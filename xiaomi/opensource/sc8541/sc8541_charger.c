// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/
/*
 * SouthChip SC8541 charge pump driver, Qualcomm IIO flavour.
 *
 * Source: cd-Crypton/android_kernel_samsung_bengal-5.15
 * drivers/power/supply/nopmi/sc8541.c (SouthChip 1.0.x_G), adapted to the
 * stock Xiaomi sapphire sc8541_charger.ko: sapphire IIO channel table
 * (incl. "sc_set_cp_workmode" for pd_policy_manager / cp_qc30), the
 * N7 cp-workmode get/set, master/slave/standalone IIO device names,
 * VBAT_INSERT/VBUS_PRESENT status fields and "sc,sc8541,ms" DT property.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

/* sapphire dt-bindings/iio/qti_power_supply_iio.h channel numbers */
#define PSY_IIO_CHARGING_ENABLED		0x76
#define PSY_IIO_STATUS				0x8f
#define PSY_IIO_PRESENT				0x90
#define PSY_IIO_SC_FAULT_STATUS			0x91
#define PSY_IIO_SC_BATTERY_PRESENT		0x92
#define PSY_IIO_SC_VBUS_PRESENT			0x93
#define PSY_IIO_SC_BATTERY_VOLTAGE		0x94
#define PSY_IIO_SC_BATTERY_CURRENT		0x95
#define PSY_IIO_SC_BATTERY_TEMPERATURE		0x96
#define PSY_IIO_SC_BUS_VOLTAGE			0x97
#define PSY_IIO_SC_BUS_CURRENT			0x98
#define PSY_IIO_SC_BUS_TEMPERATURE		0x99
#define PSY_IIO_SC_DIE_TEMPERATURE		0x9a
#define PSY_IIO_SC_ALARM_STATUS			0x9b
#define PSY_IIO_SC_VBUS_ERROR_STATUS		0x9c
#define PSY_IIO_SC_CHIP_VENDOR			0x9d
#define PSY_IIO_CHIP_ID				0xab
#define PSY_IIO_SET_BUS_PROTECTION		0xbc
#define PSY_IIO_SC_SET_CP_WORKMODE		0xe9

#define SC8541_DRV_VERSION              "1.0.0_G"

enum {
    SC8541_STANDALONG = 0,
    SC8541_MASTER,
    SC8541_SLAVE,
};

static const char* sc8541_psy_name[] = {
    [SC8541_STANDALONG] = "sc-cp-standalone",
    [SC8541_MASTER] = "sc-cp-master",
    [SC8541_SLAVE] = "sc-cp-slave",
};

static const char* sc8541_irq_name[] = {
    [SC8541_STANDALONG] = "sc8541-standalone-irq",
    [SC8541_MASTER] = "sc8541-master-irq",
    [SC8541_SLAVE] = "sc8541-slave-irq",
};

static int sc8541_mode_data[] = {
    [SC8541_STANDALONG] = SC8541_STANDALONG,
    [SC8541_MASTER] = SC8541_MASTER,
    [SC8541_SLAVE] = SC8541_SLAVE,
};

static bool is_sc8541;

enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VAC1,
    ADC_VAC2,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_TSBUS,
    ADC_TSBAT,
    ADC_TDIE,
    ADC_MAX_NUM,
}ADC_CH;

static const int sc8541_adc_m[] =
    {25, 375, 5, 5, 125, 125, 3125, 9766, 9766, 5};

static const int sc8541_adc_l[] =
    {10, 100, 1, 1, 100, 100, 1000, 100000, 100000, 10};

enum sc8541_notify {
    SC8541_NOTIFY_OTHER = 0,
	SC8541_NOTIFY_IBUSUCPF,
	SC8541_NOTIFY_VBUSOVPALM,
	SC8541_NOTIFY_VBATOVPALM,
	SC8541_NOTIFY_IBUSOCP,
	SC8541_NOTIFY_VBUSOVP,
	SC8541_NOTIFY_IBATOCP,
	SC8541_NOTIFY_VBATOVP,
	SC8541_NOTIFY_VOUTOVP,
	SC8541_NOTIFY_VDROVP,
};

enum sc8541_error_stata {
    ERROR_VBUS_HIGH = 0,
	ERROR_VBUS_LOW,
	ERROR_VBUS_OVP,
	ERROR_IBUS_OCP,
	ERROR_VBAT_OVP,
	ERROR_IBAT_OCP,
};

struct flag_bit {
    int notify;
    int mask;
    char *name;
};

struct intr_flag {
    int reg;
    int len;
    struct flag_bit bit[8];
};

static const struct intr_flag cp_intr_flag[] = {
    { .reg = 0x18, .len = 7, .bit = {
                {.mask = BIT(0), .name = "vbus ovp alm flag", .notify = SC8541_NOTIFY_VBUSOVPALM},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = SC8541_NOTIFY_VBUSOVP},
                {.mask = BIT(3), .name = "ibat ocp alm flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "ibat ocp flag", .notify = SC8541_NOTIFY_IBATOCP},
                {.mask = BIT(5), .name = "vout ovp flag", .notify = SC8541_NOTIFY_VOUTOVP},
                {.mask = BIT(6), .name = "vbat ovp alm flag", .notify = SC8541_NOTIFY_VBATOVPALM},
                {.mask = BIT(7), .name = "vbat ovp flag", .notify = SC8541_NOTIFY_VBATOVP},
                },
    },
    { .reg = 0x19, .len = 3, .bit = {
                {.mask = BIT(2), .name = "pin diag fall flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ibus ucp fall flag", .notify = SC8541_NOTIFY_IBUSUCPF},
                {.mask = BIT(7), .name = "ibus ocp flag", .notify = SC8541_NOTIFY_IBUSOCP},
                },
    },
    { .reg = 0x1a, .len = 8, .bit = {
                {.mask = BIT(0), .name = "acrb2 config flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "acrb1 config flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vac2 insert flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vac1 insert flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vat insert flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vac2 ovp flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vac1 ovp flag", .notify = SC8541_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1b, .len = 8, .bit = {
                {.mask = BIT(0), .name = "wd timeout flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "tdie alm flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "tshut flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "tsbat flt flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "tsbus flt flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "tsbus tsbat alm flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "ss timeout flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "adc done flag", .notify = SC8541_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1c, .len = 3, .bit = {
                {.mask = BIT(3), .name = "vbus errorlo flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vbus errorhi flag", .notify = SC8541_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "cp switching flag", .notify = SC8541_NOTIFY_OTHER},
                },
    },
};


/************************************************************************/
#define SC8541_DEVICE_ID                0x41

#define SC8541_REG18                    0x18
#define SC8541_REG25                    0x25
#define SC8541_REGMAX                   0x42

enum sc8541_fields {
    VBAT_OVP_DIS, VBAT_OVP,
    VBAT_OVP_ALM_DIS, VBAT_OVP_ALM,
    IBAT_OCP_DIS, IBAT_OCP,
    IBAT_OCP_ALM_DIS, IBAT_OCP_ALM,
    IBUS_UCP_DIS, VBUS_IN_RANGE_DIS,
    VBUS_PD_EN, VBUS_OVP,
    VBUS_OVP_ALM_DIS, VBUS_OVP_ALM,
    IBUS_OCP_DIS, IBUS_OCP,
    TSHUT_DIS, TSBUS_FLT_DIS, TSBAT_FLT_DIS,
    TDIE_ALM,
    TSBUS_FLT,
    TSBAT_FLT,
    VAC1_OVP, VAC2_OVP, VAC1_PD_EN, VAC2_PD_EN,
    REG_RST, OTG_EN, CHG_EN, MODE, ACDRV1_STAT, ACDRV2_STAT,
    FSW_SET, WD_TIMEOUT, WD_TIMEOUT_DIS,
    SET_IBAT_SNS_RES, SS_TIMEOUT, IBUS_UCP_FALL_DG,
    VOUT_OVP_DIS, VOUT_OVP, MS,
    VBAT_OVP_ALM_STAT, IBAT_OCP_ALM_STAT, IBUS_OCP_STAT,
    IBUS_UCP_FALL_STAT, VBAT_INSERT_STAT, VBUS_PRESENT_STAT,
    CP_SWITCHING_STAT,VBUS_ERRORHI_STAT,VBUS_ERRORLO_STAT,
    DEVICE_ID,
    ADC_EN, ADC_RATE,
    ACDRV_MANUAL_EN, ACDRV1_EN, ACDRV2_EN,
    PMID2OUT_UVP, PMID2OUT_OVP, PMID2OUT_UVP_FLAG, PMID2OUT_OVP_FLAG,
    F_MAX_FIELDS,
};


//REGISTER
static const struct reg_field sc8541_reg_fields[] = {
    /*reg00*/
    [VBAT_OVP_DIS] = REG_FIELD(0x00, 7, 7),
    [VBAT_OVP] = REG_FIELD(0x00, 0, 6),
    /*reg01*/
    [VBAT_OVP_ALM_DIS] = REG_FIELD(0x01, 7, 7),
    [VBAT_OVP_ALM] = REG_FIELD(0x01, 0, 6),
    /*reg02*/
    [IBAT_OCP_DIS] = REG_FIELD(0x02, 7, 7),
    [IBAT_OCP] = REG_FIELD(0x02, 0, 6),
    /*reg03*/
    [IBAT_OCP_ALM_DIS] = REG_FIELD(0x03, 7, 7),
    [IBAT_OCP_ALM] = REG_FIELD(0x03, 0, 6),
    /*reg05*/
    [IBUS_UCP_DIS] = REG_FIELD(0x05, 7, 7),
    [VBUS_IN_RANGE_DIS] = REG_FIELD(0x05, 2, 2),
    /*reg06*/
    [VBUS_PD_EN] = REG_FIELD(0x06, 7, 7),
    [VBUS_OVP] = REG_FIELD(0x06, 0, 6),
    /*reg07*/
    [VBUS_OVP_ALM_DIS] = REG_FIELD(0x07, 7, 7),
    [VBUS_OVP_ALM] = REG_FIELD(0x07, 0, 6),
    /*reg08*/
    [IBUS_OCP_DIS] = REG_FIELD(0x08, 7, 7),
    [IBUS_OCP] = REG_FIELD(0x08, 0, 4),
    /*reg0A*/
    [TSHUT_DIS] = REG_FIELD(0x0A, 7, 7),
    [TSBUS_FLT_DIS] = REG_FIELD(0x0A, 3, 3),
    [TSBAT_FLT_DIS] = REG_FIELD(0x0A, 2, 2),
    /*reg0B*/
    [TDIE_ALM] = REG_FIELD(0x0B, 0, 7),
    /*reg0C*/
    [TSBUS_FLT] = REG_FIELD(0x0C, 0, 7),
    /*reg0D*/
    [TSBAT_FLT] = REG_FIELD(0x0D, 0, 7),
    /*reg0E*/
    [VAC1_OVP] = REG_FIELD(0x0E, 5, 7),
    [VAC2_OVP] = REG_FIELD(0x0E, 2, 4),
    [VAC1_PD_EN] = REG_FIELD(0x0E, 1, 1),
    [VAC2_PD_EN] = REG_FIELD(0x0E, 0, 0),
    /*reg0F*/
    [REG_RST] = REG_FIELD(0x0F, 7, 7),
    [OTG_EN] = REG_FIELD(0x0F, 5, 5),
    [CHG_EN] = REG_FIELD(0x0F, 4, 4),
    [MODE] = REG_FIELD(0x0F, 3, 3),
    [ACDRV1_STAT] = REG_FIELD(0x0F, 1, 1),
    [ACDRV2_STAT] = REG_FIELD(0x0F, 0, 0),
    /*reg10*/
    [FSW_SET] = REG_FIELD(0x10, 5, 7),
    [WD_TIMEOUT] = REG_FIELD(0x10, 3, 4),
    [WD_TIMEOUT_DIS] = REG_FIELD(0x10, 2, 2),
    /*reg11*/
    [SET_IBAT_SNS_RES] = REG_FIELD(0x11, 7, 7),
    [SS_TIMEOUT] = REG_FIELD(0x11, 4, 6),
    [IBUS_UCP_FALL_DG] = REG_FIELD(0x11, 2, 3),
    /*reg12*/
    [VOUT_OVP_DIS] = REG_FIELD(0x12, 7, 7),
    [VOUT_OVP] = REG_FIELD(0x12, 5, 6),
    [MS] = REG_FIELD(0x12, 0, 1),
    /*reg15*/
    [VBAT_INSERT_STAT] = REG_FIELD(0x15, 5, 5),
    [VBUS_PRESENT_STAT] = REG_FIELD(0x15, 2, 2),
    /*reg17*/
    [CP_SWITCHING_STAT] = REG_FIELD(0x17, 6, 6),
    [VBUS_ERRORHI_STAT] = REG_FIELD(0x17, 4, 4),
    [VBUS_ERRORLO_STAT] = REG_FIELD(0x17, 3, 3),
    /*reg22*/
    [DEVICE_ID] = REG_FIELD(0x22, 0, 7),
    /*reg23*/
    [ADC_EN] = REG_FIELD(0x23, 7, 7),
    [ADC_RATE] = REG_FIELD(0x23, 6, 6),
    /*reg40*/
    [ACDRV_MANUAL_EN] = REG_FIELD(0x40, 6, 6),
    [ACDRV1_EN] = REG_FIELD(0x40, 5, 5),
    [ACDRV2_EN] = REG_FIELD(0x40, 4, 4),
    /*reg42*/
    [PMID2OUT_UVP] = REG_FIELD(0x42, 5, 7),
    [PMID2OUT_OVP] = REG_FIELD(0x42, 2, 4),
    [PMID2OUT_UVP_FLAG] = REG_FIELD(0x42, 1, 1),
    [PMID2OUT_OVP_FLAG] = REG_FIELD(0x42, 0, 0),
};

static const struct regmap_config sc8541_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = SC8541_REGMAX,
};

/************************************************************************/

struct sc8541_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp;
    int vbat_ovp_alm_dis;
    int vbat_ovp_alm;
    int ibat_ocp_dis;
    int ibat_ocp;
    int ibat_ocp_alm_dis;
    int ibat_ocp_alm;
    int ibus_ucp_dis;
    int vbus_in_range_dis;
    int vbus_pd_en;
    int vbus_ovp;
    int vbus_ovp_alm_dis;
    int vbus_ovp_alm;
    int ibus_ocp_dis;
    int ibus_ocp;
    int tshut_dis;
    int tsbus_flt_dis;
    int tsbat_flt_dis;
    int tdie_alm;
    int tsbus_flt;
    int tsbat_flt;
    int vac1_ovp;
    int vac2_ovp;
    int vac1_pd_en;
    int vac2_pd_en;
    int fsw_set;
    int wd_timeout;
    int wd_timeout_dis;
    int ibat_sns_r;
    int ss_timeout;
    int ibus_ucp_fall_dg;
    int vout_ovp_dis;
    int vout_ovp;
    int ms;
    int pmid2out_uvp;
    int pmid2out_ovp;
};

struct sc8541_chip {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];

    struct sc8541_cfg_e cfg;
    int irq_gpio;
    int irq;

    int mode;

    bool charge_enabled;
    int usb_present;
    int vbus_volt;
    int ibus_curr;
    int vbat_volt;
    int ibat_curr;
    int die_temp;

    const char *chg_dev_name;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *psy;

    struct iio_dev          *indio_dev;
    struct iio_chan_spec    *iio_chan;
    struct iio_channel      *int_iio_chans;

    bool batt_present;
    bool vbus_present;
    int vbus_error;
};

struct sc8541_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define SC8541_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define SC8541_CHAN_VOLT(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_CUR(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_TEMP(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_TEMP,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_POW(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_POWER,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_ENERGY(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_ENERGY,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define SC8541_CHAN_COUNT(_name, _num)			\
	SC8541_IIO_CHAN(_name, _num, IIO_COUNT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct sc8541_iio_channels sc8541_iio_psy_channels[] = {
	SC8541_CHAN_ENERGY("present", PSY_IIO_PRESENT)
	SC8541_CHAN_ENERGY("charging_enabled", PSY_IIO_CHARGING_ENABLED)
	SC8541_CHAN_ENERGY("status", PSY_IIO_STATUS)
	SC8541_CHAN_ENERGY("sc_battery_present", PSY_IIO_SC_BATTERY_PRESENT)
	SC8541_CHAN_ENERGY("sc_vbus_present", PSY_IIO_SC_VBUS_PRESENT)
	SC8541_CHAN_VOLT("sc_battery_voltage", PSY_IIO_SC_BATTERY_VOLTAGE)
	SC8541_CHAN_CUR("sc_battery_current", PSY_IIO_SC_BATTERY_CURRENT)
	SC8541_CHAN_TEMP("sc_battery_temperature", PSY_IIO_SC_BATTERY_TEMPERATURE)
	SC8541_CHAN_VOLT("sc_bus_voltage", PSY_IIO_SC_BUS_VOLTAGE)
	SC8541_CHAN_CUR("sc_bus_current", PSY_IIO_SC_BUS_CURRENT)
	SC8541_CHAN_TEMP("sc_bus_temperature", PSY_IIO_SC_BUS_TEMPERATURE)
	SC8541_CHAN_TEMP("sc_die_temperature", PSY_IIO_SC_DIE_TEMPERATURE)
	SC8541_CHAN_ENERGY("sc_alarm_status", PSY_IIO_SC_ALARM_STATUS)
	SC8541_CHAN_ENERGY("sc_fault_status", PSY_IIO_SC_FAULT_STATUS)
	SC8541_CHAN_ENERGY("sc_vbus_error_status", PSY_IIO_SC_VBUS_ERROR_STATUS)
	SC8541_CHAN_ENERGY("sc_chip_vendor", PSY_IIO_SC_CHIP_VENDOR)
	SC8541_CHAN_ENERGY("sc_chip_id", PSY_IIO_CHIP_ID)
	SC8541_CHAN_ENERGY("set_bus_protection", PSY_IIO_SET_BUS_PROTECTION)
	SC8541_CHAN_ENERGY("sc_set_cp_workmode", PSY_IIO_SC_SET_CP_WORKMODE)
};

static const struct sc8541_iio_channels sc8541_slave_iio_psy_channels[] = {
	SC8541_CHAN_ENERGY("present_slave", PSY_IIO_PRESENT)
	SC8541_CHAN_ENERGY("charging_enabled_slave", PSY_IIO_CHARGING_ENABLED)
	SC8541_CHAN_ENERGY("status_slave", PSY_IIO_STATUS)
	SC8541_CHAN_ENERGY("sc_battery_present_slave", PSY_IIO_SC_BATTERY_PRESENT)
	SC8541_CHAN_ENERGY("sc_vbus_present_slave", PSY_IIO_SC_VBUS_PRESENT)
	SC8541_CHAN_VOLT("sc_battery_voltage_slave", PSY_IIO_SC_BATTERY_VOLTAGE)
	SC8541_CHAN_CUR("sc_battery_current_slave", PSY_IIO_SC_BATTERY_CURRENT)
	SC8541_CHAN_TEMP("sc_battery_temperature_slave", PSY_IIO_SC_BATTERY_TEMPERATURE)
	SC8541_CHAN_VOLT("sc_bus_voltage_slave", PSY_IIO_SC_BUS_VOLTAGE)
	SC8541_CHAN_CUR("sc_bus_current_slave", PSY_IIO_SC_BUS_CURRENT)
	SC8541_CHAN_TEMP("sc_bus_temperature_slave", PSY_IIO_SC_BUS_TEMPERATURE)
	SC8541_CHAN_TEMP("sc_die_temperature_slave", PSY_IIO_SC_DIE_TEMPERATURE)
	SC8541_CHAN_ENERGY("sc_alarm_status_slave", PSY_IIO_SC_ALARM_STATUS)
	SC8541_CHAN_ENERGY("sc_fault_status_slave", PSY_IIO_SC_FAULT_STATUS)
	SC8541_CHAN_ENERGY("sc_vbus_error_status_slave", PSY_IIO_SC_VBUS_ERROR_STATUS)
	SC8541_CHAN_ENERGY("sc_chip_vendor_slave", PSY_IIO_SC_CHIP_VENDOR)
	SC8541_CHAN_ENERGY("sc_chip_id", PSY_IIO_CHIP_ID)
	SC8541_CHAN_ENERGY("set_bus_protection_slave", PSY_IIO_SET_BUS_PROTECTION)
};

/*********************************************************/
static int sc8541_field_read(struct sc8541_chip *sc,
                enum sc8541_fields field_id, int *val)
{
    int ret;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541 read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8541_field_write(struct sc8541_chip *sc,
                enum sc8541_fields field_id, int val)
{
    int ret;

    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541 read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int sc8541_read_block(struct sc8541_chip *sc,
                int reg, uint8_t *val, int len)
{
    int ret;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev, "sc8541 read %02x block failed %d\n", reg, ret);
    }

    return ret;
}



/*******************************************************/
static int sc8541_detect_device(struct sc8541_chip *sc)
{
    int ret;
    int val;

    ret = sc8541_field_read(sc, DEVICE_ID, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    if (val != SC8541_DEVICE_ID) {
        dev_err(sc->dev, "%s not find SC8541, ID = 0x%02x\n", __func__, ret);
        return -EINVAL;
    }

    return ret;
}

static int sc8541_reg_reset(struct sc8541_chip *sc)
{
    return sc8541_field_write(sc, REG_RST, 1);
}

static int sc8541_dump_reg(struct sc8541_chip *sc)
{
    int ret;
    int i;
    int val = 0;

    for (i = 0; i <= SC8541_REGMAX; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        dev_err(sc->dev, "%s reg[0x%02x] = 0x%02x\n",
                __func__, i, val);
    }

    return ret;
}

static int sc8541_enable_charge(struct sc8541_chip *sc, bool en)
{
    int ret;
    dev_info(sc->dev,"%s:%d",__func__,en);

    ret = sc8541_field_write(sc, CHG_EN, !!en);

    return ret;
}

static int sc8541_check_charge_enabled(struct sc8541_chip *sc, bool *enabled)
{
    int ret, val = 0;

    ret = sc8541_field_read(sc, CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    return ret;
}

static int sc8541_get_status(struct sc8541_chip *sc, uint32_t *status)
{
    int ret, val = 0;
    *status = 0;

    ret = sc8541_field_read(sc, VBUS_ERRORHI_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    ret = sc8541_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORLO_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_LOW);


    return ret;

}

static int sc8541_enable_adc(struct sc8541_chip *sc, bool en)
{
    dev_info(sc->dev,"%s:%d", __func__, en);
    return sc8541_field_write(sc, ADC_EN, !!en);
}

static int sc8541_get_adc_data(struct sc8541_chip *sc,
            int channel, int *result)
{
    uint8_t val[2] = {0};
    int ret;

    if(channel >= ADC_MAX_NUM)
        return -EINVAL;

    ret = sc8541_read_block(sc, SC8541_REG25 + (channel << 1), val, 2);
    if (ret < 0) {
        return ret;
    }

    *result = (val[1] | (val[0] << 8)) *
                sc8541_adc_m[channel] / sc8541_adc_l[channel];

    return ret;
}

static int sc8541_set_cp_workmode(struct sc8541_chip *sc, int workmode)
{
    int ret;

    dev_info(sc->dev, "N7:%s:%d", __func__, workmode);

    ret = sc8541_field_write(sc, MODE, workmode);

    return ret;
}

static noinline int sc8541_get_cp_workmode(struct sc8541_chip *sc, int *workmode)
{
    int ret;
    int val = 0;

    ret = sc8541_field_read(sc, MODE, &val);

    *workmode = val;
    dev_info(sc->dev, "N7:%s:%d", __func__, *workmode);

    return ret;
}

static int sc8541_init_device(struct sc8541_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc8541_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.vbat_ovp_dis},
        {VBAT_OVP, sc->cfg.vbat_ovp},
        {VBAT_OVP_ALM_DIS, sc->cfg.vbat_ovp_alm_dis},
        {VBAT_OVP_ALM, sc->cfg.vbat_ovp_alm},
        {IBAT_OCP_DIS, sc->cfg.ibat_ocp_dis},
        {IBAT_OCP, sc->cfg.ibat_ocp},
        {IBAT_OCP_ALM_DIS, sc->cfg.ibat_ocp_alm_dis},
        {IBAT_OCP_ALM, sc->cfg.ibat_ocp_alm},
        {IBUS_UCP_DIS, sc->cfg.ibus_ucp_dis},
        {VBUS_IN_RANGE_DIS, sc->cfg.vbus_in_range_dis},
        {VBUS_PD_EN, sc->cfg.vbus_pd_en},
        {VBUS_OVP, sc->cfg.vbus_ovp},
        {VBUS_OVP_ALM_DIS, sc->cfg.vbus_ovp_alm_dis},
        {VBUS_OVP_ALM, sc->cfg.vbus_ovp_alm},
        {IBUS_OCP_DIS, sc->cfg.ibus_ocp_dis},
        {IBUS_OCP, sc->cfg.ibus_ocp},
        {TSHUT_DIS, sc->cfg.tshut_dis},
        {TSBUS_FLT_DIS, sc->cfg.tsbus_flt_dis},
        {TSBAT_FLT_DIS, sc->cfg.tsbat_flt_dis},
        {TDIE_ALM, sc->cfg.tdie_alm},
        {TSBUS_FLT, sc->cfg.tsbus_flt},
        {TSBAT_FLT, sc->cfg.tsbat_flt},
        {VAC1_OVP, sc->cfg.vac1_ovp},
        {VAC2_OVP, sc->cfg.vac2_ovp},
        {VAC1_PD_EN, sc->cfg.vac1_pd_en},
        {VAC2_PD_EN, sc->cfg.vac2_pd_en},
        {FSW_SET, sc->cfg.fsw_set},
        {WD_TIMEOUT, sc->cfg.wd_timeout},
        {WD_TIMEOUT_DIS, sc->cfg.wd_timeout_dis},
        {SET_IBAT_SNS_RES, sc->cfg.ibat_sns_r},
        {SS_TIMEOUT, sc->cfg.ss_timeout},
        {IBUS_UCP_FALL_DG, sc->cfg.ibus_ucp_fall_dg},
        {VOUT_OVP_DIS, sc->cfg.vout_ovp_dis},
        {VOUT_OVP, sc->cfg.vout_ovp},
        {MS, sc->cfg.ms},
        {PMID2OUT_UVP, sc->cfg.pmid2out_uvp},
        {PMID2OUT_OVP, sc->cfg.pmid2out_ovp},
    };

    ret = sc8541_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc8541_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    sc8541_enable_adc(sc, true);

    if (sc->mode == SC8541_SLAVE) {
        ret = sc8541_field_write(sc, VBUS_IN_RANGE_DIS, 1);
        if (ret < 0) {
            dev_err(sc->dev, "%s Failed to set vbus in range(%d)\n", __func__, ret);
        }
    }

    sc8541_dump_reg(sc);

    return ret;
}

/********************creat devices note start*************************************************/
static ssize_t sc8541_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8541_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8541");
    for (addr = 0x0; addr <= SC8541_REGMAX; addr++) {
        ret = regmap_read(sc->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc8541_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8541_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= SC8541_REGMAX)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc8541_show_registers, sc8541_store_register);

static void sc8541_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
}
/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
static void sc8541_check_fault_status(struct sc8541_chip *sc)
{
    int ret;
    u8 flag = 0;
    int i,j;

    for (i=0;i < ARRAY_SIZE(cp_intr_flag);i++) {
        ret = sc8541_read_block(sc, cp_intr_flag[i].reg, &flag, 1);
        for (j=0; j <  cp_intr_flag[i].len; j++) {
            if (flag & cp_intr_flag[i].bit[j].mask) {
                dev_err(sc->dev,"trigger :%s\n",cp_intr_flag[i].bit[j].name);
            }
        }
    }
}

static irqreturn_t sc8541_irq_handler(int irq, void *data)
{
    struct sc8541_chip *sc = data;

    dev_err(sc->dev,"INT OCCURED\n");

    sc8541_check_fault_status(sc);

    power_supply_changed(sc->psy);

    return IRQ_HANDLED;
}

static int sc8541_register_interrupt(struct sc8541_chip *sc)
{
    int ret;

    if (gpio_is_valid(sc->irq_gpio)) {
        ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN,"sc8541_irq");
        if (ret) {
            dev_err(sc->dev,"failed to request sc8541_irq\n");
            return -EINVAL;
        }
        sc->irq = gpio_to_irq(sc->irq_gpio);
        if (sc->irq < 0) {
            dev_err(sc->dev,"failed to gpio_to_irq\n");
            return -EINVAL;
        }
    } else {
        dev_err(sc->dev,"irq gpio not provided\n");
        return -EINVAL;
    }

    if (sc->irq) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
                NULL, sc8541_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                sc8541_irq_name[sc->mode], sc);

        if (ret < 0) {
            dev_err(sc->dev,"request irq for irq=%d failed, ret =%d\n",
                            sc->irq, ret);
            return ret;
        }
        enable_irq_wake(sc->irq);
    }

    return ret;
}
/********************interrupte end*************************************************/


/************************psy start**************************************/
static enum power_supply_property sc8541_charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
};

static int sc8541_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8541_chip *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8541_check_charge_enabled(sc, &sc->charge_enabled);
        val->intval = sc->charge_enabled;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8541_get_adc_data(sc, ADC_VBUS, &result);
        if (!ret)
            sc->vbus_volt = result;
        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8541_get_adc_data(sc, ADC_IBUS, &result);
        if (!ret)
            sc->ibus_curr = result;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = sc8541_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = sc8541_get_adc_data(sc, ADC_IBAT, &result);
        if (!ret)
            sc->ibat_curr = result;
        val->intval = sc->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8541_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int sc8541_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    struct sc8541_chip *sc = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8541_enable_charge(sc, val->intval);
        dev_info(sc->dev, "POWER_SUPPLY_PROP_ONLINE: %s\n",
                val->intval ? "enable" : "disable");
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int sc8541_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    return 0;
}

static int sc8541_psy_register(struct sc8541_chip *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc8541_psy_name[sc->mode];

    sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    sc->psy_desc.properties = sc8541_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8541_charger_props);
    sc->psy_desc.get_property = sc8541_charger_get_property;
    sc->psy_desc.set_property = sc8541_charger_set_property;
    sc->psy_desc.property_is_writeable = sc8541_charger_is_writeable;


    sc->psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->psy)) {
        dev_err(sc->dev, "%s failed to register psy\n", __func__);
        return PTR_ERR(sc->psy);
    }

    dev_info(sc->dev, "%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}


/************************psy end**************************************/

/************************iio start**************************************/

static int sc8541_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct sc8541_chip *sc = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
		case PSY_IIO_PRESENT:
			break;
		case PSY_IIO_CHARGING_ENABLED:
			sc8541_enable_charge(sc, val1);
			sc8541_check_charge_enabled(sc, &sc->charge_enabled);
			pr_err("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
					val1 ? "enable" : "disable");
			break;
		case PSY_IIO_SC_SET_CP_WORKMODE:
			sc8541_set_cp_workmode(sc, val1);
			pr_info("sc8541 iio set workmode =%d", val1);
			break;
		case PSY_IIO_SET_BUS_PROTECTION:
			pr_err("hvdcp3_type: %d\n", val1);
			break;
		default:
			rc = -EINVAL;
			break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int sc8541_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct sc8541_chip *sc = iio_priv(indio_dev);
	int result = 0;
	int val = 0;
	int ret = 0;
	*val1 = 0;

	switch (chan->channel) {
		case PSY_IIO_CHARGING_ENABLED:
			sc8541_check_charge_enabled(sc, &sc->charge_enabled);
			*val1 = sc->charge_enabled;
			break;
		case PSY_IIO_STATUS:
			break;
		case PSY_IIO_PRESENT:
			*val1 = sc->usb_present;
			break;
		case PSY_IIO_SC_BATTERY_PRESENT:
			ret = sc8541_field_read(sc, VBAT_INSERT_STAT, &val);
			if (!ret)
				sc->batt_present = !!val;
			*val1 = sc->batt_present;
			break;
		case PSY_IIO_SC_VBUS_PRESENT:
			ret = sc8541_field_read(sc, VBUS_PRESENT_STAT, &val);
			if (!ret)
				sc->vbus_present = !!val;
			*val1 = sc->vbus_present;
			break;
		case PSY_IIO_SC_BATTERY_VOLTAGE:
			ret = sc8541_get_adc_data(sc, ADC_VBAT, &result);
	        if (!ret)
	            sc->vbat_volt = result;
			*val1 = sc->vbat_volt;
			break;
		case PSY_IIO_SC_BATTERY_CURRENT:
			ret = sc8541_get_adc_data(sc, ADC_IBAT, &result);
	        if (!ret)
	            sc->ibat_curr = result;
			*val1 = sc->ibat_curr;
			break;
		case PSY_IIO_SC_BATTERY_TEMPERATURE:
			break;
		case PSY_IIO_SC_BUS_VOLTAGE:
			ret = sc8541_get_adc_data(sc, ADC_VBUS, &result);
	        if (!ret)
	            sc->vbus_volt = result;
			*val1 = sc->vbus_volt;
			break;
		case PSY_IIO_SC_BUS_CURRENT:
			ret = sc8541_get_adc_data(sc, ADC_IBUS, &result);
	        if (!ret)
	            sc->ibus_curr = result;
			*val1 = sc->ibus_curr;
			break;
		case PSY_IIO_SC_BUS_TEMPERATURE:
			break;
		case PSY_IIO_SC_DIE_TEMPERATURE:
			ret = sc8541_get_adc_data(sc, ADC_TDIE, &result);
	        if (!ret)
	            sc->die_temp = result;
			*val1= sc->die_temp;
			break;
		case PSY_IIO_SC_ALARM_STATUS:
			break;
		case PSY_IIO_SC_FAULT_STATUS:
			break;
		case PSY_IIO_SC_VBUS_ERROR_STATUS:
			ret = sc8541_get_status(sc, (uint32_t *)&sc->vbus_error);
			*val1 = sc->vbus_error;
			break;
		case PSY_IIO_SC_CHIP_VENDOR:
			break;
		case PSY_IIO_CHIP_ID:
			ret = sc8541_field_read(sc, DEVICE_ID, &val);
			*val1 = val;
			break;
		case PSY_IIO_SC_SET_CP_WORKMODE:
			ret = sc8541_get_cp_workmode(sc, &val);
			*val1 = val;
			pr_info("sc8541 iio get workmode =%d", *val1);
			break;
		default:
			ret = -EINVAL;
			 break;
	}

	if (ret < 0) {
		pr_err("Couldn't read IIO channel %d, ret = %d\n",
			chan->channel, ret);
		return ret;
	}

	return IIO_VAL_INT;
}


static int sc8541_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct sc8541_chip *sc = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = sc->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(sc8541_iio_psy_channels);
					i++, iio_chan++) {
		if (iio_chan->channel == iiospec->args[0])
			return i;
	}
	return -EINVAL;
}

static const struct iio_info sc8541_iio_info = {
	.read_raw	= sc8541_iio_read_raw,
	.write_raw	= sc8541_iio_write_raw,
	.of_xlate	= sc8541_iio_of_xlate,
};

static int sc8541_init_iio_psy(struct sc8541_chip *chip)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int num_iio_channels = ARRAY_SIZE(sc8541_iio_psy_channels);
	int rc, i;

	pr_err("sc8541_init_iio_psy start\n");
	chip->iio_chan = devm_kcalloc(chip->dev, num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &sc8541_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = num_iio_channels;

	if (chip->mode == SC8541_MASTER) {
		indio_dev->name = "sc8541-master";
		for (i = 0; i < num_iio_channels; i++) {
			chip->int_iio_chans[i].indio_dev = indio_dev;
			chan = &chip->iio_chan[i];
			chip->int_iio_chans[i].channel = chan;
			chan->address = i;
			chan->channel = sc8541_iio_psy_channels[i].channel_num;
			chan->type = sc8541_iio_psy_channels[i].type;
			chan->datasheet_name =
				sc8541_iio_psy_channels[i].datasheet_name;
			chan->extend_name =
				sc8541_iio_psy_channels[i].datasheet_name;
			chan->info_mask_separate =
				sc8541_iio_psy_channels[i].info_mask;
		}
	} else if (chip->mode == SC8541_SLAVE) {
		indio_dev->name = "sc8541-slave";
		/* stock iterates the master channel count and overruns the slave table */
		for (i = 0; i < min_t(int, num_iio_channels, ARRAY_SIZE(sc8541_slave_iio_psy_channels)); i++) {
			chip->int_iio_chans[i].indio_dev = indio_dev;
			chan = &chip->iio_chan[i];
			chip->int_iio_chans[i].channel = chan;
			chan->address = i;
			chan->channel = sc8541_slave_iio_psy_channels[i].channel_num;
			chan->type = sc8541_slave_iio_psy_channels[i].type;
			chan->datasheet_name =
				sc8541_slave_iio_psy_channels[i].datasheet_name;
			chan->extend_name =
				sc8541_slave_iio_psy_channels[i].datasheet_name;
			chan->info_mask_separate =
				sc8541_slave_iio_psy_channels[i].info_mask;
		}
	} else {
		indio_dev->name = "sc8541-standalone";
		for (i = 0; i < num_iio_channels; i++) {
			chip->int_iio_chans[i].indio_dev = indio_dev;
			chan = &chip->iio_chan[i];
			chip->int_iio_chans[i].channel = chan;
			chan->address = i;
			chan->channel = sc8541_iio_psy_channels[i].channel_num;
			chan->type = sc8541_iio_psy_channels[i].type;
			chan->datasheet_name =
				sc8541_iio_psy_channels[i].datasheet_name;
			chan->extend_name =
				sc8541_iio_psy_channels[i].datasheet_name;
			chan->info_mask_separate =
				sc8541_iio_psy_channels[i].info_mask;
		}
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("Failed to register SC8541 IIO device, rc=%d\n", rc);

	pr_err("SC8541 IIO device, rc=%d\n", rc);
	return rc;
}

/************************iio end**************************************/

static int sc8541_set_work_mode(struct sc8541_chip *sc, int mode)
{
    sc->mode = mode;

    dev_err(sc->dev,"work mode is %s\n", sc->mode == SC8541_STANDALONG
        ? "standalone" : (sc->mode == SC8541_MASTER ? "master" : "slave"));

    return 0;
}

static int sc8541_parse_dt(struct sc8541_chip *sc, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc8541,vbat-ovp-dis", &(sc->cfg.vbat_ovp_dis)},
        {"sc,sc8541,vbat-ovp", &(sc->cfg.vbat_ovp)},
        {"sc,sc8541,vbat-ovp-alm-dis", &(sc->cfg.vbat_ovp_alm_dis)},
        {"sc,sc8541,vbat-ovp-alm", &(sc->cfg.vbat_ovp_alm)},
        {"sc,sc8541,ibat-ocp-dis", &(sc->cfg.ibat_ocp_dis)},
        {"sc,sc8541,ibat-ocp", &(sc->cfg.ibat_ocp)},
        {"sc,sc8541,ibat-ocp-alm-dis", &(sc->cfg.ibat_ocp_alm_dis)},
        {"sc,sc8541,ibat-ocp-alm", &(sc->cfg.ibat_ocp_alm)},
        {"sc,sc8541,ibus-ucp-dis", &(sc->cfg.ibus_ucp_dis)},
        {"sc,sc8541,vbus-in-range-dis", &(sc->cfg.vbus_in_range_dis)},
        {"sc,sc8541,vbus-pd-en", &(sc->cfg.vbus_pd_en)},
        {"sc,sc8541,vbus-ovp", &(sc->cfg.vbus_ovp)},
        {"sc,sc8541,vbus-ovp-alm-dis", &(sc->cfg.vbus_ovp_alm_dis)},
        {"sc,sc8541,vbus-ovp-alm", &(sc->cfg.vbus_ovp_alm)},
        {"sc,sc8541,ibus-ocp-dis", &(sc->cfg.ibus_ocp_dis)},
        {"sc,sc8541,ibus-ocp", &(sc->cfg.ibus_ocp)},
        {"sc,sc8541,tshut-dis", &(sc->cfg.tshut_dis)},
        {"sc,sc8541,tsbus-flt-dis", &(sc->cfg.tsbus_flt_dis)},
        {"sc,sc8541,tsbat-flt-dis", &(sc->cfg.tsbat_flt_dis)},
        {"sc,sc8541,tdie-alm", &(sc->cfg.tdie_alm)},
        {"sc,sc8541,tsbus-flt", &(sc->cfg.tsbus_flt)},
        {"sc,sc8541,tsbat-flt", &(sc->cfg.tsbat_flt)},
        {"sc,sc8541,vac1-ovp", &(sc->cfg.vac1_ovp)},
        {"sc,sc8541,vac2-ovp", &(sc->cfg.vac2_ovp)},
        {"sc,sc8541,vac1-pd-en", &(sc->cfg.vac1_pd_en)},
        {"sc,sc8541,vac2-pd-en", &(sc->cfg.vac2_pd_en)},
        {"sc,sc8541,fsw-set", &(sc->cfg.fsw_set)},
        {"sc,sc8541,wd-timeout", &(sc->cfg.wd_timeout)},
        {"sc,sc8541,wd-timeout-dis", &(sc->cfg.wd_timeout_dis)},
        {"sc,sc8541,ibat-sns-r", &(sc->cfg.ibat_sns_r)},
        {"sc,sc8541,ss-timeout", &(sc->cfg.ss_timeout)},
        {"sc,sc8541,ibus-ucp-fall-dg", &(sc->cfg.ibus_ucp_fall_dg)},
        {"sc,sc8541,vout-ovp-dis", &(sc->cfg.vout_ovp_dis)},
        {"sc,sc8541,vout-ovp", &(sc->cfg.vout_ovp)},
        {"sc,sc8541,ms", &(sc->cfg.ms)},
        {"sc,sc8541,pmid2out-uvp", &(sc->cfg.pmid2out_uvp)},
        {"sc,sc8541,pmid2out-ovp", &(sc->cfg.pmid2out_ovp)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            return ret;
        }
    }

    sc->irq_gpio = of_get_named_gpio(np, "sc8541,intr_gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        dev_err(sc->dev,"fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }

    if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
        sc->chg_dev_name = "charger";
        dev_err(sc->dev,"no charger name\n");
    }

    return 0;
}

/* stock sapphire has no terminating sentinel; we add it */
static struct of_device_id sc8541_charger_match_table[] = {
    {   .compatible = "sc,sc8541-standalone",
        .data = &sc8541_mode_data[SC8541_STANDALONG], },
    {   .compatible = "sc,sc8541-master",
        .data = &sc8541_mode_data[SC8541_MASTER], },
    {   .compatible = "sc,sc8541-slave",
        .data = &sc8541_mode_data[SC8541_SLAVE], },
    {},
};

static int sc8541_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc8541_chip *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret, i;
	struct iio_dev *indio_dev;

    dev_err(&client->dev, "%s (%s)\n", __func__, SC8541_DRV_VERSION);

    is_sc8541 = true;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*sc));

	sc = iio_priv(indio_dev);
	sc->indio_dev = indio_dev;

    sc->dev = &client->dev;
    sc->client = client;

    sc->regmap = devm_regmap_init_i2c(client,
                            &sc8541_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        ret = PTR_ERR(sc->regmap);
        goto err_regmap_init;
    }

    for (i = 0; i < ARRAY_SIZE(sc8541_reg_fields); i++) {
        const struct reg_field *reg_fields = sc8541_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            ret = PTR_ERR(sc->rmap_fields[i]);
            goto err_regmap_field;
        }
    }

    ret = sc8541_detect_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s detect device fail\n", __func__);
        is_sc8541 = false;
        ret = 0;
        goto err_detect_dev;
    }

    i2c_set_clientdata(client, sc);
    sc8541_create_device_node(&(client->dev));

    match = of_match_node(sc8541_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        ret = -ENODEV;
        goto err_match_node;
    }

    sc8541_set_work_mode(sc, *(int *)match->data);
    if (ret) {
        dev_err(sc->dev,"Fail to set work mode!\n");
        goto err_set_mode;
    }

    ret = sc8541_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        ret = -EIO;
        goto err_parse_dt;
    }

    ret = sc8541_init_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = sc8541_psy_register(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s psy register failed(%d)\n", __func__, ret);
        goto err_register_psy;
    }

    ret = sc8541_register_interrupt(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

	ret = sc8541_init_iio_psy(sc);
	if (ret)
		goto err_register_iio;

    dev_err(sc->dev, "sc8541[%s] probe successfully!\n",
            sc->mode == SC8541_MASTER ? "master" : "slave");
    return 0;

err_register_iio:
err_register_psy:
err_register_irq:
err_init_device:
    power_supply_unregister(sc->psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
err_regmap_init:
err_regmap_field:
    dev_err(&client->dev,"sc8541 probe fail\n");
    return ret;
}


static int sc8541_charger_remove(struct i2c_client *client)
{
    struct sc8541_chip *sc = i2c_get_clientdata(client);

    power_supply_unregister(sc->psy);
    devm_kfree(&client->dev, sc);
    return 0;
}

static void sc8541_charger_shutdown(struct i2c_client *client)
{
    struct sc8541_chip *sc = i2c_get_clientdata(client);

    if (is_sc8541 == true)
        sc8541_enable_adc(sc, false);
}

#ifdef CONFIG_PM_SLEEP
static int sc8541_suspend(struct device *dev)
{
    struct sc8541_chip *sc = dev_get_drvdata(dev);

    if (is_sc8541 == true) {
        dev_info(sc->dev, "Suspend successfully!");
        if (device_may_wakeup(dev))
            enable_irq_wake(sc->irq);
        disable_irq(sc->irq);
    }

    return 0;
}
static int sc8541_resume(struct device *dev)
{
    struct sc8541_chip *sc = dev_get_drvdata(dev);

    if (is_sc8541 == true) {
        dev_info(sc->dev, "Resume successfully!");
        if (device_may_wakeup(dev))
            disable_irq_wake(sc->irq);
        enable_irq(sc->irq);
    }

    return 0;
}

static const struct dev_pm_ops sc8541_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc8541_suspend, sc8541_resume)
};
#endif

static struct i2c_driver sc8541_charger_driver = {
    .driver     = {
        .name   = "sc8541",
        .owner  = THIS_MODULE,
        .of_match_table = sc8541_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc8541_pm,
#endif
    },
    .probe      = sc8541_charger_probe,
    .remove     = sc8541_charger_remove,
    .shutdown   = sc8541_charger_shutdown,
};

module_i2c_driver(sc8541_charger_driver);

MODULE_DESCRIPTION("SC SC8541 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
