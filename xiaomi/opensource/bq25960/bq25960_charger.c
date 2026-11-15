// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/
/*
 * TI BQ25960 charge pump driver for sapphire, built on the SouthChip
 * sc8541 driver skeleton (author Aiden-yu@southchip.com).
 *
 * Source: cd-Crypton/android_kernel_samsung_bengal-5.15
 * drivers/power/supply/nopmi/sc8541.c, renamed to bq25960 and adapted to
 * the stock Xiaomi sapphire bq25960_charger.ko: BQ25960 register map
 * (IBUS_UCP/CHG_CONFIG_2/HIZ/BYPASS/ACDRV fields), "bq,bq25960,*" DT
 * properties, standalone-only "ti-cp-standalone" psy, sapphire IIO
 * channel table (incl. "bq_set_acdrv" and "bq_set_cp_workmode"),
 * N7 cp-workmode get/set via BYPASS_EN, VAC OVP adjust.
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
#include <linux/mutex.h>

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
#define PSY_IIO_SET_ACDRV			0xc4
#define PSY_IIO_SC_SET_CP_WORKMODE		0xe9

#define BQ25960_DRV_VERSION              "1.0.0_G"

enum {
    BQ25960 = 0,
};

static const char* bq25960_psy_name[] = {
    [BQ25960] = "ti-cp-standalone",
};

static const char* bq25960_irq_name[] = {
    [BQ25960] = "ti-cp-standalone-irq",
};

static int bq25960_mode_data[] = {
    [BQ25960] = BQ25960,
};

static bool is_bq25960;

/* ADC channels (register = BQ25960_REG25 + 2 * channel) */
#define ADC_IBUS	0
#define ADC_VBUS	1
#define ADC_VAC1	2
#define ADC_VAC2	3
#define ADC_VOUT	4
#define ADC_VBAT	5
#define ADC_IBAT	6
#define ADC_TSBUS	7
#define ADC_TSBAT	8
#define ADC_TDIE	9
#define ADC_MAX_NUM	10

enum bq25960_notify {
    BQ25960_NOTIFY_OTHER = 0,
	BQ25960_NOTIFY_IBUSUCPF,
	BQ25960_NOTIFY_VBUSOVPALM,
	BQ25960_NOTIFY_VBATOVPALM,
	BQ25960_NOTIFY_IBUSOCP,
	BQ25960_NOTIFY_VBUSOVP,
	BQ25960_NOTIFY_IBATOCP,
	BQ25960_NOTIFY_VBATOVP,
	BQ25960_NOTIFY_VOUTOVP,
	BQ25960_NOTIFY_VDROVP,
};

enum bq25960_error_stata {
    ERROR_VBUS_HIGH = 0,
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
                {.mask = BIT(0), .name = "vbus ovp alm flag", .notify = BQ25960_NOTIFY_VBUSOVPALM},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = BQ25960_NOTIFY_VBUSOVP},
                {.mask = BIT(3), .name = "ibat ocp alm flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "ibat ocp flag", .notify = BQ25960_NOTIFY_IBATOCP},
                {.mask = BIT(5), .name = "vout ovp flag", .notify = BQ25960_NOTIFY_VOUTOVP},
                {.mask = BIT(6), .name = "vbat ovp alm flag", .notify = BQ25960_NOTIFY_VBATOVPALM},
                {.mask = BIT(7), .name = "vbat ovp flag", .notify = BQ25960_NOTIFY_VBATOVP},
                },
    },
    { .reg = 0x19, .len = 3, .bit = {
                {.mask = BIT(2), .name = "pin diag fall flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ibus ucp fall flag", .notify = BQ25960_NOTIFY_IBUSUCPF},
                {.mask = BIT(7), .name = "ibus ocp flag", .notify = BQ25960_NOTIFY_IBUSOCP},
                },
    },
    { .reg = 0x1a, .len = 8, .bit = {
                {.mask = BIT(0), .name = "acrb2 config flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "acrb1 config flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vac2 insert flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vac1 insert flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vat insert flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vac2 ovp flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vac1 ovp flag", .notify = BQ25960_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1b, .len = 8, .bit = {
                {.mask = BIT(0), .name = "wd timeout flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "tdie alm flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "tshut flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "tsbat flt flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "tsbus flt flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "tsbus tsbat alm flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "ss timeout flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "adc done flag", .notify = BQ25960_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1c, .len = 2, .bit = {
                {.mask = BIT(4), .name = "vbus errorhi flag", .notify = BQ25960_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "cp switching flag", .notify = BQ25960_NOTIFY_OTHER},
                },
    },
};


/************************************************************************/
#define BQ25960_DEVICE_ID                0x00

#define BQ25960_REG18                    0x18
#define BQ25960_REG25                    0x25
#define BQ25960_REGMAX                   0x37

enum bq25960_fields {
    VBAT_OVP_DIS, VBAT_OVP,
    VBAT_OVP_ALM_DIS, VBAT_OVP_ALM,
    IBAT_OCP_DIS, IBAT_OCP,
    IBAT_OCP_ALM_DIS, IBAT_OCP_ALM,
    IBUS_UCP_DIS, IBUS_UCP, CHG_CONFIG_2,
    VBUS_PD_EN, VBUS_OVP,
    VBUS_OVP_ALM_DIS, VBUS_OVP_ALM,
    IBUS_OCP,
    TSHUT_DIS, TSBUS_FLT_DIS, TSBAT_FLT_DIS,
    TDIE_ALM,
    TSBUS_FLT,
    TSBAT_FLT,
    VAC1_OVP, VAC2_OVP, VAC1_PD_EN, VAC2_PD_EN,
    REG_RST, HIZ_EN, OTG_EN, CHG_EN, BYPASS_EN, ACDRV_BOTH,
    ACDRV1_STAT, ACDRV2_STAT,
    FSW_SET, WD_TIMEOUT, WD_TIMEOUT_DIS,
    SET_IBAT_SNS_RES, SS_TIMEOUT, IBUS_UCP_FALL_DG,
    VOUT_OVP_DIS, VOUT_OVP, MS,
    VBAT_OVP_ALM_STAT, IBAT_OCP_ALM_STAT, IBUS_OCP_STAT,
    IBUS_UCP_FALL_STAT, VBAT_INSERT_STAT, VBUS_PRESENT_STAT,
    CP_SWITCHING_STAT, VBUS_ERRORHI_STAT,
    DEVICE_ID,
    ADC_EN, ADC_RATE,
    F_MAX_FIELDS,
};


//REGISTER
static const struct reg_field bq25960_reg_fields[] = {
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
    [IBUS_UCP] = REG_FIELD(0x05, 6, 6),
    [CHG_CONFIG_2] = REG_FIELD(0x05, 3, 3),
    /*reg06*/
    [VBUS_PD_EN] = REG_FIELD(0x06, 7, 7),
    [VBUS_OVP] = REG_FIELD(0x06, 0, 6),
    /*reg07*/
    [VBUS_OVP_ALM_DIS] = REG_FIELD(0x07, 7, 7),
    [VBUS_OVP_ALM] = REG_FIELD(0x07, 0, 6),
    /*reg08*/
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
    [HIZ_EN] = REG_FIELD(0x0F, 6, 6),
    [OTG_EN] = REG_FIELD(0x0F, 5, 5),
    [CHG_EN] = REG_FIELD(0x0F, 4, 4),
    [BYPASS_EN] = REG_FIELD(0x0F, 3, 3),
    [ACDRV_BOTH] = REG_FIELD(0x0F, 2, 2),
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
    /*reg13*/
    [VBAT_OVP_ALM_STAT] = REG_FIELD(0x13, 6, 6),
    [IBAT_OCP_ALM_STAT] = REG_FIELD(0x13, 3, 3),
    /*reg14*/
    [IBUS_OCP_STAT] = REG_FIELD(0x14, 7, 7),
    [IBUS_UCP_FALL_STAT] = REG_FIELD(0x14, 5, 5),
    /*reg15*/
    [VBAT_INSERT_STAT] = REG_FIELD(0x15, 5, 5),
    [VBUS_PRESENT_STAT] = REG_FIELD(0x15, 2, 2),
    /*reg17*/
    [CP_SWITCHING_STAT] = REG_FIELD(0x17, 6, 6),
    [VBUS_ERRORHI_STAT] = REG_FIELD(0x17, 4, 4),
    /*reg22*/
    [DEVICE_ID] = REG_FIELD(0x22, 0, 7),
    /*reg23*/
    [ADC_EN] = REG_FIELD(0x23, 7, 7),
    [ADC_RATE] = REG_FIELD(0x23, 6, 6),
};

static const struct regmap_config bq25960_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = BQ25960_REGMAX,
};

/************************************************************************/

struct bq25960_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp;
    int vbat_ovp_alm_dis;
    int vbat_ovp_alm;
    int ibat_ocp_dis;
    int ibat_ocp;
    int ibat_ocp_alm_dis;
    int ibat_ocp_alm;
    int ibus_ucp_dis;
    int ibus_ucp;
    int chg_config_2;
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
};

struct bq25960 {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];

    struct bq25960_cfg_e cfg;
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
    struct mutex data_lock;
    int vbus_error;
};

struct bq25960_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define BQ25960_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define BQ25960_CHAN_VOLT(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_VOLTAGE,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define BQ25960_CHAN_CUR(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define BQ25960_CHAN_TEMP(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_TEMP,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define BQ25960_CHAN_POW(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_POWER,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define BQ25960_CHAN_ENERGY(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_ENERGY,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

#define BQ25960_CHAN_COUNT(_name, _num)			\
	BQ25960_IIO_CHAN(_name, _num, IIO_COUNT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct bq25960_iio_channels bq25960_iio_psy_channels[] = {
	BQ25960_CHAN_ENERGY("bq_present", PSY_IIO_PRESENT)
	BQ25960_CHAN_ENERGY("bq_charging_enabled", PSY_IIO_CHARGING_ENABLED)
	BQ25960_CHAN_ENERGY("bq_status", PSY_IIO_STATUS)
	BQ25960_CHAN_ENERGY("bq_battery_present", PSY_IIO_SC_BATTERY_PRESENT)
	BQ25960_CHAN_ENERGY("bq_vbus_present", PSY_IIO_SC_VBUS_PRESENT)
	BQ25960_CHAN_VOLT("bq_battery_voltage", PSY_IIO_SC_BATTERY_VOLTAGE)
	BQ25960_CHAN_CUR("bq_battery_current", PSY_IIO_SC_BATTERY_CURRENT)
	BQ25960_CHAN_TEMP("bq_battery_temperature", PSY_IIO_SC_BATTERY_TEMPERATURE)
	BQ25960_CHAN_VOLT("bq_bus_voltage", PSY_IIO_SC_BUS_VOLTAGE)
	BQ25960_CHAN_CUR("bq_bus_current", PSY_IIO_SC_BUS_CURRENT)
	BQ25960_CHAN_TEMP("bq_bus_temperature", PSY_IIO_SC_BUS_TEMPERATURE)
	BQ25960_CHAN_TEMP("bq_die_temperature", PSY_IIO_SC_DIE_TEMPERATURE)
	BQ25960_CHAN_ENERGY("bq_alarm_status", PSY_IIO_SC_ALARM_STATUS)
	BQ25960_CHAN_ENERGY("bq_fault_status", PSY_IIO_SC_FAULT_STATUS)
	BQ25960_CHAN_ENERGY("bq_vbus_error_status", PSY_IIO_SC_VBUS_ERROR_STATUS)
	BQ25960_CHAN_ENERGY("bq_chip_vendor", PSY_IIO_SC_CHIP_VENDOR)
	BQ25960_CHAN_ENERGY("bq_chip_id", PSY_IIO_CHIP_ID)
	BQ25960_CHAN_ENERGY("bq_bus_protection", PSY_IIO_SET_BUS_PROTECTION)
	BQ25960_CHAN_ENERGY("bq_set_acdrv", PSY_IIO_SET_ACDRV)
	BQ25960_CHAN_ENERGY("bq_set_cp_workmode", PSY_IIO_SC_SET_CP_WORKMODE)
};

/*********************************************************/
static int bq25960_field_read(struct bq25960 *bq,
                enum bq25960_fields field_id, int *val)
{
    int ret;

    ret = regmap_field_read(bq->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(bq->dev, "bq25960 read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int bq25960_field_write(struct bq25960 *bq,
                enum bq25960_fields field_id, int val)
{
    int ret;

    ret = regmap_field_write(bq->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(bq->dev, "bq25960 read field %d fail: %d\n", field_id, ret);
    }

    return ret;
}

static int bq25960_read_block(struct bq25960 *bq,
                int reg, uint8_t *val, int len)
{
    int ret;

    ret = regmap_bulk_read(bq->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(bq->dev, "bq25960 read %02x block failed %d\n", reg, ret);
    }

    return ret;
}



/*******************************************************/
static int bq25960_detect_device(struct bq25960 *bq)
{
    int ret;
    int val = 0;

    ret = bq25960_field_read(bq, DEVICE_ID, &val);
    if (ret < 0) {
        dev_err(bq->dev, "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    if (val != BQ25960_DEVICE_ID) {
        dev_info(bq->dev, "%s not find BQ25960, ID = 0x%02x\n", __func__, ret);
        return -EINVAL;
    }

    return ret;
}

static int bq25960_reg_reset(struct bq25960 *bq)
{
    return bq25960_field_write(bq, REG_RST, 1);
}

static noinline int bq25960_enable_charge(struct bq25960 *bq, bool en)
{
    int ret;

    ret = bq25960_field_write(bq, IBUS_UCP, 1);
    ret = bq25960_field_write(bq, CHG_CONFIG_2, 1);
    ret = bq25960_field_write(bq, TSBUS_FLT_DIS, 1);
    ret = bq25960_field_write(bq, TSBAT_FLT_DIS, 1);
    ret = bq25960_field_write(bq, WD_TIMEOUT_DIS, 1);
    ret = bq25960_field_write(bq, CHG_EN, !!en);

    return ret;
}

static int bq25960_check_charge_enabled(struct bq25960 *bq, bool *enabled)
{
    int ret, val = 0;

    ret = bq25960_field_read(bq, CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    return ret;
}

static int bq25960_get_status(struct bq25960 *bq, uint32_t *status)
{
    int ret, val = 0;
    *status = 0;

    ret = bq25960_field_read(bq, VBUS_ERRORHI_STAT, &val);
    if (ret < 0) {
        dev_err(bq->dev, "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    return ret;
}

static int bq25960_enable_adc(struct bq25960 *bq, bool en)
{
    dev_err(bq->dev,"%s:%d", __func__, en);
    return bq25960_field_write(bq, ADC_EN, !!en);
}

/* returns the converted ADC value or a negative error */
static int bq25960_get_adc_data(struct bq25960 *bq, int channel)
{
    unsigned int val_l = 0, val_h = 0;
    int result;
    int ret;

    ret = regmap_read(bq->regmap, BQ25960_REG25 + (channel << 1), &val_h);
    if (ret)
        return ret;
    ret = regmap_read(bq->regmap, BQ25960_REG25 + (channel << 1) + 1, &val_l);
    if (ret)
        return ret;

    if (channel == ADC_IBAT) {
        result = (val_h << 8) | val_l;
        if (val_h & 0x80)
            result = (result ^ 0xffff) + 1;
    } else {
        result = (u16)((val_h << 8) | val_l);
        if (channel == ADC_IBUS && (val_h & 0x80))
            result = 0x10000 - result;
    }

    return result;
}

static int bq25960_set_acdrv(struct bq25960 *bq, bool en)
{
    int ret;

    dev_info(bq->dev, "%s:%d", __func__, en);

    if (en) {
        ret = bq25960_field_write(bq, OTG_EN, 1);
        msleep(10);
        ret = bq25960_field_write(bq, ACDRV_BOTH, 0);
        ret = bq25960_field_write(bq, ACDRV1_STAT, 1);
    } else {
        ret = bq25960_field_write(bq, VBUS_PD_EN, 1);
        msleep(10);
        ret = bq25960_field_write(bq, ACDRV1_STAT, 0);
        ret = bq25960_field_write(bq, OTG_EN, 0);
    }

    return ret;
}

static int bq25960_set_cp_workmode(struct bq25960 *bq, int workmode)
{
    int ret;

    dev_info(bq->dev, "N7:%s:%d", __func__, workmode);

    ret = bq25960_field_write(bq, BYPASS_EN, workmode);

    return ret;
}

static noinline int bq25960_get_cp_workmode(struct bq25960 *bq, int *workmode)
{
    int ret;
    int val = 0;

    ret = bq25960_field_read(bq, BYPASS_EN, &val);

    *workmode = val;
    dev_info(bq->dev, "N7:%s:%d", __func__, *workmode);

    return ret;
}

static noinline int bq25960_adjust_ovp(struct bq25960 *bq)
{
    int ret;

    ret = bq25960_field_write(bq, VAC1_OVP, 3);
    ret = bq25960_field_write(bq, VAC2_OVP, 3);

    return ret;
}

static int bq25960_init_device(struct bq25960 *bq)
{
    int ret = 0;
    int i;
    struct {
        enum bq25960_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, bq->cfg.vbat_ovp_dis},
        {VBAT_OVP, bq->cfg.vbat_ovp},
        {VBAT_OVP_ALM_DIS, bq->cfg.vbat_ovp_alm_dis},
        {VBAT_OVP_ALM, bq->cfg.vbat_ovp_alm},
        {IBAT_OCP_DIS, bq->cfg.ibat_ocp_dis},
        {IBAT_OCP, bq->cfg.ibat_ocp},
        {IBAT_OCP_ALM_DIS, bq->cfg.ibat_ocp_alm_dis},
        {IBAT_OCP_ALM, bq->cfg.ibat_ocp_alm},
        {IBUS_UCP_DIS, bq->cfg.ibus_ucp_dis},
        {IBUS_UCP, bq->cfg.ibus_ucp},
        {CHG_CONFIG_2, bq->cfg.chg_config_2},
        {VBUS_PD_EN, bq->cfg.vbus_pd_en},
        {VBUS_OVP, bq->cfg.vbus_ovp},
        {VBUS_OVP_ALM_DIS, bq->cfg.vbus_ovp_alm_dis},
        {VBUS_OVP_ALM, bq->cfg.vbus_ovp_alm},
        {IBUS_OCP, bq->cfg.ibus_ocp},
        {TSBUS_FLT_DIS, bq->cfg.tsbus_flt_dis},
        {TSBAT_FLT_DIS, bq->cfg.tsbat_flt_dis},
        {TDIE_ALM, bq->cfg.tdie_alm},
        {TSBUS_FLT, bq->cfg.tsbus_flt},
        {TSBAT_FLT, bq->cfg.tsbat_flt},
        {VAC1_OVP, bq->cfg.vac1_ovp},
        {VAC2_OVP, bq->cfg.vac2_ovp},
        {VAC1_PD_EN, bq->cfg.vac1_pd_en},
        {VAC2_PD_EN, bq->cfg.vac2_pd_en},
        {FSW_SET, bq->cfg.fsw_set},
        {WD_TIMEOUT, bq->cfg.wd_timeout},
        {WD_TIMEOUT_DIS, bq->cfg.wd_timeout_dis},
        {SET_IBAT_SNS_RES, bq->cfg.ibat_sns_r},
        {SS_TIMEOUT, bq->cfg.ss_timeout},
        {IBUS_UCP_FALL_DG, bq->cfg.ibus_ucp_fall_dg},
        {VOUT_OVP_DIS, bq->cfg.vout_ovp_dis},
        {VOUT_OVP, bq->cfg.vout_ovp},
        {MS, bq->cfg.ms},
    };

    ret = bq25960_reg_reset(bq);
    if (ret < 0) {
        dev_err(bq->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = bq25960_field_write(bq, props[i].field_id, props[i].conv_data);
    }

    bq25960_enable_adc(bq, true);

    return ret;
}

/********************creat devices note start*************************************************/
static ssize_t bq25960_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct bq25960 *bq = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25960");
    for (addr = 0x0; addr <= BQ25960_REGMAX; addr++) {
        ret = regmap_read(bq->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t bq25960_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct bq25960 *bq = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= BQ25960_REGMAX)
        regmap_write(bq->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, bq25960_show_registers, bq25960_store_register);

static void bq25960_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
}
/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
static void bq25960_check_fault_status(struct bq25960 *bq)
{
    int ret;
    u8 flag = 0;
    int i,j;

    for (i=0;i < ARRAY_SIZE(cp_intr_flag);i++) {
        ret = bq25960_read_block(bq, cp_intr_flag[i].reg, &flag, 1);
        for (j=0; j <  cp_intr_flag[i].len; j++) {
            if (flag & cp_intr_flag[i].bit[j].mask) {
                dev_info(bq->dev,"trigger :%s\n",cp_intr_flag[i].bit[j].name);
            }
        }
    }
}

static irqreturn_t bq25960_irq_handler(int irq, void *data)
{
    struct bq25960 *bq = data;

    dev_info(bq->dev,"INT OCCURED\n");

    bq25960_check_fault_status(bq);

    power_supply_changed(bq->psy);

    return IRQ_HANDLED;
}

static int bq25960_register_interrupt(struct bq25960 *bq)
{
    int ret;

    if (gpio_is_valid(bq->irq_gpio)) {
        ret = gpio_request_one(bq->irq_gpio, GPIOF_DIR_IN,"bq25960_irq");
        if (ret) {
            dev_err(bq->dev,"failed to request bq25960_irq\n");
            return -EINVAL;
        }
        bq->irq = gpio_to_irq(bq->irq_gpio);
        if (bq->irq < 0) {
            dev_err(bq->dev,"failed to gpio_to_irq\n");
            return -EINVAL;
        }
    } else {
        dev_info(bq->dev,"irq gpio not provided\n");
        return -EINVAL;
    }

    if (bq->irq) {
        ret = devm_request_threaded_irq(&bq->client->dev, bq->irq,
                NULL, bq25960_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                bq25960_irq_name[bq->mode], bq);

        if (ret < 0) {
            dev_err(bq->dev,"request irq for irq=%d failed, ret =%d\n",
                            bq->irq, ret);
            return ret;
        }
        enable_irq_wake(bq->irq);
    }

    return ret;
}
/********************interrupte end*************************************************/


/************************psy start**************************************/
static enum power_supply_property bq25960_charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
};

static int bq25960_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct bq25960 *bq = power_supply_get_drvdata(psy);
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        bq25960_check_charge_enabled(bq, &bq->charge_enabled);
        val->intval = bq->charge_enabled;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = bq25960_get_adc_data(bq, ADC_VBUS);
        if (ret < 0)
            return ret;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = bq25960_get_adc_data(bq, ADC_IBUS);
        if (ret < 0)
            return ret;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = bq25960_get_adc_data(bq, ADC_VBAT);
        if (ret < 0)
            return ret;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = bq25960_get_adc_data(bq, ADC_IBAT);
        if (ret < 0)
            return ret;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = bq25960_get_adc_data(bq, ADC_TSBAT);
        if (ret < 0)
            return ret;
        val->intval = ret;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int bq25960_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    struct bq25960 *bq = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_ONLINE:
        bq25960_enable_charge(bq, val->intval);
        pr_info("POWER_SUPPLY_PROP_ONLINE: %s\n",
                val->intval ? "enable" : "disable");
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int bq25960_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    return 0;
}

static int bq25960_psy_register(struct bq25960 *bq)
{
    bq->psy_cfg.drv_data = bq;
    bq->psy_cfg.of_node = bq->dev->of_node;

    bq->psy_desc.name = bq25960_psy_name[bq->mode];

    bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    bq->psy_desc.properties = bq25960_charger_props;
    bq->psy_desc.num_properties = ARRAY_SIZE(bq25960_charger_props);
    bq->psy_desc.get_property = bq25960_charger_get_property;
    bq->psy_desc.set_property = bq25960_charger_set_property;
    bq->psy_desc.property_is_writeable = bq25960_charger_is_writeable;


    bq->psy = devm_power_supply_register(bq->dev,
            &bq->psy_desc, &bq->psy_cfg);
    if (IS_ERR(bq->psy)) {
        dev_info(bq->dev, "%s failed to register psy\n", __func__);
        return PTR_ERR(bq->psy);
    }

    dev_info(bq->dev, "%s power supply register successfully\n", bq->psy_desc.name);

    return 0;
}


/************************psy end**************************************/

/************************iio start**************************************/

static int bq25960_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct bq25960 *bq = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
		case PSY_IIO_CHARGING_ENABLED:
			bq25960_enable_charge(bq, val1);
			bq25960_check_charge_enabled(bq, &bq->charge_enabled);
			break;
		case PSY_IIO_PRESENT:
			break;
		case PSY_IIO_SET_BUS_PROTECTION:
			pr_err("hvdcp3_type: %d\n", val1);
			break;
		case PSY_IIO_SET_ACDRV:
			bq25960_set_acdrv(bq, val1);
			break;
		case PSY_IIO_SC_SET_CP_WORKMODE:
			bq25960_set_cp_workmode(bq, val1);
			pr_info("iio set workmode =%d", val1);
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

static int bq25960_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct bq25960 *bq = iio_priv(indio_dev);
	int val = 0;
	int ret = 0;
	*val1 = 0;

	switch (chan->channel) {
		case PSY_IIO_CHARGING_ENABLED:
			bq25960_check_charge_enabled(bq, &bq->charge_enabled);
			*val1 = bq->charge_enabled;
			break;
		case PSY_IIO_STATUS:
			break;
		case PSY_IIO_PRESENT:
			*val1 = bq->usb_present;
			break;
		case PSY_IIO_SC_BATTERY_PRESENT:
			ret = bq25960_field_read(bq, VBAT_INSERT_STAT, &val);
			if (!ret)
				bq->batt_present = !!val;
			*val1 = bq->batt_present;
			break;
		case PSY_IIO_SC_VBUS_PRESENT:
			ret = bq25960_field_read(bq, VBUS_PRESENT_STAT, &val);
			if (!ret)
				bq->vbus_present = !!val;
			*val1 = bq->vbus_present;
			break;
		case PSY_IIO_SC_BATTERY_VOLTAGE:
			ret = bq25960_get_adc_data(bq, ADC_VBAT);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_BATTERY_CURRENT:
			ret = bq25960_get_adc_data(bq, ADC_IBAT);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_BATTERY_TEMPERATURE:
			ret = bq25960_get_adc_data(bq, ADC_TSBAT);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_BUS_VOLTAGE:
			ret = bq25960_get_adc_data(bq, ADC_VBUS);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_BUS_CURRENT:
			ret = bq25960_get_adc_data(bq, ADC_IBUS);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_BUS_TEMPERATURE:
		case PSY_IIO_SC_DIE_TEMPERATURE:
			ret = bq25960_get_adc_data(bq, ADC_TSBUS);
			if (ret < 0)
				return ret;
			*val1 = ret;
			break;
		case PSY_IIO_SC_ALARM_STATUS:
			break;
		case PSY_IIO_SC_FAULT_STATUS:
			break;
		case PSY_IIO_SC_VBUS_ERROR_STATUS:
			ret = bq25960_get_status(bq, (uint32_t *)&bq->vbus_error);
			*val1 = bq->vbus_error;
			break;
		case PSY_IIO_SC_CHIP_VENDOR:
			*val1 = 1;
			break;
		case PSY_IIO_CHIP_ID:
			ret = bq25960_field_read(bq, DEVICE_ID, &val);
			*val1 = val;
			break;
		case PSY_IIO_SC_SET_CP_WORKMODE:
			ret = bq25960_get_cp_workmode(bq, &val);
			*val1 = val;
			pr_info("iio get workmode =%d", *val1);
			break;
		default:
			ret = -EINVAL;
			 break;
	}

	if (ret < 0) {
		pr_info("Couldn't read IIO channel %d, ret = %d\n",
			chan->channel, ret);
		return ret;
	}

	return IIO_VAL_INT;
}


static int bq25960_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct bq25960 *bq = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = bq->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(bq25960_iio_psy_channels);
					i++, iio_chan++) {
		if (iio_chan->channel == iiospec->args[0])
			return i;
	}
	return -EINVAL;
}

static const struct iio_info bq25960_iio_info = {
	.read_raw	= bq25960_iio_read_raw,
	.write_raw	= bq25960_iio_write_raw,
	.of_xlate	= bq25960_iio_of_xlate,
};

static int bq25960_init_iio_psy(struct bq25960 *chip)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int num_iio_channels = ARRAY_SIZE(bq25960_iio_psy_channels);
	int rc, i;

	pr_info("bq25960_init_iio_psy start\n");
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

	indio_dev->info = &bq25960_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = num_iio_channels;

	if (chip->mode == BQ25960) {
		indio_dev->name = "bq25960-standalone";
		for (i = 0; i < num_iio_channels; i++) {
			chip->int_iio_chans[i].indio_dev = indio_dev;
			chan = &chip->iio_chan[i];
			chip->int_iio_chans[i].channel = chan;
			chan->address = i;
			chan->channel = bq25960_iio_psy_channels[i].channel_num;
			chan->type = bq25960_iio_psy_channels[i].type;
			chan->datasheet_name =
				bq25960_iio_psy_channels[i].datasheet_name;
			chan->extend_name =
				bq25960_iio_psy_channels[i].datasheet_name;
			chan->info_mask_separate =
				bq25960_iio_psy_channels[i].info_mask;
		}
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("Failed to register BQ25960 IIO device, rc=%d\n", rc);

	pr_info("BQ25960 IIO device, rc=%d\n", rc);
	return rc;
}

/************************iio end**************************************/

static int bq25960_parse_dt(struct bq25960 *bq, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"bq,bq25960,vbat-ovp-dis", &(bq->cfg.vbat_ovp_dis)},
        {"bq,bq25960,vbat-ovp", &(bq->cfg.vbat_ovp)},
        {"bq,bq25960,vbat-ovp-alm-dis", &(bq->cfg.vbat_ovp_alm_dis)},
        {"bq,bq25960,vbat-ovp-alm", &(bq->cfg.vbat_ovp_alm)},
        {"bq,bq25960,ibat-ocp-dis", &(bq->cfg.ibat_ocp_dis)},
        {"bq,bq25960,ibat-ocp", &(bq->cfg.ibat_ocp)},
        {"bq,bq25960,ibat-ocp-alm-dis", &(bq->cfg.ibat_ocp_alm_dis)},
        {"bq,bq25960,ibat-ocp-alm", &(bq->cfg.ibat_ocp_alm)},
        {"bq,bq25960,ibus-ucp-dis", &(bq->cfg.ibus_ucp_dis)},
        {"bq,bq25960,ibus-ucp", &(bq->cfg.ibus_ucp)},
        {"bq,bq25960,chg-config-2", &(bq->cfg.chg_config_2)},
        {"bq,bq25960,vbus-pd-en", &(bq->cfg.vbus_pd_en)},
        {"bq,bq25960,vbus-ovp", &(bq->cfg.vbus_ovp)},
        {"bq,bq25960,vbus-ovp-alm-dis", &(bq->cfg.vbus_ovp_alm_dis)},
        {"bq,bq25960,vbus-ovp-alm", &(bq->cfg.vbus_ovp_alm)},
        {"bq,bq25960,ibus-ocp", &(bq->cfg.ibus_ocp)},
        {"bq,bq25960,tsbus-flt-dis", &(bq->cfg.tsbus_flt_dis)},
        {"bq,bq25960,tsbat-flt-dis", &(bq->cfg.tsbat_flt_dis)},
        {"bq,bq25960,tdie-alm", &(bq->cfg.tdie_alm)},
        {"bq,bq25960,tsbus-flt", &(bq->cfg.tsbus_flt)},
        {"bq,bq25960,tsbat-flt", &(bq->cfg.tsbat_flt)},
        {"bq,bq25960,vac1-ovp", &(bq->cfg.vac1_ovp)},
        {"bq,bq25960,vac2-ovp", &(bq->cfg.vac2_ovp)},
        {"bq,bq25960,vac1-pd-en", &(bq->cfg.vac1_pd_en)},
        {"bq,bq25960,vac2-pd-en", &(bq->cfg.vac2_pd_en)},
        {"bq,bq25960,fsw-set", &(bq->cfg.fsw_set)},
        {"bq,bq25960,wd-timeout", &(bq->cfg.wd_timeout)},
        {"bq,bq25960,wd-timeout-dis", &(bq->cfg.wd_timeout_dis)},
        {"bq,bq25960,ibat-sns-r", &(bq->cfg.ibat_sns_r)},
        {"bq,bq25960,ss-timeout", &(bq->cfg.ss_timeout)},
        {"bq,bq25960,ibus-ucp-fall-dg", &(bq->cfg.ibus_ucp_fall_dg)},
        {"bq,bq25960,vout-ovp-dis", &(bq->cfg.vout_ovp_dis)},
        {"bq,bq25960,vout-ovp", &(bq->cfg.vout_ovp)},
        {"bq,bq25960,ms", &(bq->cfg.ms)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(bq->dev, "can not read %s \n", props[i].name);
            return ret;
        }
    }

    bq->irq_gpio = of_get_named_gpio(np, "bq25960,intr_gpio", 0);
    if (!gpio_is_valid(bq->irq_gpio)) {
        dev_info(bq->dev,"fail to valid gpio : %d\n", bq->irq_gpio);
        return -EINVAL;
    }

    if (of_property_read_string(np, "charger_name", &bq->chg_dev_name) < 0) {
        bq->chg_dev_name = "charger";
        dev_info(bq->dev,"no charger name\n");
    }

    return 0;
}

/* stock sapphire has no terminating sentinel; we add it */
static struct of_device_id bq25960_charger_match_table[] = {
    {   .compatible = "bq,bq25960-standalone",
        .data = &bq25960_mode_data[BQ25960], },
    {},
};

static int bq25960_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct bq25960 *bq;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret, i;
	struct iio_dev *indio_dev;

    dev_info(&client->dev, "%s (%s)\n", __func__, BQ25960_DRV_VERSION);

    is_bq25960 = true;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bq));

	bq = iio_priv(indio_dev);
	bq->indio_dev = indio_dev;

    bq->dev = &client->dev;
    bq->client = client;

    bq->regmap = devm_regmap_init_i2c(client,
                            &bq25960_regmap_config);
    if (IS_ERR(bq->regmap)) {
        dev_info(bq->dev, "Failed to initialize regmap\n");
        ret = PTR_ERR(bq->regmap);
        goto err_regmap_init;
    }

    for (i = 0; i < ARRAY_SIZE(bq25960_reg_fields); i++) {
        const struct reg_field *reg_fields = bq25960_reg_fields;

        bq->rmap_fields[i] =
            devm_regmap_field_alloc(bq->dev,
                        bq->regmap,
                        reg_fields[i]);
        if (IS_ERR(bq->rmap_fields[i])) {
            dev_info(bq->dev, "cannot allocate regmap field\n");
            ret = PTR_ERR(bq->rmap_fields[i]);
            goto err_regmap_field;
        }
    }

    ret = bq25960_detect_device(bq);
    if (ret < 0) {
        dev_err(bq->dev, "%s detect device fail\n", __func__);
        is_bq25960 = false;
        ret = 0;
        goto err_detect_dev;
    }

    i2c_set_clientdata(client, bq);
    bq25960_create_device_node(&(client->dev));

    match = of_match_node(bq25960_charger_match_table, node);
    if (match == NULL) {
        dev_err(bq->dev, "device tree match not found!\n");
        ret = -ENODEV;
        goto err_match_node;
    }

    ret = bq25960_parse_dt(bq, &client->dev);
    if (ret < 0) {
        dev_err(bq->dev, "%s parse dt failed(%d)\n", __func__, ret);
        ret = -EIO;
        goto err_parse_dt;
    }

    ret = bq25960_init_device(bq);
    if (ret < 0) {
        dev_err(bq->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = bq25960_psy_register(bq);
    if (ret < 0) {
        dev_err(bq->dev, "%s psy register failed(%d)\n", __func__, ret);
        goto err_register_psy;
    }

    ret = bq25960_register_interrupt(bq);
    if (ret < 0) {
        dev_err(bq->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

	ret = bq25960_init_iio_psy(bq);
	if (ret)
		goto err_register_iio;

    ret = bq25960_adjust_ovp(bq);
    if (ret < 0) {
        pr_err("error to adjust the ovp \n");
        goto err_adjust_ovp;
    }

    dev_info(bq->dev, "bq25960 probe successfully!\n");
    return 0;

err_register_iio:
err_register_psy:
err_register_irq:
err_init_device:
    power_supply_unregister(bq->psy);
err_adjust_ovp:
err_detect_dev:
err_match_node:
err_parse_dt:
err_regmap_init:
err_regmap_field:
    dev_info(&client->dev,"bq25960 probe fail\n");
    return ret;
}


static int bq25960_charger_remove(struct i2c_client *client)
{
    struct bq25960 *bq = i2c_get_clientdata(client);

    power_supply_unregister(bq->psy);
    devm_kfree(&client->dev, bq);
    return 0;
}

static void bq25960_charger_shutdown(struct i2c_client *client)
{
    struct bq25960 *bq = i2c_get_clientdata(client);

    if (is_bq25960 == true)
        bq25960_enable_adc(bq, false);
}

#ifdef CONFIG_PM_SLEEP
static int bq25960_suspend(struct device *dev)
{
    struct bq25960 *bq = dev_get_drvdata(dev);

    if (is_bq25960 == true) {
        dev_info(bq->dev, "Suspend successfully!");
        if (device_may_wakeup(dev))
            enable_irq_wake(bq->irq);
        disable_irq(bq->irq);
    }

    return 0;
}
static int bq25960_resume(struct device *dev)
{
    struct bq25960 *bq = dev_get_drvdata(dev);

    if (is_bq25960 == true) {
        dev_info(bq->dev, "Resume successfully!");
        if (device_may_wakeup(dev))
            disable_irq_wake(bq->irq);
        enable_irq(bq->irq);
    }

    return 0;
}

static const struct dev_pm_ops bq25960_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(bq25960_suspend, bq25960_resume)
};
#endif

static struct i2c_driver bq25960_charger_driver = {
    .driver     = {
        .name   = "bq25960",
        .owner  = THIS_MODULE,
        .of_match_table = bq25960_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &bq25960_pm,
#endif
    },
    .probe      = bq25960_charger_probe,
    .remove     = bq25960_charger_remove,
    .shutdown   = bq25960_charger_shutdown,
};

module_i2c_driver(bq25960_charger_driver);

MODULE_DESCRIPTION("TI BQ25960 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");
