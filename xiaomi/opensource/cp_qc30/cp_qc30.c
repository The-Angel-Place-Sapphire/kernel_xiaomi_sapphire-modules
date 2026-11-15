/*
 * State machine for qc3 when it works on cp
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Sapphire (Redmi Note 13 4G, khaje, 5.15 GKI) version.
 * Base: Xiaomi sunny/mojito drivers/power/supply/ti/cp_qc30.c (Fei Jiang).
 * Ported to IIO (sc8541/bq25960 charge pump and qpnp-smb5 channels),
 * BQ25960/SC8541 auto-detection, USB_ICL votable, cool/warm handling,
 * retry_enable_cp_count and the entry1/entry3 tuning, reconstructed from
 * the stock sapphire vendor_dlkm cp_qc30.ko.
 */

#define pr_fmt(fmt)	"[FC2-PM]: %s: " fmt, __func__
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>
#include <linux/iio/consumer.h>
#include <linux/qti_power_supply.h>

#include "cp_qc30.h"

#ifdef pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif

#define BATT_MAX_CHG_VOLT		4460
#define BATT_FAST_CHG_CURR		5400
#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#define BUS_VOLT_INIT_UP		150

#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD

#define VOLT_UP		true
#define VOLT_DOWN	false

#define ADC_ERR			1
#define CP_ENABLE_FAIL			2
#define TAPER_DONE			1

#define CP_WORK_INTERVAL_MS		100
#define CP_DELAY_INTERVAL_MS		1000

/*
 * Stock name kept: the value is the vbus/vbat ratio x10 used to compute
 * the adapter target voltage (vbus = vbat * 2.1 + 150mV).
 */
static int debug_mask = 21;
module_param(debug_mask, int, 0600);

static struct sys_config sys_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.ffc_bat_volt_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT/* + 1000*/,
	.bus_volt_lp_lmt		= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt		= BAT_CURR_LOOP_LMT >> 1,

	.ibus_minus_deviation_val = 400,
	.ibus_plus_deviation_val = 200,
	.ibat_minus_deviation_val = 600,
	.ibat_plus_deviation_val = 500,

	.fc2_taper_current		= 2200,
	.flash2_policy.down_steps	= -1,
	.flash2_policy.volt_hysteresis	= 50,

	.min_vbat_start_flash2		= 3500,
	.cp_sec_enable			= false,
};

struct cp_qc30_data {
	struct device *dev;
	int			bat_volt_max;
	int			ffc_bat_volt_max;
	int			bat_curr_max;
	int			bus_volt_max;
	int			bus_curr_max;
	bool			cp_sec_enable;

	/* notifiers */
	struct notifier_block	nb;

	struct delayed_work	qc3_pm_work;

	struct iio_channel	**cp_iio;
	struct iio_channel	**smb_iio;
	int			work_interval;
	bool			is_bq25960;
};

static const char * const cp_iio_chan_name[] = {
	[CHARGE_PUMP_PRESENT] = "present",
	[CHARGE_PUMP_CHARGING_ENABLED] = "charging_enabled",
	[CHARGE_PUMP_STATUS] = "status",
	[CHARGE_PUMP_SC_BATTERY_PRESENT] = "sc_battery_present",
	[CHARGE_PUMP_SC_VBUS_PRESENT] = "sc_vbus_present",
	[CHARGE_PUMP_SC_BATTERY_VOLTAGE] = "sc_battery_voltage",
	[CHARGE_PUMP_SC_BATTERY_CURRENT] = "sc_battery_current",
	[CHARGE_PUMP_SC_BATTERY_TEMPERATURE] = "sc_battery_temperature",
	[CHARGE_PUMP_SC_BUS_VOLTAGE] = "sc_bus_voltage",
	[CHARGE_PUMP_SC_BUS_CURRENT] = "sc_bus_current",
	[CHARGE_PUMP_SC_BUS_TEMPERATURE] = "sc_bus_temperature",
	[CHARGE_PUMP_SC_DIE_TEMPERATURE] = "sc_die_temperature",
	[CHARGE_PUMP_SC_ALARM_STATUS] = "sc_alarm_status",
	[CHARGE_PUMP_SC_FAULT_STATUS] = "sc_fault_status",
	[CHARGE_PUMP_SC_VBUS_ERROR_STATUS] = "sc_vbus_error_status",
	[CHARGE_PUMP_SC_SET_BUS_PROTECTION] = "sc_set_bus_protection",
	[CHARGE_PUMP_BQ_PRESENT] = "bq_present",
	[CHARGE_PUMP_BQ_CHARGING_ENABLED] = "bq_charging_enabled",
	[CHARGE_PUMP_BQ_STATUS] = "bq_status",
	[CHARGE_PUMP_BQ_BATTERY_PRESENT] = "bq_battery_present",
	[CHARGE_PUMP_BQ_VBUS_PRESENT] = "bq_vbus_present",
	[CHARGE_PUMP_BQ_BATTERY_VOLTAGE] = "bq_battery_voltage",
	[CHARGE_PUMP_BQ_BATTERY_CURRENT] = "bq_battery_current",
	[CHARGE_PUMP_BQ_BATTERY_TEMPERATURE] = "bq_battery_temperature",
	[CHARGE_PUMP_BQ_BUS_VOLTAGE] = "bq_bus_voltage",
	[CHARGE_PUMP_BQ_BUS_CURRENT] = "bq_bus_current",
	[CHARGE_PUMP_BQ_BUS_TEMPERATURE] = "bq_bus_temperature",
	[CHARGE_PUMP_BQ_DIE_TEMPERATURE] = "bq_die_temperature",
	[CHARGE_PUMP_BQ_ALARM_STATUS] = "bq_alarm_status",
	[CHARGE_PUMP_BQ_FAULT_STATUS] = "bq_fault_status",
	[CHARGE_PUMP_BQ_VBUS_ERROR_STATUS] = "bq_vbus_error_status",
	[CHARGE_PUMP_BQ_SET_BUS_PROTECTION] = "bq_set_bus_protection",
};

static const char * const smb_iio_chan_name[] = {
	[SMB_REAL_TYPE] = "real_type",
	[SMB_HVDCP3_TYPE] = "hvdcp3_type",
	[SMB_CP_INPUT_SUSPEND] = "cp_input_suspend",
	[SMB_BATTERY_CHARGING_ENABLED] = "battery_charging_enabled",
	[SMB_BATTERY_DP_DM] = "battery_dp_dm",
	[SMB_BATTERY_CHARGING_LIMITED] = "battery_charging_limited",
};

static pm_t pm_state;

static int fc2_taper_timer;

static bool is_cp_chan_valid(struct cp_qc30_data *chip,
		enum cp_iio_channels chan)
{
	int rc;

	if (IS_ERR(chip->cp_iio[chan]))
		return false;
	if (!chip->cp_iio[chan]) {
		chip->cp_iio[chan] = iio_channel_get(chip->dev,
					cp_iio_chan_name[chan]);
		if (IS_ERR(chip->cp_iio[chan])) {
			rc = PTR_ERR(chip->cp_iio[chan]);
			chip->cp_iio[chan] = NULL;
			pr_err("Failed to get IIO channel %s, rc=%d\n",
				cp_iio_chan_name[chan], rc);
			return false;
		}
	}
	return true;
}

static bool is_smb_chan_valid(struct cp_qc30_data *chip,
		enum smb_iio_channels chan)
{
	int rc;

	if (IS_ERR(chip->smb_iio[chan]))
		return false;
	if (!chip->smb_iio[chan]) {
		chip->smb_iio[chan] = iio_channel_get(chip->dev,
					smb_iio_chan_name[chan]);
		if (IS_ERR(chip->smb_iio[chan])) {
			rc = PTR_ERR(chip->smb_iio[chan]);
			chip->smb_iio[chan] = NULL;
			pr_err("Failed to get IIO channel %s, rc=%d\n",
				smb_iio_chan_name[chan], rc);
			return false;
		}
	}
	return true;
}

static noinline int cp_qc30_get_iio_channel(struct cp_qc30_data *chip,
		enum cp_iio_channels channel, int *val)
{
	int rc;

	if (chip->is_bq25960)
		channel += CP_BQ25960_IIO_CHANNEL_OFFSET;

	if (!is_cp_chan_valid(chip, channel))
		return -ENODEV;

	rc = iio_read_channel_processed(chip->cp_iio[channel], val);
	return rc < 0 ? rc : 0;
}

static noinline int cp_q30_set_iio_channel(struct cp_qc30_data *chip,
		enum cp_iio_channels channel, int val)
{
	int rc;

	if (chip->is_bq25960)
		channel += CP_BQ25960_IIO_CHANNEL_OFFSET;

	if (!is_cp_chan_valid(chip, channel))
		return -ENODEV;

	rc = iio_write_channel_raw(chip->cp_iio[channel], val);
	return rc < 0 ? rc : 0;
}

static int cp_qc30_get_smb_iio(struct cp_qc30_data *chip,
		enum smb_iio_channels channel, int *val)
{
	int rc;

	if (!is_smb_chan_valid(chip, channel))
		return -ENODEV;

	rc = iio_read_channel_processed(chip->smb_iio[channel], val);
	return rc < 0 ? rc : 0;
}

static int cp_qc30_set_smb_iio(struct cp_qc30_data *chip,
		enum smb_iio_channels channel, int val)
{
	int rc;

	if (!is_smb_chan_valid(chip, channel))
		return -ENODEV;

	rc = iio_write_channel_raw(chip->smb_iio[channel], val);
	return rc < 0 ? rc : 0;
}

static struct power_supply *cp_get_sw_psy(void)
{

	if (!pm_state.sw_psy)
		pm_state.sw_psy = power_supply_get_by_name("battery");

	return pm_state.sw_psy;
}

static struct power_supply *cp_get_usb_psy(void)
{

	if (!pm_state.usb_psy)
		pm_state.usb_psy = power_supply_get_by_name("usb");

	return pm_state.usb_psy;
}

static int cp_get_effective_fcc_val(pm_t pm_state)
{
	int effective_fcc_val = 0;

	if (!pm_state.fcc_votable)
		pm_state.fcc_votable = find_votable("FCC");

	if (!pm_state.fcc_votable)
		return -EINVAL;

	effective_fcc_val = get_effective_result(pm_state.fcc_votable);
	effective_fcc_val = effective_fcc_val / 1000;
	pr_info("effective_fcc_val: %d\n", effective_fcc_val);
	return effective_fcc_val;
}

static int cp_get_effective_usb_icl_val(void)
{
	int effective_usb_icl_val = 0;

	if (!pm_state.usb_icl_votable)
		pm_state.usb_icl_votable = find_votable("USB_ICL");

	if (!pm_state.usb_icl_votable) {
		pr_err("[%s] find votable: USB_ICL failed!\n", __func__);
		return -EINVAL;
	}

	effective_usb_icl_val = get_effective_result(pm_state.usb_icl_votable);
	pr_info("effective_usb_icl_val: %d voted by:%s\n", effective_usb_icl_val, get_effective_client(pm_state.usb_icl_votable));
	return effective_usb_icl_val;
}

static void cp_update_batt_info(void)
{
	int ret;
	struct power_supply *psy;
	union power_supply_propval val = {0,};

	psy = cp_get_sw_psy();
	if (!psy)
		return;

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		pm_state.ibat_now = val.intval / 1000;

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		pm_state.vbat_now = val.intval / 1000;

	pr_info("ibat:%d vbat_from_qg:%d \n", pm_state.ibat_now, pm_state.vbat_now);
}

static int qc3_get_bms_fastcharge_mode(void)
{
	union power_supply_propval pval = {0,};
	int rc;

	/*
	 * Stock bug fixed: stock passes a possibly NULL "battery" psy to
	 * power_supply_get_property() here.
	 */
	if (!cp_get_sw_psy())
		return 0;

	/* the stock module reads it from "battery" property #1 (CHARGE_TYPE) */
	rc = power_supply_get_property(pm_state.sw_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &pval);
	if (rc < 0) {
		pr_info("Couldn't get fastcharge mode:%d\n", rc);
		return 0;
	}

	pm_state.bms_fastcharge_mode = pval.intval;

	return pval.intval;
}


/* get thermal level from battery power supply property */
static int qc3_get_batt_current_thermal_level(int *level)
{
	int rc;
	struct power_supply *psy;
	union power_supply_propval val = {0,};

	psy = cp_get_sw_psy();
	if (!psy)
		return 0;

	rc = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);

	if (rc < 0) {
		pr_info("Couldn't get themal level:%d\n", rc);
		return rc;
	}

	pr_debug("val.intval: %d\n", val.intval);

	*level = val.intval;
	return rc;
}

static int cp_qc30_get_input_suspend(struct cp_qc30_data *chip, int *suspend)
{
	int rc;

	rc = cp_qc30_get_smb_iio(chip, SMB_CP_INPUT_SUSPEND, suspend);
	if (rc < 0)
		return rc;

	pr_info("get input suspend:%d, rc = %d\n", *suspend, rc);
	return rc;
}

/* determine whether to disable cp according to jeita status */
static bool qc3_disable_cp_by_jeita_status(struct cp_qc30_data *chip)
{
	int batt_temp = 0, bq_input_suspend = 0;
	int ret;
	struct power_supply *psy;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_input_suspend(chip, &bq_input_suspend);
	if (ret < 0) {
		pr_err("Failed to get input suspend, ret = %d\n", ret);
		return false;
	}

	psy = cp_get_sw_psy();
	if (!psy)
		return false;

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_TEMP, &val);

	if (ret < 0) {
		pr_info("Couldn't get batt temp prop:%d\n", ret);
		return false;
	}

	batt_temp = val.intval;
	pr_debug("batt_temp: %d\n", batt_temp);

	if (bq_input_suspend) {
		return true;
	} else {
		if (batt_temp >= JEITA_WARM_THR && !pm_state.jeita_triggered) {
			pm_state.jeita_triggered = true;
			pr_err("warm batt_temp: %d\n", batt_temp);
			return true;
		} else if (batt_temp <= JEITA_COOL_NOT_ALLOW_CP_THR
				&& !pm_state.jeita_triggered) {
			pm_state.jeita_triggered = true;
			pr_err("cool batt_temp: %d\n", batt_temp);
			return true;
		} else if ((batt_temp <= (JEITA_WARM_THR - JEITA_HYSTERESIS))
					&& (batt_temp >= (JEITA_COOL_NOT_ALLOW_CP_THR + JEITA_HYSTERESIS))
				&& pm_state.jeita_triggered) {
			pm_state.jeita_triggered = false;
			pr_err("return to normal batt_temp: %d\n", batt_temp);
			return false;
		} else {
			return pm_state.jeita_triggered;
		}
	}
}

static void cp_get_batt_capacity(void)
{
	int ret;
	struct power_supply *psy;
	union power_supply_propval val = {0,};

	psy = cp_get_sw_psy();
	if (!psy)
		return;

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		pm_state.capacity = val.intval;
	pr_info("capacity:%d\n", pm_state.capacity);
}

static void cp_update_fc_status(struct cp_qc30_data *chip)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_BUS_VOLTAGE, &val.intval);
	if (!ret)
		pm_state.bq2597x.vbus_volt = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_BUS_CURRENT, &val.intval);
	if (!ret)
		pm_state.bq2597x.ibus_curr = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_BUS_TEMPERATURE, &val.intval);
	if (!ret)
		pm_state.bq2597x.bus_temp = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_BATTERY_TEMPERATURE, &val.intval);
	if (!ret)
		pm_state.bq2597x.bat_temp = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_DIE_TEMPERATURE, &val.intval);
	if (!ret)
		pm_state.bq2597x.die_temp = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_BATTERY_PRESENT, &val.intval);
	if (!ret)
		pm_state.bq2597x.batt_pres = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_VBUS_PRESENT, &val.intval);
	if (!ret)
		pm_state.bq2597x.vbus_pres = val.intval;

	/* battery current and voltage are taken from the fuel gauge */
	cp_update_batt_info();
	pm_state.bq2597x.ibat_curr = pm_state.ibat_now;
	pm_state.bq2597x.vbat_volt = pm_state.vbat_now;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pm_state.bq2597x.charge_enabled = val.intval;

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_SC_ALARM_STATUS, &val.intval);
	if (!ret) {
		pm_state.bq2597x.bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK);
		pm_state.bq2597x.bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK);
		pm_state.bq2597x.bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK);
		pm_state.bq2597x.bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK);
		pm_state.bq2597x.bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK);
		pm_state.bq2597x.bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		pm_state.bq2597x.bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		pm_state.bq2597x.die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	/*
	 * Stock sapphire does not read the fault status channel here, so
	 * the *_fault flags checked in cp_flash2_charge() stay false.
	 */
}


static int cp_enable_charge_pump(struct cp_qc30_data *chip, bool enable)
{
	int ret;

	ret = cp_q30_set_iio_channel(chip, CHARGE_PUMP_CHARGING_ENABLED, enable);
	pr_err("cp enable %d,ret is %d \n", enable, ret);

	return ret;
}

static __always_inline int cp_set_qc_bus_protections(struct cp_qc30_data *chip, int hvdcp3_type)
{
	int ret;

	ret = cp_q30_set_iio_channel(chip, CHARGE_PUMP_SC_SET_BUS_PROTECTION, hvdcp3_type);
	pr_err("set bus protection %d,ret is %d \n", hvdcp3_type, ret);

	return ret;
}

static int cp_enable_sw(struct cp_qc30_data *chip, bool enable)
{
	int ret;

	ret = cp_qc30_set_smb_iio(chip, SMB_BATTERY_CHARGING_ENABLED, enable);
	if (ret < 0)
		pr_err("set battey charging fail\n");
	pr_err("set sw charging en = %d\n", enable);

	return ret;
}

static int cp_limit_sw(struct cp_qc30_data *chip, bool enable)
{
	int ret;

	ret = cp_qc30_set_smb_iio(chip, SMB_BATTERY_CHARGING_LIMITED, enable);
	if (ret < 0)
		pr_err("cp_limit_sw fail\n");

	return ret;
}

static int cp_check_charge_pump_enabled(struct cp_qc30_data *chip)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_iio_channel(chip, CHARGE_PUMP_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pm_state.bq2597x.charge_enabled = !!val.intval;

	pr_debug("pm_state.charge_pump.charge_enabled: %d\n",
			pm_state.bq2597x.charge_enabled);
	return ret;
}

static int cp_check_sw_enabled(struct cp_qc30_data *chip)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_smb_iio(chip, SMB_BATTERY_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pm_state.sw_chager.charge_enabled = !!val.intval;

	pr_debug("battery charging enabled: %d\n",
			pm_state.sw_chager.charge_enabled);
	return ret;
}

static int cp_check_sw_limited(struct cp_qc30_data *chip)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_smb_iio(chip, SMB_BATTERY_CHARGING_LIMITED, &val.intval);
	if (!ret)
		pm_state.sw_chager.charge_limited = !!val.intval;

	pr_debug("pm_state.sw_chager.charge_limited: %d\n",
			pm_state.sw_chager.charge_limited);
	return ret;
}

static void cp_update_sw_status(struct cp_qc30_data *chip)
{
	cp_check_sw_enabled(chip);
	cp_check_sw_limited(chip);
}

static int cp_tune_vbus_volt(struct cp_qc30_data *chip, bool up)
{
	int ret;
	int val;

	if (up)
		val = QTI_POWER_SUPPLY_DP_DM_DP_PULSE;
	else
		val = QTI_POWER_SUPPLY_DP_DM_DM_PULSE;

	ret = cp_qc30_set_smb_iio(chip, SMB_BATTERY_DP_DM, val);

	pr_info("tune adapter voltage %s %s\n", up ? "up" : "down",
			ret ? "fail" : "successfully");

	return ret;

}

static int cp_reset_vbus_volt(struct cp_qc30_data *chip)
{
	int ret = 0;
	int qc3p5_reset_vbus_retry = 0;

	if (pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
		cp_update_fc_status(chip);
		pr_err("vbat=%d,vbus=%d\n", pm_state.bq2597x.vbat_volt, pm_state.bq2597x.vbus_volt);
		while (pm_state.bq2597x.vbus_volt > 6550) {
			cp_tune_vbus_volt(chip, VOLT_DOWN);
			cp_update_fc_status(chip);
			qc3p5_reset_vbus_retry++;
			usleep_range(10000, 10010);
			pr_err("qc3p5_reset_vbus_retry=%d,vbus=%d\n", qc3p5_reset_vbus_retry, pm_state.bq2597x.vbus_volt);
			if (qc3p5_reset_vbus_retry > 300) {
				pr_err("Failed to reset qc3.5 adapter volt to 6.55v or less\n");
				break;
			}
		}
	} else {
		ret = cp_qc30_set_smb_iio(chip, SMB_BATTERY_DP_DM,
				QTI_POWER_SUPPLY_DP_DM_FORCE_5V);

		pr_info("reset vbus volt %s\n", ret ? "fail" : "successfully");
	}
	return ret;

}

static int cp_get_usb_type(struct cp_qc30_data *chip)
{
	int ret;
	int val = 0;

	ret = cp_qc30_get_smb_iio(chip, SMB_REAL_TYPE, &val);
	if (!ret) {
		pm_state.usb_type = val;
		pr_info("cp_get_usb_type:%d\n", pm_state.usb_type);
	}

	return ret;
}

static int cp_get_usb_present(void)
{
	int ret;
	struct power_supply *psy;
	union power_supply_propval val = {0,};

	psy = cp_get_usb_psy();
	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret)
		pm_state.usb_present = val.intval;

	return ret;
}

static int cp_get_qc_hvdcp3_type(struct cp_qc30_data *chip)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = cp_qc30_get_smb_iio(chip, SMB_HVDCP3_TYPE, &val.intval);
	if (!ret) {
		pm_state.hvdcp3_type = val.intval;
		pr_info("cp_get_qc_hvdcp3_type:%d\n", pm_state.hvdcp3_type);
	}
	return ret;
}


#define TAPER_TIMEOUT	10
static int cp_flash2_charge(struct cp_qc30_data *chip)
{
	static int ibus_limit, ibat_limit;
	int thermal_level = 0;
	uint16_t effective_fcc_val = cp_get_effective_fcc_val(pm_state);
	uint16_t effective_ibus_val = effective_fcc_val/2;

	ibus_limit = min(effective_ibus_val, pm_state.ibus_lmt_curr);

	qc3_get_bms_fastcharge_mode();
	qc3_get_batt_current_thermal_level(&thermal_level);

	pm_state.is_temp_out_fc2_range = qc3_disable_cp_by_jeita_status(chip);

	ibat_limit = min(effective_fcc_val, sys_config.bat_curr_lp_lmt);

	pr_info("vbus=%d; ibus_lmt=%d,ibus:%d; vbat_lmt=%d,vbat=%d; ibat_lmt=%d,ibat=%d\n",
			pm_state.bq2597x.vbus_volt, ibus_limit,
			pm_state.bq2597x.ibus_curr,
			sys_config.bat_volt_lp_lmt, pm_state.bq2597x.vbat_volt,
			ibat_limit, pm_state.bq2597x.ibat_curr);

	if (pm_state.bq2597x.vbus_volt <= BUS_OVP_ALARM_THRESHOLD
		&& pm_state.bq2597x.ibus_curr < ibus_limit - sys_config.ibus_minus_deviation_val
		&& pm_state.bq2597x.ibat_curr < ibat_limit - sys_config.ibat_minus_deviation_val
		&& pm_state.bq2597x.vbat_volt < sys_config.bat_volt_lp_lmt - 50) {
			pr_err("cp_tune_vbus_volt up\n");
			cp_tune_vbus_volt(chip, VOLT_UP);
		}

	if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt
		|| pm_state.bq2597x.ibat_curr > ibat_limit + sys_config.ibat_plus_deviation_val
		|| pm_state.bq2597x.ibus_curr > ibus_limit + sys_config.ibus_plus_deviation_val) {
		pr_err("cp_tune_vbus_volt down\n");
		cp_tune_vbus_volt(chip, VOLT_DOWN);
	}

	cp_check_charge_pump_enabled(chip);

	/* battery overheat, stop charge */
	if (pm_state.bq2597x.bat_therm_fault)
		return -ADC_ERR;
	else if (pm_state.bq2597x.bus_ocp_fault
			|| pm_state.bq2597x.bat_ovp_fault
			|| pm_state.bq2597x.bus_ovp_fault)
		return -ADC_ERR;
	else if (!pm_state.bq2597x.charge_enabled)
		return -CP_ENABLE_FAIL;
	else if (thermal_level >= MAX_THERMAL_LEVEL
			|| pm_state.is_temp_out_fc2_range) {
		pr_info("thermal level (%d) too high or batt temp is out of fc2 range, pm_state.is_temp_out_fc2_range:%d\n",
				thermal_level, pm_state.is_temp_out_fc2_range);
		return CP_ENABLE_FAIL;
	}
	if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 100 &&
			pm_state.bq2597x.ibat_curr < sys_config.fc2_taper_current) {
		if (fc2_taper_timer++ > TAPER_TIMEOUT) {
			fc2_taper_timer = 0;
			return TAPER_DONE;
		}
	} else {
		fc2_taper_timer = 0;
	}

	return 0;
}

const unsigned char *pm_state_str[] = {
	"CP_STATE_ENTRY",
	"CP_STATE_DISCONNECT",
	"CP_STATE_SW_ENTRY",
	"CP_STATE_SW_ENTRY_2",
//	"CP_STATE_SW_ENTRY_3",
	"CP_STATE_SW_LOOP",
	"CP_STATE_FLASH2_ENTRY",
	"CP_STATE_FLASH2_ENTRY_1",
//	"CP_STATE_FLASH2_ENTRY_2",
	"CP_STATE_FLASH2_ENTRY_3",
//	"CP_STATE_FLASH2_GET_PPS_STATUS",
	"CP_STATE_FLASH2_TUNE",
	"CP_STATE_FLASH2_DELAY",
	"CP_STATE_STOP_CHARGE",
};

static void cp_move_state(pm_sm_state_t state)
{
	pr_debug("pm_state change:%s -> %s\n",
		pm_state_str[pm_state.state], pm_state_str[state]);
	pm_state.state_log[pm_state.log_idx] = pm_state.state;
	pm_state.log_idx++;
	pm_state.log_idx %= PM_STATE_LOG_MAX;
	pm_state.state = state;
}

static void cp_statemachine(struct cp_qc30_data *chip)
{
	int ret;
	static int tune_vbus_retry, tune_vbus_count, retry_enable_cp_count;
	int thermal_level = 0, usb_icl_value = 0;
	int vbus_target, i;
	static bool recovery;

	if (!pm_state.bq2597x.vbus_pres) {
		pm_state.state = CP_STATE_DISCONNECT;
		recovery = true;
		pr_info("vbus disconnected\n");
	} else if (pm_state.state == CP_STATE_DISCONNECT) {
		pr_info("vbus connected\n");
		recovery = true;
		pm_state.jeita_triggered = false;
		pm_state.is_temp_out_fc2_range = false;
		cp_move_state(CP_STATE_ENTRY);
	}

	switch (pm_state.state) {
	case CP_STATE_DISCONNECT:
		if (!pm_state.fcc_votable)
			pm_state.fcc_votable = find_votable("FCC");
		if (pm_state.fcc_votable)
			vote(pm_state.fcc_votable, QC3P5_BQ_TAPER_FCC_VOTER,
					false, 0);
		if (pm_state.bq2597x.charge_enabled) {
			cp_enable_charge_pump(chip, false);
			cp_check_charge_pump_enabled(chip);
		}
		if (!pm_state.sw_chager.charge_enabled || pm_state.sw_chager.charge_limited) {
			cp_reset_vbus_volt(chip);
			cp_enable_sw(chip, true);
			cp_update_sw_status(chip);
		}

		tune_vbus_count = 0;
		retry_enable_cp_count = 0;
		pm_state.usb_type = 0;
		pm_state.sw_fc2_init_fail = false;
		pm_state.sw_near_cv = false;
		sys_config.bat_curr_lp_lmt = HVDCP3_CLASS_A_BAT_CURRENT_MA;
		sys_config.bus_curr_lp_lmt = HVDCP3_CLASS_A_BUS_CURRENT_MA;
		pm_state.ibus_lmt_curr = HVDCP3_CLASS_A_BUS_CURRENT_MA;
		sys_config.ibus_minus_deviation_val = 450;
		sys_config.ibus_plus_deviation_val = 100;
		sys_config.ibat_minus_deviation_val = 400;
		sys_config.ibat_plus_deviation_val = 350;
		cp_set_qc_bus_protections(chip, HVDCP3_NONE);
		break;

	case CP_STATE_ENTRY:
		chip->work_interval = CP_WORK_INTERVAL_MS;
		if (!pm_state.fcc_votable)
			pm_state.fcc_votable = find_votable("FCC");
		if (pm_state.fcc_votable)
			vote(pm_state.fcc_votable, QC3P5_BQ_TAPER_FCC_VOTER,
					false, 0);
		cp_get_usb_type(chip);
		cp_get_batt_capacity();
		qc3_get_batt_current_thermal_level(&thermal_level);
		pm_state.is_temp_out_fc2_range = qc3_disable_cp_by_jeita_status(chip);
		pr_info("is_temp_out_fc2_range:%d\n", pm_state.is_temp_out_fc2_range);

		if (pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3
			|| pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
			pr_info("vbus_volt:%d\n", pm_state.bq2597x.vbus_volt);
			cp_reset_vbus_volt(chip);
			msleep(100);
			if (thermal_level >= MAX_THERMAL_LEVEL
					|| pm_state.is_temp_out_fc2_range) {
				cp_move_state(CP_STATE_SW_ENTRY);
				pr_info("thermal too high or batt temp out of range or slowly charging, waiting...\n");
			} else if (pm_state.bq2597x.vbat_volt < sys_config.min_vbat_start_flash2)
				cp_move_state(CP_STATE_SW_ENTRY);
			else if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 250
					|| pm_state.capacity >= HIGH_CAPACITY_TRH) {
				pm_state.sw_near_cv = true;
				cp_move_state(CP_STATE_SW_ENTRY);
			} else {
				cp_move_state(CP_STATE_FLASH2_ENTRY);
			}
		}
		break;

	case CP_STATE_SW_ENTRY:
		if (!pm_state.fcc_votable)
			pm_state.fcc_votable = find_votable("FCC");
		if (pm_state.fcc_votable)
			vote(pm_state.fcc_votable, QC3P5_BQ_TAPER_FCC_VOTER,
					false, 0);
		cp_reset_vbus_volt(chip);
		pr_err("vbat=%d,vbus=%d\n", pm_state.bq2597x.vbat_volt, pm_state.bq2597x.vbus_volt);
		pr_info("enable sw charger\n");
		cp_enable_sw(chip, true);
		usb_icl_value = cp_get_effective_usb_icl_val();
		if (pm_state.usb_icl_votable && (usb_icl_value < QC3_MAIN_CHARGER_ICL))
			vote(pm_state.usb_icl_votable, MAIN_CHG_VOTER, true, QC3_MAIN_CHARGER_ICL);

		if (pm_state.bq2597x.charge_enabled) {
			cp_enable_charge_pump(chip, false);
			cp_check_charge_pump_enabled(chip);
		}

		if (!pm_state.bq2597x.charge_enabled)
			cp_move_state(CP_STATE_SW_ENTRY_2);
		break;

	case CP_STATE_SW_ENTRY_2:
		pr_info("sw check enable\n");
		cp_update_sw_status(chip);
		if (pm_state.sw_chager.charge_enabled)
			cp_move_state(CP_STATE_SW_LOOP);
		break;

	case CP_STATE_SW_LOOP:
		pr_err("vbat=%d, vbus=%d\n", pm_state.bq2597x.vbat_volt, pm_state.bq2597x.vbus_volt);
		qc3_get_batt_current_thermal_level(&thermal_level);
		if (retry_enable_cp_count >= 5) {
			pr_info("retry_enable_cp_count=%d\n", retry_enable_cp_count);
			break;
		}

		pm_state.is_temp_out_fc2_range = qc3_disable_cp_by_jeita_status(chip);
		if (thermal_level < MAX_THERMAL_LEVEL && !pm_state.is_temp_out_fc2_range && recovery) {
			if (tune_vbus_count >= 2) {
				pr_info("unsupport qc3, use sw charging\n");
				break;
			}
			tune_vbus_count++;
			pr_info("thermal or batt temp recovery...\n");
			recovery = false;
		} else if (thermal_level >= MAX_THERMAL_LEVEL || pm_state.is_temp_out_fc2_range) {
			pr_info("thermal(%d) too high or batt temp out of range\n", thermal_level);
		}
		cp_get_batt_capacity();
		if (pm_state.bq2597x.vbat_volt > sys_config.bat_volt_lp_lmt - 250
					|| pm_state.capacity >= HIGH_CAPACITY_TRH) {
				pm_state.sw_near_cv = true;
		}
		if (!pm_state.sw_near_cv && !recovery) {
			if (pm_state.bq2597x.vbat_volt > sys_config.min_vbat_start_flash2) {
				pr_info("battery volt: %d is ok, proceeding to flash charging...\n",
					pm_state.bq2597x.vbat_volt);
				usb_icl_value = cp_get_effective_usb_icl_val();
				if (pm_state.usb_icl_votable && (usb_icl_value > QC3_CHARGER_ICL))
					vote(pm_state.usb_icl_votable, MAIN_CHG_VOTER, true, QC3_CHARGER_ICL);
				cp_move_state(CP_STATE_FLASH2_ENTRY);
			}
		}
		break;

	case CP_STATE_FLASH2_ENTRY:
		if (!pm_state.sw_chager.charge_limited) {
			cp_limit_sw(chip, true);
			cp_update_sw_status(chip);
		}

		if (pm_state.sw_chager.charge_limited) {
			cp_move_state(CP_STATE_FLASH2_ENTRY_1);
			tune_vbus_retry = 0;
		}

		cp_get_qc_hvdcp3_type(chip);
		if (pm_state.hvdcp3_type == HVDCP3_CLASSB_27W) {
			sys_config.bat_curr_lp_lmt = HVDCP3_CLASS_B_BAT_CURRENT_MA;
			sys_config.bus_curr_lp_lmt = HVDCP3_CLASS_B_BUS_CURRENT_MA;
		} else {
			sys_config.bat_curr_lp_lmt = HVDCP3_CLASS_A_BAT_CURRENT_MA;
			sys_config.bus_curr_lp_lmt = HVDCP3_CLASS_A_BUS_CURRENT_MA;
		}
		pm_state.ibus_lmt_curr = sys_config.bus_curr_lp_lmt;
		break;

	case CP_STATE_FLASH2_ENTRY_1:
		cp_update_fc_status(chip);
		pr_err("entry1:vbat=%d,vbus=%d\n", pm_state.bq2597x.vbat_volt, pm_state.bq2597x.vbus_volt);
		vbus_target = pm_state.bq2597x.vbat_volt * debug_mask / 10 + BUS_VOLT_INIT_UP;
		if (pm_state.bq2597x.vbus_volt < vbus_target) {
			if (vbus_target - pm_state.bq2597x.vbus_volt > 2600) {
				/* far below the target: pulse up quickly */
				for (i = 0; i < 15; i++) {
					cp_tune_vbus_volt(chip, VOLT_UP);
					msleep(45);
					tune_vbus_retry++;
				}
			} else {
				cp_tune_vbus_volt(chip, VOLT_UP);
				tune_vbus_retry++;
			}
		} else {
			pr_err("entry1:voltage tuned above expected voltage, retry %d times\n", tune_vbus_retry);
			cp_move_state(CP_STATE_FLASH2_ENTRY_3);
			break;
		}

		if (tune_vbus_retry > 23) {
			pr_err("entry1:Failed to tune adapter volt into valid range, charge with switching charger\n");
			pm_state.sw_fc2_init_fail = true;
			recovery = true;
			cp_move_state(CP_STATE_SW_ENTRY);
		}
		break;

	case CP_STATE_FLASH2_ENTRY_3:
		if (pm_state.bq2597x.vbus_volt >
				(pm_state.bq2597x.vbat_volt * debug_mask / 10 + BUS_VOLT_INIT_UP + 450)) {
			/* voltage is too high, wait for voltage down, keep charge disabled to discharge */
			cp_tune_vbus_volt(chip, VOLT_DOWN);
			cp_update_fc_status(chip);
			pr_err("entry3:vbus volt is too high:%d, vbat:%d wait it down\n",
					pm_state.bq2597x.vbus_volt, pm_state.bq2597x.vbat_volt);
		} else {
			pr_err("entry3:vbat volt is ok, enable flash charging, charge_enabled=%d\n",
					pm_state.bq2597x.charge_enabled);
			if (!pm_state.bq2597x.charge_enabled) {
				cp_enable_charge_pump(chip, true);
				cp_check_charge_pump_enabled(chip);
			}
			if (pm_state.bq2597x.charge_enabled) {
				if (retry_enable_cp_count > 0)
					retry_enable_cp_count = 0;
				cp_move_state(CP_STATE_FLASH2_TUNE);
				if (pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
					cp_enable_sw(chip, false);
					cp_update_sw_status(chip);
				}
			} else {
				retry_enable_cp_count++;
				if (retry_enable_cp_count < 5)
					cp_move_state(CP_STATE_FLASH2_ENTRY_3);
				else
					cp_move_state(CP_STATE_SW_ENTRY);
			}
			fc2_taper_timer = 0;
		}
		break;

	case CP_STATE_FLASH2_TUNE:
		ret = cp_flash2_charge(chip);
		if (ret == -ADC_ERR) {
			pr_err("Move to stop charging:%d\n", ret);
			cp_move_state(CP_STATE_STOP_CHARGE);
			break;
		} else if (ret == -CP_ENABLE_FAIL || ret == TAPER_DONE) {
			pr_err("Move to switch charging:%d\n", ret);
			cp_move_state(CP_STATE_SW_ENTRY);
			break;
		} else if (ret == CP_ENABLE_FAIL) {
			tune_vbus_count = 0;
			pr_err("Move to switch charging, will try to recover to flash charging:%d\n",
					ret);
			recovery = true;
			cp_move_state(CP_STATE_SW_ENTRY);
		} else {// normal tune adapter output
			cp_move_state(CP_STATE_FLASH2_DELAY);
		}
		break;

	case CP_STATE_FLASH2_DELAY:
		chip->work_interval = CP_DELAY_INTERVAL_MS;
		cp_move_state(CP_STATE_FLASH2_TUNE);
		break;

	case CP_STATE_STOP_CHARGE:
		pr_err("Stop charging\n");
		if (pm_state.bq2597x.charge_enabled) {
			cp_enable_charge_pump(chip, false);
			cp_check_charge_pump_enabled(chip);
		}
		if (pm_state.sw_chager.charge_enabled) {
			cp_enable_sw(chip, false);
			cp_update_sw_status(chip);
		}
		break;

	default:
		pr_err("No state defined! Move to stop charging\n");
		cp_move_state(CP_STATE_STOP_CHARGE);
		break;
	}
}

static void cp_workfunc(struct work_struct *work)
{
	struct cp_qc30_data *chip = container_of(work, struct cp_qc30_data,
					qc3_pm_work.work);

	cp_get_usb_type(chip);

	cp_update_sw_status(chip);
	cp_update_fc_status(chip);

	cp_statemachine(chip);

	cp_get_usb_present();
	/* check whether usb is present */
	if (pm_state.usb_present == 0) {
		cp_set_qc_bus_protections(chip, HVDCP3_NONE);
		return;
	}

	if (pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3)
		schedule_delayed_work(&chip->qc3_pm_work,
				msecs_to_jiffies(chip->work_interval));
}

static int cp_qc30_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct cp_qc30_data *chip = container_of(nb, struct cp_qc30_data, nb);
	struct power_supply *psy = v;
	static bool usb_hvdcp3_on;
	static bool first_flag;

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!first_flag) {
		if (is_cp_chan_valid(chip, CHARGE_PUMP_PRESENT)) {
			chip->is_bq25960 = false;
			first_flag = true;
			pr_err("[cp_qc_psy_notifier_cb] SC8541\n");
		} else if (is_cp_chan_valid(chip, CHARGE_PUMP_BQ_PRESENT)) {
			chip->is_bq25960 = true;
			first_flag = true;
			pr_err("[cp_qc_psy_notifier_cb] BQ25960\n");
		} else {
			first_flag = false;
			pr_err("qc3.0 can't find charge pump\n");
		}
	}

	if (strcmp(psy->desc->name, "usb") == 0) {
		cp_get_usb_type(chip);
		if (pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3 || pm_state.usb_type == QTI_POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
			if (!delayed_work_pending(&chip->qc3_pm_work))
				schedule_delayed_work(&chip->qc3_pm_work, 3*HZ);
			usb_hvdcp3_on = true;
		} else if (pm_state.usb_type == POWER_SUPPLY_TYPE_UNKNOWN && usb_hvdcp3_on == true) {
			cancel_delayed_work(&chip->qc3_pm_work);
			schedule_delayed_work(&chip->qc3_pm_work, 0);
			pr_info("pm_state.usb_type: %d\n", pm_state.usb_type);
			usb_hvdcp3_on = false;
		}
	}

	return NOTIFY_OK;
}

static int cp_qc30_register_notifier(struct cp_qc30_data *chip)
{
	int rc;

	chip->nb.notifier_call = cp_qc30_notifier_call;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int cp_qc30_iio_init(struct cp_qc30_data *chip)
{
	pr_err("cp_qc30_iio_init start\n");

	chip->cp_iio = devm_kcalloc(chip->dev, ARRAY_SIZE(cp_iio_chan_name),
				sizeof(*chip->cp_iio), GFP_KERNEL);
	if (!chip->cp_iio)
		return -ENOMEM;

	chip->smb_iio = devm_kcalloc(chip->dev, ARRAY_SIZE(smb_iio_chan_name),
				sizeof(*chip->smb_iio), GFP_KERNEL);
	if (!chip->smb_iio)
		return -ENOMEM;

	pr_err("cp_qc30_iio_init end\n");
	return 0;
}

static int cp_qc30_parse_dt(struct cp_qc30_data *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node,
			"mi,qc3-bat-volt-max", &chip->bat_volt_max);
	if (rc < 0)
		pr_err("qc3-bat-volt-max property missing, use default val\n");
	else
		sys_config.bat_volt_lp_lmt = chip->bat_volt_max;

	rc = of_property_read_u32(node,
			"mi,qc3-ffc-bat-volt-max", &chip->ffc_bat_volt_max);
	if (rc < 0)
		pr_err("qc3-ffc-bat-volt-max property missing, use default\n");
	else
		sys_config.ffc_bat_volt_lmt = chip->ffc_bat_volt_max;

	rc = of_property_read_u32(node,
			"mi,qc3-bat-curr-max", &chip->bat_curr_max);
	if (rc < 0)
		pr_err("qc3-bat-curr-max property missing, use default val\n");
	else
		sys_config.bat_curr_lp_lmt = chip->bat_curr_max;

	rc = of_property_read_u32(node,
			"mi,qc3-bus-volt-max", &chip->bus_volt_max);
	if (rc < 0)
		pr_err("qc3-bus-volt-max property missing, use default val\n");
	else
		sys_config.bus_volt_lp_lmt = chip->bus_volt_max;

	rc = of_property_read_u32(node,
			"mi,qc3-bus-curr-max", &chip->bus_curr_max);
	if (rc < 0)
		pr_err("qc3-bus-curr-max property missing, use default val\n");
	else
		sys_config.bus_curr_lp_lmt = chip->bus_curr_max;

	chip->cp_sec_enable = of_property_read_bool(node,
				"mi,cp-sec-enable");

	sys_config.cp_sec_enable = chip->cp_sec_enable;

	return rc;
}

static int cp_qc30_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct cp_qc30_data *chip;

	pr_info("%s enter\n", __func__);

	chip = devm_kzalloc(dev, sizeof(struct cp_qc30_data), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	ret = cp_qc30_parse_dt(chip);
	if (ret < 0) {
		pr_err("Couldn't parse device tree rc=%d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, chip);

	pm_state.state = CP_STATE_DISCONNECT;
	pm_state.usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
	pm_state.ibus_lmt_curr = sys_config.bus_curr_lp_lmt;
	chip->work_interval = CP_WORK_INTERVAL_MS;

	/* as in stock, a failed allocation is not fatal here */
	cp_qc30_iio_init(chip);

	INIT_DELAYED_WORK(&chip->qc3_pm_work, cp_workfunc);

	cp_qc30_register_notifier(chip);

	pr_info("charge pump qc3 probe success\n");

	return ret;
}

static int cp_qc30_remove(struct platform_device *pdev)
{
	struct cp_qc30_data *chip = platform_get_drvdata(pdev);

	cancel_delayed_work(&chip->qc3_pm_work);
	return 0;
}

static const struct of_device_id cp_qc30_of_match[] = {
	{ .compatible = "xiaomi,cp-qc30", },
	{},
};

static struct platform_driver cp_qc30_driver = {
	.driver = {
		.name = "cp-qc30",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cp_qc30_of_match),
	},
	.probe = cp_qc30_probe,
	.remove = cp_qc30_remove,
};

static int __init cp_qc30_init(void)
{
	return platform_driver_register(&cp_qc30_driver);
}

module_init(cp_qc30_init);

static void __exit cp_qc30_exit(void)
{
	return platform_driver_unregister(&cp_qc30_driver);
}
module_exit(cp_qc30_exit);

MODULE_AUTHOR("Fei Jiang<jiangfei1@xiaomi.com>");
MODULE_DESCRIPTION("Xiaomi cp qc30");
MODULE_LICENSE("GPL");
/* stock module is built in-tree (no out-of-tree taint) */
MODULE_INFO(intree, "Y");
