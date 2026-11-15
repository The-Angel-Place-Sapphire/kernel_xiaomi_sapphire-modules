// SPDX-License-Identifier: GPL-2.0
/*
 * USB Power Delivery charge pump policy manager.
 *
 * Sapphire (Redmi Note 13 4G, khaje, 5.15 GKI) version.
 * Base: Xiaomi sunny/mojito drivers/power/supply/ti/pd_policy_manager.c
 * (smb5 state machine), IIO helpers after MiCode topaz-t-oss
 * drivers/power/supply/pd_policy_manager.c, and the sapphire-only "N7"
 * layer (2:1 / 1:1 charge pump work mode, LOW_FAST votable, smart and
 * night charging hold, SC8541/BQ25960 detection) reconstructed from the
 * stock sapphire vendor_dlkm pd_policy_manager.ko.
 */

#define pr_fmt(fmt)	"[USBPD-PM]: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>
#include <linux/iio/consumer.h>
#include <linux/qti_power_supply.h>

#include "pd_policy_manager.h"

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define BATT_MAX_CHG_VOLT		4500
#define BATT_FAST_CHG_CURR		6000
#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#define BATT_WARM_CHG_VOLT		4100

#define BUS_VOLT_INIT_UP		400

#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD

/* the 5V fixed PDO requested while the charge pump is not running */
#define PD_FIXED_5V_PDO			1

enum {
	PM_ALGO_RET_OK,
	PM_ALGO_RET_THERM_FAULT,
	PM_ALGO_RET_OTHER_FAULT,
	PM_ALGO_RET_CHG_DISABLED,
	PM_ALGO_RET_SMART_RELEASE,
	PM_ALGO_RET_SMART_HOLD,
	PM_ALGO_RET_TAPER_DONE,
};

static int fast_work_time = 200;
module_param(fast_work_time, int, 0600);
static int stable_work_time = 1500;
module_param(stable_work_time, int, 0600);
static int fast_tune_step = 20;
module_param(fast_tune_step, int, 0600);
static int stable_tune_step = 3;
module_param(stable_tune_step, int, 0600);

static struct pdpm_config pm_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT + 1000,
	.bus_volt_lp_lmt		= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt		= BAT_CURR_LOOP_LMT >> 1,
	.bus_curr_compensate	= 200,

	.fc2_taper_current		= 2300,
	.fc2_steps			= 1,

	.min_adapter_volt_required	= 10000,
	.min_adapter_curr_required	= 2000,

	.min_vbat_for_cp		= 3000,

	.cp_sec_enable			= false,
	.fc2_disable_sw			= true,
};

static int fc2_taper_timer;
static int ibus_lmt_change_timer;

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
	[CHARGE_PUMP_SC_SET_CP_WORKMODE] = "sc_set_cp_workmode",
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
	[CHARGE_PUMP_BQ_SET_CP_WORKMODE] = "bq_set_cp_workmode",
};

static const char * const smb_iio_chan_name[] = {
	[SMB_PD_ACTIVE] = "pd_active",
	[SMB_CP_INPUT_SUSPEND] = "cp_input_suspend",
	[SMB_BATTERY_CHARGING_ENABLED] = "battery_charging_enabled",
	[SMB_APDO_VOLT] = "apdo_volt",
	[SMB_APDO_CURR] = "apdo_curr",
};

static bool is_cp_chan_valid(struct usbpd_pm *chip,
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

static bool is_smb_chan_valid(struct usbpd_pm *chip,
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

int usbpd_get_iio_channel(struct usbpd_pm *chg,
		enum cp_iio_type type, int channel, int *val)
{
	struct iio_channel *iio_chan_list;
	int rc;

	switch (type) {
	case CP_MASTER:
		if (chg->is_bq25960)
			channel += CP_BQ25960_IIO_CHANNEL_OFFSET;
		if (!is_cp_chan_valid(chg, channel))
			return -ENODEV;
		iio_chan_list = chg->cp_iio[channel];
		break;
	case SMB:
		if (!is_smb_chan_valid(chg, channel))
			return -ENODEV;
		iio_chan_list = chg->smb_iio[channel];
		break;
	default:
		pr_err_ratelimited("iio_type %d is not supported\n", type);
		return -EINVAL;
	}

	rc = iio_read_channel_processed(iio_chan_list, val);
	return rc < 0 ? rc : 0;
}

int usbpd_set_iio_channel(struct usbpd_pm *chg,
		enum cp_iio_type type, int channel, int val)
{
	struct iio_channel *iio_chan_list;
	int rc;

	switch (type) {
	case CP_MASTER:
		if (chg->is_bq25960)
			channel += CP_BQ25960_IIO_CHANNEL_OFFSET;
		if (!is_cp_chan_valid(chg, channel))
			return -ENODEV;
		iio_chan_list = chg->cp_iio[channel];
		break;
	case SMB:
		if (!is_smb_chan_valid(chg, channel))
			return -ENODEV;
		iio_chan_list = chg->smb_iio[channel];
		break;
	default:
		pr_err_ratelimited("iio_type %d is not supported\n", type);
		return -EINVAL;
	}

	rc = iio_write_channel_raw(iio_chan_list, val);
	return rc < 0 ? rc : 0;
}

int usbpd_set_cp_workmode(struct usbpd_pm *pdpm, int workmode)
{
	int rc;

	rc = usbpd_set_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_SET_CP_WORKMODE, workmode);
	pr_info("N7:set cp workmode is %d \n", workmode);

	return rc;
}

int usbpd_get_cp_workmode(struct usbpd_pm *pdpm)
{
	int rc;
	int workmode = 0;

	rc = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_SET_CP_WORKMODE, &workmode);
	if (!rc) {
		pr_info("N7:get cp workmode is %d \n", workmode);
		return workmode;
	}

	return rc;
}

static void usbpd_check_usb_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->usb_psy) {
		pdpm->usb_psy = power_supply_get_by_name("usb");
		if (!pdpm->usb_psy)
			pr_err("usb psy not found!\n");
	}
}

static void usbpd_check_batt_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->sw_psy) {
		pdpm->sw_psy = power_supply_get_by_name("battery");
		if (!pdpm->sw_psy)
			pr_err("batt psy not found!\n");
	}
}

/* get thermal level from battery power supply property */
static int pd_get_batt_current_thermal_level(struct usbpd_pm *pdpm, int *level)
{
	union power_supply_propval pval = {0,};
	int rc = 0;

	usbpd_check_batt_psy(pdpm);

	if (!pdpm->sw_psy)
		return -ENODEV;

	rc = power_supply_get_property(pdpm->sw_psy,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &pval);
	if (rc < 0) {
		pr_info("Couldn't get fastcharge mode:%d\n", rc);
		return rc;
	}

	*level = pval.intval;
	return rc;
}

/* determine whether to disable cp according to jeita status */
static bool pd_disable_cp_by_jeita_status(struct usbpd_pm *pdpm)
{
	union power_supply_propval pval = {0,};
	int batt_temp = 0, bq_input_suspend = 0;
	int rc;

	usbpd_check_batt_psy(pdpm);

	if (!pdpm->sw_psy)
		return -ENODEV;

	rc = usbpd_get_iio_channel(pdpm, SMB, SMB_CP_INPUT_SUSPEND,
			&bq_input_suspend);
	if (rc < 0) {
		pr_err("Failed to get input suspend, rc = %d\n", rc);
		return false;
	}

	rc = power_supply_get_property(pdpm->sw_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_info("Couldn't get batt temp prop:%d\n", rc);
		return false;
	}

	batt_temp = pval.intval;
	if (bq_input_suspend) {
		return true;
	} else {
		if (batt_temp >= JEITA_WARM_THR && !pdpm->jeita_triggered) {
			pr_err("jeita upper limit reached, batt_temp:%d\n", batt_temp);
			pdpm->jeita_triggered = true;
			return true;
		} else if (batt_temp <= JEITA_COOL_NOT_ALLOW_CP_THR && !pdpm->jeita_triggered) {
			pr_err("jeita lower limit reached, batt_temp:%d\n", batt_temp);
			pdpm->jeita_triggered = true;
			return true;
		} else if (pdpm->bq_cool_warm_done) {
			if (((pdpm->cp.vbat_volt < (pm_config.bat_volt_lp_lmt - 100)) && (batt_temp <= COOL_HYS_THRESHOLDS)) ||
			((pdpm->cp.vbat_volt < (BATT_WARM_CHG_VOLT - 100)) && (batt_temp >= WARM_HYS_THRESHOLDS)) ||
			((batt_temp < WARM_HYS_THRESHOLDS) && (batt_temp > COOL_HYS_THRESHOLDS))) {
				pr_err("after cool and warm state recharge, batt_temp:%d,vbat_volt=%d,bat_volt_lp_lmt=%d,bq_cool_warm_done=%d\n",
					batt_temp, pdpm->cp.vbat_volt, pm_config.bat_volt_lp_lmt, pdpm->bq_cool_warm_done);
				pdpm->bq_cool_warm_done = false;
				return false;
			} else {
				return true;
			}

		} else if ((batt_temp <= (JEITA_WARM_THR - JEITA_HYSTERESIS))
				&& (batt_temp >= (JEITA_COOL_NOT_ALLOW_CP_THR + JEITA_HYSTERESIS))
				&& pdpm->jeita_triggered) {
			pr_err("jeita returned to normal, batt_temp:%d\n", batt_temp);
			pdpm->jeita_triggered = false;
			return false;
		} else {
			return pdpm->jeita_triggered;
		}
	}
}

static bool is_cool_charge(struct usbpd_pm *pdpm)
{
	union power_supply_propval pval = {0,};
	int batt_temp = 0;
	int rc;

	/* stock bug fixed: stock dereferences a possibly NULL battery psy */
	if (!pdpm->sw_psy)
		return false;

	rc = power_supply_get_property(pdpm->sw_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_info("Couldn't get batt temp prop:%d\n", rc);
		return false;
	}
	batt_temp = pval.intval;

	if (batt_temp < (COOL_HYS_THRESHOLDS - 30))
		return true;
	return false;
}

static bool is_warm_charge(struct usbpd_pm *pdpm)
{
	union power_supply_propval pval = {0,};
	int batt_temp = 0;
	int rc;

	/* stock bug fixed: stock dereferences a possibly NULL battery psy */
	if (!pdpm->sw_psy)
		return false;

	rc = power_supply_get_property(pdpm->sw_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_info("Couldn't get batt temp prop:%d\n", rc);
		return false;
	}
	batt_temp = pval.intval;

	if (batt_temp > (WARM_HYS_THRESHOLDS + 30))
		return true;
	return false;
}

static int usbpd_get_effective_fcc_val(struct usbpd_pm *pdpm)
{
	int effective_fcc_val = 0;

	if (!pdpm->fcc_votable)
		pdpm->fcc_votable = find_votable("FCC");

	if (!pdpm->fcc_votable)
		return -EINVAL;

	effective_fcc_val = get_effective_result(pdpm->fcc_votable);
	effective_fcc_val = effective_fcc_val / 1000;
	return effective_fcc_val;
}

static void usbpd_pm_update_cp_status(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_BUS_VOLTAGE, &val.intval);
	if (!ret)
		pdpm->cp.vbus_volt = val.intval;

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_BUS_CURRENT, &val.intval);
	if (!ret)
		pdpm->cp.ibus_curr = val.intval;

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_ALARM_STATUS, &val.intval);
	if (!ret) {
		pdpm->cp.bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK);
		pdpm->cp.bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK);
		pdpm->cp.bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK);
		pdpm->cp.bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK);
		pdpm->cp.bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK);
		pdpm->cp.bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		pdpm->cp.bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		pdpm->cp.die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_SC_FAULT_STATUS, &val.intval);
	if (!ret) {
		pdpm->cp.bat_ovp_fault = !!(val.intval & BAT_OVP_FAULT_MASK);
		pdpm->cp.bat_ocp_fault = !!(val.intval & BAT_OCP_FAULT_MASK);
		pdpm->cp.bus_ovp_fault = !!(val.intval & BUS_OVP_FAULT_MASK);
		pdpm->cp.bus_ocp_fault = !!(val.intval & BUS_OCP_FAULT_MASK);
		pdpm->cp.bat_therm_fault = !!(val.intval & BAT_THERM_FAULT_MASK);
		pdpm->cp.bus_therm_fault = !!(val.intval & BUS_THERM_FAULT_MASK);
		pdpm->cp.die_therm_fault = !!(val.intval & DIE_THERM_FAULT_MASK);
	}
}

static int usbpd_pm_enable_cp(struct usbpd_pm *pdpm, bool enable)
{
	return usbpd_set_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_CHARGING_ENABLED, enable);
}

static __always_inline int usbpd_pm_check_cp_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = usbpd_get_iio_channel(pdpm, CP_MASTER,
			CHARGE_PUMP_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	return ret;
}

static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;

	ret = usbpd_set_iio_channel(pdpm, SMB,
			SMB_BATTERY_CHARGING_ENABLED, enable);
	if (!ret)
		pdpm->sw.charge_enabled = enable;

	pr_err("set battey charging en = %d, ret = %d\n", enable, ret);

	return ret;
}

/*
 * qpnp-smb5 on 5.15 has no "battery charging limited" channel: limiting
 * the switching charger simply disables it, and the limited flag is only
 * kept in software.
 */
static int usbpd_pm_limit_sw(struct usbpd_pm *pdpm, bool enable)
{
	if (enable)
		usbpd_pm_enable_sw(pdpm, false);
	pdpm->sw.charge_limited = enable;

	return 0;
}

static int usbpd_pm_check_sw_limited(struct usbpd_pm *pdpm)
{
	pdpm->sw.charge_limited = false;

	return 0;
}

static int usbpd_pm_check_sw_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	ret = usbpd_get_iio_channel(pdpm, SMB,
			SMB_BATTERY_CHARGING_ENABLED, &val.intval);
	if (!ret)
		pdpm->sw.charge_enabled = !!val.intval;
	else
		pr_err("Failed to check sw enabled, ret = %d\n", ret);

	return ret;
}

static void usbpd_pm_update_sw_status(struct usbpd_pm *pdpm)
{
	usbpd_pm_check_sw_enabled(pdpm);
	usbpd_pm_check_sw_limited(pdpm);
}

static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{
	int ret;
	int i;

	if (!pdpm->pd) {
		pdpm->pd = smb_get_usbpd();
		if (!pdpm->pd) {
			pr_err("couldn't get usbpd device\n");
			return;
		}
	}

	ret = usbpd_fetch_pdo(pdpm->pd, pdpm->pdo);
	if (ret) {
		pr_err("Failed to fetch pdo info\n");
		return;
	}

	pdpm->apdo_max_volt = pm_config.min_adapter_volt_required;
	pdpm->apdo_max_curr = pm_config.min_adapter_curr_required;

	for (i = 0; i < PDO_MAX_NUM; i++) {
		if (pdpm->pdo[i].type == PD_SRC_PDO_TYPE_AUGMENTED
			&& pdpm->pdo[i].pps && pdpm->pdo[i].pos) {
			if (pdpm->pdo[i].max_volt_mv >= pdpm->apdo_max_volt
					&& pdpm->pdo[i].curr_ma >= pdpm->apdo_max_curr) {
				pdpm->apdo_max_volt = pdpm->pdo[i].max_volt_mv;
				pdpm->apdo_max_curr = pdpm->pdo[i].curr_ma;
				pdpm->apdo_selected_pdo = pdpm->pdo[i].pos;
				pdpm->pps_supported = true;
				break;
			}
		}
	}

	if (pdpm->pps_supported) {
		usbpd_set_iio_channel(pdpm, SMB, SMB_APDO_VOLT, pdpm->apdo_max_volt);
		usbpd_set_iio_channel(pdpm, SMB, SMB_APDO_CURR, pdpm->apdo_max_curr);
		pr_info("PPS supported, preferred APDO pos:%d, max volt:%d, current:%d\n",
				pdpm->apdo_selected_pdo,
				pdpm->apdo_max_volt,
				pdpm->apdo_max_curr);
		if (pdpm->apdo_max_curr <= LOW_POWER_PPS_CURR_THR)
			pdpm->apdo_max_curr = XIAOMI_LOW_POWER_PPS_CURR_MAX;
		if (!IS_ERR(pdpm->pd) &&
				pdpm->src_cap_id != usbpd_get_src_cap_id(pdpm->pd))
			pdpm->src_cap_id = usbpd_get_src_cap_id(pdpm->pd);
	} else {
		pr_info("Not qualified PPS adapter\n");
	}
}

static int usbpd_update_ibat_curr(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = power_supply_get_by_name("battery");
		if (!pdpm->sw_psy) {
			pr_err("Failed to get battery psy.\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(pdpm->sw_psy,
		POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		pdpm->cp.ibat_curr = (int)(val.intval / 1000);

	ret = power_supply_get_property(pdpm->sw_psy,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		pdpm->cp.vbat_volt = (int)(val.intval / 1000);

	return ret;
}

/* N7: choose 2:1 or 1:1 charge pump mode from the requested battery current */
static void pdm_multi_mode_switch(struct usbpd_pm *pdpm)
{
	int ibat_request;

	if (!pdpm || pdpm->is_temp_out_fc2_range)
		return;

	ibat_request = usbpd_get_effective_fcc_val(pdpm);
	pr_info("N7:ibat_request = %d, 2_1 mode = %d, 1_1 mode = %d\n",
			ibat_request, pdpm->cp_mode_2_1, pdpm->cp_mode_1_1);

	pdpm->cp_mode_2_1 = true;
	pdpm->cp_mode_1_1 = false;
	if (ibat_request >= CP_1_1_MODE_IBAT_MIN &&
			ibat_request <= CP_1_1_MODE_IBAT_MAX) {
		pdpm->cp_1_1_request = true;
		pdpm->cp_mode_2_1 = false;
		pdpm->cp_mode_1_1 = true;
		pdpm->dive_rate = 1;
	} else {
		pdpm->cp_1_1_request = false;
		pdpm->dive_rate = 2;
	}
}

static void pdm_multi_mode_init(struct usbpd_pm *pdpm, bool start)
{
	if (pdpm) {
		if (start) {
			pdpm->cp_mode_2_1 = true;
			pdpm->cp_mode_1_1 = false;
			pdpm->dive_rate = 2;
		} else {
			pdpm->cp_1_1_request = false;
			pdpm->cp_mode_2_1 = false;
			pdpm->cp_mode_1_1 = false;
			pdpm->dive_rate = 0;
		}
	}
}

/*
 * N7: bring the charge pump work mode in line with the wanted mode.
 * Returns true when the mode was changed (charge pump restarted).
 */
static bool update_smart_chg_cp_workmode(struct usbpd_pm *pdpm)
{
	struct votable *low_fast_votable;
	int low_fast_working_flag;
	int cp_workmode;
	int new_mode;

	low_fast_votable = find_votable("LOW_FAST");
	if (!low_fast_votable) {
		pr_err("%s failed to get low_fast_votable\n", __func__);
		return false;
	}

	low_fast_working_flag = get_effective_result(low_fast_votable);
	if (low_fast_working_flag < 0) {
		pr_err("%s failed to get low_fast_working_flag\n", __func__);
		return false;
	}

	cp_workmode = usbpd_get_cp_workmode(pdpm);
	pr_info("N7: update cp mode, get cp mode is %d, 2_1 mode is %d, 1_1 mode is %d, low_fast_working_flag is %d\n",
			cp_workmode, pdpm->cp_mode_2_1, pdpm->cp_mode_1_1,
			low_fast_working_flag);

	if (cp_workmode != CP_WORKMODE_2_1 && pdpm->cp_mode_2_1)
		new_mode = CP_WORKMODE_2_1;
	else if (cp_workmode == CP_WORKMODE_2_1 && pdpm->cp_mode_1_1 &&
			!low_fast_working_flag)
		new_mode = CP_WORKMODE_1_1;
	else
		return false;

	usbpd_pm_enable_cp(pdpm, false);
	msleep(1000);
	usbpd_set_cp_workmode(pdpm, new_mode);

	return true;
}

#define TAPER_TIMEOUT	10
#define IBUS_CHANGE_TIMEOUT  5
static int usbpd_pm_fc2_charge_algo(struct usbpd_pm *pdpm)
{
	int steps = 0, sw_ctrl_steps = 0, hw_ctrl_steps = 0;
	int step_vbat = 0, step_ibus = 0, step_ibat = 0;
	int step_bat_reg = 0;
	int ibus_total = 0;
	int fcc_vote_val = 0, effective_fcc_taper = 0;
	int fcc_ibatt_diff = 0, sicl_ibus_diff = 0;
	static int ibus_limit, fcc_limit;
	static int cool_warm_overcharge_timer;
	static int g_smartchg_val, g_nightchg_val;
	int time_delta = 0;
	int thermal_level = 0;

	pd_get_batt_current_thermal_level(pdpm, &thermal_level);
	time_delta = ktime_ms_delta(ktime_get(), pdpm->entry_bq_cv_time);

	fcc_vote_val = usbpd_get_effective_fcc_val(pdpm);
	fcc_limit = min(fcc_vote_val, pm_config.bat_curr_lp_lmt);
	if (pdpm->cp_mode_2_1 || !pdpm->cp_mode_1_1)
		ibus_limit = fcc_limit >> 1;
	else
		ibus_limit = fcc_limit;
	pr_info("N7:get fcc_limit = %d, ibus_limit = %d\n", fcc_limit, ibus_limit);

	/* N7: smart charging / night charging hold */
	pdpm->chg_disable_votable = find_votable("CHG_DISABLE");
	if (pdpm->chg_disable_votable) {
		g_smartchg_val = get_client_vote(pdpm->chg_disable_votable,
					SMART_CHARGING_VOTER);
		g_nightchg_val = get_client_vote(pdpm->chg_disable_votable,
					NIGHT_CHARGING_VOTER);
	}
	pr_info("N7:smart chg ctrl val = %d\n", g_smartchg_val);
	if (g_smartchg_val || g_nightchg_val) {
		pr_info("N7:get smart chg ctrl val = %d, night_charging ctrl val = %d\n",
				g_smartchg_val, g_nightchg_val);
		pdpm->smart_chg_hold = true;
		return PM_ALGO_RET_SMART_HOLD;
	} else if (pdpm->smart_chg_hold) {
		pdpm->smart_chg_hold = false;
		pr_info("N7:release ctrl\n");
		return PM_ALGO_RET_SMART_RELEASE;
	}

	/* reduce bus current in cv loop */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - BQ_TAPER_HYS_MV) {
		if (ibus_lmt_change_timer++ > IBUS_CHANGE_TIMEOUT) {
			ibus_lmt_change_timer = 0;
			ibus_limit = ibus_limit - 100;
			effective_fcc_taper = (fcc_vote_val - BQ_TAPER_DECREASE_STEP_MA) * 1000;
			if (pdpm->fcc_votable && effective_fcc_taper >= 2000000) {
				vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER, true, effective_fcc_taper);
				pr_info("bq set taper fcc to : %d uA\n", effective_fcc_taper);
			}
		}
	} else {
		ibus_lmt_change_timer = 0;
	}

	/* battery voltage loop*/
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt)
		step_vbat = -pm_config.fc2_steps;
	else if (pdpm->cp.vbat_volt < pm_config.bat_volt_lp_lmt - 10)
		step_vbat = pm_config.fc2_steps;

	/* battery charge current loop*/
	if (pdpm->cp.ibat_curr < fcc_limit)
		step_ibat = pm_config.fc2_steps;
	else if (pdpm->cp.ibat_curr > fcc_limit + 100)
		step_ibat = -pm_config.fc2_steps;

	/* bus current loop*/
	ibus_total = pdpm->cp.ibus_curr;
	if (pm_config.cp_sec_enable)
		ibus_total += pdpm->cp_sec.ibus_curr;

	if (ibus_total < ibus_limit - 80)
		step_ibus = pm_config.fc2_steps;
	else if (ibus_total > ibus_limit - 80)
		step_ibus = -pm_config.fc2_steps;

	pr_info("vbat:%d,lmt:%d; ibat:%d,lmt:%d; ibus:%d,lmt:%d",
			pdpm->cp.vbat_volt, pm_config.bat_volt_lp_lmt,
			pdpm->cp.ibat_curr, fcc_limit, ibus_total, ibus_limit);

	/* hardware regulation loop*/
	step_bat_reg = pm_config.fc2_steps;

	sw_ctrl_steps = min(min(step_vbat, step_ibus), step_ibat);
	sw_ctrl_steps = min(sw_ctrl_steps, step_bat_reg);

	/* hardware alarm loop */
	if (pdpm->cp.bat_ocp_alarm
		|| pdpm->cp.bus_ocp_alarm || pdpm->cp.bus_ovp_alarm)
		hw_ctrl_steps = -pm_config.fc2_steps;
	else
		hw_ctrl_steps = pm_config.fc2_steps;

	/* check if cp disabled due to other reason*/
	usbpd_pm_check_cp_enabled(pdpm);
	if (!pdpm->cp.charge_enabled) {
		pr_err("cp disabled due to other reason, return\n");
		return PM_ALGO_RET_CHG_DISABLED;
	}

	pdpm->is_temp_out_fc2_range = pd_disable_cp_by_jeita_status(pdpm);

	if (pdpm->cp.bat_therm_fault) { /* battery overheat, stop charge*/
		pr_err("bat_therm_fault:%d\n", pdpm->cp.bat_therm_fault);
		return PM_ALGO_RET_THERM_FAULT;
	} else if (thermal_level >= MAX_THERMAL_LEVEL
			|| pdpm->is_temp_out_fc2_range) {
		pr_info("system thermal level too high or batt temp is out of fc2 range\n");
		return PM_ALGO_RET_CHG_DISABLED;
	} else if (pdpm->cp.bat_ocp_fault || pdpm->cp.bus_ocp_fault
			|| pdpm->cp.bat_ovp_fault || pdpm->cp.bus_ovp_fault) {
		pr_err("bat_ocp_fault:%d, bus_ocp_fault:%d, bat_ovp_fault:%d,bus_ovp_fault:%d\n",
				pdpm->cp.bat_ocp_fault, pdpm->cp.bus_ocp_fault,
				pdpm->cp.bat_ovp_fault, pdpm->cp.bus_ovp_fault);
		return PM_ALGO_RET_OTHER_FAULT; /* go to switch, and try to ramp up*/
	} else if (!pdpm->cp.charge_enabled) {
		pr_err("cp.charge_enabled:%d\n", pdpm->cp.charge_enabled);
		return PM_ALGO_RET_CHG_DISABLED;
	}

	/*check overcharge when it is cool*/
	if ((pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt && is_cool_charge(pdpm))
		|| (pdpm->cp.vbat_volt > BATT_WARM_CHG_VOLT && is_warm_charge(pdpm))) {
		if (cool_warm_overcharge_timer++ > TAPER_TIMEOUT) {
			pr_info("cool warm overcharge\n");
			cool_warm_overcharge_timer = 0;
			pdpm->bq_cool_warm_done = true;
			return PM_ALGO_RET_CHG_DISABLED;
		}
	} else {
		cool_warm_overcharge_timer = 0;
	}

	/* charge pump taper charge */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - TAPER_VOL_HYS
			&& pdpm->cp.ibat_curr < pm_config.fc2_taper_current) {
		if (fc2_taper_timer++ > TAPER_TIMEOUT) {
			pr_err("charge pump taper charging done\n");
			fc2_taper_timer = 0;
			return PM_ALGO_RET_TAPER_DONE;
		}
	} else {
		fc2_taper_timer = 0;
	}

	/*TODO: customer can add hook here to check system level
	 * thermal mitigation*/

	steps = min(sw_ctrl_steps, hw_ctrl_steps);

	/* raise the adapter voltage quickly right after entering cp charging */
	if (pdpm->cp.ibat_curr > 0 && ibus_total > 0 &&
			time_delta < QUICK_RAISE_VOLT_INTERVAL_MS) {
		fcc_ibatt_diff = (pdpm->cp.ibat_curr > fcc_limit) ?
			(pdpm->cp.ibat_curr - fcc_limit) : (fcc_limit - pdpm->cp.ibat_curr);
		sicl_ibus_diff = (ibus_total > ibus_limit) ?
			(ibus_total - ibus_limit) : (ibus_limit - ibus_total);
		pr_err("fcc_ibatt_diff:%d sicl_ibus_diff:%d\n", fcc_ibatt_diff, sicl_ibus_diff);
		if (fcc_ibatt_diff > 1200 && sicl_ibus_diff > 500) {
			pdpm->pm_work_time = fast_work_time;
			steps = steps * fast_tune_step;
		} else if (fcc_ibatt_diff > 500 && fcc_ibatt_diff <= 1200 &&
				sicl_ibus_diff <= 500 && sicl_ibus_diff > 250) {
			pdpm->pm_work_time = stable_work_time;
			steps = steps * stable_tune_step;
		}
	}

	pr_info("step_vbat:%d step_ibat:%d step_ibus:%d sw_ctrl_steps:%d hw_ctrl_steps:%d steps:%d\n",
			step_vbat, step_ibat, step_ibus, sw_ctrl_steps,
			hw_ctrl_steps, steps);

	pdpm->request_voltage += steps * STEP_MV;

	/* 11V3A PD charger cause the battery current over 6A, reduce the request_voltage */
	if (pdpm->request_voltage > (pdpm->apdo_max_volt == PPS_VOL_MAX ?
			PPS_VOL_MAX - PPS_VOL_HYS : pdpm->apdo_max_volt))
		pdpm->request_voltage = (pdpm->apdo_max_volt == PPS_VOL_MAX ?
			PPS_VOL_MAX - PPS_VOL_HYS : pdpm->apdo_max_volt);

	return PM_ALGO_RET_OK;
}

static const unsigned char *pm_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_FC2_ENTRY",
	"PD_PM_STATE_FC2_ENTRY_1",
	"PD_PM_STATE_FC2_ENTRY_2",
	"PD_PM_STATE_FC2_ENTRY_3",
	"PD_PM_STATE_FC2_TUNE",
	"PD_PM_STATE_HOLD",
	"PD_PM_STATE_FC2_EXIT",
};

static void usbpd_pm_move_state(struct usbpd_pm *pdpm, enum pm_state state)
{
	pr_info("state change:%s -> %s\n",
		pm_str[pdpm->state], pm_str[state]);
	pdpm->state = state;
}

static int usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	int ret;
	static int tune_vbus_retry;
	static bool stop_sw;
	static bool recover;
	static int g_status;
	int effective_fcc_val = 0;
	int thermal_level = 0;
	int curr_fcc_lmt, curr_ibus_lmt;
	static int retry_count;

	if (update_smart_chg_cp_workmode(pdpm)) {
		usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		pr_info("N7:need switch cp mode\n");
	}

	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		stop_sw = false;
		recover = false;
		pdpm->pm_work_time = fast_work_time;

		if (!pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm, true);

		pd_get_batt_current_thermal_level(pdpm, &thermal_level);
		pdpm->is_temp_out_fc2_range = pd_disable_cp_by_jeita_status(pdpm);
		pr_info("is_temp_out_fc2_range:%d\n", pdpm->is_temp_out_fc2_range);

		effective_fcc_val = usbpd_get_effective_fcc_val(pdpm);

		if (effective_fcc_val > 0) {
			curr_fcc_lmt = min(pm_config.bat_curr_lp_lmt, effective_fcc_val);
			curr_ibus_lmt = curr_fcc_lmt >> 1;
			pr_info("curr_ibus_lmt:%d\n", curr_ibus_lmt);
		}

		if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp) {
			usbpd_select_pdo(pdpm->pd, PD_FIXED_5V_PDO, 0, 0);
			pr_info("batt_volt %d, waiting...\n", pdpm->cp.vbat_volt);
		} else if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 100) {
			pr_info("batt_volt %d is too high for cp,\t\t\t\t\tcharging with switch charger\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			if (pm_config.bat_volt_lp_lmt < BAT_VOLT_LOOP_LMT)
				recover = true;
		} else if (thermal_level >= MAX_THERMAL_LEVEL
				|| pdpm->is_temp_out_fc2_range) {
			usbpd_select_pdo(pdpm->pd, PD_FIXED_5V_PDO, 0, 0);
			pr_info("thermal too high or batt temp is out of fc2 range, waiting...\n");
		} else {
			pr_info("batt_volt-%d is ok, start flash charging\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY);
		}
		break;

	case PD_PM_STATE_FC2_ENTRY:
		if (pm_config.fc2_disable_sw) {
			if (!pdpm->sw.charge_limited) {
				usbpd_pm_limit_sw(pdpm, true);
				usbpd_pm_update_sw_status(pdpm);
			}
		}
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		retry_count = 0;
		break;

	case PD_PM_STATE_FC2_ENTRY_1:
		pr_info("N7:dive rate = %d in state FC2_ENTRY_1\n", pdpm->dive_rate);
		if (retry_count >= 1)
			pdpm->request_voltage += STEP_MV;
		else
			pdpm->request_voltage = pdpm->cp.vbat_volt * pdpm->dive_rate + BUS_VOLT_INIT_UP;
		pdpm->request_current = pdpm->apdo_max_curr;

		usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
				pdpm->request_voltage * 1000, pdpm->request_current * 1000);
		pr_info("entry1 request_voltage:%d, request_current:%d\n",
				pdpm->request_voltage, pdpm->request_current);

		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_2);

		tune_vbus_retry = 0;
		break;

	case PD_PM_STATE_FC2_ENTRY_2:
		pr_info("tune adapter vbus_volt %d , vbat_volt %d tune_vbus_retry=%d\n",
					pdpm->cp.vbus_volt, pdpm->cp.vbat_volt, tune_vbus_retry);
		pr_info("N7:dive rate = %d in state FC2_ENTRY_2\n", pdpm->dive_rate);
		if (pdpm->cp.vbus_volt < (pdpm->cp.vbat_volt * pdpm->dive_rate + BUS_VOLT_INIT_UP - 50)) {
			tune_vbus_retry++;
			pdpm->request_voltage += STEP_MV;
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_info("entry2 add request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		} else if (pdpm->cp.vbus_volt > (pdpm->cp.vbat_volt * pdpm->dive_rate + BUS_VOLT_INIT_UP + 200)) {
			tune_vbus_retry++;
			pdpm->request_voltage -= STEP_MV;
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_info("entry2 reduce request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		} else {
			pr_info("adapter volt tune ok, retry %d times\n", tune_vbus_retry);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_3);
			break;
		}

		if (tune_vbus_retry > 33) {
			if (retry_count < 1) {
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
				retry_count++;
				pr_info("Failed to tune adapter volt into valid range, retry again\n");
			} else {
				pr_info("Failed to tune adapter volt into valid range, charge with switching charger\n");
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			}
		}
		break;

	case PD_PM_STATE_FC2_ENTRY_3:
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
			msleep(30);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pdpm->cp.charge_enabled) {
			if (pdpm->sw.charge_enabled) {
				usbpd_pm_enable_sw(pdpm, false);
				usbpd_pm_update_sw_status(pdpm);
			}
			if ((pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled)
					|| !pm_config.cp_sec_enable) {
				pdpm->entry_bq_cv_time = ktime_get();
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
				ibus_lmt_change_timer = 0;
				fc2_taper_timer = 0;
			}
		}
		break;

	case PD_PM_STATE_FC2_TUNE:
		if (IS_ERR(pdpm->pd) ||
				pdpm->src_cap_id != usbpd_get_src_cap_id(pdpm->pd)) {
			if (!IS_ERR(pdpm->pd))
				pdpm->src_cap_id = usbpd_get_src_cap_id(pdpm->pd);
			usbpd_pm_evaluate_src_caps(pdpm);
		}

		ret = usbpd_pm_fc2_charge_algo(pdpm);
		g_status = ret;
		if (ret == PM_ALGO_RET_THERM_FAULT) {
			pr_info("Move to stop charging:%d\n", ret);
			stop_sw = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_OTHER_FAULT || ret == PM_ALGO_RET_TAPER_DONE) {
			pr_info("Move to switch charging:%d\n", ret);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_CHG_DISABLED) {
			pr_info("Move to switch charging, will try to recover flash charging:%d\n",
					ret);
			recover = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_SMART_RELEASE || ret == PM_ALGO_RET_SMART_HOLD) {
			pr_info("because smart ctrl, cp enter hold state = %d\n", ret);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_HOLD);
			break;
		} else {
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_info("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
		}
		break;

	case PD_PM_STATE_HOLD:
		pr_info("N7:enter HOLD state\n");
		if (g_status == PM_ALGO_RET_SMART_RELEASE) {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
			pr_info("N7:switch PD_PM_STATE_ENTRY state\n");
		} else if (g_status == PM_ALGO_RET_SMART_HOLD) {
			usbpd_select_pdo(pdpm->pd, PD_FIXED_5V_PDO, 0, 0);
			msleep(100);
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
			pr_info("N7:keep HOLD state\n");
		}
		break;

	case PD_PM_STATE_FC2_EXIT:
		usbpd_select_pdo(pdpm->pd, PD_FIXED_5V_PDO, 0, 0);

		if (pdpm->fcc_votable)
			vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER,
					false, 0);

		usbpd_pm_limit_sw(pdpm, false);

		if (!stop_sw && (!pdpm->sw.charge_enabled || pdpm->sw.charge_limited))
			usbpd_pm_enable_sw(pdpm, true);

		if (stop_sw && (pdpm->sw.charge_enabled || pdpm->sw.charge_limited))
			usbpd_pm_enable_sw(pdpm, false);

		usbpd_pm_update_sw_status(pdpm);

		if (pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (recover)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		break;

	default:
		usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		break;
	}

	return false; //return rc
}

static void usbpd_pm_workfunc(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					pm_work.work);

	usbpd_pm_update_sw_status(pdpm);
	usbpd_pm_update_cp_status(pdpm);
	usbpd_update_ibat_curr(pdpm);

	pdm_multi_mode_switch(pdpm);

	usbpd_pm_sm(pdpm);

	if (pdpm->pd_active)
		schedule_delayed_work(&pdpm->pm_work,
				msecs_to_jiffies(pdpm->pm_work_time));
}

static void usbpd_pm_disconnect(struct usbpd_pm *pdpm)
{
	cancel_delayed_work_sync(&pdpm->pm_work);

	if (pdpm->fcc_votable)
		vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER,
				false, 0);

	if (!pdpm->sw.charge_enabled) {
		usbpd_pm_enable_sw(pdpm, true);
		usbpd_pm_check_sw_enabled(pdpm);
	}
	pdpm->pps_supported = false;
	pdpm->jeita_triggered = false;
	pdpm->is_temp_out_fc2_range = false;
	pdpm->bq_cool_warm_done = false;
	pdpm->smart_chg_hold = false;
	pdpm->apdo_selected_pdo = 0;
	memset(&pdpm->pdo, 0, sizeof(pdpm->pdo));
	pm_config.bat_curr_lp_lmt = pdpm->bat_curr_max;
	if (!pdpm->sw.charge_enabled || pdpm->sw.charge_limited)
		usbpd_pm_enable_sw(pdpm, true);
	usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
}

static void usbpd_pd_contact(struct usbpd_pm *pdpm, bool connected)
{
	pdpm->pd_active = connected;

	pr_info("pd_active = %d\n", connected);

	if (connected) {
		usbpd_pm_evaluate_src_caps(pdpm);
		if (pdpm->pps_supported)
			schedule_delayed_work(&pdpm->pm_work, 0);
	} else {
		usbpd_pm_disconnect(pdpm);
	}
}

static void usb_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					usb_psy_change_work);
	int pd_active = 0;
	int ret = 0;

	ret = usbpd_get_iio_channel(pdpm, SMB, SMB_PD_ACTIVE, &pd_active);
	if (ret) {
		pr_err("Failed to get usb pd active state\n");
		goto out;
	}

	if (!pdpm->pd_active && (pd_active == QTI_POWER_SUPPLY_PD_PPS_ACTIVE)) {
		usbpd_pd_contact(pdpm, true);
		pdm_multi_mode_init(pdpm, true);
		pr_info("N7:start cp statemachine\n");
	} else if (pdpm->pd_active && (pd_active != QTI_POWER_SUPPLY_PD_PPS_ACTIVE)) {
		usbpd_pd_contact(pdpm, false);
		pdm_multi_mode_init(pdpm, false);
		pr_info("N7:stop cp statemachine\n");
	}
out:
	pdpm->psy_change_running = false;
}

/* battery "voltage_max" follows the cycle count based float voltage */
static void cycle_count_reduce_voltage(struct usbpd_pm *pdpm,
		struct power_supply *psy)
{
	union power_supply_propval val = {0,};
	int ret;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret < 0) {
		pr_err("voltage_max is not ready fail ret = %d\n", ret);
		pm_config.bat_volt_lp_lmt = BAT_VOLT_LOOP_LMT;
	} else {
		pm_config.bat_volt_lp_lmt = val.intval / 1000;
	}
}

static int usbpd_psy_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, nb);
	struct power_supply *psy = data;
	unsigned long flags;
	static bool first_flag;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!first_flag) {
		if (is_cp_chan_valid(pdpm, CHARGE_PUMP_PRESENT)) {
			pdpm->is_bq25960 = false;
			first_flag = true;
			pr_err("[cp_qc_psy_notifier_cb] SC8541\n");
		} else if (is_cp_chan_valid(pdpm, CHARGE_PUMP_BQ_PRESENT)) {
			pdpm->is_bq25960 = true;
			first_flag = true;
			pr_err("[cp_qc_psy_notifier_cb] BQ25960\n");
		} else {
			first_flag = false;
			pr_err("qc3.0 can't find charge pump\n");
		}
	}

	usbpd_check_usb_psy(pdpm);
	usbpd_check_batt_psy(pdpm);

	if (pdpm->sw_psy && psy == pdpm->sw_psy)
		cycle_count_reduce_voltage(pdpm, psy);

	if (psy == pdpm->usb_psy) {
		spin_lock_irqsave(&pdpm->psy_change_lock, flags);
		if (!pdpm->psy_change_running) {
			pdpm->psy_change_running = true;
			schedule_work(&pdpm->usb_psy_change_work);
		}
		spin_unlock_irqrestore(&pdpm->psy_change_lock, flags);
	}

	return NOTIFY_OK;
}

static int pd_policy_parse_dt(struct usbpd_pm *pdpm)
{
	struct device_node *node = pdpm->dev->of_node;
	int rc = 0;

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node,
			"mi,pd-bat-volt-max", &pdpm->bat_volt_max);
	if (rc < 0)
		pr_err("pd-bat-volt-max property missing, use default val\n");
	else
		pm_config.bat_volt_lp_lmt = pdpm->bat_volt_max;
	pr_info("pm_config.bat_volt_lp_lmt:%d\n", pm_config.bat_volt_lp_lmt);

	rc = of_property_read_u32(node,
			"mi,pd-bat-curr-max", &pdpm->bat_curr_max);
	if (rc < 0)
		pr_err("pd-bat-curr-max property missing, use default val\n");
	else
		pm_config.bat_curr_lp_lmt = pdpm->bat_curr_max;
	pr_info("pm_config.bat_curr_lp_lmt:%d\n", pm_config.bat_curr_lp_lmt);

	rc = of_property_read_u32(node,
			"mi,pd-bus-volt-max", &pdpm->bus_volt_max);
	if (rc < 0)
		pr_err("pd-bus-volt-max property missing, use default val\n");
	else
		pm_config.bus_volt_lp_lmt = pdpm->bus_volt_max;
	pr_info("pm_config.bus_volt_lp_lmt:%d\n", pm_config.bus_volt_lp_lmt);

	rc = of_property_read_u32(node,
			"mi,pd-bus-curr-max", &pdpm->bus_curr_max);
	if (rc < 0)
		pr_err("pd-bus-curr-max property missing, use default val\n");
	else
		pm_config.bus_curr_lp_lmt = pdpm->bus_curr_max;
	pr_info("pm_config.bus_curr_lp_lmt:%d\n", pm_config.bus_curr_lp_lmt);

	rc = of_property_read_u32(node,
			"mi,pd-bus-curr-compensate", &pdpm->bus_curr_compensate);
	if (rc < 0)
		pr_err("pd-bus-curr-compensate property missing, use default val\n");
	else
		pm_config.bus_curr_compensate = pdpm->bus_curr_compensate;
	pr_info("pm_config.bus_curr_compensate:%d\n", pm_config.bus_curr_compensate);

	pdpm->cp_sec_enable = of_property_read_bool(node,
				"mi,cp-sec-enable");
	pm_config.cp_sec_enable = pdpm->cp_sec_enable;

	rc = of_property_read_u32(node,
			"mi,pd-ffc-bat-volt-max", &pdpm->ffc_bat_volt_max);
	pr_info("pdpm->ffc_bat_volt_max:%d\n",
				pdpm->ffc_bat_volt_max);

	return rc;
}

static int usbpd_iio_init(struct usbpd_pm *pdpm)
{
	pr_err("usbpd_iio_init start\n");

	pdpm->cp_iio = devm_kcalloc(pdpm->dev, ARRAY_SIZE(cp_iio_chan_name),
				sizeof(*pdpm->cp_iio), GFP_KERNEL);
	if (!pdpm->cp_iio)
		return -ENOMEM;

	pdpm->smb_iio = devm_kcalloc(pdpm->dev, ARRAY_SIZE(smb_iio_chan_name),
				sizeof(*pdpm->smb_iio), GFP_KERNEL);
	if (!pdpm->smb_iio)
		return -ENOMEM;

	pr_err("usbpd_iio_init end\n");
	return 0;
}

static int usbpd_pm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct usbpd_pm *pdpm;
	struct power_supply *usb_psy;

	pr_err("%s enter\n", __func__);

	usb_psy = power_supply_get_by_name("usb");
	if (IS_ERR_OR_NULL(usb_psy))
		return -EPROBE_DEFER;

	pdpm = kzalloc(sizeof(struct usbpd_pm), GFP_KERNEL);
	if (!pdpm)
		return -ENOMEM;

	pdpm->dev = dev;

	ret = pd_policy_parse_dt(pdpm);
	if (ret < 0) {
		pr_err("Couldn't parse device tree rc=%d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pdpm);

	spin_lock_init(&pdpm->psy_change_lock);

	usbpd_check_usb_psy(pdpm);

	/* as in stock, a failed allocation is not fatal here */
	usbpd_iio_init(pdpm);

	INIT_WORK(&pdpm->usb_psy_change_work, usb_psy_change_work);
	INIT_DELAYED_WORK(&pdpm->pm_work, usbpd_pm_workfunc);

	pdpm->nb.notifier_call = usbpd_psy_notifier_cb;
	power_supply_reg_notifier(&pdpm->nb);

	/* pick up a PD/PPS adapter that is already attached */
	usbpd_psy_notifier_cb(&pdpm->nb, PSY_EVENT_PROP_CHANGED, pdpm->usb_psy);

	return ret;
}

static int usbpd_pm_remove(struct platform_device *pdev)
{
	struct usbpd_pm *pdpm = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&pdpm->nb);
	cancel_delayed_work(&pdpm->pm_work);
	cancel_work_sync(&pdpm->usb_psy_change_work);

	return 0;
}

static const struct of_device_id usbpd_pm_of_match[] = {
	{ .compatible = "xiaomi,usbpd-pm", },
	{},
};

static struct platform_driver usbpd_pm_driver = {
	.driver = {
		.name = "usbpd-pm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(usbpd_pm_of_match),
	},
	.probe = usbpd_pm_probe,
	.remove = usbpd_pm_remove,
};

static int __init usbpd_pm_init(void)
{
	return platform_driver_register(&usbpd_pm_driver);
}

module_init(usbpd_pm_init);

static void __exit usbpd_pm_exit(void)
{
	return platform_driver_unregister(&usbpd_pm_driver);
}
module_exit(usbpd_pm_exit);

MODULE_DESCRIPTION("USB Power Delivery Policy Driver");
MODULE_LICENSE("GPL v2");
/* stock module is built in-tree (no out-of-tree taint) */
MODULE_INFO(intree, "Y");
