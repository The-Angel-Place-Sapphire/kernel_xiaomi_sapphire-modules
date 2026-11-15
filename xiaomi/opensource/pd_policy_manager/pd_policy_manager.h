/*
 * usb_pd_policy_manager.h
 *
 *  Created on: Mar 27, 2017
 *      Author: a0220433
 *
 * Xiaomi sapphire (khaje, 5.15 GKI) flavour: IIO charge pump (SC8541 or
 * BQ25960) and qpnp-smb5 channels, N7 2:1/1:1 charge pump mode switching
 * and smart/night charging hold.
 */

#ifndef SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#define SRC_PDLIB_USB_PD_POLICY_MANAGER_H_
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/ktime.h>
#include <linux/usb/usbpd.h>

enum pm_state {
    PD_PM_STATE_ENTRY,
    PD_PM_STATE_FC2_ENTRY,
    PD_PM_STATE_FC2_ENTRY_1,
    PD_PM_STATE_FC2_ENTRY_2,
    PD_PM_STATE_FC2_ENTRY_3,
    PD_PM_STATE_FC2_TUNE,
    PD_PM_STATE_HOLD,
    PD_PM_STATE_FC2_EXIT,
};

#define	BAT_OVP_FAULT_SHIFT			0
#define	BAT_OCP_FAULT_SHIFT			1
#define	BUS_OVP_FAULT_SHIFT			2
#define	BUS_OCP_FAULT_SHIFT			3
#define	BAT_THERM_FAULT_SHIFT			4
#define	BUS_THERM_FAULT_SHIFT			5
#define	DIE_THERM_FAULT_SHIFT			6

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)

/* voters for usbpd */
#define STEP_BMS_CHG_VOTER	"STEP_BMS_CHG_VOTER"
#define BQ_TAPER_FCC_VOTER	"BQ_TAPER_FCC_VOTER"
#define SMART_CHARGING_VOTER	"SMART_CHARGING_VOTER"
#define NIGHT_CHARGING_VOTER	"NIGHT_CHARGING_VOTER"

/* defined for non_verified pps charger maxium fcc */
#define NON_VERIFIED_PPS_FCC_MAX		3000

#define MAX_THERMAL_LEVEL			10
/* jeita related */
#define JEITA_WARM_THR			480
#define JEITA_COOL_NOT_ALLOW_CP_THR			100
#define COOL_HYS_THRESHOLDS			180
#define WARM_HYS_THRESHOLDS			450

#define PDO_MAX_NUM			7
/*
 * add hysteresis for warm threshold to avoid flash
 * charge and normal charge switch frequently at
 * the warm threshold
 */
#define JEITA_HYSTERESIS			20

/* product related */
#define LOW_POWER_PPS_CURR_THR			2000
#define XIAOMI_LOW_POWER_PPS_CURR_MAX			1500
#define PPS_VOL_MAX			11000
#define PPS_VOL_HYS			1000

#define STEP_MV			20
#define TAPER_VOL_HYS			80
#define TAPER_WITH_IBUS_HYS			60
#define TAPER_IBUS_THR			450
#define BQ_TAPER_HYS_MV			10
#define BQ_TAPER_DECREASE_STEP_MA			200

/* quick raise of the adapter voltage after entering cp charging */
#define QUICK_RAISE_VOLT_INTERVAL_MS		10000

/* N7: ibat request window in which the charge pump runs in 1:1 mode */
#define CP_1_1_MODE_IBAT_MIN			2001
#define CP_1_1_MODE_IBAT_MAX			2400

enum cp_workmode {
	CP_WORKMODE_2_1,
	CP_WORKMODE_1_1,
};

enum cp_iio_type {
	CP_MASTER,
	SMB,
};

#define CP_BQ25960_IIO_CHANNEL_OFFSET	16

enum cp_iio_channels {
	CHARGE_PUMP_PRESENT,
	CHARGE_PUMP_CHARGING_ENABLED,
	CHARGE_PUMP_STATUS,
	CHARGE_PUMP_SC_BATTERY_PRESENT,
	CHARGE_PUMP_SC_VBUS_PRESENT,
	CHARGE_PUMP_SC_BATTERY_VOLTAGE,
	CHARGE_PUMP_SC_BATTERY_CURRENT,
	CHARGE_PUMP_SC_BATTERY_TEMPERATURE,
	CHARGE_PUMP_SC_BUS_VOLTAGE,
	CHARGE_PUMP_SC_BUS_CURRENT,
	CHARGE_PUMP_SC_BUS_TEMPERATURE,
	CHARGE_PUMP_SC_DIE_TEMPERATURE,
	CHARGE_PUMP_SC_ALARM_STATUS,
	CHARGE_PUMP_SC_FAULT_STATUS,
	CHARGE_PUMP_SC_VBUS_ERROR_STATUS,
	CHARGE_PUMP_SC_SET_CP_WORKMODE,
	CHARGE_PUMP_BQ_PRESENT = CP_BQ25960_IIO_CHANNEL_OFFSET,
	CHARGE_PUMP_BQ_CHARGING_ENABLED,
	CHARGE_PUMP_BQ_STATUS,
	CHARGE_PUMP_BQ_BATTERY_PRESENT,
	CHARGE_PUMP_BQ_VBUS_PRESENT,
	CHARGE_PUMP_BQ_BATTERY_VOLTAGE,
	CHARGE_PUMP_BQ_BATTERY_CURRENT,
	CHARGE_PUMP_BQ_BATTERY_TEMPERATURE,
	CHARGE_PUMP_BQ_BUS_VOLTAGE,
	CHARGE_PUMP_BQ_BUS_CURRENT,
	CHARGE_PUMP_BQ_BUS_TEMPERATURE,
	CHARGE_PUMP_BQ_DIE_TEMPERATURE,
	CHARGE_PUMP_BQ_ALARM_STATUS,
	CHARGE_PUMP_BQ_FAULT_STATUS,
	CHARGE_PUMP_BQ_VBUS_ERROR_STATUS,
	CHARGE_PUMP_BQ_SET_CP_WORKMODE,
};

enum smb_iio_channels {
	SMB_PD_ACTIVE,
	SMB_CP_INPUT_SUSPEND,
	SMB_BATTERY_CHARGING_ENABLED,
	SMB_APDO_VOLT,
	SMB_APDO_CURR,
};

struct sw_device {
	bool charge_enabled;
	bool charge_limited;
};

struct cp_device {
	bool charge_enabled;

	bool batt_pres;
	bool vbus_pres;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int  vout_volt;
	int  vbat_volt;
	int  vbus_volt;
	int  ibat_curr;
	int  ibus_curr;

	int  bat_temp;
	int  bus_temp;
	int  die_temp;
};

#define PM_STATE_LOG_MAX    32

struct usbpd_pm {
	struct device *dev;

	enum pm_state state;

	struct cp_device cp;
	struct cp_device cp_sec;

	struct sw_device sw;

	bool	cp_sec_stopped;

	bool	pd_active;
	bool	pps_supported;

	int	request_voltage;
	int	request_current;
	int	pm_work_time;	/* ms between two state machine runs */

	struct usbpd *pd;
	struct usbpd_pdo pdo[7];

	int	src_cap_id;
	int	apdo_max_volt;
	int	apdo_max_curr;
	int	apdo_selected_pdo;

	int	adapter_voltage;
	int	adapter_current;
	int	adapter_ptf;
	bool	adapter_omf;

	struct delayed_work pm_work;

	struct notifier_block nb;

	bool   psy_change_running;
	struct work_struct cp_psy_change_work;
	struct work_struct usb_psy_change_work;
	spinlock_t psy_change_lock;

	struct votable		*fcc_votable;
	struct votable		*chg_disable_votable;

	/* N7: smart/night charging hold and charge pump work mode */
	bool			smart_chg_hold;
	bool			cp_1_1_request;
	int			dive_rate;	/* vbus/vbat ratio of the cp mode */
	bool			cp_mode_2_1;
	bool			cp_mode_1_1;

	struct power_supply *sw_psy;
	struct power_supply *usb_psy;

	struct iio_channel	**cp_iio;
	struct iio_channel	**smb_iio;

	/* dtsi properties */
	int			bat_volt_max;
	int			ffc_bat_volt_max;
	int			bat_curr_max;
	int			bus_volt_max;
	int			bus_curr_max;
	int			bus_curr_compensate;
	bool		cp_sec_enable;

	/* jeita or thermal related */
	bool			jeita_triggered;
	bool			is_temp_out_fc2_range;
	bool			bq_cool_warm_done;

	bool			is_bq25960;
	ktime_t			entry_bq_cv_time;
};

struct pdpm_config {
	int	bat_volt_lp_lmt; /*bat volt loop limit*/
	int	bat_curr_lp_lmt;
	int	bus_volt_lp_lmt;
	int	bus_curr_lp_lmt;
	int	bus_curr_compensate;

	int	fc2_taper_current;
	int	fc2_steps;

	int	min_adapter_volt_required;
	int min_adapter_curr_required;

	int	min_vbat_for_cp;

	bool	cp_sec_enable;
	bool	fc2_disable_sw;		/* disable switching charger during flash charge*/

};

int usbpd_get_iio_channel(struct usbpd_pm *pdpm,
		enum cp_iio_type type, int channel, int *val);
int usbpd_set_iio_channel(struct usbpd_pm *pdpm,
		enum cp_iio_type type, int channel, int val);
int usbpd_get_cp_workmode(struct usbpd_pm *pdpm);
int usbpd_set_cp_workmode(struct usbpd_pm *pdpm, int workmode);

extern int usbpd_get_pps_status(struct usbpd *pd, u32 *status);
extern int usbpd_fetch_pdo(struct usbpd *pd, struct usbpd_pdo *pdos);
extern int usbpd_select_pdo(struct usbpd *pd, int pdo, int uv, int ua);
extern int usbpd_get_src_cap_id(struct usbpd *pd);
extern struct usbpd *smb_get_usbpd(void);
#endif /* SRC_PDLIB_USB_PD_POLICY_MANAGER_H_ */
