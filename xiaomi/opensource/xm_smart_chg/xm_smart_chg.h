/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 Xiaomi Inc.
 * Author Tianye<tianye9@xiaomi.com>
 *
 * Qualcomm (qpnp-smb5 / QG / panel_event_notifier) flavour for sapphire.
 */
#ifndef __XM_SMART_CHG_H__
#define __XM_SMART_CHG_H__

#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/kstrtox.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/notifier.h>
#include <linux/time64.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

#define XM_CHARGE_WORK_MS		5000
#define MAX_THERMAL_LEVELS		15

#define CYCLE_LOW_TEMP			150
#define CYCLE_HIGH_TEMP			450

/* voters */
#define SMART_CHARGING_VOTER		"SMART_CHARGING_VOTER"
#define NIGHT_CHARGING_VOTER		"NIGHT_CHARGING_VOTER"
#define LOW_FAST_VOTER			"LOW_FAST_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"

/* qpnp-smb5 notifier events */
enum qpnp_smb5_notifier_event {
	QPNP_SMB5_CHARGER_PLUGIN_EVENT = 1,
	QPNP_SMB5_THERMAL_LEVEL_EVENT,
	QPNP_SMB5_SMART_CHG_EVENT,
	QPNP_SMB5_SMART_BATT_EVENT,
	QPNP_SMB5_NIGHT_CHARGING_EVENT,
};

/* mi_thermal_interface notifier events */
#define MI_THERMAL_BOARD_TEMP_EVENT	0

/* xm_smart_chg IIO channel (consumers refer to it by this number in DT) */
#define XM_SMART_CHG_IIO_INT_LABEL	0xf1

enum smart_chg_functype{
	SMART_CHG_STATUS_FLAG = 0,
	SMART_CHG_FEATURE_MIN_NUM = 1,
	SMART_CHG_NAVIGATION = 1,
	SMART_CHG_OUTDOOR_CHARGE = 2,
	SMART_CHG_LOW_FAST = 3,
	SMART_CHG_ENDURANCE_PRO = 4,
	/* add new func here */
	SMART_CHG_FEATURE_MAX_NUM = 15,
};

struct smart_chg {
	bool en_ret;
	int active_status;
	int func_val;
};

struct charger_screen_monitor {
	struct notifier_block charger_panel_notifier;
	int screen_state;
};

enum blank_flag{
	SNORMAL = 0,
	BLACK_TO_BRIGHT = 1,
	BRIGHT = 2,
	BLACK = 3,
};

struct xm_smart_chg_info {
	struct device *dev;
	int smart_batt;
	int fv_ageing;
	bool night_charging;
	bool night_charging_flag;
	const char *label;
	int *pps_thermal_mitigation;
	int *pps_thermal_mitigation_fast;
	int thermal_levels;
	int screen_state;	/* 0: bright, 1: black */
	int thermal_level;
	struct charger_screen_monitor sm;

	struct votable *fv_votable;
	struct votable *fcc_votable;
	struct votable *chg_disable_votable;
	struct votable *low_fast_votable;
	struct votable *night_charging_votable;

	struct smart_chg smart_charge[SMART_CHG_FEATURE_MAX_NUM + 1];
	int smart_chg_cmd;
	int soc;
	int raw_soc;
	struct power_supply *batt_psy;
	bool smart_ctrl_en;
	bool ffc_en;
	int thermal_board_temp;
	int pd_active;
	bool low_fast_plugin_flag;
	bool pps_fast_mode;
	enum blank_flag b_flag;
	struct notifier_block charger_thermal_nb;
	/*charger_plugin_event 0:none 1:plugin 2:plugout*/
	int charger_plugin_event;
	struct notifier_block smart_chg_nb;
	struct delayed_work xm_charge_work;
	struct delayed_work xm_smart_chg_post_init_work;

	/* QG IIO channels */
	struct iio_channel *capacity_chan;
	struct iio_channel *pd_active_chan;
	struct iio_channel *capacity_raw_chan;

	/* xm_smart_chg IIO device */
	struct iio_dev *indio_dev;
	struct iio_chan_spec *iio_chan;
	struct iio_channel *int_iio_chans;
	int int_label;
};

void set_error(struct xm_smart_chg_info *info);
void set_success(struct xm_smart_chg_info *info);
int smart_chg_is_error(struct xm_smart_chg_info *info);
void handle_smart_chg_functype(struct xm_smart_chg_info *info,
	const int func_type, const int en_ret, const int func_val);
int handle_smart_chg_functype_status(struct xm_smart_chg_info *info);
void handle_smart_chg_value(int val);
void monitor_smart_chg(void);
void monitor_night_charging(void);
void monitor_low_fast_strategy(void);
void get_fv_ageing(int cyclecount);
void get_drop_floatvolatge(void);
void xm_charge_update_date(void);
void xm_charge_work(struct work_struct *work);
int xm_smart_charge_add_to_batt_psy(void);
int xm_smart_chg_register_panel_notifier(void);
int xm_smart_chg_parse_dt(struct platform_device *pdev);
int xm_smart_chg_post_init(void);
int xm_smart_chg_info_init(struct platform_device *pdev);

/* qpnp-smb5-main exports (stock CRCs) */
struct smb_charger;
extern struct smb_charger *g_chg;
int qpnp_smb5_reg_notifier(struct notifier_block *nb);
int qpnp_smb5_unreg_notifier(struct notifier_block *nb);
void set_fv_ageing(int fv_ageing);
void set_blank_status(int status);

/* mi_thermal_interface exports */
int mi_thermal_reg_notifier(struct notifier_block *nb);
int mi_thermal_unreg_notifier(struct notifier_block *nb);

#endif
