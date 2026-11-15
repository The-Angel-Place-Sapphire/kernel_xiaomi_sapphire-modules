// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Xiaomi Inc.
 * Author Tianye<tianye9@xiaomi.com>
 *
 * Sapphire (Redmi Note 13 4G, khaje, 5.15 GKI) version.
 * Base: Xiaomi ums9230 drivers/power/policy/xm_smart_chg (smart charging
 * functype/status handling, night charging, low-battery fast charging
 * strategy, float voltage ageing). The Qualcomm glue (qpnp-smb5 notifier,
 * QG IIO channels, the xm_smart_chg IIO device, panel_event_notifier screen
 * state, mi_thermal board temperature notifier, pmic-voter votables, miev
 * DFX events, raw_soc sysfs) is reconstructed from the stock sapphire
 * vendor_dlkm xm_smart_chg.ko.
 */
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/of_platform.h>
#include <linux/pmic-voter.h>
#include <linux/qti_power_supply.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include <drm/drm_panel.h>
#include <miev/mievent.h>
#include "xm_smart_chg.h"

/*
 * Offset of "int real_type" in the stock qpnp-smb5-main struct smb_charger
 * (g_chg). The stock struct layout is ABI here, so it is not taken from our
 * headers.
 */
#define SMB_CHARGER_REAL_TYPE_OFFSET	0x128c

#define MIEV_CODE_NONE_STANDARD_CHG	909002001
#define MIEV_CODE_BATT_LINKER_ABSENT	909005003

struct xm_smart_chg_info *info;

static struct drm_panel *prim_panel;
static void *cookie;

static const int cycle_count_conf[] = {
	100, 300, 800, 65535
};
static const int dropfv_conf[] = {
	0, 10, 20, 40
};

/*
 * Stock bug fixed in the three helpers below: stock passes en_ret for the
 * "%s" conversion (no function name argument).
 */
void set_error(struct xm_smart_chg_info *info)
{
	info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret = 1;
	pr_err("xm %s en_ret=%d\n", __func__, info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret);
}

void set_success(struct xm_smart_chg_info *info)
{
	info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret = 0;
	pr_err("xm %s en_ret=%d\n", __func__, info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret);
	pr_info("N7:set successful\n");
}

int smart_chg_is_error(struct xm_smart_chg_info *info)
{
	return info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret;
}

void handle_smart_chg_functype(struct xm_smart_chg_info *info,
	const int func_type, const int en_ret, const int func_val)
{
	switch (func_type)
	{
	case SMART_CHG_FEATURE_MIN_NUM ... SMART_CHG_FEATURE_MAX_NUM:
		info->smart_charge[func_type].en_ret = en_ret;
		info->smart_charge[func_type].active_status = false;
		info->smart_charge[func_type].func_val = func_val;
		set_success(info);
		pr_err("xm set func_type:%d, en_ret = %d\n", func_type, en_ret);
		break;
	default:
		pr_err("xm ERROR: Not supported func type: %d\n", func_type);
		set_error(info);
		break;
	}
}

int handle_smart_chg_functype_status(struct xm_smart_chg_info *info)
{
	int i;
	int all_func_status = 0;
	all_func_status |= !!info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret;	//handle bit0

	pr_err("all_func_status =%#X, en_ret=%d\n", all_func_status, info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret);

	/* save functype[i] enable status in all_func_status bit[i] */
	for (i = SMART_CHG_FEATURE_MIN_NUM; i <= SMART_CHG_FEATURE_MAX_NUM; i++){  //handle bit1 ~ bit SMART_CHG_FEATURE_MAX_NUM
		if (info->smart_charge[i].en_ret)
			all_func_status |= BIT_MASK(i);
		else
			all_func_status &= ~BIT_MASK(i);

		pr_err("type:%d, en_ret=%d, active_status=%d,func_val=%d, all_func_status=%#X\n",
			i, info->smart_charge[i].en_ret, info->smart_charge[i].active_status, info->smart_charge[i].func_val, all_func_status);
	}

	pr_err("all_func_status:%#X\n", all_func_status);
	return all_func_status;
}

/* decode a smart_chg command: bit0 enable, bit1..15 function type, bit16.. value */
static __always_inline void xm_smart_chg_handle_cmd(int val)
{
	bool en_ret;
	unsigned long func_type;
	int func_val;
	int bit_pos;
	int all_func_status;

	en_ret = val & 0x1;
	func_type = (val & 0xFFFE) >> 1;
	func_val = val >> 16;

	pr_info("N7:get val:%#X, func_type:%#X, en_ret:%d, func_val:%d\n",
		val, (unsigned int)func_type, en_ret, func_val);
	bit_pos = find_first_bit(&func_type, SMART_CHG_FEATURE_MAX_NUM);

	if (bit_pos == SMART_CHG_FEATURE_MAX_NUM || find_next_bit(&func_type, SMART_CHG_FEATURE_MAX_NUM , bit_pos + 1) != SMART_CHG_FEATURE_MAX_NUM){
		pr_err("ERROR: zero or more than one func type!\n");
		pr_err("find_next_bit = %d, bit_pos = %d\n",
		(int)find_next_bit(&func_type, SMART_CHG_FEATURE_MAX_NUM , bit_pos + 1), bit_pos);
		set_error(info);
	} else
		set_success(info);

	// if func_type bit0 is 1, bit_pos = 0, not 1. so ++bit_pos.
	if (!smart_chg_is_error(info))
		handle_smart_chg_functype(info, ++bit_pos, en_ret, func_val);

	/* update smart_chg[0] status */
	all_func_status = handle_smart_chg_functype_status(info);
	info->smart_chg_cmd = all_func_status;
	info->smart_charge[SMART_CHG_STATUS_FLAG].en_ret = all_func_status & 0x1;
	info->smart_charge[SMART_CHG_STATUS_FLAG].active_status = (all_func_status & 0xFFFE) >> 1;
}

void handle_smart_chg_value(int val)
{
	xm_smart_chg_handle_cmd(val);
}

void monitor_smart_chg(void)
{
	static int g_smart_soc = 50;
	struct xm_smart_chg_info *chip = info;
	union power_supply_propval pval = {0,};
	struct power_supply *batt_psy;
	struct votable *chg_disable_votable;
	struct smart_chg *nav = &chip->smart_charge[SMART_CHG_NAVIGATION];
	struct smart_chg *low_fast = &chip->smart_charge[SMART_CHG_LOW_FAST];
	int ret;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return;

	pr_info("N7 call monitor_smart_chg\n");
	chg_disable_votable = find_votable("CHG_DISABLE");
	if (!chg_disable_votable)
		return;

	chip->batt_psy = batt_psy;
	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (ret < 0)
		pr_info("get battery soc error.\n");
	else
		chip->soc = pval.intval;

	pr_info("N7 enter monitor_smart_chg, en_ret = %d, fun_val = %d, active_status = %d, smart_soc = %d, ui_soc =%d, smart_ctrl_en = %d\n",
		nav->en_ret, nav->func_val, nav->active_status,
		g_smart_soc, chip->soc, chip->smart_ctrl_en);

	if (nav->en_ret && chip->soc >= nav->func_val && !nav->active_status) {
		nav->active_status = true;
		g_smart_soc = nav->func_val;
		vote(chg_disable_votable, SMART_CHARGING_VOTER, true, 0);
		chip->smart_ctrl_en = true;
		pr_info("N7 monitor disable charger,soc >= %d\n", nav->func_val);
	} else if ((((!nav->en_ret || chip->soc <= nav->func_val - 5) && nav->active_status) ||
			(!nav->en_ret && !nav->active_status && chip->soc >= nav->func_val)) &&
			chip->smart_ctrl_en) {
		nav->active_status = false;
		vote(chg_disable_votable, SMART_CHARGING_VOTER, false, 0);
		chip->smart_ctrl_en = false;
		pr_info("N7 monitor enable charger, soc <= %d\n", nav->func_val - 5);
	}

	if (low_fast->en_ret && !low_fast->active_status) {
		low_fast->active_status = true;
		pr_info("N7 set smart_charge[SMART_CHG_LOW_FAST].en_ret = %d, enable low_fast\n", low_fast->en_ret);
	} else if (!low_fast->en_ret && low_fast->active_status) {
		low_fast->active_status = false;
		pr_info("N7 set smart_charge[SMART_CHG_LOW_FAST].en_ret = %d, disable low_fast\n", low_fast->en_ret);
	}
}

static ssize_t smart_chg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	if (kstrtoint(buf, 16, &val))
		return -EINVAL;

	xm_smart_chg_handle_cmd(val);

	return count;
}

static ssize_t smart_chg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	return sprintf(buf, "%d\n", info->smart_chg_cmd);
}

static struct device_attribute smart_chg_attr =
		__ATTR(smart_chg, 0644, smart_chg_show, smart_chg_store);

/* the stock show handlers also count the terminating NUL */
static ssize_t smart_batt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_err("%s smart_batt = %d", __func__, info->smart_batt);
	return sprintf(buf, "%d\n", info->smart_batt) + 1;
}

static ssize_t smart_batt_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (kstrtoint(buf, 0, &info->smart_batt))
		return -EINVAL;
	pr_err("%s smart_batt = %d", __func__, info->smart_batt);

	return size;
}
static struct device_attribute smart_batt_attrs =
	__ATTR(smart_batt, 0644, smart_batt_show, smart_batt_store);

static ssize_t night_charging_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_err("%s night_charging = %d", __func__, info->night_charging);
	return sprintf(buf, "%d\n", info->night_charging) + 1;
}

static ssize_t night_charging_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (kstrtobool(buf, &info->night_charging))
		return -EINVAL;
	pr_err("%s night_charging = %d", __func__, info->night_charging);

	return size;
}
static struct device_attribute night_charging_attrs =
	__ATTR(night_charging, 0644, night_charging_show, night_charging_store);

static ssize_t raw_soc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_err("%s raw_soc = %d", __func__, info->raw_soc);
	return sprintf(buf, "%d\n", info->raw_soc) + 1;
}

static ssize_t raw_soc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (kstrtoint(buf, 0, &info->raw_soc))
		return -EINVAL;
	pr_err("%s raw_soc = %d", __func__, info->raw_soc);

	return size;
}
static struct device_attribute raw_soc_attrs =
	__ATTR(raw_soc, 0644, raw_soc_show, raw_soc_store);

static struct attribute *batt_psy_attrs[] = {
	&smart_batt_attrs.attr,
	&night_charging_attrs.attr,
	&smart_chg_attr.attr,
	&raw_soc_attrs.attr,
	NULL,
};

static const struct attribute_group batt_psy_attrs_group = {
	.attrs = batt_psy_attrs,
};

int xm_smart_charge_add_to_batt_psy(void)
{
	int ret = 0;
	struct power_supply *batt_psy;

	pr_err("%s ++\n", __func__);
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy){
		pr_err("%s failed to get batt_psy", __func__);
		return -EPROBE_DEFER;
	}
	pr_err("%s success to get batt_psy", __func__);

	ret = sysfs_create_group(&(batt_psy->dev.kobj), &batt_psy_attrs_group);
	if(ret){
		pr_err("%s failed to creat batt_psy group, ret = %d\n", __func__, ret);
		return -EPROBE_DEFER;
	}

	pr_err("%s --", __func__);
	return ret;
}

void get_fv_ageing(int cyclecount)
{
	int i = 0;

	while (cyclecount > cycle_count_conf[i])
		i++;
	info->fv_ageing = dropfv_conf[i];
	set_fv_ageing(info->fv_ageing);
}

void get_drop_floatvolatge(void)
{
	int ret = 0;
	union power_supply_propval pval = {0,};
	struct power_supply *batt_psy;
	static int last_fv_ageing = 0;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("%s failed to get batt_psy, use default fv_aging: 0", __func__);
		info->fv_ageing = 0;
		return;
	}

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CYCLE_COUNT, &pval);
	if (ret < 0) {
		pr_err("%s failed to get cycle_count_prop, use default fv_aging: 0", __func__);
		info->fv_ageing = 0;
		return;
	}

	get_fv_ageing(pval.intval);
	if (info->fv_ageing != last_fv_ageing && info->fv_votable)
		rerun_election(info->fv_votable);
	last_fv_ageing = info->fv_ageing;
}

void monitor_night_charging(void)
{
	bool disable;
	int ret;

	if (info->night_charging && (info->soc >= 80)) {
		pr_err("%s disable charging\n", __func__);
		info->night_charging_flag = true;
		disable = true;
	} else if (info->night_charging_flag && (!info->night_charging || info->soc <= 75)) {
		pr_err("%s enable charging\n", __func__);
		info->night_charging_flag = false;
		disable = false;
	} else {
		return;
	}

	ret = vote(info->chg_disable_votable, NIGHT_CHARGING_VOTER, disable, 0);
	if (ret < 0) {
		pr_err("%s Couldn't vote to %s chg_disable_votable, ret = %d\n",
			__func__, NIGHT_CHARGING_VOTER, ret);
		return;
	}
	vote(info->night_charging_votable, NIGHT_CHARGING_VOTER, disable, 0);
}

static int xm_thermal_mitigation(int level)
{
	/*
	 * Stock bug fixed: stock indexes pps_thermal_mitigation[] with an
	 * out-of-range thermal level when the level is invalid.
	 */
	if (level < 0 || level >= info->thermal_levels)
		level = info->thermal_levels - 1;
	if (level < 0 || !info->pps_thermal_mitigation)
		return 0;
	return info->pps_thermal_mitigation[level];
}

void monitor_low_fast_strategy(void)
{
	bool fast_flag = false;
	time64_t time_now = 0, delta_time = 0;
	static time64_t time_last = 0;
	static int count;
	static int last_level;
	static bool hot_flag;
	int thermal_vote_current;

	if (count++ == 5) {
		pr_err("%s soc = %d, thermal_level = %d, thermal_board_temp = %d, pd_active = %d, low_fast_plugin_flag = %d, low_fast_enable = %d, screen_state = %d, b_flag = %d\n",
			__func__, info->soc, info->thermal_level, info->thermal_board_temp, info->pd_active, info->low_fast_plugin_flag,
			info->smart_charge[SMART_CHG_LOW_FAST].active_status, info->screen_state, info->b_flag);
		count = 0;
	}

	if (info->pd_active != QTI_POWER_SUPPLY_PD_PPS_ACTIVE || info->soc > 40 ||
			!info->low_fast_plugin_flag ||
			!info->smart_charge[SMART_CHG_LOW_FAST].active_status)
		goto out;

	if (info->thermal_level > MAX_THERMAL_LEVELS) {
		pr_err("%s: thermal_level is invalid\n", __func__);
		vote(info->low_fast_votable, LOW_FAST_VOTER, false, 0);
		vote(info->fcc_votable, THERMAL_DAEMON_VOTER, true,
			xm_thermal_mitigation(info->thermal_level));
		goto out;
	}

	// screen_state 0:bright, 1:black
	if (info->b_flag == SNORMAL && !info->screen_state) {  //black to bright
		info->b_flag = BLACK_TO_BRIGHT;
		time_last = ktime_get_seconds();
		fast_flag = true;
		pr_err("%s switch to bright time_last = %d\n", __func__, (int)time_last);
	} else if ((info->b_flag == BLACK_TO_BRIGHT || info->b_flag == BRIGHT) && !info->screen_state) {  //still bright
		info->b_flag = BRIGHT;
		time_now = ktime_get_seconds();
		delta_time = time_now - time_last;
		pr_err("%s still_bright time_now = %d, time_last = %d, delta_time = %d\n",
			__func__, (int)time_now, (int)time_last, (int)delta_time);
		if (delta_time <= 15) {
			fast_flag = true;
			pr_err("%s still_bright delta_time = %d, stay fast\n", __func__, (int)delta_time);
		} else {
			fast_flag = false;
			pr_err("%s still_bright delta_time = %d, exit fast\n", __func__, (int)delta_time);
		}
	} else { //black
		info->b_flag = BLACK;
		fast_flag = true;
		pr_err("%s black stay fast\n", __func__);
	}

	/* avoid the board temperature rising too fast */
	if (last_level == 5) {
		if ((info->thermal_level == 8 || info->thermal_level == 5) &&
				hot_flag && info->thermal_board_temp >= 416)
			fast_flag = false;
		else
			hot_flag = false;
	} else if (last_level == 8 && info->thermal_level == 5 &&
			info->thermal_board_temp >= 416) {
		hot_flag = true;
		pr_err("%s avoid thermal_board_temp raise too fast, exit fast mode\n", __func__);
		fast_flag = false;
	} else {
		hot_flag = false;
	}

	if (info->thermal_board_temp > 420 || !fast_flag || info->soc > 38) {  //exit fast strategy
		info->pps_fast_mode = false;
		vote(info->low_fast_votable, LOW_FAST_VOTER, false, 0);
		thermal_vote_current = info->pps_thermal_mitigation[info->thermal_level];
		vote(info->fcc_votable, THERMAL_DAEMON_VOTER, true, thermal_vote_current);
		pr_err("%s exit fast, thermal_vote_current = %d\n", __func__, thermal_vote_current);
	} else {  //stay fast strategy
		info->pps_fast_mode = true;
		thermal_vote_current = info->pps_thermal_mitigation_fast[info->thermal_level];

		if (info->thermal_board_temp > 410) {
			if (thermal_vote_current >= 5150000)
				thermal_vote_current -= 3000000;
			else
				thermal_vote_current = 2150000;
			pr_err("%s stay fast but decrease 3, thermal_board_temp = %d, thermal_vote_current = %d\n",
				__func__, info->thermal_board_temp, thermal_vote_current);
		} else if ((info->soc > 30) && (info->soc <= 38) && (info->thermal_board_temp < 390)) {
			if (thermal_vote_current <= 3950000)
				thermal_vote_current += 2000000;
			else
				thermal_vote_current = 5950000;
			pr_err("%s stay fast but add 2, thermal_board_temp = %d, thermal_vote_current = %d\n",
				__func__, info->thermal_board_temp, thermal_vote_current);
		} else if ((info->soc >= 15) && (info->soc <= 37) && (info->thermal_board_temp < 400)) {
			if (thermal_vote_current <= 2950000)
				thermal_vote_current += 3000000;
			else
				thermal_vote_current = 5950000;
			pr_err("%s stay fast but add 3, thermal_board_temp = %d, thermal_vote_current = %d\n",
				__func__, info->thermal_board_temp, thermal_vote_current);
		}

		vote(info->low_fast_votable, LOW_FAST_VOTER, true, 0);
		vote(info->fcc_votable, THERMAL_DAEMON_VOTER, true, thermal_vote_current);
		pr_err("%s stay fast, thermal_vote_current = %d\n", __func__, thermal_vote_current);
	}

out:
	last_level = info->thermal_level;
}

void xm_charge_update_date(void)
{
	static int count;
	int ret;

	info->fv_votable = find_votable("FV");
	if (!info->fv_votable) {
		pr_err("%s failed to get fv_votable\n", __func__);
		return;
	}

	info->fcc_votable = find_votable("FCC");
	if (!info->fcc_votable) {
		pr_err("%s failed to get fcc_votable\n", __func__);
		return;
	}

	info->chg_disable_votable = find_votable("CHG_DISABLE");
	if (!info->chg_disable_votable) {
		pr_err("%s failed to get chg_disable_votable\n", __func__);
		return;
	}

	info->low_fast_votable = find_votable("LOW_FAST");
	if (!info->low_fast_votable) {
		pr_err("%s failed to get low_fast_votable\n", __func__);
		return;
	}

	info->night_charging_votable = find_votable("NIGHT_CHARGING");
	if (!info->night_charging_votable) {
		pr_err("%s failed to get night_charging_votable\n", __func__);
		return;
	}

	ret = iio_read_channel_processed(info->capacity_raw_chan, &info->raw_soc);
	if (ret < 0)
		pr_err("%s raw_soc channel read failed, ret=%d\n", __func__, ret);

	ret = iio_read_channel_processed(info->capacity_chan, &info->soc);
	if (ret < 0) {
		pr_err("%s capacity channel read failed, ret=%d\n", __func__, ret);
		return;
	}

	ret = iio_read_channel_processed(info->pd_active_chan, &info->pd_active);
	if (ret < 0)
		pr_err("%s pd_active channel read failed, ret=%d\n", __func__, ret);

	get_drop_floatvolatge();

	if (count++ == 5) {
		pr_err("%s raw_soc = %d, soc = %d, pd_active = %d, fv_ageing = %d\n",
			__func__, info->raw_soc, info->soc, info->pd_active, info->fv_ageing);
		count = 0;
	}
}

static void mievent_upload(int miev_code, int para_cnt, ...)
{
	struct misight_mievent *event;
	va_list args;
	const char *type, *key;
	int i;

	event = cdev_tevent_alloc(miev_code);
	if (!event) {
		pr_err("CHG:DFX:Failed to allocate event\n");
		return;
	}

	pr_err("CHG:DFX:miev_code=%d, para_cnt=%d \n", miev_code, para_cnt);

	va_start(args, para_cnt);
	for (i = 0; i < para_cnt; i++) {
		type = va_arg(args, const char *);
		key = va_arg(args, const char *);
		if (!strcmp(type, "char")) {
			cdev_tevent_add_str(event, key, va_arg(args, const char *));
		} else if (!strcmp(type, "int")) {
			int val_long = va_arg(args, int);

			pr_err("CHG:DFX:key=%s, val_long=%d\n", key, val_long);
			cdev_tevent_add_int(event, key, val_long);
		} else {
			pr_err("CHG:DFX:unsupported type");
		}
	}
	va_end(args);

	cdev_tevent_write(event);
	cdev_tevent_destroy(event);
}

static void xm_dfx_none_standard_check(void)
{
	int real_type;

	if (IS_ERR_OR_NULL(g_chg)) {
		pr_err("CHG:DFX:point info is err or null\n");
		pr_err("CHG:DFX: dfx none standard check fail\n");
		return;
	}

	real_type = *(int *)((char *)g_chg + SMB_CHARGER_REAL_TYPE_OFFSET);
	if (real_type == QTI_POWER_SUPPLY_TYPE_USB_FLOAT) {
		mievent_upload(MIEV_CODE_NONE_STANDARD_CHG, 1, "char", "chgErrInfo", "noneStandartChg");
		pr_err("CHG:DFX:mievent_upload:real type:%d, chgErrInfo:noneStandartChg\n", real_type);
	}
}

static void xm_dfx_batt_linker_absent_check(void)
{
	union power_supply_propval pval = {0,};
	struct power_supply *batt_psy;
	int ret;

	if (IS_ERR_OR_NULL(info)) {
		pr_err("CHG:DFX:point info is err or null\n");
		goto fail;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (IS_ERR_OR_NULL(batt_psy)) {
		pr_err("CHG:DFX:power supply get battery is fail\n");
		goto fail;
	}

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_PRESENT, &pval);
	if (ret < 0 || pval.intval <= 0) {
		mievent_upload(MIEV_CODE_BATT_LINKER_ABSENT, 1, "char", "chgErrInfo", "battLinkerAbsent");
		pr_err("CHG:DFX:mievent_upload:battery linker absent, chgErrInfo:battLinkerAbsent\n");
	}
	return;
fail:
	pr_err("CHG:DFX: dfx batt linker absent check fail\n");
}

void xm_charge_work(struct work_struct *work)
{
	xm_charge_update_date();
	monitor_night_charging();
	monitor_smart_chg();
	monitor_low_fast_strategy();

	xm_dfx_none_standard_check();
	xm_dfx_batt_linker_absent_check();

	schedule_delayed_work(&info->xm_charge_work,
		info->soc > 40 ? msecs_to_jiffies(XM_CHARGE_WORK_MS) : msecs_to_jiffies(1000));
}

static int qpnp_smb5_notifier_event_callback(struct notifier_block *notifier,
	unsigned long chg_event, void *val)
{
	int ret;

	switch (chg_event) {
	/*charger_plugin_event 0:none 1:plugin 2:plugout*/
	case QPNP_SMB5_CHARGER_PLUGIN_EVENT:
		info->charger_plugin_event = *(int *)val;
		pr_info("%s: get charger_plugin_event: %d\n", __func__, info->charger_plugin_event);

		ret = iio_read_channel_processed(info->capacity_chan, &info->soc);
		if (ret < 0) {
			pr_err("%s capacity channel read failed, ret=%d\n", __func__, ret);
			return ret;
		}
		pr_err("%s soc = %d\n", __func__, info->soc);

		if (info->charger_plugin_event == 2)
			info->low_fast_plugin_flag = 0;
		else if ((info->charger_plugin_event == 1) && (info->soc <= 40) &&
				(info->thermal_board_temp <= 390))
			info->low_fast_plugin_flag = 1;
		info->smart_ctrl_en = false;
		info->pps_fast_mode = false;
		info->b_flag = SNORMAL;
		vote(info->low_fast_votable, LOW_FAST_VOTER, false, 0);
		return NOTIFY_DONE;
	case QPNP_SMB5_THERMAL_LEVEL_EVENT:
		info->thermal_level = *(int *)val;
		pr_info("%s: get thermal_level_event: %d\n", __func__, info->thermal_level);
		break;
	case QPNP_SMB5_SMART_CHG_EVENT:
		handle_smart_chg_value(*(int *)val);
		pr_info("%s: get smart_chg_event, analyzing...\n", __func__);
		break;
	case QPNP_SMB5_SMART_BATT_EVENT:
		info->smart_batt = *(int *)val;
		pr_info("%s: get smart_batt_event: %d\n", __func__, info->smart_batt);
		break;
	case QPNP_SMB5_NIGHT_CHARGING_EVENT:
		info->night_charging = !!*(int *)val;
		pr_info("%s: get night_charging_event = %d\n", __func__, info->night_charging);
		break;
	default:
		pr_err("%s: not supported charger notifier event: %d\n", __func__, (int)chg_event);
		break;
	}

	return NOTIFY_DONE;
}

static int mi_thermal_notifier_event_callback(struct notifier_block *notifier,
	unsigned long event, void *val)
{
	if (event == MI_THERMAL_BOARD_TEMP_EVENT) {
		info->thermal_board_temp = *(int *)val;
		pr_err("%s: get thermal_board_temp: %d\n", __func__, info->thermal_board_temp);
	} else {
		pr_err("%s: not supported mi_thermal_notifier_event_callback: %d\n", __func__, (int)event);
	}

	return NOTIFY_DONE;
}

static void screen_state_for_xm_smart_chg_callback(enum panel_event_notifier_tag tag,
	struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		pr_err("%s:Invalid notification\n", __func__);
		return;
	}

	if (notification->notif_data.early_trigger)
		return;

	if (tag == PANEL_EVENT_NOTIFICATION_PRIMARY) {
		switch (notification->notif_type) {
		case DRM_PANEL_EVENT_BLANK:
			info->screen_state = 1;
			break;
		case DRM_PANEL_EVENT_UNBLANK:
			info->screen_state = 0;
			break;
		case DRM_PANEL_EVENT_BLANK_LP:
			info->screen_state = 1;
			break;
		default:
			return;
		}
		pr_err("%s: screen_state = %d\n", __func__, info->screen_state);
	}

	set_blank_status(info->screen_state);
}

static int xm_smart_chg_check_panel(void)
{
	struct device_node *node, *pnode;
	int count, i;

	node = of_find_node_by_name(NULL, "charger-screen");
	if (!node) {
		pr_err("%s ERROR: Cannot find charger_screen_node with panel!", __func__);
		return -ENODEV;
	}

	count = of_count_phandle_with_args(node, "panel", NULL);
	pr_err("%s: count of panel in node is: %d\n", __func__, count);
	if (count <= 0)
		return -ENODEV;

	for (i = 0; i < count; i++) {
		pnode = of_parse_phandle(node, "panel", i);
		/* stock passes the node pointer itself to "%s" */
		pr_err("%s: try to add of node panel: %s\n", __func__,
			pnode ? pnode->name : NULL);
		prim_panel = of_drm_find_panel(pnode);
		if (!IS_ERR(prim_panel))
			break;
		if (PTR_ERR(prim_panel) == -EPROBE_DEFER) {
			pr_err("%s ERROR: Cannot find prim_panel of panel_node!", __func__);
			prim_panel = NULL;
			return -EPROBE_DEFER;
		}
		prim_panel = NULL;
	}
	pr_err("%s: count of panel in panel_node PTR_ERR_prim_panel  is: %d\n",
		__func__, (int)PTR_ERR(prim_panel));

	return 0;
}

int xm_smart_chg_register_panel_notifier(void)
{
	int ret = 0;
	int i;

	for (i = 0; i < 10; i++) {
		ret = xm_smart_chg_check_panel();
		if (ret < 0) {
			pr_err("%s check panel fail(%d), i = %d\n", __func__, ret, i);
			if (ret == -EPROBE_DEFER)
				return ret;
		}
		if (prim_panel) {
			pr_err("%s success to check panel\n", __func__);
			break;
		}
	}

	if (!prim_panel) {
		pr_err("%s:Failed to get prim_panel\n", __func__);
		return -ENODEV;
	}

	if (cookie)
		return ret;

	cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_XM_SMART_CHG, prim_panel,
			screen_state_for_xm_smart_chg_callback, NULL);
	if (IS_ERR(cookie)) {
		pr_err("%s:Failed to register for prim_panel events\n", __func__);
		return -EPROBE_DEFER;
	}

	pr_err("%s:prim_panel_event_notifier_register register succeed\n", __func__);
	return 0;
}

int xm_smart_chg_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int ret = 0;
	int byte_len = 0;

	ret = of_property_read_string(node, "label", &info->label);
	if (ret < 0) {
		pr_err("%s failed to read label(%d), use default label\n", __func__, ret);
		info->label = "smart_charger";
	}

	if (of_find_property(node, "xiaomi,pd-thermal-mitigation-fast", &byte_len)) {
		info->pps_thermal_mitigation_fast = kzalloc(byte_len, GFP_KERNEL);
		if (info->pps_thermal_mitigation_fast == NULL){
			pr_err("%s failed to alloc mem for pps_thermal_mitigation_fast\n", __func__);
			return -ENOMEM;
		}
		info->thermal_levels = byte_len / sizeof(u32);
		ret = of_property_read_u32_array(node, "xiaomi,pd-thermal-mitigation-fast",
			info->pps_thermal_mitigation_fast,
			info->thermal_levels);
		if (ret < 0) {
			pr_err("%s failed read pps_thermal_mitigation_fast from dtsi, ret = %d\n", __func__, ret);
			return ret;
		}
	}

	if (of_find_property(node, "xiaomi,pd-thermal-mitigation", &byte_len)) {
		info->pps_thermal_mitigation = kzalloc(byte_len, GFP_KERNEL);
		if (info->pps_thermal_mitigation == NULL){
			pr_err("%s failed to alloc mem for pps_thermal_mitigation\n", __func__);
			return -ENOMEM;
		}
		info->thermal_levels = byte_len / sizeof(u32);
		ret = of_property_read_u32_array(node, "xiaomi,pd-thermal-mitigation",
			info->pps_thermal_mitigation,
			info->thermal_levels);
		if (ret < 0) {
			pr_err("%s failed read pps_thermal_mitigation from dtsi, ret = %d\n", __func__, ret);
			return ret;
		}
	}

	return 0;
}

int xm_smart_chg_post_init(void)
{
	int ret = 0;

	pr_err("%s ++\n", __func__);

	/*add sysfs node to battery psy*/
	ret = xm_smart_charge_add_to_batt_psy();
	if (ret) {
		pr_err("%s xm_smart_charge_add_to_batt_psy failed(%d)\n", __func__, ret);
		return ret;
	}

	ret = xm_smart_chg_register_panel_notifier();
	if (ret < 0) {
		pr_err("%s xm_smart_chg_register_panel_notifier failed(%d)\n", __func__, ret);
		return ret;
	}

	info->thermal_board_temp = 250;

	info->charger_thermal_nb.notifier_call = mi_thermal_notifier_event_callback;
	mi_thermal_reg_notifier(&info->charger_thermal_nb);

	info->smart_chg_nb.notifier_call = qpnp_smb5_notifier_event_callback;
	qpnp_smb5_reg_notifier(&info->smart_chg_nb);

	return 0;
}

static int xm_smart_chg_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	int rc = 0;

	pr_err("%s ++\n", __func__);
	*val1 = 0;

	switch (chan->channel) {
	case XM_SMART_CHG_IIO_INT_LABEL:
		*val1 = info->int_label;
		pr_err("%s intLabel = %d\n", __func__, *val1);
		break;
	default:
		pr_err("%s Unsupported QG IIO chan %d\n", __func__, chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err_ratelimited("%s Couldn't read IIO channel %d, rc = %d\n",
			__func__, chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int xm_smart_chg_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	int rc = 0;

	pr_err("%s ++\n", __func__);

	switch (chan->channel) {
	case XM_SMART_CHG_IIO_INT_LABEL:
		info->int_label = val1;
		pr_err("%s intLabel = %d\n", __func__, val1);
		break;
	default:
		pr_err("Unsupported xm_smart_chg IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err_ratelimited("%s Couldn't write IIO channel %d, rc = %d\n",
			__func__, chan->channel, rc);

	return rc;
}

static int xm_smart_chg_iio_of_xlate(struct iio_dev *indio_dev,
		const struct of_phandle_args *iiospec)
{
	struct iio_chan_spec *iio_chan = info->iio_chan;
	int i = 0;

	pr_err("%s ++\n", __func__);
	while (i < 1) {
		if (iio_chan->channel == iiospec->args[0])
			return i;
		i++;
		iio_chan++;
		pr_err("%s ++, i = %d\n", __func__, i);
	}

	return -EINVAL;
}

static const struct iio_info xm_smart_chg_iio_info = {
	.read_raw	= xm_smart_chg_iio_read_raw,
	.write_raw	= xm_smart_chg_iio_write_raw,
	.of_xlate	= xm_smart_chg_iio_of_xlate,
};

static int xm_smart_chg_init_iio(struct xm_smart_chg_info *chip)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int rc = 0;

	pr_err("%s ++\n", __func__);

	chip->capacity_chan = iio_channel_get(chip->dev, "capacity");
	if (IS_ERR(chip->capacity_chan)) {
		rc = PTR_ERR(chip->capacity_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("%s capacity channel unavailable, rc=%d\n", __func__, rc);
		chip->capacity_chan = NULL;
		return rc;
	}

	chip->pd_active_chan = iio_channel_get(chip->dev, "pd_active");
	if (IS_ERR(chip->pd_active_chan)) {
		rc = PTR_ERR(chip->pd_active_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("%s pd_active channel unavailable, rc=%d\n", __func__, rc);
		chip->pd_active_chan = NULL;
		return rc;
	}

	chip->capacity_raw_chan = iio_channel_get(chip->dev, "capacity_raw");
	if (IS_ERR(chip->capacity_raw_chan)) {
		rc = PTR_ERR(chip->capacity_raw_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("%s capacity_raw channel unavailable, rc=%d\n", __func__, rc);
		chip->capacity_raw_chan = NULL;
		return rc;
	}

	chip->iio_chan = devm_kcalloc(chip->dev, 1, sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan) {
		pr_err("%s failed to alloc memory for iio_chan_spec, rc=%d\n", __func__, rc);
		return -ENOMEM;
	}

	chip->int_iio_chans = devm_kcalloc(chip->dev, 1, sizeof(*chip->int_iio_chans), GFP_KERNEL);
	if (!chip->int_iio_chans) {
		pr_err("%s failed to alloc memory for iio_channel, rc=%d\n", __func__, rc);
		return -ENOMEM;
	}

	indio_dev->info = &xm_smart_chg_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = "xm_smart_chg";
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = 1;

	chip->int_iio_chans[0].indio_dev = indio_dev;
	chan = &chip->iio_chan[0];
	chip->int_iio_chans[0].channel = chan;
	chan->address = 0;
	chan->channel = XM_SMART_CHG_IIO_INT_LABEL;
	chan->type = IIO_ENERGY;
	chan->datasheet_name = "intLabel";
	chan->extend_name = "intLabel";
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED);

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("%s failed to register xm_smart_chg IIO device, rc=%d\n", __func__, rc);
	else
		pr_err("%s success to register xm_smart_chg IIO device, rc=%d\n", __func__, rc);

	return rc;
}

int xm_smart_chg_info_init(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev) {
		pr_err("%s failed to alloc memory for indio_dev", __func__);
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	info->indio_dev = indio_dev;
	info->smart_batt = 0;
	info->int_label = 33;
	info->night_charging_flag = false;
	info->smart_ctrl_en = false;

	INIT_DELAYED_WORK(&info->xm_charge_work, xm_charge_work);
	return 0;
}

static int xm_smart_chg_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_err("%s ++\n", __func__);

	ret = xm_smart_chg_info_init(pdev);
	if (ret < 0) {
		pr_err("%s xm_smart_chg_info_init failed, xm_smart_chg driver exit\n", __func__);
		return ret;
	}
	info->dev = &pdev->dev;

	ret = xm_smart_chg_parse_dt(pdev);
	if (ret < 0) {
		pr_err("%s xm_smart_chg_parse_dt failed, xm_smart_chg driver exit\n", __func__);
		return ret;
	}

	ret = xm_smart_chg_init_iio(info);
	if (ret < 0) {
		pr_err("%s xm_smart_chg_init_iio failed, xm_smart_chg driver exit\n", __func__);
		return -EPROBE_DEFER;
	}
	pr_err("%s xm_smart_chg_init_iio success\n", __func__);

	ret = xm_smart_chg_post_init();
	if (ret < 0) {
		pr_err("%s xm_smart_chg_post_init failed, xm_smart_chg driver exit\n", __func__);
		return ret;
	}
	pr_err("%s xm_smart_chg_post_init success\n", __func__);

	schedule_delayed_work(&info->xm_charge_work, msecs_to_jiffies(XM_CHARGE_WORK_MS * 3));

	pr_err("%s --\n", __func__);

	return ret;
}

static int xm_smart_chg_remove(struct platform_device *pdev)
{
	pr_err("%s ++\n", __func__);
	return 0;
}

static const struct of_device_id xm_smart_chg_of_ids[] = {
	{
		.compatible = "xiaomi,xm_smart_chg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, xm_smart_chg_of_ids);
static struct platform_driver xm_smart_chg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "xm_smart_chg",
		.of_match_table = xm_smart_chg_of_ids,
	},
	.probe = xm_smart_chg_probe,
	.remove = xm_smart_chg_remove,
};

/******************* Module Init ***********************************/
static int __init xm_smart_chg_init(void)
{
	pr_err("%s ++\n", __func__);
	return platform_driver_register(&xm_smart_chg_driver);
}

static void __exit xm_smart_chg_exit(void)
{
	struct power_supply *batt_psy;

	pr_err("%s ++\n", __func__);
	cancel_delayed_work(&info->xm_charge_work);
	mi_thermal_unreg_notifier(&info->charger_thermal_nb);
	/* stock bug fixed: stock never unregisters the qpnp-smb5 notifier */
	qpnp_smb5_unreg_notifier(&info->smart_chg_nb);

	if (prim_panel && !IS_ERR(cookie))
		panel_event_notifier_unregister(cookie);
	else
		pr_err("%s:prim_panel_event_notifier_unregister falt\n", __func__);

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		pr_err("%s failed to get batt_psy", __func__);
	else
		sysfs_remove_group(&batt_psy->dev.kobj, &batt_psy_attrs_group);

	/* stock bug fixed: stock skips this when the battery psy is gone */
	platform_driver_unregister(&xm_smart_chg_driver);
}
module_init(xm_smart_chg_init);
module_exit(xm_smart_chg_exit);

MODULE_DESCRIPTION("XM SMART CHARGE Driver");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: qpnp-smb5-main");
/* stock module is built in-tree (no out-of-tree taint) */
MODULE_INFO(intree, "Y");
