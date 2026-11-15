/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*
 * sapphire (Redmi Note 13 4G, FT5452J "ft3519t" module):
 * FocalTech V3.4 core (MiCode sunstone FT3519T) with the Longcheer lct_tp
 * hooks (MiCode spes ft3418_i2c), the qpnp-smb5 USB plug-in callback, the
 * Qualcomm panel_event_notifier screen state and the Xiaomi FOD/touch-mode
 * glue reconstructed from the stock sapphire focaltech_ts_i2c.ko.
 */

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irqdesc.h>
#include <linux/fs.h>
#include <linux/moduleparam.h>
#include <drm/drm_panel.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "focaltech_core.h"
#include <linux/string.h>

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME					 "fts_ts"
#define INTERVAL_READ_REG				   50   /* unit:ms */
#define TIMEOUT_READ_REG					1000 /* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV					  2800000
#define FTS_VTG_MAX_UV					  3300000
#define FTS_I2C_VTG_MIN_UV				  1800000
#define FTS_I2C_VTG_MAX_UV				  1800000
#endif

#define FTS_CHIP_IDH						0x54	/* FT5452J ('T') */
#define FTS_REG_FOD_DATA					0xE1
#define FTS_FOD_DATA_LEN					9
#define FTS_FOD_EVENT_ID					0x26
#define FTS_FOD_OVERLAP_AREA				100
#define FTS_REG_PALM_DATA					0x9B
#define FTS_REG_GAME_ACTIVE					0x93
#define FTS_TP_DEV_MINOR					0x38

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
static struct drm_panel *active_panel;
static struct xiaomi_touch_interface xiaomi_touch_interfaces;

static void fts_ts_usb_plugin_work_func(struct work_struct *work);
DECLARE_WORK(fts_usb_plugin_work, fts_ts_usb_plugin_work_func);

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

/*****************************************************************************
*  Longcheer / qpnp-smb5 glue
*****************************************************************************/
static void fts_ts_usb_plugin_work_func(struct work_struct *work)
{
	struct fts_ts_data *ts_data = fts_data;

	if (ts_data->suspended) {
		msleep(8);
		return;
	}

	if (!ts_data->fw_loading && !ts_data->selftest_running)
		lct_fts_set_charger_mode(g_touchscreen_usb_pulgin.usb_plugged_in);
}

int lct_fts_tp_gesture_callback(bool flag)
{
	struct fts_ts_data *ts_data = fts_data;

	if (ts_data->suspended) {
		FTS_INFO("The gesture mode will be %s the next time you wakes up.",
				 flag ? "enabled" : "disabled");
		return -EPERM;
	}

	set_lct_tp_gesture_status(flag);
	if (flag) {
		ts_data->gesture_mode = ENABLE;
		ts_data->lct_gesture_mode = ENABLE;
	} else {
		ts_data->gesture_mode = DISABLE;
		ts_data->lct_gesture_mode = DISABLE;
	}
	return 0;
}

void fts_ts_usb_event_callback(void)
{
	schedule_work(&fts_usb_plugin_work);
}

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
	int i = 0;
	struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
	u8 cid_h = 0x0;

	if (cid->type == 0)
		return -ENODATA;

	for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
		cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
		if (cid_h && (id_h == cid_h)) {
			return 0;
		}
	}

	return -ENODATA;
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*		 need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
	int ret = 0;
	int cnt = 0;
	u8 idh = 0;
	struct fts_ts_data *ts_data = fts_data;
	u8 chip_idh = ts_data->ic_info.ids.chip_idh;

	do {
		ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
		if ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0))
			return 0;
		FTS_ERROR("TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	fts_wait_tp_to_valid();
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(ts_data);
	/* recover TP gesture state 0xD0 */
	fts_gesture_recovery(ts_data);
	/* recover FOD state 0xCF (and gesture 0xD0 while suspended) */
	if ((ts_data->fod_status != -1) && (ts_data->fod_status != 0)) {
		if (ts_data->suspended)
			fts_gesture_reg_write(0x01, true);
		fts_fod_reg_write(0x02, true);
	}
	FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
	FTS_DEBUG("tp reset");
	gpio_direction_output(fts_data->pdata->reset_gpio, 0);
	msleep(1);
	gpio_direction_output(fts_data->pdata->reset_gpio, 1);
	if (hdelayms) {
		msleep(hdelayms);
	}

	return 0;
}

/* after a reset in resume: wait for the fw, then restore FOD */
int fts_recover_after_reset(void)
{
	int i = 0;
	u8 idh = 0;

	for (i = 0; i < 20; i++) {
		fts_read_reg(FTS_REG_CHIP_ID, &idh);
		if (idh == FTS_CHIP_IDH)
			break;
		msleep(10);
	}
	if (i >= 20)
		FTS_ERROR("wait tp fw valid timeout");

	if ((fts_data->fod_status != -1) && (fts_data->fod_status != 0)) {
		FTS_INFO("fod_status = %d\n", fts_data->fod_status);
		fts_fod_recovery();
	}

	return 0;
}

void fts_irq_disable(void)
{
	unsigned long irqflags;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (!fts_data->irq_disabled) {
		disable_irq_nosync(fts_data->irq);
		fts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
	unsigned long irqflags = 0;
	struct irq_desc *desc = irq_to_desc(fts_data->irq);

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if ((desc->depth == 1) || fts_data->irq_disabled) {
		enable_irq(fts_data->irq);
		fts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
	int ret = 0;
	u8 buf[3] = {0xEB, 0xAA, 0x09};

	if (fts_data->bus_type != BUS_TYPE_I2C)
		return;

	ret = fts_write(buf, 3);
	if (ret < 0) {
		FTS_ERROR("hid2std cmd write fail");
	} else {
		msleep(10);
		buf[0] = buf[1] = buf[2] = 0;
		ret = fts_read(NULL, 0, buf, 3);
		if (ret < 0) {
			FTS_ERROR("hid2std cmd read fail");
		} else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
			FTS_DEBUG("hidi2c change to stdi2c successful");
		} else {
			FTS_DEBUG("hidi2c change to stdi2c not support or fail");
		}
	}
}

static int fts_get_chip_types(
	struct fts_ts_data *ts_data,
	u8 id_h, u8 id_l, bool fw_valid)
{
	u32 i = 0;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((0x0 == id_h) || (0x0 == id_l)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_INFO("verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (VALID == fw_valid) {
			if ((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
				|| ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
				|| ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
				break;
			}
		}
	}

	if (i >= ctype_entries) {
		return -ENODATA;
	}

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };
	u32 id_cmd_len = 0;

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = fts_write(id_cmd, 2);
	if (ret < 0) {
		FTS_ERROR("start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
	if (ts_data->ic_info.is_incell)
		id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
	else
		id_cmd_len = FTS_CMD_READ_ID_LEN;
	ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
	if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
		FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*		ill know which IC is it.
*		If cant get the ic information, maybe not focaltech's touch IC, need
*		unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };

	ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

	do {
		ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id[0]);
		ret = fts_read_reg(FTS_REG_CHIP_ID2, &chip_id[1]);
		if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
			FTS_DEBUG("chip id read invalid, read:0x%02x%02x",
					  chip_id[0], chip_id[1]);
		} else {
			ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
			if (!ret)
				break;
			else
				FTS_ERROR("TP not ready, read:0x%02x%02x",
						  chip_id[0], chip_id[1]);
		}

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
		FTS_INFO("fw is invalid, need read boot id");
		if (ts_data->ic_info.hid_supported) {
			fts_hid2std();
		}

		ret = fts_read_bootid(ts_data, &chip_id[0]);
		if (ret <  0) {
			FTS_ERROR("read boot id fail");
			return ret;
		}

		ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
		if (ret < 0) {
			FTS_ERROR("can't get ic informaton");
			return ret;
		}
	}

	FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
			 ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
			 ts_data->ic_info.cid.type);

	return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void fts_show_touch_buffer(u8 *data, u32 datalen)
{
	u32 i = 0;
	u32 count = 0;
	char *tmpbuf = NULL;

	tmpbuf = kzalloc(1024, GFP_KERNEL);
	if (!tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return;
	}

	for (i = 0; i < datalen; i++) {
		count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
		if (count >= 1024)
			break;
	}
	FTS_DEBUG("touch_buf:%s", tmpbuf);

	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
}

void fts_release_all_finger(void)
{
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;
	u32 finger_count = 0;
	u32 max_touches = ts_data->pdata->max_touch_number;

	ts_data->finger_in_fod = false;
	ts_data->overlap_area = 0;

	mutex_lock(&ts_data->report_mutex);
	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(input_dev, BTN_INFO, 0);
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	ts_data->touch_points = 0;
	ts_data->key_state = 0;
	mutex_unlock(&ts_data->report_mutex);
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*		if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*		need report it to key event.
*		x_dim: parse from dts, means key x_coordinate, dimension:+-50
*		y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *ts_data, struct ts_event *kevent)
{
	int i = 0;
	int x = kevent->x;
	int y = kevent->y;
	int *x_dim = &ts_data->pdata->key_x_coords[0];
	int *y_dim = &ts_data->pdata->key_y_coords[0];

	if (!ts_data->pdata->have_key) {
		return -EINVAL;
	}
	for (i = 0; i < ts_data->pdata->key_number; i++) {
		if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
			(y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
			if (EVENT_DOWN(kevent->flag)
				&& !(ts_data->key_state & (1 << i))) {
				input_report_key(ts_data->input_dev, ts_data->pdata->keys[i], 1);
				ts_data->key_state |= (1 << i);
				FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
			} else if (EVENT_UP(kevent->flag)
					   && (ts_data->key_state & (1 << i))) {
				input_report_key(ts_data->input_dev, ts_data->pdata->keys[i], 0);
				ts_data->key_state &= ~(1 << i);
				FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
			}
			return 0;
		}
	}
	return -EINVAL;
}

static int fts_input_report_b(struct fts_ts_data *ts_data, struct ts_event *events)
{
	int i = 0;
	int touch_down_point_cur = 0;
	int touch_point_pre = ts_data->touch_points;
	u32 max_touch_num = ts_data->pdata->max_touch_number;
	bool touch_event_coordinate = false;
	struct input_dev *input_dev = ts_data->input_dev;

	for (i = 0; i < ts_data->touch_event_num; i++) {
		if (fts_input_report_key(ts_data, &events[i]) == 0) {
			continue;
		}

		touch_event_coordinate = true;
		if (EVENT_DOWN(events[i].flag)) {
			input_mt_slot(input_dev, events[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
			/* finger lies on the FOD area while the screen is on */
			if (!ts_data->fod_finger_skip &&
				(ts_data->overlap_area == FTS_FOD_OVERLAP_AREA) &&
				!ts_data->suspended) {
				input_report_key(input_dev, BTN_INFO, 1);
				update_fod_press_status(1);
			}
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, events[i].y);

			touch_down_point_cur |= (1 << events[i].id);
			touch_point_pre |= (1 << events[i].id);

			if ((ts_data->log_level >= 2) ||
				((1 == ts_data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
				FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
						  events[i].id, events[i].x, events[i].y,
						  events[i].p, events[i].area);
			}
		} else {
			input_mt_slot(input_dev, events[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			touch_point_pre &= ~(1 << events[i].id);
			if (ts_data->log_level >= 1)
				FTS_DEBUG("[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(touch_point_pre ^ touch_down_point_cur)) {
		for (i = 0; i < max_touch_num; i++)  {
			if ((1 << i) & (touch_point_pre ^ touch_down_point_cur)) {
				input_mt_slot(input_dev, i);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			}
		}
	}

	if (touch_down_point_cur)
		input_report_key(input_dev, BTN_TOUCH, 1);
	else if (touch_event_coordinate || ts_data->touch_points) {
		update_fod_press_status(0);
		input_report_key(input_dev, BTN_TOUCH, 0);
	}

	ts_data->touch_points = touch_down_point_cur;
	input_sync(input_dev);
	return 0;
}

/*
 * FOD event block (0xE1, 9 bytes):
 * [0] point id, [1] 0x26 = FOD event, [3] area, [4..5] x, [6..7] y,
 * [8] 0 = finger down / non-zero = finger up
 */
static void fts_read_and_report_foddata(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 buf[FTS_FOD_DATA_LEN] = { 0 };
	struct input_dev *input_dev = ts_data->input_dev;
	int x = 0, y = 0, z = 0;

	ts_data->fod_addr = FTS_REG_FOD_DATA;
	ts_data->fod_size = FTS_FOD_DATA_LEN;
	ret = fts_read(&ts_data->fod_addr, 1, buf, ts_data->fod_size);
	if (ret < 0) {
		FTS_ERROR("read fod failed, ret:%d", ret);
		return;
	}

	if (buf[1] != FTS_FOD_EVENT_ID) {
		ts_data->overlap_area = 0;
		return;
	}

	x = (buf[4] << 8) | buf[5];
	y = (buf[6] << 8) | buf[7];
	z = buf[3];

	if (buf[8] == 0) {
		/* finger down in the FOD area */
		mutex_lock(&ts_data->report_mutex);
		if (!ts_data->fod_finger_skip)
			ts_data->overlap_area = FTS_FOD_OVERLAP_AREA;
		if (ts_data->old_point_id != buf[0]) {
			if (ts_data->old_point_id == 0xFF)
				ts_data->old_point_id = buf[0];
			else
				ts_data->point_id_changed = true;
		}
		ts_data->finger_in_fod = true;
		if (ts_data->suspended && ts_data->fod_status) {
			if (ts_data->nonui_status == 2) {
				FTS_INFO("nonui_status is two, don't report 152");
			} else if (!ts_data->fod_finger_skip) {
				input_mt_slot(input_dev, buf[0]);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
				input_report_key(input_dev, BTN_INFO, 1);
				update_fod_press_status(1);
				input_report_key(input_dev, BTN_TOUCH, 1);
				input_report_key(input_dev, BTN_TOOL_FINGER, 1);
				input_report_abs(input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, ts_data->overlap_area);
				input_report_abs(input_dev, ABS_MT_WIDTH_MINOR, ts_data->overlap_area);
				input_report_abs(input_dev, ABS_MT_PRESSURE, z);
				input_sync(input_dev);
			}
		}
		mutex_unlock(&ts_data->report_mutex);
	} else {
		/* finger up */
		input_report_key(input_dev, BTN_INFO, 0);
		update_fod_press_status(0);
		input_sync(input_dev);
		ts_data->point_id_changed = false;
		ts_data->finger_in_fod = false;
		ts_data->fod_finger_skip = false;
		ts_data->old_point_id = 0xFF;
		FTS_INFO("Report_0x152 UP for FingerPrint");
		ts_data->overlap_area = 0;
		if (!ts_data->suspended) {
			if (!ts_data->fod_status)
				fts_fod_reg_write(0x02, false);
			return;
		}
		mutex_lock(&ts_data->report_mutex);
		input_mt_slot(input_dev, buf[0]);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
		input_sync(input_dev);
		mutex_unlock(&ts_data->report_mutex);
	}
}

static int fts_read_touchdata(struct fts_ts_data *ts_data, u8 *buf)
{
	int ret = 0;
	u32 touch_max_size = 0;
	u32 max_touch_num = ts_data->pdata->max_touch_number;
	u8 event = 0xFF;

	ts_data->touch_addr = 0x01;
	ret = fts_read(&ts_data->touch_addr, 1, buf, ts_data->touch_size);

	fts_read_and_report_foddata(ts_data);

	if (ret < 0) {
		FTS_ERROR("read touchdata fails,ret:%d", ret);
		return ret;
	}

	if (ts_data->aod_changed && !ts_data->gesture_mode &&
		(fts_gesture_readdata(ts_data, NULL) == 0)) {
		FTS_INFO("succuss to get gesture data in irq handler");
		return 1;
	}

	event = (buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F;
	if (event == TOUCH_DEFAULT) {
		if (buf[ts_data->touch_size - 1] != 0xFF)
			touch_max_size = max_touch_num * FTS_ONE_TCH_LEN + 2;
	} else if (event == TOUCH_EVENT_NUM) {
		touch_max_size = (buf[FTS_TOUCH_E_NUM] & 0x0F) * FTS_ONE_TCH_LEN + 2;
	} else if (event == TOUCH_EXTRA_MSG) {
		touch_max_size = (buf[FTS_TOUCH_E_NUM] & 0x0F) * FTS_ONE_TCH_LEN + \
						 4 + ((buf[2] << 8) + buf[3]);
		if (touch_max_size > FTS_MAX_TOUCH_BUF)
			touch_max_size = FTS_MAX_TOUCH_BUF;
	}

	if (touch_max_size > ts_data->touch_size) {
		ts_data->ta_size = touch_max_size;
		ts_data->touch_addr += ts_data->touch_size;
		ret = fts_read(&ts_data->touch_addr, 1, buf + ts_data->touch_size, \
					   touch_max_size - ts_data->touch_size);
		if (ret < 0) {
			FTS_ERROR("read touchdata2 fails,ret:%d", ret);
			return ret;
		}
	}

	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
	int ret = 0;
	u8 gesture_en = 0xFF;

	memset(touch_buf, 0xFF, FTS_MAX_TOUCH_BUF);
	ts_data->ta_size = ts_data->touch_size;

	/*read touch data*/
	ret = fts_read_touchdata(ts_data, touch_buf);
	if (ret < 0) {
		FTS_ERROR("read touch data fails");
		return TOUCH_ERROR;
	}

	if (ts_data->log_level >= 3)
		fts_show_touch_buffer(touch_buf, ts_data->ta_size);

	if (ret)
		return TOUCH_IGNORE;

	/*gesture*/
	if (ts_data->suspended && ts_data->gesture_mode) {
		ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
		if ((ret >= 0) && (gesture_en == ENABLE))
			return TOUCH_GESTURE;
		else
			FTS_DEBUG("gesture not enable in fw, don't process gesture");
	}

	if ((touch_buf[1] == 0xFF) && (touch_buf[2] == 0xFF)
		&& (touch_buf[3] == 0xFF) && (touch_buf[4] == 0xFF)) {
		FTS_INFO("touch buff is 0xff, need recovery state");
		return TOUCH_FW_INIT;
	}

	return ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);
}

static int fts_palm_sensor_cmd(int value)
{
	int ret = 0;

	ret = fts_write_reg(FTS_REG_PALM_MODE_EN, value ? 0x05 : 0x00);
	if (ret < 0)
		FTS_ERROR("Set palm sensor switch failed!\n");
	else
		FTS_INFO("Set palm sensor switch: %d\n", value);

	return ret;
}

static int fts_read_palm_data(void)
{
	int ret = 0;
	u8 reg_value = 0;

	if (fts_data == NULL)
		return -EINVAL;

	ret = fts_read_reg(FTS_REG_PALM_DATA, &reg_value);
	if (ret < 0) {
		FTS_ERROR("read palm data error\n");
		return -EINVAL;
	}

	if (reg_value == 0x01)
		update_palm_sensor_value(1);
	else if (reg_value == 0x00)
		update_palm_sensor_value(0);

	return 0;
}

static int fts_irq_read_report(struct fts_ts_data *ts_data)
{
	int i = 0;
	int max_touch_num = ts_data->pdata->max_touch_number;
	int touch_etype = 0;
	u8 event_num = 0;
	u8 finger_num = 0;
	u8 pointid = 0;
	u8 base = 0;
	u8 *touch_buf = ts_data->touch_buf;
	struct ts_event *events = ts_data->events;

	touch_etype = fts_read_parse_touchdata(ts_data, touch_buf);
	switch (touch_etype) {
	case TOUCH_DEFAULT:
		finger_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (finger_num > max_touch_num) {
			FTS_ERROR("invalid point_num(%d)", finger_num);
			return -EIO;
		}

		for (i = 0; i < max_touch_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= FTS_MAX_ID)
				break;
			else if (pointid >= max_touch_num) {
				FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			/*
			 * sapphire (as stock): X in 1/8 pixel (12 + 3 bits),
			 * Y in 1/16 pixel (12 + 4 bits, LSB in bit 7 of the area
			 * byte). DT display-coords is 1080x16 by 2400x16.
			 */
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 11) \
						  + ((touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF) << 3)\
						  + ((touch_buf[FTS_TOUCH_OFF_PRE + base] & 0xC0) >> 5)\
						  + ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x20) >> 5);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 12) \
						  + ((touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF) << 4)\
						  + ((touch_buf[FTS_TOUCH_OFF_PRE + base] & 0x30) >> 2)\
						  + ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x10) >> 3)\
						  + ((touch_buf[FTS_TOUCH_OFF_AREA + base] & 0x80) >> 7);
			events[i].p = (touch_buf[FTS_TOUCH_OFF_PRE + base] & 0x0F);
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			if (events[i].area <= 0)
				events[i].area = 0x09;

			event_num++;
			if (EVENT_DOWN(events[i].flag) && (finger_num == 0)) {
				FTS_INFO("abnormal touch data from fw");
				return -EIO;
			}
		}

		if (event_num == 0) {
			FTS_INFO("no touch point information(%02x)", touch_buf[2]);
			return -EIO;
		}
		ts_data->touch_event_num = event_num;

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;

	case TOUCH_EVENT_NUM:
		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!event_num || (event_num > max_touch_num)) {
			FTS_ERROR("invalid touch event num(%d)", event_num);
			return -EIO;
		}

		ts_data->touch_event_num = event_num;
		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_touch_num) {
				FTS_ERROR("touch point ID(%d) beyond max_touch_number(%d)",
						  pointid, max_touch_num);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) \
						  + (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) \
						  + (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
			events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			if (events[i].p <= 0)
				events[i].p = 0x3F;
			if (events[i].area <= 0)
				events[i].area = 0x09;
		}

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;

	case TOUCH_EXTRA_MSG:
		if (!ts_data->touch_analysis_support) {
			FTS_ERROR("touch_analysis is disabled");
			return -EINVAL;
		}

		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!event_num || (event_num > max_touch_num)) {
			FTS_ERROR("invalid touch event num(%d)", event_num);
			return -EIO;
		}

		ts_data->touch_event_num = event_num;
		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 4;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_touch_num) {
				FTS_ERROR("touch point ID(%d) beyond max_touch_number(%d)",
						  pointid, max_touch_num);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) \
						  + (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) \
						  + (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
			events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			if (events[i].p <= 0)
				events[i].p = 0x3F;
			if (events[i].area <= 0)
				events[i].area = 0x09;
		}

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;

	case TOUCH_GESTURE:
		if (0 == fts_gesture_readdata(ts_data, touch_buf)) {
			FTS_INFO("succuss to get gesture data in irq handler");
		}
		break;

	case TOUCH_FW_INIT:
		fts_release_all_finger();
		fts_tp_state_recovery(ts_data);
		break;

	case TOUCH_IGNORE:
	case TOUCH_ERROR:
		break;

	default:
		FTS_INFO("unknown touch event(%d)", touch_etype);
		break;
	}

	if (ts_data->palm_sensor_switch)
		fts_read_palm_data();

	return 0;
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
	struct fts_ts_data *ts_data = fts_data;
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	int ret = 0;

	if ((ts_data->suspended) && (ts_data->pm_suspend)) {
		ret = wait_for_completion_timeout(
				  &ts_data->pm_completion,
				  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
		if (!ret) {
			FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
			return IRQ_HANDLED;
		}
	}
#endif

	ts_data->intr_jiffies = jiffies;
	fts_prc_queue_work(ts_data);
	fts_irq_read_report(ts_data);
	if (ts_data->touch_analysis_support && ts_data->ta_flag) {
		ts_data->ta_flag = 0;
		if (ts_data->ta_buf && ts_data->ta_size)
			memcpy(ts_data->ta_buf, ts_data->touch_buf, ts_data->ta_size);
		wake_up_interruptible(&ts_data->ts_waitqueue);
	}

	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
	ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
							   pdata->irq_gpio_flags,
							   FTS_DRIVER_NAME, ts_data);

	return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int key_num = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev;

	FTS_FUNC_ENTER();
	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	if (ts_data->bus_type == BUS_TYPE_I2C)
		input_dev->id.bustype = BUS_I2C;
	else
		input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = ts_data->dev;

	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->have_key) {
		FTS_INFO("set key capabilities");
		for (key_num = 0; key_num < pdata->key_number; key_num++)
			input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
	}

	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	/* FOD */
	input_set_capability(input_dev, EV_KEY, BTN_INFO);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, pdata->x_min, pdata->x_max - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, pdata->x_min, pdata->x_max - 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed");
		input_set_drvdata(input_dev, NULL);
		input_free_device(input_dev);
		input_dev = NULL;
		return ret;
	}

	ts_data->input_dev = input_dev;
	FTS_FUNC_EXIT();
	return 0;
}

static int fts_buffer_init(struct fts_ts_data *ts_data)
{
	ts_data->touch_buf = (u8 *)kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
	if (!ts_data->touch_buf) {
		FTS_ERROR("failed to alloc memory for touch buf");
		return -ENOMEM;
	}

	ts_data->touch_size = FTS_SIZE_DEFAULT;

	ts_data->touch_analysis_support = 0;
	ts_data->ta_flag = 0;
	ts_data->ta_size = 0;

	return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl) {
		devm_pinctrl_put(ts->pinctrl);
	}
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			FTS_ERROR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d", ret);
		}
	}

	return ret;
}
#endif /* FTS_PINCTRL_EN */

/*
 * sapphire: the panel supply comes from avdd-gpio; only the optional
 * vcc_i2c regulator is switched.
 */
static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	if (enable) {
		if (ts_data->power_disabled) {
			FTS_DEBUG("regulator enable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			gpio_direction_output(ts_data->pdata->avdd_gpio, 1);
			if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
				ret = regulator_enable(ts_data->vcc_i2c);
				if (ret) {
					FTS_ERROR("enable vcc_i2c regulator failed,ret=%d", ret);
				}
			}
			ts_data->power_disabled = false;
		}
	} else {
		if (!ts_data->power_disabled) {
			FTS_DEBUG("regulator disable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			gpio_direction_output(ts_data->pdata->avdd_gpio, 0);
			if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
				ret = regulator_disable(ts_data->vcc_i2c);
				if (ret) {
					FTS_ERROR("disable vcc_i2c regulator failed,ret=%d", ret);
				}
			}
			ts_data->power_disabled = true;
		}
	}

	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*		vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*		Must be call after fts_gpio_configure() execute,because this function
*		will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ts_data->vcc_i2c = regulator_get(ts_data->dev, "vcc_i2c");
	if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
		if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
			ret = regulator_set_voltage(ts_data->vcc_i2c,
										FTS_I2C_VTG_MIN_UV,
										FTS_I2C_VTG_MAX_UV);
			if (ret) {
				FTS_ERROR("vcc_i2c regulator set_vtg failed,ret=%d", ret);
				regulator_put(ts_data->vcc_i2c);
				/* stock keeps using the released regulator */
				ts_data->vcc_i2c = NULL;
			}
		}
	}

#if FTS_PINCTRL_EN
	fts_pinctrl_init(ts_data);
	fts_pinctrl_select_normal(ts_data);
#endif

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret) {
		FTS_ERROR("fail to enable power(regulator)");
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_PARAM_ERR, 0,
				"TpParamParseFail", "focal", ERROR_REGULATOR_INIT);
	}

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif

	fts_power_source_ctrl(ts_data, DISABLE);

	if (!IS_ERR_OR_NULL(ts_data->vdd)) {
		if (regulator_count_voltages(ts_data->vdd) > 0)
			regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
		regulator_put(ts_data->vdd);
	}

	if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
		if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
			regulator_set_voltage(ts_data->vcc_i2c, 0, FTS_I2C_VTG_MAX_UV);
		regulator_put(ts_data->vcc_i2c);
	}

	return 0;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
	int ret = 0;

#if FTS_PINCTRL_EN
	fts_pinctrl_select_normal(ts_data);
#endif
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret < 0) {
		FTS_ERROR("power on fail, ret=%d", ret);
	}

	return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->pdata->irq_gpio);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->reset_gpio, 0);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	/* request avdd gpio (panel/touch supply enable) */
	if (gpio_is_valid(data->pdata->avdd_gpio)) {
		ret = gpio_request(data->pdata->avdd_gpio, "fts_avdd_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]vdd gpio avdd failed");
			goto err_reset_gpio_dir;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	FTS_FUNC_EXIT();
	return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
							 struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		FTS_ERROR("invalid:%s, size:%d", name, coords_size);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, name, coords, coords_size);
	if (ret < 0) {
		FTS_ERROR("Unable to read %s, please check dts", name);
		pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
		pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
		pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
		pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
		return -ENODATA;
	} else {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}

	FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
			 pdata->y_min, pdata->y_max);
	return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	u32 temp_val = 0;

	FTS_FUNC_ENTER();

	ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (ret < 0)
		FTS_ERROR("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key number undefined!");

		ret = of_property_read_u32_array(np, "focaltech,keys",
										 pdata->keys, pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Keys undefined!");
		else if (pdata->key_number > FTS_MAX_KEYS)
			pdata->key_number = FTS_MAX_KEYS;

		ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
										 pdata->key_x_coords,
										 pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key Y Coords undefined!");

		ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
										 pdata->key_y_coords,
										 pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key X Coords undefined!");

		FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
				 "coords:(%d,%d),(%d,%d),(%d,%d)",
				 pdata->key_number,
				 pdata->keys[0], pdata->keys[1], pdata->keys[2],
				 pdata->key_x_coords[0], pdata->key_y_coords[0],
				 pdata->key_x_coords[1], pdata->key_y_coords[1],
				 pdata->key_x_coords[2], pdata->key_y_coords[2]);
	}

	/* reset, irq, avdd gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
						0, &pdata->reset_gpio_flags);
	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
					  0, &pdata->irq_gpio_flags);
	pdata->avdd_gpio = of_get_named_gpio_flags(np, "focaltech,avdd-gpio",
					  0, &pdata->avdd_gpio_flags);

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get max-touch-number, please check dts");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		if (temp_val < 2)
			pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
			 pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

	FTS_FUNC_EXIT();
	return 0;
}

static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
								  resume_work);

	fts_ts_resume(ts_data->dev);
}

/*
 * Stock queues this work for DRM_PANEL_EVENT_BLANK_LP but never initialises
 * it (that would call a NULL work function); here it runs the suspend path.
 */
static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
								  suspend_work);

	fts_ts_suspend(ts_data->dev);
}

/*
 * FOD enabled while the IC sits in deep sleep (0xD0 == 0): reset it and put
 * it into gesture mode so the fingerprint area keeps reporting.
 */
static void fts_sleep2gesture_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
								  sleep2gesture_work);

	fts_irq_disable();
	fts_reset_proc(50);
	fts_tp_state_recovery(ts_data);
	fts_irq_enable();
	if (ts_data->suspended) {
		ts_data->gesture_mode = ENABLE;
		fts_gesture_suspend(ts_data);
	}
}

static int drm_check_dt(struct device_node *np)
{
	int i = 0;
	int count = 0;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0) {
		FTS_ERROR("find drm_panel count(%d) fail", count);
		return -ENODEV;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			FTS_INFO("find drm_panel successfully");
			active_panel = panel;
			return 0;
		}
	}

	FTS_ERROR("no find drm_panel");
	return PTR_ERR(panel);
}

static int fts_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}

	if (ts_data->palm_sensor_switch) {
		FTS_INFO("palm sensor ON, switch to OFF");
		update_palm_sensor_value(0);
		fts_palm_sensor_cmd(0);
	}

	/* keep the IC in gesture mode for FOD, double-tap or AOD single-tap */
	ts_data->gesture_mode = ts_data->fod_enabled || ts_data->lct_gesture_mode ||
							ts_data->aod_changed;

	if ((ts_data->fod_status <= 0) || (ts_data->fod_status == 100)) {
		ret = fts_fod_reg_write(0x02, false);
		if (ret < 0)
			FTS_ERROR("%s fts_fod_reg_write failed", __func__);
	}
	if ((ts_data->fod_status > 0) && (ts_data->fod_status != 100)) {
		FTS_INFO("set CF reg");
		ret = fts_fod_reg_write(0x02, true);
		if (ret < 0)
			FTS_ERROR("%s fts_write_reg CF failed", __func__);
		ret = fts_gesture_reg_write(0x01, true);
		if (ret < 0)
			FTS_ERROR("%s fts_fod_reg_write failed\n", __func__);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend(ts_data);
#endif

	FTS_INFO("gesture_support:%d  poweroff_on_sleep:%d ",
			 ts_data->gesture_mode, ts_data->poweroff_on_sleep);
	if (ts_data->gesture_mode && !ts_data->poweroff_on_sleep) {
		fts_gesture_suspend(ts_data);
	} else {
		fts_irq_disable();

		FTS_INFO("make TP enter into sleep mode");
		ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
		if (ret < 0)
			FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);
	}

	fts_release_all_finger();
	ts_data->suspended = true;
	FTS_FUNC_EXIT();
	return 0;
}

static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_INFO("Already in awake state");
		return 0;
	}

	ts_data->suspended = false;
	FTS_INFO("finger_in_fod:%d fod_finger_skip:%d  ts_data->fod_pressed:%d",
			 ts_data->finger_in_fod, ts_data->fod_finger_skip,
			 ts_data->fod_pressed);
	if (!ts_data->finger_in_fod && !ts_data->fod_finger_skip &&
		!ts_data->fod_pressed) {
		FTS_INFO("resume reset");
		fts_reset_proc(50);
		fts_recover_after_reset();
		fts_release_all_finger();
		update_fod_press_status(0);
	}
	ts_data->fod_pressed = false;

	if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
		fts_power_source_resume(ts_data);
#endif
	}

	fts_wait_tp_to_valid();
	fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume(ts_data);
#endif

	if (ts_data->palm_sensor_switch) {
		fts_palm_sensor_cmd(1);
		FTS_INFO("palm sensor OFF, switch to ON");
	}

	FTS_INFO("gesture_support:%d  poweroff_on_sleep:%d",
			 ts_data->gesture_mode, ts_data->poweroff_on_sleep);
	if (ts_data->gesture_mode && !ts_data->poweroff_on_sleep)
		fts_gesture_resume(ts_data);
	ts_data->poweroff_on_sleep = false;

	if (get_lct_tp_work_status())
		fts_irq_enable();
	else
		FTS_ERROR("Touchscreen Disabled, Can't enable irq.");

	FTS_INFO("make TP enter resume mode");
	if (g_touchscreen_usb_pulgin.valid)
		g_touchscreen_usb_pulgin.event_callback();

	FTS_FUNC_EXIT();
	return 0;
}

static void fts_ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
		struct panel_event_notification *notification, void *client_data)
{
	struct fts_ts_data *ts_data = client_data;

	if (!notification) {
		FTS_ERROR("Invalid notification\n");
		return;
	}

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		if (!notification->notif_data.early_trigger)
			return;
		FTS_INFO("%s: FB_BLANK_UNBLANK", __func__);
		if (!ts_data->suspended)
			return;
		FTS_INFO("%s: FB_BLANK_UNBLANK, resume function will be called", __func__);
		flush_workqueue(ts_data->ts_workqueue);
		queue_work(ts_data->ts_workqueue, &ts_data->resume_work);
		break;
	case DRM_PANEL_EVENT_BLANK:
		if (!notification->notif_data.early_trigger)
			return;
		cancel_work_sync(&fts_data->resume_work);
		fts_ts_suspend(ts_data->dev);
		break;
	case DRM_PANEL_EVENT_BLANK_LP:
		if (notification->notif_data.early_trigger)
			return;
		FTS_INFO("%s: FB_BLANK %s", __func__, "POWER DOWN");
		if (ts_data->suspended)
			return;
		FTS_INFO("%s: FB_BLANK %s, suspend function will be called", __func__,
				 (notification->notif_type == DRM_PANEL_EVENT_BLANK_LP) ? "POWER DOWN" : "LP");
		flush_workqueue(ts_data->ts_workqueue);
		queue_work(ts_data->ts_workqueue, &ts_data->suspend_work);
		break;
	default:
		break;
	}
}

static void fts_ts_register_for_panel_events(struct fts_ts_data *ts_data)
{
	void *cookie = NULL;

	cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, active_panel,
			fts_ts_panel_notifier_callback, ts_data);
	/* stock only tests for NULL, but the notifier returns ERR_PTR on failure */
	if (IS_ERR_OR_NULL(cookie)) {
		FTS_ERROR("Failed to register for panel events\n");
		return;
	}

	FTS_INFO("registered for panel notifications panel: 0x%x\n", active_panel);
	ts_data->notifier_cookie = cookie;
}

/*****************************************************************************
*  Xiaomi touch feature (xiaomi_touch_game) interface
*****************************************************************************/
static int fts_palm_sensor_write(int value)
{
	int ret = 0;

	if (!fts_data)
		return -EINVAL;

	fts_data->palm_sensor_switch = value;
	if (fts_data->suspended)
		return 0;

	ret = fts_palm_sensor_cmd(value);
	if (ret < 0)
		FTS_ERROR("set palm sensor cmd failed: %d\n", value);

	return ret;
}

static void fts_init_touchmode_data(void)
{
	int i;

	FTS_INFO("ENTER into");
	/* Touch Game Mode Switch */
	xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][GET_MAX_VALUE] = 1;
	xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][GET_DEF_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][SET_CUR_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][GET_CUR_VALUE] = 0;

	/* Active Mode */
	xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][GET_MAX_VALUE] = 1;
	xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][GET_DEF_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][SET_CUR_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][GET_CUR_VALUE] = 0;

	/* sensivity */
	xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][GET_MAX_VALUE] = 4;
	xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][GET_DEF_VALUE] = 2;
	xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][SET_CUR_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][GET_CUR_VALUE] = 0;

	/*  Tolerance */
	xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][GET_MAX_VALUE] = 4;
	xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][GET_DEF_VALUE] = 2;
	xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][SET_CUR_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][GET_CUR_VALUE] = 0;

	/* edge filter orientation */
	xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][GET_MAX_VALUE] = 3;
	xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][GET_DEF_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][SET_CUR_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][GET_CUR_VALUE] = 0;

	/* edge filter area */
	xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][GET_MAX_VALUE] = 3;
	xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][GET_MIN_VALUE] = 0;
	xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][GET_DEF_VALUE] = 2;
	xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][SET_CUR_VALUE] = 2;
	xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][GET_CUR_VALUE] = 2;

	for (i = 0; i < Touch_Mode_NUM; i++) {
		FTS_INFO("mode:%d, set cur:%d, get cur:%d, def:%d min:%d max:%d",
				 i,
				 xiaomi_touch_interfaces.touch_mode[i][SET_CUR_VALUE],
				 xiaomi_touch_interfaces.touch_mode[i][GET_CUR_VALUE],
				 xiaomi_touch_interfaces.touch_mode[i][GET_DEF_VALUE],
				 xiaomi_touch_interfaces.touch_mode[i][GET_MIN_VALUE],
				 xiaomi_touch_interfaces.touch_mode[i][GET_MAX_VALUE]);
	}
}

static int fts_set_cur_value(int fts_mode, int fts_value)
{
	u8 fts_game_value[2] = { 0 };
	u8 temp_value = 0;
	u8 reg_value = 0;
	u8 gesture_en = 0;
	struct fts_ts_data *ts_data = fts_data;
	int *cur;

	if ((fts_mode >= Touch_Mode_NUM) || (fts_mode < 0)) {
		FTS_ERROR("fts mode not support:%d", fts_mode);
		return 0;
	}

	cur = &xiaomi_touch_interfaces.touch_mode[fts_mode][SET_CUR_VALUE];
	if (*cur > xiaomi_touch_interfaces.touch_mode[fts_mode][GET_MAX_VALUE])
		*cur = xiaomi_touch_interfaces.touch_mode[fts_mode][GET_MAX_VALUE];
	else if (*cur < xiaomi_touch_interfaces.touch_mode[fts_mode][GET_MIN_VALUE])
		*cur = xiaomi_touch_interfaces.touch_mode[fts_mode][GET_MIN_VALUE];

	if ((fts_mode == Touch_FodIcon_Enable) && (fts_value >= 0)) {
		FTS_INFO("Mode:FodIcon  fod_icon_status = %d", fts_value);
		ts_data->fod_icon_status = fts_value;
		return 0;
	}

	if ((fts_mode == Touch_Nonui_Mode) && (fts_value >= 0)) {
		FTS_INFO("Mode:Nonui_Mode  nonui_status = %d", fts_value);
		ts_data->nonui_status = fts_value;
		return 0;
	}

	if ((fts_mode == Touch_Fod_Enable) && (fts_value >= 0)) {
		FTS_INFO("Mode:FOD  fod_status = %d", fts_value);
		ts_data->fod_status = fts_value;
		if (fts_value) {
			ts_data->fod_enabled = true;
		} else {
			ts_data->fod_enabled = false;
			if (!ts_data->finger_in_fod)
				fts_fod_reg_write(0x02, false);
		}

		if (!fts_value && ts_data->finger_in_fod)
			goto out_fod;

		if (ts_data->fod_status == 2)
			fts_fod_recovery();

		if (ts_data->fod_status == 1) {
			if (ts_data->suspended) {
				flush_workqueue(ts_data->ts_workqueue);
				fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
				FTS_INFO("1.FTS_REG_GESTURE_EN = %d", gesture_en);
				if (gesture_en == 0) {
					flush_workqueue(fts_data->ts_workqueue);
					fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
					FTS_INFO("2.FTS_REG_GESTURE_EN = %d", gesture_en);
					if (gesture_en == 0) {
						queue_work(fts_data->ts_workqueue,
								   &fts_data->sleep2gesture_work);
						return 0;
					}
				}
			}
			fts_fod_recovery();
			return 0;
		}
out_fod:
		*cur = fts_value;
		goto out;
	}

	*cur = fts_value;

	switch (fts_mode) {
	case Touch_Active_MODE:
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_Active_MODE][SET_CUR_VALUE];
		reg_value = (temp_value == 1);
		fts_write_reg(FTS_REG_GAME_ACTIVE, reg_value);
		fts_game_value[0] = 0x90;
		fts_game_value[1] = reg_value;
		break;

	case Touch_UP_THRESHOLD:
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_UP_THRESHOLD][SET_CUR_VALUE];
		if ((temp_value >= 1) && (temp_value <= 4))
			reg_value = temp_value;
		else
			reg_value = 0;
		fts_game_value[0] = 0x81;
		fts_game_value[1] = reg_value;
		break;

	case Touch_Tolerance:
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_Tolerance][SET_CUR_VALUE];
		if ((temp_value >= 1) && (temp_value <= 4))
			reg_value = temp_value;
		else
			reg_value = 0;
		fts_game_value[0] = 0x85;
		fts_game_value[1] = reg_value;
		break;

	case Touch_Edge_Filter:
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_Edge_Filter][SET_CUR_VALUE];
		if (temp_value <= 3)
			reg_value = temp_value + 1;
		else
			reg_value = 0;
		fts_game_value[0] = 0x8D;
		fts_game_value[1] = reg_value;
		break;

	case Touch_Panel_Orientation:
		/* 0,1,2,3 = 0, 90, 180,270 */
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_Panel_Orientation][SET_CUR_VALUE];
		if (temp_value == 1)
			reg_value = 0x01;
		else if (temp_value == 3)
			reg_value = 0x02;
		else
			reg_value = 0;
		fts_game_value[0] = 0x8C;
		fts_game_value[1] = reg_value;
		break;

	case Touch_Aod_Enable:
		temp_value = xiaomi_touch_interfaces.touch_mode[Touch_Aod_Enable][SET_CUR_VALUE];
		if (temp_value == 1)
			ts_data->aod_changed = ENABLE;
		else
			ts_data->aod_changed = DISABLE;
		FTS_INFO("temp_value:%d,aod_changed:%d", temp_value, ts_data->aod_changed);
		break;

	case Touch_Game_Mode:
	case Touch_Wgh_Min:
	case Touch_Wgh_Max:
	case Touch_Wgh_Step:
	case Touch_Report_Rate:
	case Touch_Fod_Enable:
	default:
		/* Don't support */
		break;
	}

out:
	FTS_INFO("mode:%d, value:%d,temp_value:%d, game value:0x%x,0x%x",
			 fts_mode, fts_value, temp_value, fts_game_value[0], fts_game_value[1]);

	xiaomi_touch_interfaces.touch_mode[fts_mode][GET_CUR_VALUE] =
		xiaomi_touch_interfaces.touch_mode[fts_mode][SET_CUR_VALUE];
	if (xiaomi_touch_interfaces.touch_mode[Touch_Game_Mode][SET_CUR_VALUE]) {
		fts_write_reg(fts_game_value[0], fts_game_value[1]);
		FTS_INFO("fts_game_value[0]:%d,fts_game_value[1]:%d",
				 fts_game_value[0], fts_game_value[1]);
	}

	return 0;
}

static int fts_get_mode_value(int mode, int value_type)
{
	int value = -1;

	if ((mode < Touch_Mode_NUM) && (mode >= 0))
		value = xiaomi_touch_interfaces.touch_mode[mode][value_type];
	else
		FTS_ERROR("don't support\n");

	return value;
}

static int fts_get_mode_all(int mode, int *value)
{
	if ((mode < Touch_Mode_NUM) && (mode >= 0)) {
		value[0] = xiaomi_touch_interfaces.touch_mode[mode][GET_CUR_VALUE];
		value[1] = xiaomi_touch_interfaces.touch_mode[mode][GET_DEF_VALUE];
		value[2] = xiaomi_touch_interfaces.touch_mode[mode][GET_MIN_VALUE];
		value[3] = xiaomi_touch_interfaces.touch_mode[mode][GET_MAX_VALUE];
	} else {
		FTS_ERROR("don't support\n");
	}
	FTS_INFO("mode:%d, value:%d:%d:%d:%d", mode, value[0], value[1], value[2], value[3]);

	return 0;
}

static int fts_reset_mode(int mode)
{
	int i = 0;

	FTS_INFO("fts_reset_game_mode enter");

	if ((mode < Touch_Mode_NUM) && (mode > 0)) {
		xiaomi_touch_interfaces.touch_mode[mode][SET_CUR_VALUE] =
			xiaomi_touch_interfaces.touch_mode[mode][GET_DEF_VALUE];
		fts_set_cur_value(mode, xiaomi_touch_interfaces.touch_mode[mode][SET_CUR_VALUE]);
	} else if (mode == 0) {
		for (i = Touch_Mode_NUM - 1; i >= 0; i--) {
			if ((i == Touch_Game_Mode) || (i == Touch_Active_MODE) ||
				(i == Touch_UP_THRESHOLD) || (i == Touch_Tolerance) ||
				(i == Touch_Edge_Filter)) {
				xiaomi_touch_interfaces.touch_mode[i][SET_CUR_VALUE] =
					xiaomi_touch_interfaces.touch_mode[i][GET_DEF_VALUE];
				fts_set_cur_value(i, xiaomi_touch_interfaces.touch_mode[i][SET_CUR_VALUE]);
			}
		}
	} else {
		FTS_ERROR("don't support");
	}

	FTS_ERROR("mode:%d", mode);

	return 0;
}

static void fts_touch_dfs_test(int value)
{
	switch (value) {
	case TOUCH_EVENT_TRANSFER_ERR:
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_TRANSFER_ERR, 0,
				"TpTransferErr", "focal", -1);
		break;
	case TOUCH_EVENT_FWLOAD_ERR:
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_FWLOAD_ERR, 0,
				"TpFirmwareLoadFail", "focal", -1);
		break;
	case TOUCH_EVENT_PARAM_ERR:
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_PARAM_ERR, 0,
				"TpParamParseFail", "focal", ERROR_DTS_PARSE);
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_PARAM_ERR, 0,
				"TpParamParseFail", "focal", ERROR_REGULATOR_INIT);
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_PARAM_ERR, 0,
				"TpParamParseFail", "focal", ERROR_GPIO_REQUEST);
		break;
	case TOUCH_EVENT_OPENTEST_FAIL:
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_OPENTEST_FAIL, 0,
				"TpOpenTestFail", "focal", -1);
		break;
	case TOUCH_EVENT_SHORTTEST_FAIL:
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_SHORTTEST_FAIL, 0,
				"TpShortTestFail", "focal", -1);
		break;
	default:
		FTS_ERROR("don't support touch dfs test");
		break;
	}
}

/* /sys/class/touch/tp_dev/fod_test: simulate a FOD press (1) / release (0) */
static ssize_t fts_fod_test_store(struct device *dev,
								  struct device_attribute *attr,
								  const char *buf, size_t count)
{
	int value = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	FTS_INFO("%s,buf:%s,count:%u\n", __func__, buf, (unsigned int)count);
	if (kstrtoint(buf, 10, &value))
		return -ENODEV;

	if (value) {
		input_report_key(ts_data->input_dev, BTN_INFO, 1);
		update_fod_press_status(1);
		input_sync(ts_data->input_dev);
		input_mt_slot(ts_data->input_dev, 0);
		input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 1);
		input_report_key(ts_data->input_dev, BTN_TOUCH, 1);
		input_report_key(ts_data->input_dev, BTN_TOOL_FINGER, 1);
		input_report_abs(ts_data->input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(ts_data->input_dev, ABS_MT_WIDTH_MINOR, 1);
		input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X, 8640);
		input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y, 33744);
	} else {
		input_mt_slot(ts_data->input_dev, 0);
		input_report_abs(ts_data->input_dev, ABS_MT_WIDTH_MINOR, 0);
		input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 0);
		input_report_abs(ts_data->input_dev, ABS_MT_TRACKING_ID, -1);
		input_report_key(ts_data->input_dev, BTN_INFO, 0);
		update_fod_press_status(0);
	}
	input_sync(ts_data->input_dev);

	return count;
}

static DEVICE_ATTR(fod_test, (S_IRUGO | S_IWUSR), NULL, fts_fod_test_store);

static void fts_xiaomi_touch_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	if (ts_data->fts_tp_class)
		return;

	ts_data->fts_tp_class = get_xiaomi_touch_class();
	if (!ts_data->fts_tp_class)
		return;

	ts_data->fts_touch_dev = device_create(ts_data->fts_tp_class, NULL,
										   FTS_TP_DEV_MINOR, ts_data, "tp_dev");
	if (IS_ERR(ts_data->fts_touch_dev)) {
		FTS_ERROR("Failed to create device !\n");
		goto err_device;
	}
	dev_set_drvdata(ts_data->fts_touch_dev, ts_data);

	ret = sysfs_create_file(&ts_data->fts_touch_dev->kobj, &dev_attr_fod_test.attr);
	if (ret) {
		FTS_ERROR("Failed to create fod_test sysfs group!\n");
		goto err_device;
	}

	memset(&xiaomi_touch_interfaces, 0, sizeof(struct xiaomi_touch_interface));
	xiaomi_touch_interfaces.getModeValue = fts_get_mode_value;
	xiaomi_touch_interfaces.setModeValue = fts_set_cur_value;
	xiaomi_touch_interfaces.palm_sensor_write = fts_palm_sensor_write;
	xiaomi_touch_interfaces.getModeAll = fts_get_mode_all;
	xiaomi_touch_interfaces.resetMode = fts_reset_mode;
	xiaomi_touch_interfaces.touch_dfs_test = fts_touch_dfs_test;
	fts_init_touchmode_data();
	xiaomitouch_register_modedata(&xiaomi_touch_interfaces);
	return;

	/*
	 * Stock tears the whole probe down here (after the IRQ is live) and
	 * then still reports success; only drop the xiaomi node instead.
	 */
err_device:
	device_destroy(ts_data->fts_tp_class, FTS_TP_DEV_MINOR);
	ts_data->fts_touch_dev = NULL;
	ts_data->fts_tp_class = NULL;
}

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int pdata_size = sizeof(struct fts_ts_platform_data);

	FTS_FUNC_ENTER();
	FTS_INFO("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
	if (!ts_data->pdata) {
		FTS_ERROR("allocate memory for platform_data fail");
		return -ENOMEM;
	}

	if (ts_data->dev->of_node) {
		ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
		if (ret)
			FTS_ERROR("device-tree parse fail");

		ret = drm_check_dt(ts_data->dev->of_node);
		if (ret) {
			FTS_ERROR("parse drm-panel fail");
		}
	} else {
		if (ts_data->dev->platform_data) {
			memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
		} else {
			FTS_ERROR("platform_data is null");
			ret = -ENODEV;
			goto err_platform_data;
		}
	}

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue) {
		FTS_ERROR("create fts workqueue fail");
	}

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);
	init_waitqueue_head(&ts_data->ts_waitqueue);
	mutex_init(&ts_data->fod_mutex);
	ts_data->fod_pressed = false;
	ts_data->fod_status = -1;
	ts_data->fod_icon_status = 1;
	ts_data->old_point_id = 0xFF;

	/* Init communication interface */
	ret = fts_bus_init(ts_data);
	if (ret) {
		FTS_ERROR("bus initialize fail");
		goto err_bus_init;
	}

	ret = fts_input_init(ts_data);
	if (ret) {
		FTS_ERROR("input initialize fail");
		goto err_input_init;
	}

	ret = fts_buffer_init(ts_data);
	if (ret) {
		FTS_ERROR("buffer init fail");
		goto err_buffer_init;
	}

	ret = fts_gpio_configure(ts_data);
	if (ret) {
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_PARAM_ERR, 0,
				"TpParamParseFail", "focal", ERROR_GPIO_REQUEST);
		FTS_ERROR("configure the gpios fail");
		goto err_gpio_config;
	}

#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_init(ts_data);
	if (ret) {
		FTS_ERROR("fail to get power(regulator)");
		goto err_power_init;
	}
#endif

	ret = gpio_direction_output(ts_data->pdata->reset_gpio, 1);
	if (ret)
		FTS_ERROR("set gpio reset to high failed");
	msleep(100);

	if (ts_data->fod_status > 0) {
		FTS_INFO("fod_status = %d\n", ts_data->fod_status);
		fts_fod_recovery();
	}

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_TRANSFER_ERR, 0,
				"TpTransferErr", "focal", ret);
		FTS_ERROR("not focal IC, unregister driver");
		goto err_irq_req;
	}

	ret = fts_create_apk_debug_channel(ts_data);
	if (ret) {
		FTS_ERROR("create apk debug node fail");
	}

	ret = lct_create_procfs(ts_data);
	if (ret < 0) {
		FTS_ERROR("create procfs node fail");
	}

	ret = fts_create_sysfs(ts_data);
	if (ret) {
		FTS_ERROR("create sysfs node fail");
	}

	ret = fts_point_report_check_init(ts_data);
	if (ret) {
		FTS_ERROR("init point report check fail");
	}

	ret = fts_ex_mode_init(ts_data);
	if (ret) {
		FTS_ERROR("init glove/cover/charger fail");
	}

	ret = fts_gesture_init(ts_data);
	if (ret) {
		FTS_ERROR("init gesture fail");
	}

#if FTS_TEST_EN
	ret = fts_test_init(ts_data);
	if (ret) {
		FTS_ERROR("init host test fail");
	}
#endif

#if FTS_ESDCHECK_EN
	ret = fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
	}
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_irq_req;
	}

	ret = fts_fwupg_init(ts_data);
	if (ret) {
		xiaomi_touch_mievent_report_int(TOUCH_EVENT_FWLOAD_ERR, 0,
				"TpFirmwareLoadFail", "focal", ret);
		FTS_ERROR("init fw upgrade fail");
	}

	if (ts_data->ts_workqueue) {
		INIT_WORK(&ts_data->resume_work, fts_resume_work);
		INIT_WORK(&ts_data->suspend_work, fts_suspend_work);
		INIT_WORK(&ts_data->sleep2gesture_work, fts_sleep2gesture_work);
	}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	init_completion(&ts_data->pm_completion);
	ts_data->pm_suspend = false;
#endif

	if (active_panel)
		fts_ts_register_for_panel_events(ts_data);

	g_touchscreen_usb_pulgin.event_callback = fts_ts_usb_event_callback;

	fts_xiaomi_touch_init(ts_data);

	FTS_FUNC_EXIT();
	return 0;

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
	fts_power_source_exit(ts_data);
#endif
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
	if (gpio_is_valid(ts_data->pdata->avdd_gpio))
		gpio_free(ts_data->pdata->avdd_gpio);
err_gpio_config:
	kfree_safe(ts_data->touch_buf);
err_buffer_init:
	input_unregister_device(ts_data->input_dev);
err_input_init:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
	kfree_safe(ts_data->bus_tx_buf);
	kfree_safe(ts_data->bus_rx_buf);
err_platform_data:
	kfree_safe(ts_data->pdata);
	FTS_FUNC_EXIT();
	return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	cancel_work_sync(&fts_data->resume_work);
	fts_point_report_check_exit(ts_data);
	fts_release_apk_debug_channel(ts_data);
	lct_remove_procfs(ts_data);
	fts_remove_sysfs(ts_data);
	fts_ex_mode_exit(ts_data);

	fts_fwupg_exit(ts_data);

#if FTS_TEST_EN
	fts_test_exit(ts_data);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif

	fts_gesture_exit(ts_data);

	free_irq(ts_data->irq, ts_data);

	fts_bus_exit(ts_data);

	input_unregister_device(ts_data->input_dev);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

	if (active_panel && ts_data->notifier_cookie)
		panel_event_notifier_unregister(ts_data->notifier_cookie);

	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

	if (gpio_is_valid(ts_data->pdata->avdd_gpio))
		gpio_free(ts_data->pdata->avdd_gpio);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
#endif

	if (ts_data->fts_tp_class)
		device_destroy(ts_data->fts_tp_class, FTS_TP_DEV_MINOR);
	ts_data->fts_tp_class = NULL;

	kfree_safe(ts_data->touch_buf);
	kfree_safe(ts_data->pdata);
	kfree_safe(ts_data);

	FTS_FUNC_EXIT();

	return 0;
}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	ts_data->pm_suspend = true;
	reinit_completion(&ts_data->pm_completion);
	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	ts_data->pm_suspend = false;
	complete(&ts_data->pm_completion);
	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;

	FTS_INFO("Touch Screen(I2C BUS) driver prboe...");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		return -ENODEV;
	}

	/* malloc memory for global struct variable */
	ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("allocate memory for fts_data fail");
		return -ENOMEM;
	}

	fts_data = ts_data;
	ts_data->client = client;
	ts_data->dev = &client->dev;
	ts_data->log_level = 1;
	ts_data->fw_is_running = 0;
	ts_data->bus_type = BUS_TYPE_I2C;
	ts_data->poweroff_on_sleep = false;
	i2c_set_clientdata(client, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen(I2C BUS) driver probe fail");
		kfree_safe(ts_data);
		return ret;
	}

	FTS_INFO("Touch Screen(I2C BUS) driver prboe successfully");
	return 0;
}

static int fts_ts_remove(struct i2c_client *client)
{
	return fts_ts_remove_entry(i2c_get_clientdata(client));
}

static const struct i2c_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};
static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,fts", },
	{},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
		.pm = &fts_dev_pm_ops,
#endif
		.of_match_table = of_match_ptr(fts_dt_match),
	},
	.id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret != 0) {
		FTS_ERROR("Focaltech touch screen driver init failed!");
	}
	FTS_FUNC_EXIT();
	return ret;
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}
module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: msm_drm");
