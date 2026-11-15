/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define KEY_GESTURE_UP						  KEY_UP
#define KEY_GESTURE_DOWN						KEY_DOWN
#define KEY_GESTURE_LEFT						KEY_LEFT
#define KEY_GESTURE_RIGHT					   KEY_RIGHT
#define KEY_GESTURE_O						   KEY_O
#define KEY_GESTURE_E						   KEY_E
#define KEY_GESTURE_M						   KEY_M
#define KEY_GESTURE_L						   KEY_L
#define KEY_GESTURE_W						   KEY_W
#define KEY_GESTURE_S						   KEY_S
#define KEY_GESTURE_V						   KEY_V
#define KEY_GESTURE_C						   KEY_C
#define KEY_GESTURE_Z						   KEY_Z
#define KEY_GESTURE_AOD						   KEY_GOTO

#define GESTURE_DOUBLECLICK					 0x24
#define GESTURE_FOD							   0x26
#define GESTURE_AOD							   0x27
#define	WAKEUP_OFF							   0x04
#define	WAKEUP_ON							   0x05

#define FTS_REG_FOD_EN						   0xCF
#define FTS_REG_WRITE_RETRY					 5

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* gesture_id	- mean which gesture is recognised
* point_num	 - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
*/
struct fts_gesture_st {
	u8 gesture_id;
	u8 point_num;
	u16 coordinate_x[FTS_GESTURE_POINTS_MAX];
	u16 coordinate_y[FTS_GESTURE_POINTS_MAX];
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	u8 val = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	mutex_lock(&ts_data->input_dev->mutex);
	fts_read_reg(FTS_REG_GESTURE_EN, &val);
	count = snprintf(buf, PAGE_SIZE, "Gesture Mode:%s\n",
					 ts_data->gesture_mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0)=%d\n", val);
	mutex_unlock(&ts_data->input_dev->mutex);

	return count;
}

static ssize_t fts_gesture_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	mutex_lock(&ts_data->input_dev->mutex);
	if (buf[0] == '1')
		lct_fts_tp_gesture_callback(true);
	else if (buf[0] == '0')
		lct_fts_tp_gesture_callback(false);
	mutex_unlock(&ts_data->input_dev->mutex);

	return count;
}

static ssize_t fts_gesture_buf_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct input_dev *input_dev = ts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\n", gesture->gesture_id);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum:%d\n",
					  gesture->point_num);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Points Buffer:\n");

	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
						  gesture->coordinate_x[i], gesture->coordinate_y[i]);
		if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t fts_gesture_buf_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return -EPERM;
}

static ssize_t fts_gesture_bm_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	mutex_lock(&ts_data->input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "gesture bmode:%d\n",
					 ts_data->gesture_bmode);
	mutex_unlock(&ts_data->input_dev->mutex);

	return count;
}

static ssize_t fts_gesture_bm_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int value = 0xFF;
	int ret = 0;

	mutex_lock(&ts_data->input_dev->mutex);
	ret = sscanf(buf, "%d", &value);
	if (ret == 1) {
		FTS_DEBUG("gesture bmode:%d->%d", ts_data->gesture_bmode, value);
		ts_data->gesture_bmode = value;
	}
	mutex_unlock(&ts_data->input_dev->mutex);

	return count;
}

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode	   ---read gesture mode
 *   write example:echo 1 > fts_gesture_mode   --- write gesture mode to 1
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show,
				   fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf		--- read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR,
				   fts_gesture_buf_show, fts_gesture_buf_store);

static DEVICE_ATTR(fts_gesture_bm, S_IRUGO | S_IWUSR,
				   fts_gesture_bm_show, fts_gesture_bm_store);

static struct attribute *fts_gesture_mode_attrs[] = {
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
	&dev_attr_fts_gesture_bm.attr,
	NULL,
};

static struct attribute_group fts_gesture_group = {
	.attrs = fts_gesture_mode_attrs,
};

static int fts_create_gesture_sysfs(struct device *dev)
{
	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
	if (ret) {
		FTS_ERROR("gesture sys node create fail");
		sysfs_remove_group(&dev->kobj, &fts_gesture_group);
		return ret;
	}

	return 0;
}

/*
 * sapphire: only double-tap (wakeup, when enabled through lct_tp gesture)
 * and the single-tap code (AOD, or FOD icon tap while FOD is enabled) are
 * reported.
 */
static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
	int gesture;

	FTS_DEBUG("gesture_id:0x%x", gesture_id);
	switch (gesture_id) {
	case GESTURE_DOUBLECLICK:
		if (!fts_data->lct_gesture_mode)
			return;
		gesture = KEY_WAKEUP;
		FTS_INFO("Gesture Code=%d", gesture);
		break;
	case GESTURE_AOD:
		gesture = KEY_GESTURE_AOD;
		if (fts_data->aod_changed) {
			FTS_INFO("AOD Code=%d", gesture);
		} else if (fts_data->fod_icon_status && fts_data->fod_enabled) {
			FTS_INFO("Click Code=%d", gesture);
		} else {
			return;
		}
		break;
	default:
		return;
	}

	/* report event key */
	input_report_key(input_dev, gesture, 1);
	input_sync(input_dev);
	input_report_key(input_dev, gesture, 0);
	input_sync(input_dev);
}

/*****************************************************************************
* Name: fts_gesture_readdata
* Brief: Read information about gesture: enable flag/gesture points..., if ges-
*		ture enable, save gesture points' information, and report to OS.
*		It will be called this function every intrrupt when FTS_GESTURE_EN = 1
*
*		gesture data length: 1(enable) + 1(reserve) + 2(header) + 6 * 4
* Input: ts_data - global struct data
*		data	- gesture data buffer if non-flash, else NULL
* Output:
* Return: 0 - read gesture data successfully, the report data is gesture data
*		 -Exx - tp not in suspend/gesture not enable, or error
*****************************************************************************/
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
	int ret = 0;
	int i = 0;
	int index = 0;
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };
	struct input_dev *input_dev = ts_data->input_dev;
	struct fts_gesture_st *gesture = &fts_gesture_data;

	if (!ts_data->suspended || (!ts_data->gesture_mode && !ts_data->aod_changed))
		return -EINVAL;

	/*
	 * Stock dereferences touch_buf unconditionally in buffer mode, but the
	 * touch-data path calls this with NULL; fall back to the register read.
	 */
	if ((ts_data->gesture_bmode == GESTURE_BM_TOUCH) && touch_buf) {
		memcpy(buf, touch_buf + FTS_TOUCH_DATA_LEN, FTS_GESTURE_DATA_LEN);
	} else {
		buf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
		ret = fts_read(&buf[2], 1, &buf[2], FTS_GESTURE_DATA_LEN - 2);
		if (ret < 0) {
			FTS_ERROR("read gesture header data fail");
			return ret;
		}
	}

	gesture->gesture_id = buf[2];
	gesture->point_num = buf[3];
	if (gesture->gesture_id == GESTURE_FOD)
		ts_data->fod_pressed = true;

	/* save point data,max:6 */
	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		index = 4 * i + 4;
		gesture->coordinate_x[i] = (u16)(((buf[0 + index] & 0x0F) << 8)
										 + buf[1 + index]);
		gesture->coordinate_y[i] = (u16)(((buf[2 + index] & 0x0F) << 8)
										 + buf[3 + index]);
	}

	/* report gesture to OS */
	fts_gesture_report(input_dev, gesture->gesture_id);
	return 0;
}

void fts_gesture_recovery(struct fts_ts_data *ts_data)
{
	if (ts_data->gesture_mode && ts_data->suspended) {
		FTS_DEBUG("gesture recovery...");
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
	}
}

/* re-enable FOD (0xCF bit1) and, while suspended, gesture (0xD0 bit0) */
void fts_fod_recovery(void)
{
	if (fts_data->suspended) {
		FTS_INFO("%s, tp is in suspend mode, write 0xD0 to 1", __func__);
		fts_gesture_reg_write(0x01, true);
	}
	fts_fod_reg_write(0x02, true);
}

/* set/clear @mask in the gesture enable register 0xD0 */
int fts_gesture_reg_write(u8 mask, bool enable)
{
	int i = 0;
	u8 reg_value = 0;
	u8 reg_check = 0;

	for (i = 0; i < FTS_REG_WRITE_RETRY; i++) {
		fts_read_reg(FTS_REG_GESTURE_EN, &reg_value);
		if (enable)
			reg_value |= mask;
		else
			reg_value &= ~mask;
		fts_write_reg(FTS_REG_GESTURE_EN, reg_value);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &reg_check);
		if (reg_check == reg_value)
			break;
	}

	if (i >= FTS_REG_WRITE_RETRY) {
		FTS_ERROR("[GESTURE]Write gesture reg failed!\n");
		return -EIO;
	}

	FTS_INFO("[GESTURE]Write gesture reg success!\n");
	return 0;
}

/* set/clear @mask in the FOD enable register 0xCF */
int fts_fod_reg_write(u8 mask, bool enable)
{
	int i = 0;
	u8 reg_value = 0;
	u8 reg_new = 0;
	u8 reg_check = 0;

	for (i = 0; i < FTS_REG_WRITE_RETRY; i++) {
		fts_read_reg(FTS_REG_FOD_EN, &reg_value);
		if (enable)
			reg_new = reg_value | mask;
		else
			reg_new = reg_value & ~mask;
		if (reg_new == reg_value) {
			FTS_INFO("reg 0xCF do not need to be modified, reg_value = %02X",
					 reg_value);
			return 0;
		}
		fts_write_reg(FTS_REG_FOD_EN, reg_new);
		msleep(1);
		fts_read_reg(FTS_REG_FOD_EN, &reg_check);
		if (reg_check == reg_new)
			break;
	}

	if (i >= FTS_REG_WRITE_RETRY) {
		FTS_ERROR("[GESTURE]Write fod reg failed!\n");
		return -EIO;
	}

	FTS_INFO("[GESTURE]Write fod reg:%d success!\n", reg_check);
	return 0;
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
	if (enable_irq_wake(ts_data->irq)) {
		FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
	}

	for (i = 0; i < 5; i++) {
		fts_write_reg(0xD1, 0xFF);
		fts_write_reg(0xD2, 0xFF);
		fts_write_reg(0xD5, 0xFF);
		fts_write_reg(0xD6, 0xFF);
		fts_write_reg(0xD7, 0xFF);
		fts_write_reg(0xD8, 0xFF);
		fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == ENABLE)
			break;
	}

	if (i >= 5)
		FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
	else
		FTS_INFO("Enter into gesture(suspend) successfully");

	FTS_FUNC_EXIT();
	return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
	int i = 0;
	u8 state = 0xFF;

	FTS_FUNC_ENTER();
	if (disable_irq_wake(ts_data->irq)) {
		FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
	}

	for (i = 0; i < 5; i++) {
		fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
		msleep(1);
		fts_read_reg(FTS_REG_GESTURE_EN, &state);
		if (state == DISABLE)
			break;
	}

	if (i >= 5)
		FTS_ERROR("make IC exit gesture(resume) fail,state:%x", state);
	else
		FTS_INFO("resume from gesture successfully");

	FTS_FUNC_EXIT();
	return 0;
}

/* input event hook: SYN_CONFIG 4/5 from userspace turns double-tap off/on */
int fts_gesture_switch(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	FTS_INFO("Enter. type = %u, code = %u, value = %d", type, code, value);
	if (type == EV_SYN && code == SYN_CONFIG) {
		if (value == WAKEUP_OFF)
			lct_fts_tp_gesture_callback(false);
		else if (value == WAKEUP_ON)
			lct_fts_tp_gesture_callback(true);
	}
	FTS_INFO("Exit");
	return 0;
}

int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct input_dev *input_dev = ts_data->input_dev;

	FTS_FUNC_ENTER();
	input_set_capability(input_dev, EV_KEY, KEY_GOTO);
	input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

	__set_bit(KEY_GOTO, input_dev->keybit);
	__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	__set_bit(KEY_GESTURE_UP, input_dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	__set_bit(KEY_SLEEP, input_dev->keybit);
	__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_WAKEUP, input_dev->keybit);

	input_dev->event = fts_gesture_switch;
	fts_create_gesture_sysfs(ts_data->dev);
	ts_data->gesture_mode = DISABLE;
	ts_data->gesture_bmode = GESTURE_BM_REG;
	memset(&fts_gesture_data, 0, sizeof(struct fts_gesture_st));

	FTS_FUNC_EXIT();
	return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
	FTS_FUNC_EXIT();
	return 0;
}
