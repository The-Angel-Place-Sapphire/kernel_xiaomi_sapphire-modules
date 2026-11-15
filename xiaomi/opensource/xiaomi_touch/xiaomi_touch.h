#ifndef __XIAOMI__TOUCH_H
#define __XIAOMI__TOUCH_H
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>

#include <miev/mievent.h>

/*CUR,DEFAULT,MIN,MAX*/
#define VALUE_TYPE_SIZE 6
#define VALUE_GRIP_SIZE 9
#define MAX_BUF_SIZE 256
enum MODE_CMD {
	SET_CUR_VALUE = 0,
	GET_CUR_VALUE,
	GET_DEF_VALUE,
	GET_MIN_VALUE,
	GET_MAX_VALUE,
	GET_MODE_VALUE,
	RESET_MODE,
	SET_LONG_VALUE,
};

/*
 * sapphire: 18 modes (stock focaltech_ts_i2c initialises modes 0..17 and the
 * stock layout has touch_mode[18][6]).  Modes 16/17 are named after the usual
 * Xiaomi xiaomi_touch.h numbering; only the value of Touch_Mode_NUM matters for
 * the layout and the export CRCs.
 */
enum  MODE_TYPE {
	Touch_Game_Mode        = 0,
	Touch_Active_MODE      = 1,
	Touch_UP_THRESHOLD     = 2,
	Touch_Tolerance        = 3,
	Touch_Wgh_Min          = 4,
	Touch_Wgh_Max          = 5,
	Touch_Wgh_Step         = 6,
	Touch_Edge_Filter      = 7,
	Touch_Panel_Orientation = 8,
	Touch_Report_Rate      = 9,
	Touch_Fod_Enable       = 10,
	Touch_Aod_Enable       = 11,
	Touch_Resist_RF        = 12,
	Touch_Idle_Time        = 13,
	Touch_Doubletap_Mode   = 14,
	Touch_Grip_Mode        = 15,
	Touch_FodIcon_Enable   = 16,
	Touch_Nonui_Mode       = 17,
	Touch_Mode_NUM         = 18,
};

struct xiaomi_touch_interface {
	int touch_mode[Touch_Mode_NUM][VALUE_TYPE_SIZE];
	int (*setModeValue)(int Mode, int value);
	int (*setModeLongValue)(int Mode, int value_len, int *value);
	int (*getModeValue)(int Mode, int value_type);
	int (*getModeAll)(int Mode, int *modevalue);
	int (*resetMode)(int Mode);
	int (*palm_sensor_read)(void);
	int (*palm_sensor_write)(int on);
	int long_mode_len;
	int long_mode_value[MAX_BUF_SIZE];
	void (*touch_dfs_test)(int value);
};

struct xiaomi_touch {
	struct miscdevice 	misc_dev;
	struct device *dev;
	struct class *class;
	struct attribute_group attrs;
	struct mutex  mutex;
	struct mutex  palm_mutex;
	struct mutex  psensor_mutex;
	struct mutex  fod_press_status_mutex;
	wait_queue_head_t 	wait_queue;
};

struct xiaomi_touch_pdata{
	struct xiaomi_touch *device;
	struct xiaomi_touch_interface *touch_data;
	int fod_press_status_value;
	int reserved[3];	/* unused, keeps the stock layout (palm_value at +0x20) */
	int palm_value;
	bool palm_changed;
	const char *name;
};

enum touch_mievent_code {
	TOUCH_EVENT_TRANSFER_ERR = 912001001,
	TOUCH_EVENT_FWLOAD_ERR = 912001002,
	TOUCH_EVENT_PARAM_ERR = 912001003,
	TOUCH_EVENT_OPENTEST_FAIL = 912002001,
	TOUCH_EVENT_SHORTTEST_FAIL = 912002002,
};
enum param_parse_fail_type {
	ERROR_REGULATOR_INIT,
	ERROR_GPIO_REQUEST,
	ERROR_DTS_PARSE,
};

struct xiaomi_touch *xiaomi_touch_dev_get(int minor);

extern struct class *get_xiaomi_touch_class(void);

extern struct device *get_xiaomi_touch_dev(void);

extern int update_palm_sensor_value(int value);

extern int update_fod_press_status(int value);

int xiaomitouch_register_modedata(struct xiaomi_touch_interface *data);

void xiaomi_touch_mievent_report_int(unsigned int code, int panel_id, const char *fault_name, const char *vendor_name, long error_code);
void xiaomi_touch_mievent_report_str(unsigned int code, int panel_id, const char *fault_name, const char *vendor_name);

#endif
