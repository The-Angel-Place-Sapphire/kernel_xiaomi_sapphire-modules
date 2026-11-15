// SPDX-License-Identifier: GPL-2.0
/*
 * Xiaomi/Longcheer battery authentication front-end ("batt_verify" psy +
 * "auth_battery" IIO device).
 *
 * Source: Huaqin battery_secrete/auth_battery.c (picasso09 moon, N19A
 * HQ-360184 three-supplier version), converted to the sapphire platform
 * driver "lc,auth-battery" with the Qualcomm 5.15 IIO channel pattern of
 * MiCode topaz battery_secret/ds28e16 (ds_init_iio_psy), as on stock
 * sapphire auth_battery.ko.
 */
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

#include "battery_auth_class.h"

/* sapphire qti_power_supply_iio.h channel numbers used by auth_battery */
#define PSY_IIO_AUTH_AUTHEN_RESULT	0xb4
#define PSY_IIO_AUTH_BATT_ID		0xb5
#define PSY_IIO_AUTH_CYCLE_COUNT	0xb6
#define PSY_IIO_AUTH_AUTHENTIC		0xb7
#define PSY_IIO_AUTH_CHIP_ID		0xb8
#define PSY_IIO_AUTH_MANUFACTURER	0xb9
#define PSY_IIO_BATT_MANUFACTURER	0xba
#define PSY_IIO_AUTH_CHIP_OK		0xbb

enum {
	MAIN_SUPPLY = 0,
	SECEON_SUPPLY,
	THIRD_SUPPLY,
	MAX_SUPPLY,
};

static const char *auth_device_name[] = {
	"main_suppiler",
	"second_supplier",
	"third_supplier",
	"unknown",
};

struct auth_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define AUTH_IIO_CHAN(_name, _num, _type, _mask)	\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define AUTH_CHAN_CURRENT(_name, _num)			\
	AUTH_IIO_CHAN(_name, _num, IIO_CURRENT,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct auth_iio_channels auth_iio_psy_channels[] = {
	AUTH_CHAN_CURRENT("auth_authen_result", PSY_IIO_AUTH_AUTHEN_RESULT)
	AUTH_CHAN_CURRENT("auth_batt_id", PSY_IIO_AUTH_BATT_ID)
	AUTH_CHAN_CURRENT("auth_cycle_count", PSY_IIO_AUTH_CYCLE_COUNT)
	AUTH_CHAN_CURRENT("auth_manufacturer", PSY_IIO_AUTH_MANUFACTURER)
	AUTH_CHAN_CURRENT("auth_chip_id", PSY_IIO_AUTH_CHIP_ID)
	AUTH_CHAN_CURRENT("batt_manufacturer", PSY_IIO_BATT_MANUFACTURER)
	AUTH_CHAN_CURRENT("auth_chip_ok", PSY_IIO_AUTH_CHIP_OK)
};

struct auth_data {
	struct platform_device *pdev;
	struct auth_device *auth_dev[MAX_SUPPLY];

	struct power_supply *verify_psy;
	struct power_supply_desc desc;

	bool auth_result;
	u8 batt_id;

	struct iio_chan_spec *iio_chan;
	struct device *dev;
	struct iio_channel *int_iio_chans;
	struct iio_dev *indio_dev;
	bool auth_chip_ok;
};

static struct auth_data *g_info;
static int auth_index = 0;

static enum power_supply_property verify_props[] = {
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int verify_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct auth_data *info = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_AUTHENTIC:
		val->intval = info->auth_result;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->batt_id;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = auth_device_name[auth_index];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int auth_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct auth_data *info = iio_priv(indio_dev);
	int rc = 0;

	*val1 = 0;

	switch (chan->channel) {
	case PSY_IIO_AUTH_AUTHEN_RESULT:
		*val1 = info->auth_result;
		break;
	case PSY_IIO_AUTH_BATT_ID:
		*val1 = info->batt_id;
		break;
	case PSY_IIO_AUTH_CYCLE_COUNT:
		break;
	case PSY_IIO_AUTH_MANUFACTURER:
		*val1 = auth_index;
		break;
	case PSY_IIO_AUTH_CHIP_OK:
		*val1 = info->auth_chip_ok;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int auth_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_AUTH_CYCLE_COUNT:
	case PSY_IIO_AUTH_AUTHENTIC:
		break;
	default:
		pr_err("Unsupported AUTH IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int auth_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct auth_data *info = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = info->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(auth_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info auth_iio_info = {
	.read_raw	= auth_iio_read_raw,
	.write_raw	= auth_iio_write_raw,
	.of_xlate	= auth_iio_of_xlate,
};

static int auth_iio_init(struct auth_data *info)
{
	struct iio_dev *indio_dev = info->indio_dev;
	struct iio_chan_spec *chan = NULL;
	int num_iio_channels = ARRAY_SIZE(auth_iio_psy_channels);
	int rc = 0, i = 0;

	pr_err("auth_iio_init start\n");
	info->iio_chan = devm_kcalloc(info->dev, num_iio_channels,
				sizeof(*info->iio_chan), GFP_KERNEL);
	if (!info->iio_chan)
		return -ENOMEM;

	info->int_iio_chans = devm_kcalloc(info->dev,
				num_iio_channels,
				sizeof(*info->int_iio_chans),
				GFP_KERNEL);
	if (!info->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &auth_iio_info;
	indio_dev->dev.parent = info->dev;
	indio_dev->dev.of_node = info->dev->of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = info->iio_chan;
	indio_dev->num_channels = num_iio_channels;
	indio_dev->name = "auth_battery";
	for (i = 0; i < num_iio_channels; i++) {
		info->int_iio_chans[i].indio_dev = indio_dev;
		chan = &info->iio_chan[i];
		info->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = auth_iio_psy_channels[i].channel_num;
		chan->type = auth_iio_psy_channels[i].type;
		chan->datasheet_name =
			auth_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			auth_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			auth_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(info->dev, indio_dev);
	if (rc)
		pr_err("Failed to register battery IIO device, rc=%d\n", rc);

	pr_err("auth_iio_init end\n");
	return 0;
}

static int lc_auth_battery_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct auth_data *info;
	struct iio_dev *indio_dev;
	struct power_supply_config cfg = { };

	if (pdev->dev.of_node) {
		indio_dev = devm_iio_device_alloc(&pdev->dev,
						  sizeof(struct auth_data));
		if (!indio_dev) {
			pr_err("Failed to allocate memory\n");
			return -ENOMEM;
		}
		info = iio_priv(indio_dev);
		info->indio_dev = indio_dev;
	} else {
		info = pdev->dev.platform_data;
	}

	if (!info)
		pr_err("No platform data found\n");

	for (i = 0; i < MAX_SUPPLY; i++)
		info->auth_dev[i] = get_batt_auth_by_name(auth_device_name[i]);

	cfg.drv_data = info;
	info->desc.name = "batt_verify";
	info->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	info->desc.properties = verify_props;
	info->desc.num_properties = ARRAY_SIZE(verify_props);
	info->desc.get_property = verify_get_property;
	info->verify_psy =
	    power_supply_register(NULL, &(info->desc), &cfg);
	if (!(info->verify_psy)) {
		pr_err("%s register verify psy fail\n", __func__);
	}

	info->auth_chip_ok = false;
	g_info = info;
	info->auth_result = false;
	info->batt_id = 0xff;

	if (info->auth_dev[MAIN_SUPPLY])
		auth_device_get_batt_id(info->auth_dev[MAIN_SUPPLY],
					&(info->batt_id));

	for (i = 0; i < MAX_SUPPLY; i++) {
		if (!info->auth_dev[i])
			continue;
		ret = auth_device_start_auth(info->auth_dev[i]);
		if (!ret) {
			auth_device_get_batt_id(info->auth_dev[i],
						&(info->batt_id));
			auth_index = i;
			info->auth_result = true;
			info->auth_chip_ok = true;
		}
		break;
	}

	info->dev = &pdev->dev;
	info->pdev = pdev;
	platform_set_drvdata(pdev, info);

	ret = auth_iio_init(info);
	if (ret < 0)
		pr_err("Failed to initialize AUTH IIO PSY, ret=%d\n", ret);

	info->dev->driver_data = NULL;

	return 0;
}

static int lc_auth_battery_remove(struct platform_device *pdev)
{
	int i = 0;

	pr_err("%s remove\n", __func__);

	power_supply_unregister(g_info->verify_psy);

	for (i = 0; i < MAX_SUPPLY; i++)
		auth_device_unregister(g_info->auth_dev[i]);

	kfree(g_info);

	return 0;
}

static const struct of_device_id auth_battery_dt_match[] = {
	{.compatible = "lc,auth-battery"},
	{},
};

static struct platform_driver lc_auth_battery_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lc,auth-battery",
		.of_match_table = auth_battery_dt_match,
	},
	.probe = lc_auth_battery_probe,
	.remove = lc_auth_battery_remove,
};

static int __init auth_battery_init(void)
{
	pr_err("%s enter\n", __func__);

	return platform_driver_register(&lc_auth_battery_driver);
}

static void __exit auth_battery_exit(void)
{
	pr_err("%s enter\n", __func__);

	platform_driver_unregister(&lc_auth_battery_driver);
}

module_init(auth_battery_init);
module_exit(auth_battery_exit);
MODULE_DESCRIPTION("auth battery driver");
MODULE_LICENSE("GPL");
