// SPDX-License-Identifier: GPL-2.0
/*
 * Xiaomi battery authentication class + onewire_gpio platform driver.
 *
 * Source: Huaqin battery_secrete/battery_auth_class.c (picasso09 moon,
 * Niyush sm4450, linastorvaldz gold).  As on stock sapphire, the
 * onewire_gpio platform driver (MiCode topaz battery_secret/onewire_gpio.c)
 * is merged into this module and creates the class on probe.
 */
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>

#include "battery_auth_class.h"

static struct class *auth_class;
static struct gpio_desc *gpiod;
static bool onewire_gpio_detected;

static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct auth_device *auth = to_auth_device(dev);

	return snprintf(buf, 20, "%s\n",
			auth->name ? auth->name : "anonymous");
}

static DEVICE_ATTR_RO(name);

static struct attribute *auth_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group auth_group = {
	.attrs = auth_attrs,
};

static const struct attribute_group *auth_groups[] = {
	&auth_group,
	NULL,
};

int auth_device_start_auth(struct auth_device *auth_dev)
{
	if (auth_dev != NULL && auth_dev->ops != NULL &&
	    auth_dev->ops->auth_battery != NULL) {
		pr_err("%s: name = %s\n", __func__, dev_name(&auth_dev->dev));
		return auth_dev->ops->auth_battery(auth_dev);
	}

	pr_err("%s: name = %s not fund\n", __func__, dev_name(&auth_dev->dev));
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(auth_device_start_auth);

int auth_device_get_batt_id(struct auth_device *auth_dev, u8 * id)
{
	if (auth_dev != NULL && auth_dev->ops != NULL &&
	    auth_dev->ops->get_battery_id != NULL) {
		pr_err("%s: battery id name = %s\n", __func__,
		       dev_name(&auth_dev->dev));
		return auth_dev->ops->get_battery_id(auth_dev, id);
	}

	pr_err("%s: battery id name = %s not fund\n", __func__,
	       dev_name(&auth_dev->dev));
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(auth_device_get_batt_id);

int auth_device_probe_done(struct auth_device *auth_dev)
{
	if (auth_dev != NULL && auth_dev->ops != NULL &&
	    auth_dev->ops->probe_done != NULL)
		return auth_dev->ops->probe_done(auth_dev);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(auth_device_probe_done);

int auth_device_get_chip_ok(struct auth_device *auth_dev)
{
	if (auth_dev != NULL && auth_dev->ops != NULL &&
	    auth_dev->ops->get_chip_ok != NULL) {
		pr_err("%s: name = %s\n", __func__, dev_name(&auth_dev->dev));
		return auth_dev->ops->get_chip_ok(auth_dev);
	}

	pr_err("%s: name = %s not fund\n", __func__, dev_name(&auth_dev->dev));
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(auth_device_get_chip_ok);

static void auth_device_release(struct device *dev)
{
	struct auth_device *auth_dev = to_auth_device(dev);

	kfree(auth_dev);
}

struct auth_device *auth_device_register(const char *name,
					 struct device *parent,
					 void *devdata,
					 const struct auth_ops *ops)
{
	struct auth_device *auth_dev;

	int rc;

	pr_err("%s: name = %s\n", __func__, name);

	auth_dev = kzalloc(sizeof(*auth_dev), GFP_KERNEL);
	if (!auth_dev)
		return ERR_PTR(-ENOMEM);

	raw_spin_lock_init(&auth_dev->io_lock);

	auth_dev->dev.class = auth_class;
	auth_dev->dev.parent = parent;
	auth_dev->dev.release = auth_device_release;
	auth_dev->gpiod = gpiod;
	dev_set_name(&auth_dev->dev, name);
	dev_set_drvdata(&auth_dev->dev, devdata);

	rc = device_register(&auth_dev->dev);
	if (rc) {
		kfree(auth_dev);
		return ERR_PTR(rc);
	}

	auth_dev->ops = ops;

	pr_err("%s: name = %s end \n", __func__, name);

	return auth_dev;
}
EXPORT_SYMBOL(auth_device_register);

void auth_device_unregister(struct auth_device *auth_dev)
{
	if (!auth_dev)
		return;

	auth_dev->ops = NULL;

	device_unregister(&auth_dev->dev);
}
EXPORT_SYMBOL(auth_device_unregister);

static int auth_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct auth_device *get_batt_auth_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct auth_device *) NULL;

	dev = class_find_device(auth_class, NULL, name,
				auth_match_device_by_name);

	return dev ? to_auth_device(dev) : NULL;
}
EXPORT_SYMBOL(get_batt_auth_by_name);

int onewire_gpio_get_status(void)
{
	return onewire_gpio_detected;
}
EXPORT_SYMBOL(onewire_gpio_get_status);

static int onewire_gpio_probe(struct platform_device *pdev)
{
	enum gpiod_flags gflags = GPIOD_OUT_LOW_OPEN_DRAIN;

	auth_class = class_create(THIS_MODULE, "battery_auth");
	if (IS_ERR(auth_class)) {
		pr_err("Unable to create auth class(%d)\n",
			  PTR_ERR(auth_class));
		return PTR_ERR(auth_class);
	}

	auth_class->dev_groups = auth_groups;

	pr_err("%s onewire_gpio_probe start\n", __func__);

	gpiod = devm_gpiod_get_index(&(pdev->dev), "onewire", 0, gflags);
	if (IS_ERR(gpiod)) {
		pr_err("%s gpio_request (pin) failed\n", __func__);
		return PTR_ERR(gpiod);
	}
	gpiod_direction_output(gpiod, 1);

	return 0;
}

static const struct of_device_id onewire_gpio_dt_match[] = {
	{.compatible = "xiaomi,onewire_gpio"},
	{},
};

static struct platform_driver onewire_gpio_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "onewire_gpio",
		   .of_match_table = onewire_gpio_dt_match,
		   },
	.probe = onewire_gpio_probe,
};

static int __init onewire_gpio_init(void)
{
	int retval;

	onewire_gpio_detected = false;
	pr_err("onewire gpio init entry.\n");

	retval = platform_driver_register(&onewire_gpio_driver);
	if (retval) {
		pr_err(" platform_driver_register fail retval = %d \n ", retval);
		onewire_gpio_detected = false;
		pr_err(" probe failed, exi t!\n");
		return retval;
	}

	onewire_gpio_detected = true;
	pr_err(" probe successfully, onewire_gpio_detected = %d .\n",
	       onewire_gpio_detected);

	return retval;
}

static void __exit onewire_gpio_exit(void)
{
	pr_err("onewire gpio exit entry.\n");
	platform_driver_unregister(&onewire_gpio_driver);
}

module_init(onewire_gpio_init);
module_exit(onewire_gpio_exit);

MODULE_LICENSE("GPL");
