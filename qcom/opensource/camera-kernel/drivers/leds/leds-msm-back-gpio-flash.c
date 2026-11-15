// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
 *
 * Back (rear camera) GPIO flash/torch LED driver, as bundled by Xiaomi in
 * the sapphire camera module ("qcom,leds-back-gpio-flash"). Reconstructed
 * from the stock sapphire camera.ko; based on the msm leds-msm-gpio-flash
 * driver (MiCode spes-r-oss drivers/leds/leds-msm-back-gpio-flash.c).
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include "leds-msm-back-gpio-flash.h"

#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,leds-back-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

/* brightness above this value selects the flash (not torch) sequence */
#define LED_GPIO_FLASH_THRESHOLD	200

enum msm_flash_seq_type_t {
	FLASH_EN,
	FLASH_NOW,
};

struct msm_flash_ctrl_seq {
	enum msm_flash_seq_type_t seq_type;
	uint8_t flash_on_val;
	uint8_t torch_on_val;
};

struct led_gpio_flash_data {
	int flash_en;
	int flash_now;
	int brightness;
	struct led_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct msm_flash_ctrl_seq ctrl_seq[2];
};

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	int brightness = value;
	int flash_en = 0, flash_now = 0;

	if (brightness > LED_GPIO_FLASH_THRESHOLD) {
		flash_en =
			flash_led->ctrl_seq[FLASH_EN].flash_on_val;
		flash_now =
			flash_led->ctrl_seq[FLASH_NOW].flash_on_val;
	} else if (brightness > LED_OFF) {
		flash_en =
			flash_led->ctrl_seq[FLASH_EN].torch_on_val;
		flash_now =
			flash_led->ctrl_seq[FLASH_NOW].torch_on_val;
	} else {
		flash_en = 0;
		flash_now = 0;
	}

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		pr_err("%s:Failed to set gpio %d\n", __func__,
		       flash_led->flash_en);
		goto err;
	}
	rc = gpio_direction_output(flash_led->flash_now, flash_now);
	if (rc) {
		pr_err("%s:Failed to set gpio %d\n", __func__,
		       flash_led->flash_now);
		goto err;
	}

	pr_err("Brave %s:brightness %d, flash_en=%d, flash_now=%d\n",
		__func__, brightness, flash_en, flash_now);
	flash_led->brightness = brightness;
err:
	return;
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	return flash_led->brightness;
}

static int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	const char *seq_name = NULL;
	uint32_t array_flash_seq[2];
	uint32_t array_torch_seq[2];
	int i = 0;

	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	pr_err("***************************Brave************************\n");
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"flash_sleep");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	pr_err("Brave flash_led->flash_on = %d \n", flash_led->flash_en);
	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		/* stock returned the stale rc (0) here: fail the probe instead */
		rc = flash_led->flash_en;
		goto error;
	}

	flash_led->flash_now = of_get_named_gpio(node, "qcom,flash-now", 0);
	if (flash_led->flash_now < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-now", node->full_name, flash_led->flash_now);
		/* stock returned the stale rc (0) here: fail the probe instead */
		rc = flash_led->flash_now;
		goto error;
	}

	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error;
	}

	rc = of_property_read_u32_array(node, "qcom,flash-seq-val",
		array_flash_seq, 2);
	if (rc < 0) {
		pr_err("%s get flash op seq failed %d\n", __func__, __LINE__);
		goto error;
	}

	rc = of_property_read_u32_array(node, "qcom,torch-seq-val",
		array_torch_seq, 2);
	if (rc < 0) {
		pr_err("%s get torch op seq failed %d\n", __func__, __LINE__);
		goto error;
	}

	for (i = 0; i < 2; i++) {
		rc = of_property_read_string_index(node,
			"qcom,op-seq", i, &seq_name);
		pr_err("*****************Brave**********%s seq_name[%d] = %s\n",
			__func__, i, seq_name);
		if (rc < 0) {
			dev_err(&pdev->dev, "%s failed %d\n",
				__func__, __LINE__);
			/* stock went on to strcmp() a NULL seq_name here */
			continue;
		}

		if (!strcmp(seq_name, "flash_en")) {
			flash_led->ctrl_seq[FLASH_EN].seq_type = FLASH_EN;
			pr_err("**********************Brave********%s:%d seq_type[%d] %d\n",
				__func__, __LINE__, i,
				flash_led->ctrl_seq[FLASH_EN].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_EN].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_EN].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_EN].torch_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_EN].torch_on_val =
					GPIO_OUT_HIGH;
		} else if (!strcmp(seq_name, "flash_now")) {
			flash_led->ctrl_seq[FLASH_NOW].seq_type = FLASH_NOW;
			pr_err("*****************Brave********%s:%d seq_type[%d] %d\n",
				__func__, __LINE__, i,
				flash_led->ctrl_seq[i].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_NOW].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_NOW].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_NOW].torch_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_NOW].torch_on_val =
					GPIO_OUT_HIGH;
		}
	}

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	pr_err("%s:probe successfully!\n", __func__);
	return 0;

error:
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);

	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

static int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

int led_gpio_flash_init(void)
{
	return platform_driver_register(&led_gpio_flash_driver);
}

void led_gpio_flash_exit(void)
{
	platform_driver_unregister(&led_gpio_flash_driver);
}

MODULE_DESCRIPTION("QCOM BACK GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-msm-back-gpio-flash");
