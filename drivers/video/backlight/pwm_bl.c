/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION, All rights reserved.
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
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/edp.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	unsigned int		pwm_gpio;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
	struct edp_client *pwm_bl_edp_client;
	int *edp_brightness_states;
};

static int pwm_backlight_set(struct backlight_device *bl, int brightness)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		int duty_cycle;

		if (pb->levels) {
			duty_cycle = pb->levels[brightness];
			max = pb->levels[max];
		} else {
			duty_cycle = brightness;
		}

		duty_cycle = pb->lth_brightness +
		     (duty_cycle * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, duty_cycle, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_set_with_edp(struct backlight_device *bl,
	int brightness)
{
	struct pwm_bl_data *data = bl_get_data(bl);
	unsigned int approved;
	int ret;
	unsigned int edp_state;
	unsigned int i;
	if (data->pwm_bl_edp_client) {
		for (i = 0; i < PWM_BL_EDP_NUM_STATES; i++) {
			if (brightness >= data->edp_brightness_states[i])
				break;
		}
		edp_state = i;
		ret = edp_update_client_request(data->pwm_bl_edp_client,
							edp_state, &approved);
		if (ret) {
			dev_err(data->dev, "E state transition failed\n");
			return ret;
		}
		pwm_backlight_set(bl, data->edp_brightness_states[approved]);
	} else {
		pwm_backlight_set(bl, brightness);
	}
	return 0;
}

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	return pwm_backlight_set_with_edp(bl, brightness);
}

static void pwm_backlight_edpcb(unsigned int new_state, void *priv_data)
{
	struct backlight_device *bl_device =
		(struct backlight_device *) priv_data;
	struct pwm_bl_data *data = bl_get_data(bl_device);
	pwm_backlight_set(bl_device, data->edp_brightness_states[new_state]);
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int length;
	u32 value;
	int ret;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;

		if (value >= data->max_brightness) {
			dev_warn(dev, "invalid default brightness level: %u, using %u\n",
				 value, data->max_brightness - 1);
			value = data->max_brightness - 1;
		}

		data->dft_brightness = value;
		data->max_brightness--;
	}

	/*
	 * TODO: Most users of this driver use a number of GPIOs to control
	 *       backlight power. Support for specifying these needs to be
	 *       added.
	 */

	return 0;
}

static struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	struct edp_manager *battery_manager = NULL;
	unsigned int max;
	int ret;

	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		max = data->levels[data->max_brightness];
		pb->levels = data->levels;
	} else
		max = data->max_brightness;

	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;
	pb->pwm_gpio = data->pwm_gpio;
	pb->edp_brightness_states = data->edp_brightness;

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
		if (IS_ERR(pb->pwm)) {
			dev_err(&pdev->dev, "unable to request legacy PWM\n");
			ret = PTR_ERR(pb->pwm);
			goto err_alloc;
		}
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data.
	 */
	if (data->pwm_period_ns > 0)
		pwm_set_period(pb->pwm, data->pwm_period_ns);

	pb->period = pwm_get_period(pb->pwm);
	pb->lth_brightness = data->lth_brightness * (pb->period / max);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;

	if (gpio_is_valid(pb->pwm_gpio)) {
		ret = gpio_request(pb->pwm_gpio, "disp_bl");
		if (ret)
			dev_err(&pdev->dev, "backlight gpio request failed\n");
	}

	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_alloc;
	}

	if (pb->edp_brightness_states) {
		pb->pwm_bl_edp_client = devm_kzalloc(&pdev->dev,
				sizeof(struct edp_client), GFP_KERNEL);
		if (IS_ERR_OR_NULL(pb->pwm_bl_edp_client)) {
			dev_err(&pdev->dev, "could not allocate edp client\n");
			return PTR_ERR(pb->pwm_bl_edp_client);
		}

		strncpy(pb->pwm_bl_edp_client->name,
						"backlight", EDP_NAME_LEN - 1);
		pb->pwm_bl_edp_client->name[EDP_NAME_LEN - 1] = '\0';
		pb->pwm_bl_edp_client->states = data->edp_states;
		pb->pwm_bl_edp_client->num_states = PWM_BL_EDP_NUM_STATES;
		pb->pwm_bl_edp_client->e0_index = PWM_BL_EDP_ZERO;
		pb->pwm_bl_edp_client->private_data = bl;
		pb->pwm_bl_edp_client->priority = EDP_MAX_PRIO + 2;
		pb->pwm_bl_edp_client->throttle = pwm_backlight_edpcb;
		pb->pwm_bl_edp_client->notify_promotion = pwm_backlight_edpcb;

		battery_manager = edp_get_manager("battery");
		if (!battery_manager) {
			dev_err(&pdev->dev, "unable to get edp manager\n");
			goto err_edp_init;
		} else {
			ret = edp_register_client(battery_manager,
						pb->pwm_bl_edp_client);
			if (ret) {
				dev_err(&pdev->dev, "unable to register edp client\n");
				goto err_edp_init;
			} else {
				ret = edp_update_client_request(
						pb->pwm_bl_edp_client,
							PWM_BL_EDP_ZERO, NULL);
				if (ret) {
					dev_err(&pdev->dev,
						"unable to set E0 EDP state\n");
					edp_unregister_client(
						pb->pwm_bl_edp_client);
					goto err_edp_init;
				}
				goto success_edp_init;
			}
		}
err_edp_init:
		devm_kfree(&pdev->dev, pb->pwm_bl_edp_client);
		pb->pwm_bl_edp_client = NULL;
success_edp_init:;
	} else {
		dev_info(&pdev->dev, "edp manager not supported\n");
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	if (gpio_is_valid(pb->pwm_gpio))
		gpio_free(pb->pwm_gpio);

	platform_set_drvdata(pdev, bl);
	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->exit)
		pb->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	return pwm_backlight_set_with_edp(bl, 0);
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);
	return 0;
}

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm		= &pwm_backlight_pm_ops,
#endif
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

