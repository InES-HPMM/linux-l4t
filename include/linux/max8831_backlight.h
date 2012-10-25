/*
 * Generic PWM backlight driver data - see drivers/video/backlight/pwm_bl.c
 */
#ifndef __LINUX_MAX8831_BACKLIGHT_H
#define __LINUX_PWM8831_BACKLIGHT_H

#include <linux/backlight.h>

struct platform_max8831_backlight_data {
	int id;
	const char *name;
	unsigned int max_brightness;
	unsigned int dft_brightness;
	int (*notify)(struct device *dev, int brightness);
	bool (*is_powered)(void);
};

#endif
