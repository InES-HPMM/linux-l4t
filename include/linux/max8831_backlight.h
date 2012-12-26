/*
 * Generic PWM backlight driver data - see drivers/video/backlight/pwm_bl.c
 */
#ifndef __LINUX_MAX8831_BACKLIGHT_H
#define __LINUX_PWM8831_BACKLIGHT_H

#include <linux/backlight.h>

enum max8831_edp_states {
	MAX8831_EDP_NEG_3,
	MAX8831_EDP_NEG_2,
	MAX8831_EDP_NEG_1,
	MAX8831_EDP_ZERO,
	MAX8831_EDP_1,
	MAX8831_EDP_2,
	MAX8831_EDP_3,
	MAX8831_EDP_4,
	MAX8831_EDP_5,
	MAX8831_EDP_6,
	MAX8831_EDP_7,
	MAX8831_EDP_NUM_STATES,
};

#define MAX8831_EDP_BRIGHTNESS_UNIT	25

struct platform_max8831_backlight_data {
	int id;
	const char *name;
	unsigned int max_brightness;
	unsigned int dft_brightness;
	int (*notify)(struct device *dev, int brightness);
	bool (*is_powered)(void);
	unsigned int edp_states[MAX8831_EDP_NUM_STATES];
	unsigned int edp_brightness[MAX8831_EDP_NUM_STATES];
};

#endif
