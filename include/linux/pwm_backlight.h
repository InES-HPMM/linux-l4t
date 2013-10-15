/*
 * Generic PWM backlight driver data - see drivers/video/backlight/pwm_bl.c
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
 */
#ifndef __LINUX_PWM_BACKLIGHT_H
#define __LINUX_PWM_BACKLIGHT_H

#include <linux/backlight.h>

enum pwm_bl_edp_states {
	PWM_BL_EDP_NEG_3,
	PWM_BL_EDP_NEG_2,
	PWM_BL_EDP_NEG_1,
	PWM_BL_EDP_ZERO,
	PWM_BL_EDP_1,
	PWM_BL_EDP_2,
	PWM_BL_EDP_3,
	PWM_BL_EDP_4,
	PWM_BL_EDP_5,
	PWM_BL_EDP_6,
	PWM_BL_EDP_NUM_STATES,
};

#define PWM_BL_EDP_BRIGHTNESS_UNIT	25

struct platform_pwm_backlight_data {
	int pwm_id;
	unsigned int max_brightness;
	unsigned int dft_brightness;
	unsigned int lth_brightness;
	unsigned int pwm_period_ns;
	unsigned int *levels;
	unsigned int pwm_gpio;
	int (*init)(struct device *dev);
	int (*notify)(struct device *dev, int brightness);
	void (*notify_after)(struct device *dev, int brightness);
	void (*exit)(struct device *dev);
	int (*check_fb)(struct device *dev, struct fb_info *info);
	unsigned int *edp_states;
	unsigned int *edp_brightness;
};

#endif
