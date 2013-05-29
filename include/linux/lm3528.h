#ifndef __LINUX_LM3528_BACKLIGHT_H
#define __LINUX_LM3528_BACKLIGHT_H

#include <linux/backlight.h>

enum LM3528_edp_states {
	LM3528_EDP_NEG_8,
	LM3528_EDP_NEG_7,
	LM3528_EDP_NEG_6,
	LM3528_EDP_NEG_5,
	LM3528_EDP_NEG_4,
	LM3528_EDP_NEG_3,
	LM3528_EDP_NEG_2,
	LM3528_EDP_NEG_1,
	LM3528_EDP_ZERO,
	LM3528_EDP_1,
	LM3528_EDP_2,
	LM3528_EDP_NUM_STATES,
};

#define LM3528_EDP_BRIGHTNESS_UNIT	25

struct lm3528_platform_data {
	unsigned int dft_brightness;
	bool (*is_powered)(void);
	int (*notify)(struct device *dev, int brightness);
	unsigned int *edp_states;
	unsigned int *edp_brightness;
};

#endif

