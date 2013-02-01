/*
 * include/linux/regulator/max77660-regulator.h
 * Maxim LDO and Buck regulators driver
 *
 * Copyright 2011-2012 Maxim Integrated Products, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __LINUX_REGULATOR_MAX77660_REGULATOR_H__
#define __LINUX_REGULATOR_MAX77660_REGULATOR_H__

#include <linux/regulator/machine.h>

#define max77660_rails(_name)	"max77660_"#_name

enum max77660_regulator_id {
	MAX77660_REGULATOR_ID_BUCK1,
	MAX77660_REGULATOR_ID_BUCK2,
	MAX77660_REGULATOR_ID_BUCK3,
	MAX77660_REGULATOR_ID_BUCK4,
	MAX77660_REGULATOR_ID_BUCK5,
	MAX77660_REGULATOR_ID_BUCK6,
	MAX77660_REGULATOR_ID_BUCK7,
	MAX77660_REGULATOR_ID_LDO1,
	MAX77660_REGULATOR_ID_LDO2,
	MAX77660_REGULATOR_ID_LDO3,
	MAX77660_REGULATOR_ID_LDO4,
	MAX77660_REGULATOR_ID_LDO5,
	MAX77660_REGULATOR_ID_LDO6,
	MAX77660_REGULATOR_ID_LDO7,
	MAX77660_REGULATOR_ID_LDO8,
	MAX77660_REGULATOR_ID_LDO9,
	MAX77660_REGULATOR_ID_LDO10,
	MAX77660_REGULATOR_ID_LDO11,
	MAX77660_REGULATOR_ID_LDO12,
	MAX77660_REGULATOR_ID_LDO13,
	MAX77660_REGULATOR_ID_LDO14,
	MAX77660_REGULATOR_ID_LDO15,
	MAX77660_REGULATOR_ID_LDO16,
	MAX77660_REGULATOR_ID_LDO17,
	MAX77660_REGULATOR_ID_LDO18,
	MAX77660_REGULATOR_ID_SW1,
	MAX77660_REGULATOR_ID_SW2,
	MAX77660_REGULATOR_ID_SW3,
	MAX77660_REGULATOR_ID_SW4,
	MAX77660_REGULATOR_ID_SW5,
	MAX77660_REGULATOR_ID_NR,
};

/* FPS Power Up/Down Period */
enum max77660_regulator_fps_power_period {
	FPS_POWER_PERIOD_0,
	FPS_POWER_PERIOD_1,
	FPS_POWER_PERIOD_2,
	FPS_POWER_PERIOD_3,
	FPS_POWER_PERIOD_DEF = -1,
};

/* FPS Time Period */
enum max77660_regulator_fps_time_period {
	FPS_TIME_PERIOD_31US,    /* 0b000 */
	FPS_TIME_PERIOD_61US,    /* 0b001 */
	FPS_TIME_PERIOD_122US,   /* 0b010 */
	FPS_TIME_PERIOD_244US,   /* 0b011 */
	FPS_TIME_PERIOD_488US,   /* 0b100 */
	FPS_TIME_PERIOD_977US,   /* 0b101 */
	FPS_TIME_PERIOD_1953US,  /* 0b110 */
	FPS_TIME_PERIOD_3960US,  /* 0b111 */
	FPS_TIME_PERIOD_DEF = -1,
};


/* FPS Source */
enum max77660_regulator_fps_src {
	FPS_SRC_0,
	FPS_SRC_1,
	FPS_SRC_2,
	FPS_SRC_3,
	FPS_SRC_4,
	FPS_SRC_5,
	FPS_SRC_6,
	FPS_SRC_NONE,
	FPS_SRC_DEF = -1,
};


/* SD Forced PWM Mode */
#define SD_FORCED_PWM_MODE	0x20

/* SD Failling Slew Rate Active-Discharge Mode */
#define SD_FSRADE_DISABLE	0x40

/* Group Low-Power Mode */
#define GLPM_ENABLE		0x80

/* EN enable */
#define ENABLE_EN		0x01
#define ENABLE_EN1		(0x02 | ENABLE_EN)
#define ENABLE_EN2		(0x04 | ENABLE_EN)
#define ENABLE_EN3		(0x08 | ENABLE_EN)

/* Disable DVFS */
#define DISABLE_DVFS		0x10

/* Tracking for LDO4 */
#define LDO4_EN_TRACKING	0x100

struct max77660_regulator_fps_cfg {
	enum max77660_regulator_fps_time_period tu_ap_off;
	enum max77660_regulator_fps_time_period td_ap_off;
	enum max77660_regulator_fps_time_period tu_ap_slp;
	enum max77660_regulator_fps_time_period td_ap_slp;
	enum max77660_regulator_fps_time_period tu_fps_6;
	enum max77660_regulator_fps_time_period td_fps_6;
};

struct max77660_regulator_platform_data {
	struct regulator_init_data *reg_init_data;
	int id;
	enum max77660_regulator_fps_src fps_src;
	enum max77660_regulator_fps_power_period fps_pu_period;
	enum max77660_regulator_fps_power_period fps_pd_period;

	int num_fps_cfgs;
	struct max77660_regulator_fps_cfg *fps_cfgs;

	unsigned int flags;
};

#endif /* __LINUX_REGULATOR_MAX77660_REGULATOR_H__ */
