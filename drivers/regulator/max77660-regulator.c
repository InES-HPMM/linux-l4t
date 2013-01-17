/*
 * drivers/regulator/max77660-regulator.c
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

#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max77660/max77660-regulator.h>

/* Regulator types */
#define REGULATOR_TYPE_BUCK			0
#define REGULATOR_TYPE_LDO_N		1
#define REGULATOR_TYPE_LDO_P		2
#define REGULATOR_TYPE_SW			3


#define MAX77660_REG_INVALID		0xFF
#define MAX77660_REG_FPS_NONE		MAX77660_REG_INVALID

/* FPS Registers */
#define MAX77660_REG_CNFG_FPS_AP_OFF 0x23
#define MAX77660_REG_CNFG_FPS_AP_SLP 0x24
#define MAX77660_REG_CNFG_FPS_6		 0x25

#define MAX77660_REG_FPS_RSO		0x26
#define MAX77660_REG_FPS_BUCK1		0x27
#define MAX77660_REG_FPS_BUCK2		0x28
#define MAX77660_REG_FPS_BUCK3		0x29
#define MAX77660_REG_FPS_BUCK4		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_BUCK5		0x2A
#define MAX77660_REG_FPS_BUCK6		0x2B
#define MAX77660_REG_FPS_BUCK7		0x2C


#define MAX77660_REG_FPS_SW4		0x2E
#define MAX77660_REG_FPS_SW5		0x2D
#define MAX77660_REG_FPS_LDO1		0x2E
#define MAX77660_REG_FPS_LDO2		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO3		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO4		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO5		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO6		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO7		0x2F
#define MAX77660_REG_FPS_LDO8		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO9		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO10		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO11		0x30
#define MAX77660_REG_FPS_LDO12		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO13		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO14		0x31
#define MAX77660_REG_FPS_LDO15		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO16		MAX77660_REG_FPS_NONE
#define MAX77660_REG_FPS_LDO17		0x32
#define MAX77660_REG_FPS_LDO18		0x33

#define MAX77660_REG_FPS_GPIO1		0x34
#define MAX77660_REG_FPS_GPIO2		0x35
#define MAX77660_REG_FPS_GPIO3		0x36

#define MAX77660_REG_SW_EN          0x43
/* BUCK and LDO Registers */
#define MAX77660_REG_BUCK1_VOUT	0x46
#define MAX77660_REG_BUCK2_VOUT	0x47
#define MAX77660_REG_BUCK3_VOUT	0x48
#define MAX77660_REG_BUCK3_VDVS	0x49
#define MAX77660_REG_BUCK5_VOUT	0x4A
#define MAX77660_REG_BUCK5_VDVS	0x4B
#define MAX77660_REG_BUCK6_VOUT	0x4C
#define MAX77660_REG_BUCK7_VOUT	0x4D


#define MAX77660_REG_BUCK6_CNFG	0x4C
#define MAX77660_REG_BUCK7_CNFG	0x4D
#define MAX77660_REG_BUCK1_CNFG	0x4E
#define MAX77660_REG_BUCK2_CNFG	0x4F
#define MAX77660_REG_BUCK3_CNFG	0x50
#define MAX77660_REG_BUCK4_CNFG	0x51
#define MAX77660_REG_BUCK5_CNFG	0x52
#define MAX77660_REG_BUCK4_VOUT	0x53


#define MAX77660_REG_LDO1_CNFG		0x54
#define MAX77660_REG_LDO2_CNFG		0x55
#define MAX77660_REG_LDO3_CNFG		0x56
#define MAX77660_REG_LDO4_CNFG		0x57
#define MAX77660_REG_LDO5_CNFG		0x58
#define MAX77660_REG_LDO6_CNFG		0x59
#define MAX77660_REG_LDO7_CNFG		0x5A
#define MAX77660_REG_LDO8_CNFG		0x5B
#define MAX77660_REG_LDO9_CNFG		0x5C
#define MAX77660_REG_LDO10_CNFG		0x5D
#define MAX77660_REG_LDO11_CNFG		0x5E
#define MAX77660_REG_LDO12_CNFG		0x5F
#define MAX77660_REG_LDO13_CNFG		0x60
#define MAX77660_REG_LDO14_CNFG		0x61
#define MAX77660_REG_LDO15_CNFG		0x62
#define MAX77660_REG_LDO16_CNFG		0x63
#define MAX77660_REG_LDO17_CNFG		0x64
#define MAX77660_REG_LDO18_CNFG		0x65

#define MAX77660_REG_SW1_CNFG       0x66
#define MAX77660_REG_SW2_CNFG       0x67
#define MAX77660_REG_SW3_CNFG       0x68
#define MAX77660_REG_SW4_CNFG       MAX77660_REG_INVALID
#define MAX77660_REG_SW5_CNFG       0x69


#define MAX77660_REG_BUCK_PWR_MODE1 0x37
#define MAX77660_REG_BUCK_PWR_MODE2 0x38

#define MAX77660_REG_LDO_PWR_MODE1  0x3E
#define MAX77660_REG_LDO_PWR_MODE2  0x3F
#define MAX77660_REG_LDO_PWR_MODE3  0x40
#define MAX77660_REG_LDO_PWR_MODE4  0x41
#define MAX77660_REG_LDO_PWR_MODE5  0x42

/* Power Mode */
#define POWER_MODE_NORMAL		3
#define POWER_MODE_LPM			2
#define POWER_MODE_GLPM			1
#define POWER_MODE_DISABLE		0

#define BUCK_POWER_MODE_MASK	0x3
#define BUCK_POWER_MODE_SHIFT	0
#define LDO_POWER_MODE_MASK		0x3
#define LDO_POWER_MODE_SHIFT	0


/* LDO Configuration 3 */
#define TRACK4_MASK			0x20
#define TRACK4_SHIFT			5

/* Voltage */
#define SDX_VOLT_MASK			0xFF
#define SD1_VOLT_MASK			0x3F
#define LDO_VOLT_MASK			0x3F



/*
#define FPS_TIME_PERIOD_MASK		0x38
#define FPS_TIME_PERIOD_SHIFT		3
#define FPS_EN_SRC_MASK			0x07
#define FPS_EN_SRC_SHIFT		1
#define FPS_SW_EN_MASK			0x01
#define FPS_SW_EN_SHIFT			0
*/

#define FPS_SRC_MASK			(BIT(6) | BIT(5) | BIT(4))
#define FPS_SRC_SHIFT			 4
#define FPS_PU_PERIOD_MASK		(BIT(2) | BIT(3))
#define FPS_PU_PERIOD_SHIFT		 2
#define FPS_PD_PERIOD_MASK		(BIT(0) | BIT(1))
#define FPS_PD_PERIOD_SHIFT		 0

#define CNFG_FPS_AP_OFF_TU_MASK   (BIT(6) | BIT(5) | BIT(4))
#define CNFG_FPS_AP_OFF_TU_SHIFT   4
#define CNFG_FPS_AP_OFF_TD_MASK   (BIT(0) | BIT(1) | BIT(2))
#define CNFG_FPS_AP_OFF_TD_SHIFT   0

#define CNFG_FPS_AP_SLP_TU_MASK   (BIT(6) | BIT(5) | BIT(4))
#define CNFG_FPS_AP_SLP_TU_SHIFT   4
#define CNFG_FPS_AP_SLP_TD_MASK   (BIT(0) | BIT(1) | BIT(2))
#define CNFG_FPS_AP_SLP_TD_SHIFT   0

#define CNFG_FPS_6_TU_MASK		  (BIT(6) | BIT(5) | BIT(4))
#define CNFG_FPS_6_TU_SHIFT        4
#define CNFG_FPS_6_TD_MASK	      (BIT(0) | BIT(1) | BIT(2))
#define CNFG_FPS_6_TD_SHIFT        0

#define BUCK6_7_CNFG_ADE_MASK       BIT(7)
#define BUCK6_7_CNFG_ADE_SHIFT      7
#define BUCK6_7_CNFG_FPWM_MASK      BIT(6)
#define BUCK6_7_CNFG_FPWM_SHIFT     6
#define BUCK6_7_CNFG_VOUT_MASK      0x3F    /* BIT(0-5) */
#define BUCK6_7_CNFG_VOUT_SHIFT     0
#define BUCK1_5_CNFG_RAMP_MASK     (BIT(7)|BIT(6))
#define BUCK1_5_CNFG_RAMP_SHIFT     6
#define BUCK1_5_CNFG_ADE_MASK       BIT(3)
#define BUCK1_5_CNFG_ADE_SHIFT      3
#define BUCK1_5_CNFG_FPWM_MASK      BIT(2)
#define BUCK1_5_CNFG_FPWM_SHIFT     2
#define BUCK1_5_CNFG_DVFS_EN_MASK   BIT(1)
#define BUCK1_5_CNFG_DVFS_EN_SHIFT  1
#define BUCK1_5_CNFG_FSRADE_MASK    BIT(0)
#define BUCK1_5_CNFG_FSRADE_SHIFT   0

#define LDO1_18_CNFG_ADE_MASK       BIT(6)
#define LDO1_18_CNFG_ADE_SHIFT      6
#define LDO1_18_CNFG_VOUT_MASK      0x3F  /*  BIT(0-5) */
#define LDO1_18_CNFG_VOUT_SHIFT     0


#if 0
#define CID_DIDM_MASK			0xF0
#define CID_DIDM_SHIFT			4
#endif
#define SD_SAFE_DOWN_UV			50000 /* 50mV */



enum {
	VOLT_REG = 0,  /* XX_VOUT */
	CFG_REG,       /* XX_CNFG */
	FPS_REG,       /* XX_FPS */
	PWR_MODE_REG   /* XX_PWR_MODE */
};

struct max77660_register {
	u8 addr;
	u8 val;
};

struct max77660_regulator_info {
	u8 id;
	u8 type;
	u32 min_uV;
	u32 max_uV;
	u32 step_uV;

	struct max77660_register regs[4]; /* volt, cfg, fps, pwrmode */

	struct regulator_desc desc;

	u8 volt_mask;

	u8 power_mode_mask;
	u8 power_mode_shift;
};

struct max77660_regulator {
	struct max77660_regulator_info *rinfo;
	struct regulator_dev *rdev;
	struct device *dev;
	struct max77660_regulator_platform_data *pdata;
	u32 regulator_mode;
	u8 power_mode;
	enum max77660_regulator_fps_src fps_src;
	u8 val[4]; /* volt, cfg, fps, power mode */
	int safe_down_uV; /* for stable down scaling */
};

#define fps_src_name(fps_src)	\
	(fps_src == FPS_SRC_0 ? "FPS_SRC_0" :	\
	fps_src == FPS_SRC_1 ? "FPS_SRC_1" :	\
	fps_src == FPS_SRC_2 ? "FPS_SRC_2" :	\
	fps_src == FPS_SRC_3 ? "FPS_SRC_3" :	\
	fps_src == FPS_SRC_4 ? "FPS_SRC_4" :	\
	fps_src == FPS_SRC_5 ? "FPS_SRC_5" :	\
	fps_src == FPS_SRC_6 ? "FPS_SRC_6" : "FPS_SRC_NONE")

static int fps_cfg_init;
static struct max77660_register fps_cfg_regs[] = {
	{
		.addr = MAX77660_REG_CNFG_FPS_AP_OFF,
	},
	{
		.addr = MAX77660_REG_CNFG_FPS_AP_SLP,
	},
	{
		.addr = MAX77660_REG_CNFG_FPS_6,
	},
};



static inline int max77660_regulator_cache_write(struct max77660_regulator *reg,
					u8 addr, u8 mask, u8 val, u8 *cache)
{
	struct device *dev = reg->dev;
	u8 new_val;
	int ret;

	new_val = (*cache & ~mask) | (val & mask);
	if (*cache != new_val) {
		ret = max77660_write(dev, addr, &new_val, 1, MAX77660_I2C_PMIC);
		if (ret < 0)
			return ret;

		*cache = new_val;
	}
	return 0;
}

static int
max77660_regulator_set_fps_src(struct max77660_regulator *reg,
			       enum max77660_regulator_fps_src fps_src)
{
	int ret;
	struct max77660_regulator_info *rinfo = reg->rinfo;

	if ((rinfo->regs[FPS_REG].addr == MAX77660_REG_FPS_NONE) ||
			(reg->fps_src == fps_src))
		return 0;

	switch (fps_src) {
	case FPS_SRC_0:
	case FPS_SRC_1:
	case FPS_SRC_2:
	case FPS_SRC_3:
	case FPS_SRC_4:
	case FPS_SRC_5:
	case FPS_SRC_6:
		break;
	case FPS_SRC_NONE:
	case FPS_SRC_DEF:
		return 0;
	default:
		return -EINVAL;
	}

	ret = max77660_regulator_cache_write(reg, rinfo->regs[FPS_REG].addr,
					FPS_SRC_MASK, fps_src << FPS_SRC_SHIFT,
					&reg->val[FPS_REG]);
	if (ret < 0)
		return ret;

	reg->fps_src = fps_src;
	return 0;
}

static int max77660_regulator_set_fps(struct max77660_regulator *reg)
{
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 fps_val = 0, fps_mask = 0;
	int ret = 0;

	if (reg->rinfo->regs[FPS_REG].addr == MAX77660_REG_FPS_NONE)
		return 0;

	if (reg->fps_src == FPS_SRC_NONE)
		return 0;

	/* FPS power up period setting */
	if (pdata->fps_pu_period != FPS_POWER_PERIOD_DEF) {
		fps_val |= (pdata->fps_pu_period << FPS_PU_PERIOD_SHIFT);
		fps_mask |= FPS_PU_PERIOD_MASK;
	}

	/* FPS power down period setting */
	if (pdata->fps_pd_period != FPS_POWER_PERIOD_DEF) {
		fps_val |= (pdata->fps_pd_period << FPS_PD_PERIOD_SHIFT);
		fps_mask |= FPS_PD_PERIOD_MASK;
	}

	/* FPS SRC setting */
	if (pdata->fps_src != FPS_SRC_NONE) {
		fps_val |= (pdata->fps_pd_period << FPS_SRC_SHIFT);
		fps_mask |= FPS_SRC_MASK;
	}

	fps_val |= reg->val[FPS_REG];
	if (fps_val || fps_mask)
		ret = max77660_regulator_cache_write(reg,
					rinfo->regs[FPS_REG].addr, fps_mask,
					fps_val, &reg->val[FPS_REG]);

	return ret;
}

static int
max77660_regulator_set_fps_cfg(struct max77660_regulator *reg,
			       struct max77660_regulator_fps_cfg *fps_cfg)
{
	u8 val = 0, mask = 0;
	int ret;

	if ((reg->fps_src < FPS_SRC_0) || (reg->fps_src >= FPS_SRC_NONE))
		return -EINVAL;

	if ((reg->fps_src >= FPS_SRC_0) && (reg->fps_src <= FPS_SRC_5))	{
		if (fps_cfg->tu_ap_off != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_OFF_TU_SHIFT);
			mask |= CNFG_FPS_AP_OFF_TU_MASK;
		}

		if (fps_cfg->td_ap_off != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_OFF_TD_SHIFT);
			mask |= CNFG_FPS_AP_OFF_TD_MASK;
		}

		ret = max77660_regulator_cache_write(reg,
					fps_cfg_regs[0].addr, mask,
					val, &fps_cfg_regs[0].val);

		if (fps_cfg->tu_ap_slp != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_SLP_TU_SHIFT);
			mask |= CNFG_FPS_AP_SLP_TU_MASK;
		}

		if (fps_cfg->td_ap_slp != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_SLP_TD_SHIFT);
			mask |= CNFG_FPS_AP_SLP_TD_MASK;
		}

		ret = max77660_regulator_cache_write(reg,
					fps_cfg_regs[1].addr, mask,
					val, &fps_cfg_regs[1].val);

	}

	if (reg->fps_src == FPS_SRC_6) {
		if (fps_cfg->tu_fps_6 != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_fps_6 << CNFG_FPS_6_TU_SHIFT);
			mask |= CNFG_FPS_6_TU_MASK;
		}

		if (fps_cfg->tu_fps_6 != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->td_ap_off << CNFG_FPS_6_TD_SHIFT);
			mask |= CNFG_FPS_AP_SLP_TD_MASK;
		}

		ret = max77660_regulator_cache_write(reg,
					fps_cfg_regs[2].addr, mask,
					val, &fps_cfg_regs[2].val);
	}


	return ret;
}

static int
max77660_regulator_set_fps_cfgs(struct max77660_regulator *reg,
				struct max77660_regulator_fps_cfg *fps_cfgs,
				int num_fps_cfgs)
{
	struct device *dev = reg->dev->parent;
	int i, ret, num_fps_cfg_regs;

	if (fps_cfg_init)
		return 0;

	num_fps_cfg_regs = sizeof(fps_cfg_regs) /
				sizeof(struct max77660_register);
	for (i = 0; i < num_fps_cfg_regs; i++) {
		ret = max77660_read(dev, fps_cfg_regs[i].addr,
				    &fps_cfg_regs[i].val, 1, 0);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < num_fps_cfgs; i++) {
		ret = max77660_regulator_set_fps_cfg(reg, &fps_cfgs[i]);
		if (ret < 0)
			return ret;
	}

	fps_cfg_init = 1;

	return 0;
}

static int
max77660_regulator_set_power_mode(struct max77660_regulator *reg, u8 power_mode)
{
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;
	int ret = 0;

	if (rinfo->type == REGULATOR_TYPE_BUCK)
		ret = max77660_regulator_cache_write(reg,
					rinfo->regs[PWR_MODE_REG].addr,
					mask, power_mode << shift,
					&reg->val[PWR_MODE_REG]);
	else if ((rinfo->type == REGULATOR_TYPE_LDO_N) ||
			(rinfo->type == REGULATOR_TYPE_LDO_P))
		ret = max77660_regulator_cache_write(reg,
					rinfo->regs[PWR_MODE_REG].addr,
					mask, power_mode << shift,
					&reg->val[PWR_MODE_REG]);
	else if (rinfo->type == REGULATOR_TYPE_SW)
		ret = 0;

	if (ret < 0)
		return ret;

	reg->power_mode = power_mode;
	return ret;
}

static u8 max77660_regulator_get_power_mode(struct max77660_regulator *reg)
{
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 mask = rinfo->power_mode_mask;
	u8 shift = rinfo->power_mode_shift;

	switch (rinfo->type) {
	case REGULATOR_TYPE_BUCK:
		reg->power_mode = (reg->val[PWR_MODE_REG] & mask) >> shift;
		break;
	case REGULATOR_TYPE_LDO_N:
	case REGULATOR_TYPE_LDO_P:
		reg->power_mode = (reg->val[PWR_MODE_REG] & mask) >> shift;
		break;
	}
	return reg->power_mode;
}

static int max77660_regulator_do_set_voltage(struct max77660_regulator *reg,
					     int min_uV, int max_uV)
{
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 addr = rinfo->regs[VOLT_REG].addr;
	u8 mask = rinfo->volt_mask;
	u8 *cache = &reg->val[VOLT_REG];
	u8 val;
	int old_uV, new_uV, safe_uV;
	int i, steps = 1;
	int ret = 0;

	if (min_uV < rinfo->min_uV || max_uV > rinfo->max_uV)
		return -EDOM;

	old_uV = (*cache & mask) * rinfo->step_uV + rinfo->min_uV;

	if ((old_uV > min_uV) && (reg->safe_down_uV >= rinfo->step_uV)) {
		steps = DIV_ROUND_UP(old_uV - min_uV, reg->safe_down_uV);
		safe_uV = -reg->safe_down_uV;
	}

	if (steps == 1) {
		val = (min_uV - rinfo->min_uV) / rinfo->step_uV;
		ret = max77660_regulator_cache_write(reg, addr, mask, val,
						     cache);
	} else {
		for (i = 0; i < steps; i++) {
			if (abs(min_uV - old_uV) > abs(safe_uV))
				new_uV = old_uV + safe_uV;
			else
				new_uV = min_uV;

			dev_dbg(&reg->rdev->dev, "do_set_voltage: name=%s, %d/%d, old_uV=%d, new_uV=%d\n",
				reg->rdev->desc->name, i + 1, steps, old_uV,
				new_uV);

			val = (new_uV - rinfo->min_uV) / rinfo->step_uV;
			ret = max77660_regulator_cache_write(reg, addr, mask,
							     val, cache);
			if (ret < 0)
				return ret;

			old_uV = new_uV;
		}
	}

	return ret;
}

static int max77660_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uV, int max_uV,
					  unsigned *selector)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);

	dev_dbg(&rdev->dev, "set_voltage: name=%s, min_uV=%d, max_uV=%d\n",
		rdev->desc->name, min_uV, max_uV);
	return max77660_regulator_do_set_voltage(reg, min_uV, max_uV);
}

static int max77660_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int volt;

	volt = (reg->val[VOLT_REG] & rinfo->volt_mask)
		* rinfo->step_uV + rinfo->min_uV;

	dev_dbg(&rdev->dev, "get_voltage: name=%s, volt=%d, val=0x%02x\n",
		rdev->desc->name, volt, reg->val[VOLT_REG]);
	return volt;
}

static int max77660_regulator_enable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	int power_mode = (pdata->flags & GLPM_ENABLE) ?
			 POWER_MODE_GLPM : POWER_MODE_NORMAL;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "enable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}
#if 0
	if ((rinfo->id == MAX77660_REGULATOR_ID_SD0)
			&& (pdata->flags & EN2_CTRL_SD0)) {
		dev_dbg(&rdev->dev,
			"enable: Regulator %s is controlled by EN2\n",
			rdev->desc->name);
		return 0;
	}
#endif
	/* N-Channel LDOs don't support Low-Power mode. */
	if ((rinfo->type != REGULATOR_TYPE_LDO_N) &&
			(reg->regulator_mode == REGULATOR_MODE_STANDBY))
		power_mode = POWER_MODE_LPM;

	return max77660_regulator_set_power_mode(reg, power_mode);
}

static int max77660_regulator_disable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);

	int power_mode = POWER_MODE_DISABLE;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "disable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}
	return max77660_regulator_set_power_mode(reg, power_mode);
}

static int max77660_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	int ret = 1;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "is_enable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 1;
	}

	if (max77660_regulator_get_power_mode(reg) == POWER_MODE_DISABLE)
		ret = 0;

	return ret;
}

static int max77660_regulator_set_mode(struct regulator_dev *rdev,
				       unsigned int mode)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 power_mode;
	int ret;

	if (mode == REGULATOR_MODE_NORMAL)
		power_mode = (pdata->flags & GLPM_ENABLE) ?
			     POWER_MODE_GLPM : POWER_MODE_NORMAL;
	else if (mode == REGULATOR_MODE_STANDBY) {
		/* N-Channel LDOs don't support Low-Power mode. */
		power_mode = (rinfo->type != REGULATOR_TYPE_LDO_N) ?
			     POWER_MODE_LPM : POWER_MODE_NORMAL;
	} else
		return -EINVAL;

	ret = max77660_regulator_set_power_mode(reg, power_mode);
	if (!ret)
		reg->regulator_mode = mode;

	return ret;
}

static unsigned int max77660_regulator_get_mode(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);

	return reg->regulator_mode;
}

static int max77660_switch_enable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int idx, ret;

	idx = rinfo->id - MAX77660_REGULATOR_ID_SW1;
	ret = max77660_set_bits(reg->dev, rinfo->regs[VOLT_REG].addr,
					1 << idx, 1 << idx, MAX77660_I2C_PMIC);

	return ret;
}

static int max77660_switch_disable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int idx, ret;

	idx = rinfo->id - MAX77660_REGULATOR_ID_SW1;
	ret = max77660_set_bits(reg->dev, rinfo->regs[VOLT_REG].addr,
					1 << idx, 0, MAX77660_I2C_PMIC);

	return ret;
}

static int max77660_switch_is_enabled(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int idx, val, ret;

	idx = rinfo->id - MAX77660_REGULATOR_ID_SW1;
	ret = max77660_read(reg->dev, rinfo->regs[VOLT_REG].addr,
				&val, 1, MAX77660_I2C_PMIC);

	return val & (1 << idx);
}

static struct regulator_ops max77660_ldo_ops = {
	.set_voltage = max77660_regulator_set_voltage,
	.get_voltage = max77660_regulator_get_voltage,
	.enable = max77660_regulator_enable,
	.disable = max77660_regulator_disable,
	.is_enabled = max77660_regulator_is_enabled,
	.set_mode = max77660_regulator_set_mode,
	.get_mode = max77660_regulator_get_mode,
};

static struct regulator_ops max77660_sw_ops = {
	.enable = max77660_switch_enable,
	.disable = max77660_switch_disable,
	.is_enabled = max77660_switch_is_enabled,
};

static int max77660_regulator_preinit(struct max77660_regulator *reg)
{
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	struct max77660_regulator_info *rinfo = reg->rinfo;
	struct device *dev = reg->dev->parent;
	int i;
	u8 idx;
	u8 val, mask;
	int ret;

	/* Update Power Mode register mask and offset */
	if (rinfo->type == REGULATOR_TYPE_BUCK) {

		idx = reg->rinfo->id - MAX77660_REGULATOR_ID_BUCK1;
		/*  4 Bucks Pwr Mode in 1 register */
		reg->rinfo->regs[PWR_MODE_REG].addr =
				MAX77660_REG_BUCK_PWR_MODE1 + (idx/4);
		reg->rinfo->power_mode_shift = (idx%4)*2 ;
		reg->rinfo->power_mode_mask = reg->rinfo->power_mode_mask <<
						(reg->rinfo->power_mode_shift);
	}

	if ((rinfo->type == REGULATOR_TYPE_LDO_N) ||
			(rinfo->type == REGULATOR_TYPE_LDO_P)) {

		idx = reg->rinfo->id - MAX77660_REGULATOR_ID_LDO1;
		/* 4 LDOs Pwr Mode in 1 register  */
		reg->rinfo->regs[PWR_MODE_REG].addr =
				MAX77660_REG_LDO_PWR_MODE1 + + (idx/4);
		reg->rinfo->power_mode_shift = (idx%4)*2 ;
		reg->rinfo->power_mode_mask = reg->rinfo->power_mode_mask <<
						(reg->rinfo->power_mode_shift);
	}

	/* Update registers */
	for (i = 0; i <= PWR_MODE_REG; i++) {
		ret = max77660_read(dev, rinfo->regs[i].addr,
				    &reg->val[i], 1, 0);
		if (ret < 0) {
			dev_err(reg->dev,
				"preinit: Failed to get register 0x%x\n",
				rinfo->regs[i].addr);
			return ret;
		}
	}

	/* Update FPS source */
	if (rinfo->regs[FPS_REG].addr == MAX77660_REG_FPS_NONE)
		reg->fps_src = FPS_SRC_NONE;
	else
		reg->fps_src = (reg->val[FPS_REG] & FPS_SRC_MASK) >>
			FPS_SRC_SHIFT;

	dev_dbg(reg->dev, "preinit: initial fps_src=%s\n",
		fps_src_name(reg->fps_src));

	/* Update power mode */
	max77660_regulator_get_power_mode(reg);

	/* Check Chip Identification */
	ret = max77660_read(dev, MAX77660_REG_CID5, &val, 1, 0);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to get register 0x%x\n",
			MAX77660_REG_CID5);
		return ret;
	}

#if 0
	/* If metal revision is less than rev.3,
	 * set safe_down_uV for stable down scaling. */
	if ((rinfo->type == REGULATOR_TYPE_BUCK) &&
			((val & CID_DIDM_MASK) >> CID_DIDM_SHIFT) <= 2)
		reg->safe_down_uV = SD_SAFE_DOWN_UV;
	else
		reg->safe_down_uV = 0;
#endif
	/* Set FPS */
	ret = max77660_regulator_set_fps_cfgs(reg, pdata->fps_cfgs,
					      pdata->num_fps_cfgs);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPSCFG\n");
		return ret;
	}

	/* N-Channel LDOs don't support Low-Power mode. */
	if ((rinfo->type == REGULATOR_TYPE_LDO_N) &&
			(pdata->flags & GLPM_ENABLE))
		pdata->flags &= ~GLPM_ENABLE;

	/* To prevent power rail turn-off when change FPS source,
	 * it must set power mode to NORMAL before change FPS source to NONE
	 * from SRC_0, SRC_1 and SRC_2. */
	if ((reg->fps_src != FPS_SRC_NONE) && (pdata->fps_src == FPS_SRC_NONE)
			&& (reg->power_mode != POWER_MODE_NORMAL)) {
		val = (pdata->flags & GLPM_ENABLE) ?
		      POWER_MODE_GLPM : POWER_MODE_NORMAL;
		ret = max77660_regulator_set_power_mode(reg, val);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to set power mode to POWER_MODE_NORMAL\n");
			return ret;
		}
	}

	ret = max77660_regulator_set_fps_src(reg, pdata->fps_src);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPSSRC to %d\n",
			pdata->fps_src);
		return ret;
	}

	ret = max77660_regulator_set_fps(reg);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to set FPS\n");
		return ret;
	}
	if (rinfo->type == REGULATOR_TYPE_BUCK) {
		val = 0;
		mask = 0;
		if ((reg->rinfo->id >= MAX77660_REGULATOR_ID_BUCK1) &&
			(reg->rinfo->id <= MAX77660_REGULATOR_ID_BUCK5)) {
			mask |= BUCK1_5_CNFG_FPWM_MASK;
			if (pdata->flags & SD_FORCED_PWM_MODE)
				val |= BUCK1_5_CNFG_FPWM_MASK;

			mask |= BUCK1_5_CNFG_FSRADE_MASK;
			if (pdata->flags & SD_FSRADE_DISABLE)
				val |= BUCK1_5_CNFG_FSRADE_MASK;
		} else if ((reg->rinfo->id >= MAX77660_REGULATOR_ID_BUCK6) &&
			(reg->rinfo->id <= MAX77660_REGULATOR_ID_BUCK7)) {
			mask |= BUCK6_7_CNFG_FPWM_MASK;
			if (pdata->flags & SD_FORCED_PWM_MODE)
				val |= BUCK6_7_CNFG_FPWM_MASK;
		}


		ret = max77660_regulator_cache_write(reg,
				rinfo->regs[CFG_REG].addr, mask, val,
				&reg->val[CFG_REG]);
		if (ret < 0) {
			dev_err(reg->dev, "%s:Failed to set register 0x%x\n",
				__func__, rinfo->regs[CFG_REG].addr);
			return ret;
		}
	}
	return 0;
}

#define REGULATOR_BUCK(_id, _volt_mask, _fps_reg, _min_uV, _max_uV, _step_uV) \
	[MAX77660_REGULATOR_ID_##_id] = {			\
		.id = MAX77660_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_BUCK,			\
		.volt_mask = _volt_mask##_VOLT_MASK,		\
		.regs = {					\
			[VOLT_REG] = {				\
				.addr = MAX77660_REG_##_id##_VOUT,	\
			},					\
			[CFG_REG] = {				\
				.addr = MAX77660_REG_##_id##_CNFG, \
			},					\
			[FPS_REG] = {				\
				.addr = MAX77660_REG_FPS_##_id, \
			},					\
		},						\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.power_mode_mask = BUCK_POWER_MODE_MASK,		\
		.power_mode_shift = BUCK_POWER_MODE_SHIFT,	\
		.desc = {					\
			.name = max77660_rails(_id),		\
			.id = MAX77660_REGULATOR_ID_##_id,	\
			.ops = &max77660_ldo_ops,		\
			.type = REGULATOR_VOLTAGE,		\
			.owner = THIS_MODULE,			\
		},						\
	}

#define REGULATOR_LDO(_id, _type, _min_uV, _max_uV, _step_uV)	\
	[MAX77660_REGULATOR_ID_##_id] = {			\
		.id = MAX77660_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_LDO_##_type,		\
		.volt_mask = LDO_VOLT_MASK,			\
		.regs = {					\
			[VOLT_REG] = {				\
				.addr = MAX77660_REG_##_id##_CNFG, \
			},					\
			[CFG_REG] = {				\
				.addr = MAX77660_REG_##_id##_CNFG, \
			},					\
			[FPS_REG] = {				\
				.addr = MAX77660_REG_FPS_##_id,	\
			},					\
		},						\
		.min_uV = _min_uV,				\
		.max_uV = _max_uV,				\
		.step_uV = _step_uV,				\
		.power_mode_mask = LDO_POWER_MODE_MASK,		\
		.power_mode_shift = LDO_POWER_MODE_SHIFT,	\
		.desc = {					\
			.name = max77660_rails(_id),		\
			.id = MAX77660_REGULATOR_ID_##_id,	\
			.ops = &max77660_ldo_ops,		\
			.type = REGULATOR_VOLTAGE,		\
			.owner = THIS_MODULE,			\
		},						\
	}

#define REGULATOR_SW(_id)	\
	[MAX77660_REGULATOR_ID_##_id] = {			\
		.id = MAX77660_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_SW,		\
		.volt_mask = 0,	\
		.regs = { \
			[VOLT_REG] = { \
				.addr = MAX77660_REG_SW_EN,	\
			},					\
			[CFG_REG] = {				\
				.addr = MAX77660_REG_##_id##_CNFG, \
			},					\
			[FPS_REG] = {				\
				.addr = MAX77660_REG_FPS_NONE,	\
			},					\
		},						\
		.desc = {					\
			.name = max77660_rails(_id),		\
			.id = MAX77660_REGULATOR_ID_##_id,	\
			.ops = &max77660_sw_ops,		\
			.type = REGULATOR_VOLTAGE,		\
			.owner = THIS_MODULE,			\
		},						\
	}

static struct max77660_regulator_info
	max77660_regs_info[MAX77660_REGULATOR_ID_NR] = {
	REGULATOR_BUCK(BUCK1, SDX, BUCK1,  600000, 1500000, 6250),
	REGULATOR_BUCK(BUCK2, SDX, BUCK2,  600000, 1500000, 6250),
	REGULATOR_BUCK(BUCK3, SDX, BUCK3,  600000, 3787500, 12500),
	REGULATOR_BUCK(BUCK4, SDX, BUCK4,  800000, 1500000, 6250),
	REGULATOR_BUCK(BUCK5, SDX, BUCK5,  600000, 3787500, 12500),
	REGULATOR_BUCK(BUCK6, SD1, BUCK6, 1000000, 4150000, 50000),
	REGULATOR_BUCK(BUCK7, SD1, BUCK7, 1000000, 4150000, 50000),

	REGULATOR_LDO(LDO1, N, 600000, 2175000, 25000),
	REGULATOR_LDO(LDO2, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO3, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO4, P, 800000, 3950000, 12500),
	REGULATOR_LDO(LDO5, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO6, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO7, N, 600000, 2175000, 50000),
	REGULATOR_LDO(LDO8, N, 600000, 2175000, 50000),
	REGULATOR_LDO(LDO9, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO10, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO11, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO12, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO13, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO14, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO15, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO16, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO17, P, 800000, 3950000, 50000),
	REGULATOR_LDO(LDO18, P, 800000, 3950000, 50000),

	REGULATOR_SW(SW1),
	REGULATOR_SW(SW2),
	REGULATOR_SW(SW3),
	REGULATOR_SW(SW4),
	REGULATOR_SW(SW5),
};

static int max77660_regulator_probe(struct platform_device *pdev)
{
	struct max77660_platform_data *pdata =
					dev_get_platdata(pdev->dev.parent);
	struct regulator_desc *rdesc;
	struct max77660_regulator *reg;
	struct max77660_regulator *max_regs;
	struct max77660_regulator_platform_data *reg_pdata;
	int ret = 0;
	int id;
	int reg_id;
	int reg_count;

	if (!pdata) {
		dev_err(&pdev->dev, "No Platform data\n");
		return -ENODEV;
	}

	reg_count = pdata->num_regulator_pdata;
	max_regs = devm_kzalloc(&pdev->dev,
			reg_count * sizeof(*max_regs), GFP_KERNEL);
	if (!max_regs) {
		dev_err(&pdev->dev, "mem alloc for reg failed\n");
		return -ENOMEM;
	}

	for (id = 0; id < reg_count; ++id) {
		reg_pdata = pdata->regulator_pdata[id];
		if (!reg_pdata) {
			dev_err(&pdev->dev,
				"Regulator pltform data not there\n");
			goto clean_exit;
		}

		reg_id = reg_pdata->id;
		reg  = &max_regs[id];
		rdesc = &max77660_regs_info[reg_id].desc;
		reg->rinfo = &max77660_regs_info[reg_id];
		reg->dev = &pdev->dev;
		reg->pdata = reg_pdata;
		reg->regulator_mode = REGULATOR_MODE_NORMAL;
		reg->power_mode = POWER_MODE_NORMAL;

		platform_set_drvdata(pdev, max_regs);

		dev_dbg(&pdev->dev, "probe: name=%s\n", rdesc->name);

		ret = max77660_regulator_preinit(reg);
		if (ret) {
			dev_err(&pdev->dev, "Failed to preinit regulator %s\n",
				rdesc->name);
			goto clean_exit;
		}

		reg->rdev = regulator_register(rdesc, &pdev->dev,
					reg->pdata->reg_init_data, reg, NULL);
		if (IS_ERR(reg->rdev)) {
			dev_err(&pdev->dev, "Failed to register regulator %s\n",
			rdesc->name);
			ret = PTR_ERR(reg->rdev);
			goto clean_exit;
		}
	}

	return 0;

clean_exit:
	while (--id >= 0) {
		reg  = &max_regs[id];
		regulator_unregister(reg->rdev);
	}
	return ret;
}

static int max77660_regulator_remove(struct platform_device *pdev)
{
	struct max77660_regulator *max_regs = platform_get_drvdata(pdev);
	struct max77660_regulator *reg;
	struct max77660_platform_data *pdata =
					dev_get_platdata(pdev->dev.parent);
	int reg_count;

	if (!pdata)
		return 0;

	reg_count = pdata->num_regulator_pdata;
	while (--reg_count >= 0) {
		reg  = &max_regs[reg_count];
		regulator_unregister(reg->rdev);
	}

	return 0;
}

static struct platform_driver max77660_regulator_driver = {
	.probe = max77660_regulator_probe,
	.remove = __devexit_p(max77660_regulator_remove),
	.driver = {
		.name = "max77660-pmic",
		.owner = THIS_MODULE,
	},
};

static int __init max77660_regulator_init(void)
{
	return platform_driver_register(&max77660_regulator_driver);
}
subsys_initcall(max77660_regulator_init);

static void __exit max77660_reg_exit(void)
{
	platform_driver_unregister(&max77660_regulator_driver);
}
module_exit(max77660_reg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77660 Regulator Driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Maxim Integrated");
MODULE_ALIAS("platform:max77660-regulator");
