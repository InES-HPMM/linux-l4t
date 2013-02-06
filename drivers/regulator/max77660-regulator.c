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


#define MAX77660_REG_BUCK_PWR_MODE1	0x37
#define MAX77660_REG_BUCK_PWR_MODE2	0x38
#define MAX77660_REG_BUCK4_DVFS_CNFG	0x39
#define MAX77660_REG_BUCK4_VBR		0x3A
#define MAX77660_REG_BUCK4_PWM		0x3B
#define MAX77660_REG_BUCK4_MVR		0x3C
#define MAX77660_REG_BUCK4_VSR		0x3C
#define MAX77660_REG_LDO_PWR_MODE1	0x3E
#define MAX77660_REG_LDO_PWR_MODE2	0x3F
#define MAX77660_REG_LDO_PWR_MODE3	0x40
#define MAX77660_REG_LDO_PWR_MODE4	0x41
#define MAX77660_REG_LDO_PWR_MODE5	0x42

/* BUCK4 DVFS */
#define DVFS_BASE_VOLTAGE_UV	600000
#define DVFS_VOLTAGE_STEP_UV	6250
#define BUCK4_DVFS_EN_MASK	BIT(1)
#define BUCK4_DVFS_EN_SHIFT	1
#define PWMRST_SHIFT		7
#define PWMEN_SHIFT		2

#define MAX77660_REG_GLBLCNFG7	0xC0

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

/* FPS */
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

static inline struct device *to_max77660_chip(struct max77660_regulator *reg)
{
	return reg->dev->parent;
}

static int
max77660_regulator_set_fps(struct max77660_regulator *reg)
{
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 val;
	u8 mask;
	int ret;

	if ((rinfo->regs[FPS_REG].addr == MAX77660_REG_FPS_NONE) ||
			(reg->fps_src ==  pdata->fps_src))
		return 0;

	switch (pdata->fps_src) {
	case FPS_SRC_0:
	case FPS_SRC_1:
	case FPS_SRC_2:
	case FPS_SRC_3:
	case FPS_SRC_4:
	case FPS_SRC_5:
	case FPS_SRC_6:
		/* FPS SRC setting */
		val = pdata->fps_src << FPS_SRC_SHIFT;
		mask = FPS_SRC_MASK;
		break;
	case FPS_SRC_NONE:
	case FPS_SRC_DEF:
		return 0;
	default:
		return -EINVAL;
	}

	/* FPS power up period setting */
	if (pdata->fps_pu_period != FPS_POWER_PERIOD_DEF) {
		val |= (pdata->fps_pu_period << FPS_PU_PERIOD_SHIFT);
		mask |= FPS_PU_PERIOD_MASK;
	}

	/* FPS power down period setting */
	if (pdata->fps_pd_period != FPS_POWER_PERIOD_DEF) {
		val |= (pdata->fps_pd_period << FPS_PD_PERIOD_SHIFT);
		mask |= FPS_PD_PERIOD_MASK;
	}

	ret = max77660_reg_update(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[FPS_REG].addr, val,
					mask);
	if (ret < 0)
		return ret;

	reg->fps_src = pdata->fps_src;
	return 0;
}

static int
max77660_regulator_set_fps_cfg(struct max77660_regulator *reg,
			       struct max77660_regulator_fps_cfg *fps_cfg)
{
	u8 val = 0, mask = 0;
	int ret = 0;

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

		ret = max77660_reg_update(to_max77660_chip(reg),
						MAX77660_PWR_SLAVE,
						fps_cfg_regs[0].addr, val,
						mask);
		mask = 0;
		val = 0;
		if (fps_cfg->tu_ap_slp != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_SLP_TU_SHIFT);
			mask |= CNFG_FPS_AP_SLP_TU_MASK;
		}

		if (fps_cfg->td_ap_slp != FPS_TIME_PERIOD_DEF) {
			val |= (fps_cfg->tu_ap_off << CNFG_FPS_AP_SLP_TD_SHIFT);
			mask |= CNFG_FPS_AP_SLP_TD_MASK;
		}

		ret = max77660_reg_update(to_max77660_chip(reg),
						MAX77660_PWR_SLAVE,
						fps_cfg_regs[1].addr, val,
						mask);
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

		ret = max77660_reg_update(to_max77660_chip(reg),
						MAX77660_PWR_SLAVE,
						fps_cfg_regs[2].addr, val,
						mask);
	}

	return ret;
}

static int
max77660_regulator_set_fps_cfgs(struct max77660_regulator *reg,
				struct max77660_regulator_fps_cfg *fps_cfgs,
				int num_fps_cfgs)
{
	int i, ret;
	static int fps_cfg_init;

	if (fps_cfg_init)
		return 0;

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

	if (rinfo->type == REGULATOR_TYPE_SW)
		return 0;
	ret = max77660_reg_update(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[PWR_MODE_REG].addr,
					power_mode << shift, mask);
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
	int ret;
	u8 val;

	if (rinfo->type == REGULATOR_TYPE_SW)
		return 0;

	ret = max77660_reg_read(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
				rinfo->regs[PWR_MODE_REG].addr, &val);
	if (ret < 0)
		return ret;

	reg->power_mode = (val & mask) >> shift;
	return reg->power_mode;
}

static int max77660_regulator_do_set_voltage(struct max77660_regulator *reg,
					     int min_uV, int max_uV)
{
	struct max77660_regulator_info *rinfo = reg->rinfo;
	u8 addr = rinfo->regs[VOLT_REG].addr;
	u8 mask = rinfo->volt_mask;
	u8 val;
	int old_uV, new_uV, safe_uV;
	int i, steps = 1;
	int ret = 0;

	if (min_uV < rinfo->min_uV || max_uV > rinfo->max_uV)
		return -EDOM;

	ret = max77660_reg_read(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					addr, &val);
	if (ret < 0)
		return ret;

	old_uV = (val & mask) * rinfo->step_uV + rinfo->min_uV;

	if ((old_uV > min_uV) && (reg->safe_down_uV >= rinfo->step_uV)) {
		steps = DIV_ROUND_UP(old_uV - min_uV, reg->safe_down_uV);
		safe_uV = -reg->safe_down_uV;
	}

	if (steps == 1) {
		val = (min_uV - rinfo->min_uV) / rinfo->step_uV;
		ret = max77660_reg_update(to_max77660_chip(reg),
						MAX77660_PWR_SLAVE,
						addr, val, mask);
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
			ret = max77660_reg_update(to_max77660_chip(reg),
							MAX77660_PWR_SLAVE,
							addr, val, mask);
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
	int volt, ret;
	u8 val;

	ret = max77660_reg_read(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[VOLT_REG].addr, &val);
	if (ret < 0)
		return ret;

	volt  = (val & rinfo->volt_mask) * rinfo->step_uV + rinfo->min_uV;
	dev_dbg(&rdev->dev, "get_voltage: name=%s, volt=%d, val=0x%02x\n",
		rdev->desc->name, volt, val);
	return volt;
}

static int max77660_regulator_enable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	int power_mode = (pdata->flags & GLPM_ENABLE) ?
			 POWER_MODE_GLPM : POWER_MODE_NORMAL;
	u8 val;
	int ret;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "enable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}

	/* N-Channel LDOs don't support Low-Power mode. */
	if ((rinfo->type != REGULATOR_TYPE_LDO_N) &&
			(reg->regulator_mode == REGULATOR_MODE_STANDBY))
		power_mode = POWER_MODE_LPM;

	if (pdata->flags & ENABLE_EN) {
		ret = max77660_reg_read(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLBLCNFG7, &val);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to get GLBLCNFG7 register 0x%x\n",
				MAX77660_REG_GLBLCNFG7);
			return ret;
		}
		/* if pdata->flags has enable_en3,
		 * when regulator_enable for buck4 or ldo8,
		 * en3 will be actually enabled. */
		if (pdata->flags & ENABLE_EN3) {
			val &= ~GLBLCNFG7_EN3_MASK_MASK;
			power_mode = POWER_MODE_DISABLE;
		}

		ret = max77660_reg_write(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLBLCNFG7, val);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to set GLBLCNFG7 register 0x%x\n",
				MAX77660_REG_GLBLCNFG7);
			return ret;

		}
	}

	return max77660_regulator_set_power_mode(reg, power_mode);
}

static int max77660_regulator_disable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	u8 val;
	int ret;

	int power_mode = POWER_MODE_DISABLE;

	if (reg->fps_src != FPS_SRC_NONE) {
		dev_dbg(&rdev->dev, "disable: Regulator %s using %s\n",
			rdev->desc->name, fps_src_name(reg->fps_src));
		return 0;
	}

	if (pdata->flags & ENABLE_EN) {
		ret = max77660_reg_read(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLBLCNFG7, &val);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to get GLBLCNFG7 register 0x%x\n",
				MAX77660_REG_GLBLCNFG7);
			return ret;
		}
		/* if pdata->flags has enable_en3,
		 * when regulator_disable for buck4 or ldo8,
		 * en3 will be actually disabled. */
		if (pdata->flags & ENABLE_EN3)
			val |= GLBLCNFG7_EN3_MASK_MASK;

		ret = max77660_reg_write(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE,
				MAX77660_REG_GLBLCNFG7, val);
		if (ret < 0) {
			dev_err(reg->dev, "preinit: Failed to set GLBLCNFG7 register 0x%x\n",
				MAX77660_REG_GLBLCNFG7);
			return ret;

		}
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
	u8 mask;
	u8 val = 0;
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

	if (reg->rinfo->id == MAX77660_REGULATOR_ID_BUCK6 ||
			reg->rinfo->id == MAX77660_REGULATOR_ID_BUCK7) {
		mask = BUCK6_7_CNFG_FPWM_MASK;
		switch (mode) {
		case REGULATOR_MODE_FAST:
			val = 0;
			break;
		case REGULATOR_MODE_NORMAL:
			val = 1 << BUCK6_7_CNFG_FPWM_SHIFT;
			break;
		}

		ret = max77660_reg_update(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE, rinfo->regs[CFG_REG].addr,
				mask, val);
		if (ret < 0)
			return ret;
	}

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
	ret = max77660_reg_set_bits(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[VOLT_REG].addr, 1 << idx);
	return ret;
}

static int max77660_switch_disable(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int idx, ret;

	idx = rinfo->id - MAX77660_REGULATOR_ID_SW1;
	ret = max77660_reg_clr_bits(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[VOLT_REG].addr, 1 << idx);
	return ret;
}

static int max77660_switch_is_enabled(struct regulator_dev *rdev)
{
	struct max77660_regulator *reg = rdev_get_drvdata(rdev);
	struct max77660_regulator_info *rinfo = reg->rinfo;
	int idx, ret;
	u8 val = 0;

	idx = rinfo->id - MAX77660_REGULATOR_ID_SW1;
	ret = max77660_reg_read(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					rinfo->regs[VOLT_REG].addr, &val);

	return val & (1 << idx);
}

static int max77660_reg_enable_time(struct regulator_dev *dev)
{
	return 500;
}

static struct regulator_ops max77660_regulator_ops = {
	.set_voltage = max77660_regulator_set_voltage,
	.get_voltage = max77660_regulator_get_voltage,
	.enable_time = max77660_reg_enable_time,
	.enable = max77660_regulator_enable,
	.disable = max77660_regulator_disable,
	.is_enabled = max77660_regulator_is_enabled,
	.set_mode = max77660_regulator_set_mode,
	.get_mode = max77660_regulator_get_mode,
};

static struct regulator_ops max77660_sw_ops = {
	.enable = max77660_switch_enable,
	.disable = max77660_switch_disable,
	.enable_time = max77660_reg_enable_time,
	.is_enabled = max77660_switch_is_enabled,
};

static int max77660_regulator_preinit(struct max77660_regulator *reg)
{
	struct max77660_regulator_platform_data *pdata = reg->pdata;
	struct max77660_regulator_info *rinfo = reg->rinfo;
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
	ret = max77660_reg_read(to_max77660_chip(reg), MAX77660_PWR_SLAVE,
					MAX77660_REG_CID5, &val);
	if (ret < 0) {
		dev_err(reg->dev, "preinit: Failed to get register 0x%x\n",
			MAX77660_REG_CID5);
		return ret;
	}

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

	/* enable EN */
	if (pdata->flags & ENABLE_EN) {
		if (pdata->flags & ENABLE_EN3)
			max77660_regulator_set_power_mode(reg,
				POWER_MODE_DISABLE);
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

			mask |= BUCK1_5_CNFG_DVFS_EN_MASK;
			if (pdata->flags & DISABLE_DVFS)
				val |= BUCK1_5_CNFG_DVFS_EN_MASK;
		} else if ((reg->rinfo->id >= MAX77660_REGULATOR_ID_BUCK6) &&
			(reg->rinfo->id <= MAX77660_REGULATOR_ID_BUCK7)) {
			mask |= BUCK6_7_CNFG_FPWM_MASK;
			if (pdata->flags & SD_FORCED_PWM_MODE)
				val |= BUCK6_7_CNFG_FPWM_MASK;
		}

		ret = max77660_reg_update(to_max77660_chip(reg),
						MAX77660_PWR_SLAVE,
						rinfo->regs[CFG_REG].addr, val,
						mask);
		if (ret < 0) {
			dev_err(reg->dev, "%s:Failed to set register 0x%x\n",
				__func__, rinfo->regs[CFG_REG].addr);
			return ret;
		}
	}
	return 0;
}

#define REGULATOR_BUCK(_id, _volt_mask, _fps_reg, _min_uV, _max_uV, _step_uV)	\
	[MAX77660_REGULATOR_ID_##_id] = {					\
		.id = MAX77660_REGULATOR_ID_##_id,				\
		.type = REGULATOR_TYPE_BUCK,					\
		.volt_mask = _volt_mask##_VOLT_MASK,				\
		.regs = {							\
			[VOLT_REG] = {						\
				.addr = MAX77660_REG_##_id##_VOUT,		\
			},							\
			[CFG_REG] = {						\
				.addr = MAX77660_REG_##_id##_CNFG,		\
			},							\
			[FPS_REG] = {						\
				.addr = MAX77660_REG_FPS_##_id, 		\
			},							\
		},								\
		.min_uV = _min_uV,						\
		.max_uV = _max_uV,						\
		.step_uV = _step_uV,						\
		.power_mode_mask = BUCK_POWER_MODE_MASK,			\
		.power_mode_shift = BUCK_POWER_MODE_SHIFT,			\
		.desc = {							\
			.name = max77660_rails(_id),				\
			.id = MAX77660_REGULATOR_ID_##_id,			\
			.ops = &max77660_regulator_ops,				\
			.type = REGULATOR_VOLTAGE,				\
			.owner = THIS_MODULE,					\
		},								\
	}

#define REGULATOR_LDO(_id, _type, _min_uV, _max_uV, _step_uV)			\
	[MAX77660_REGULATOR_ID_##_id] = {					\
		.id = MAX77660_REGULATOR_ID_##_id,				\
		.type = REGULATOR_TYPE_LDO_##_type,				\
		.volt_mask = LDO_VOLT_MASK,					\
		.regs = {							\
			[VOLT_REG] = {						\
				.addr = MAX77660_REG_##_id##_CNFG, 		\
			},							\
			[CFG_REG] = {						\
				.addr = MAX77660_REG_##_id##_CNFG,		\
			},							\
			[FPS_REG] = {						\
				.addr = MAX77660_REG_FPS_##_id,			\
			},							\
		},								\
		.min_uV = _min_uV,						\
		.max_uV = _max_uV,						\
		.step_uV = _step_uV,						\
		.power_mode_mask = LDO_POWER_MODE_MASK,				\
		.power_mode_shift = LDO_POWER_MODE_SHIFT,			\
		.desc = {							\
			.name = max77660_rails(_id),				\
			.id = MAX77660_REGULATOR_ID_##_id,			\
			.ops = &max77660_regulator_ops,				\
			.type = REGULATOR_VOLTAGE,				\
			.owner = THIS_MODULE,					\
		},								\
	}

#define REGULATOR_SW(_id)					\
	[MAX77660_REGULATOR_ID_##_id] = {			\
		.id = MAX77660_REGULATOR_ID_##_id,		\
		.type = REGULATOR_TYPE_SW,			\
		.volt_mask = 0,					\
		.regs = { 					\
			[VOLT_REG] = { 				\
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

static int max77660_pwm_dvfs_init(struct device *parent,
					struct max77660_platform_data *pdata)
{
	u8 val = 0;
	int ret;
	struct max77660_pwm_dvfs_init_data *dvfs_pd = &pdata->dvfs_pd;

	if (!dvfs_pd->en_pwm)
		return 0;

	val = DIV_ROUND_UP((dvfs_pd->default_voltage_uV - DVFS_BASE_VOLTAGE_UV),
			DVFS_VOLTAGE_STEP_UV);
	ret = max77660_reg_write(parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_BUCK4_VSR, val);
	if (ret < 0)
		return ret;

	ret = max77660_reg_update(parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_BUCK4_CNFG,
			1 << BUCK4_DVFS_EN_SHIFT,
			BUCK4_DVFS_EN_MASK);
	if (ret < 0)
		return ret;

	val = (1 << PWMEN_SHIFT);
	switch (dvfs_pd->step_voltage_uV) {
	case 12250:
		val |= 0x1;
		break;
	case 25000:
		val |= 0x2;
		break;
	}

	ret = max77660_reg_write(parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_BUCK4_DVFS_CNFG, val);
	if (ret < 0)
		return ret;

	val = DIV_ROUND_UP((dvfs_pd->base_voltage_uV - DVFS_BASE_VOLTAGE_UV),
			DVFS_VOLTAGE_STEP_UV);
	ret = max77660_reg_write(parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_BUCK4_VBR, val);
	if (ret < 0)
		return ret;

	val = DIV_ROUND_UP((dvfs_pd->max_voltage_uV - DVFS_BASE_VOLTAGE_UV),
			DVFS_VOLTAGE_STEP_UV);
	ret = max77660_reg_write(parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_BUCK4_MVR, val);
	return ret;

}

static int max77660_regulator_probe(struct platform_device *pdev)
{
	struct max77660_platform_data *pdata =
					dev_get_platdata(pdev->dev.parent);
	struct regulator_desc *rdesc;
	struct max77660_regulator *reg;
	struct max77660_regulator *max_regs;
	struct regulator_config config = { };
	int ret = 0;
	int id;
	int reg_id;

	if (!pdata) {
		dev_err(&pdev->dev, "No Platform data\n");
		return -ENODEV;
	}

	max_regs = devm_kzalloc(&pdev->dev,
			MAX77660_REGULATOR_ID_NR * sizeof(*max_regs), GFP_KERNEL);
	if (!max_regs) {
		dev_err(&pdev->dev, "mem alloc for reg failed\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, max_regs);

	for (id = 0; id < MAX77660_REGULATOR_ID_NR; ++id) {
		struct max77660_regulator_platform_data *reg_pdata;
		struct regulator_init_data *reg_init_data = NULL;

		reg_pdata = pdata->regulator_pdata[id];

		reg_id = id;
		reg  = &max_regs[id];
		rdesc = &max77660_regs_info[reg_id].desc;
		reg->rinfo = &max77660_regs_info[reg_id];
		reg->dev = &pdev->dev;
		reg->pdata = reg_pdata;
		if (reg_pdata)
			reg_init_data = reg_pdata->reg_init_data;

		reg->regulator_mode = REGULATOR_MODE_NORMAL;
		reg->power_mode = POWER_MODE_NORMAL;

		dev_dbg(&pdev->dev, "probe: name=%s\n", rdesc->name);

		if (reg_pdata) {
			ret = max77660_regulator_preinit(reg);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"Preinit regualtor %s failed: %d\n",
					rdesc->name, ret);
				goto clean_exit;
			}
		}

		/* ES1.0 errata: Clear active discharge for LDO1 */
		if (max77660_is_es_1_0(&pdev->dev) &&
			(id == MAX77660_REGULATOR_ID_LDO1)) {
			ret = max77660_reg_clr_bits(to_max77660_chip(reg),
				MAX77660_PWR_SLAVE, MAX77660_REG_LDO1_CNFG,
				LDO1_18_CNFG_ADE_MASK);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"LDO1_CNFG update failed: %d\n", ret);
				goto clean_exit;
			}
		}

		config.dev = &pdev->dev;
		config.init_data = reg_init_data;
		config.driver_data = reg;

		reg->rdev = regulator_register(rdesc, &config);
		if (IS_ERR(reg->rdev)) {
			ret = PTR_ERR(reg->rdev);
			dev_err(&pdev->dev,
				"regulator %s register failed: %d\n",
				rdesc->name, ret);
			goto clean_exit;
		}
	}

	ret = max77660_pwm_dvfs_init(pdev->dev.parent, pdata);
	if (ret)
		dev_err(&pdev->dev, "Failed to initialize BUCK4 dvfs");

	return 0;

clean_exit:
	while (--id >= 0) {
		reg  = &max_regs[id];
		if (reg->dev)
			regulator_unregister(reg->rdev);
	}
	return ret;
}

static int max77660_regulator_remove(struct platform_device *pdev)
{
	struct max77660_regulator *max_regs = platform_get_drvdata(pdev);
	struct max77660_regulator *reg;
	int reg_count = MAX77660_REGULATOR_ID_NR;

	while (--reg_count >= 0) {
		reg  = &max_regs[reg_count];
		if (reg->dev)
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
