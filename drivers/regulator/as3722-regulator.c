/*
 * as3722-regulator.c - voltage regulator support for AS3722
 *
 * Copyright (C) 2013 ams
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/regulator/driver.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>

struct as3722_register_mapping {
	u8 reg_id;
	u8 reg_vsel;
	u32 reg_enable;
	u8 enable_bit;
	u32 sleep_ctrl_reg;
	u8 sleep_ctrl_bit_mask_mask;
};

struct as3722_register_mapping as3722_reg_lookup[] = {
	{
		.reg_id = AS3722_SD0,
		.reg_vsel = AS3722_SD0_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD0_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL1,
		.sleep_ctrl_bit_mask_mask = AS3722_SD0_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD1,
		.reg_vsel = AS3722_SD1_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD1_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL1,
		.sleep_ctrl_bit_mask_mask = AS3722_SD1_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD2,
		.reg_vsel = AS3722_SD2_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD2_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL1,
		.sleep_ctrl_bit_mask_mask = AS3722_SD2_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD3,
		.reg_vsel = AS3722_SD3_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD3_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL1,
		.sleep_ctrl_bit_mask_mask = AS3722_SD3_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD4,
		.reg_vsel = AS3722_SD4_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD4_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL2,
		.sleep_ctrl_bit_mask_mask = AS3722_SD4_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD5,
		.reg_vsel = AS3722_SD5_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD5_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL2,
		.sleep_ctrl_bit_mask_mask = AS3722_SD5_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_SD6,
		.reg_vsel = AS3722_SD6_VOLTAGE_REG,
		.reg_enable = AS3722_SD_CONTROL_REG,
		.enable_bit = AS3722_SD6_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL2,
		.sleep_ctrl_bit_mask_mask = AS3722_SD6_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO0,
		.reg_vsel = AS3722_LDO0_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO0_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL3,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO0_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO1,
		.reg_vsel = AS3722_LDO1_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO1_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL3,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO1_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO2,
		.reg_vsel = AS3722_LDO2_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO2_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL3,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO2_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO3,
		.reg_vsel = AS3722_LDO3_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO3_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL3,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO3_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO4,
		.reg_vsel = AS3722_LDO4_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO4_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL4,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO4_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO5,
		.reg_vsel = AS3722_LDO5_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO5_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL4,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO5_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO6,
		.reg_vsel = AS3722_LDO6_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO6_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL4,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO6_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO7,
		.reg_vsel = AS3722_LDO7_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL0_REG,
		.enable_bit = AS3722_LDO7_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL4,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO7_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO9,
		.reg_vsel = AS3722_LDO9_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL1_REG,
		.enable_bit = AS3722_LDO9_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL5,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO9_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO10,
		.reg_vsel = AS3722_LDO10_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL1_REG,
		.enable_bit = AS3722_LDO10_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL5,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO10_EXTERNAL_ENABLE_MASK,
	},
	{
		.reg_id = AS3722_LDO11,
		.reg_vsel = AS3722_LDO11_VOLTAGE_REG,
		.reg_enable = AS3722_LDOCONTROL1_REG,
		.enable_bit = AS3722_LDO11_ON,
		.sleep_ctrl_reg = AS3722_EXTERNAL_ENABLE_CTRL5,
		.sleep_ctrl_bit_mask_mask = AS3722_LDO11_EXTERNAL_ENABLE_MASK,
	},
};

/*
 * as3722 ldo0 extended input range (0.825-1.25V)  */
static int as3722_ldo0_is_enabled(struct regulator_dev *dev)
{
	u32 val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, AS3722_LDOCONTROL0_REG, &val);
	return (val & AS3722_LDO0_CTRL_MASK) != 0;
}

static int as3722_ldo0_enable(struct regulator_dev *dev)
{
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_enable,
			AS3722_LDO0_CTRL_MASK, AS3722_LDO0_ON);
}

static int as3722_ldo0_disable(struct regulator_dev *dev)
{
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_enable,
			AS3722_LDO0_CTRL_MASK, 0);
}

static int as3722_ldo0_list_voltage(struct regulator_dev *dev,
		unsigned selector)
{
	if (selector >= AS3722_LDO0_VSEL_MAX)
		return -EINVAL;

	return 800000 + (selector + 1) * 25000;
}

static int as3722_ldo0_get_voltage(struct regulator_dev *dev)
{
	u32 val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel, &val);
	val &= AS3722_LDO_VSEL_MASK;
	if (val > 0)
		val--;  /* ldo vsel has min value of 1, selector starts
			   at 0 */

	return as3722_ldo0_list_voltage(dev, val);
}

static int as3722_ldo0_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 reg_val;
	int val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	if (min_uV > 1250000 || max_uV < 825000)
		return -EINVAL;

	/* 25mV steps from 0.825V-1.25V */
	val = (min_uV - 800001) / 25000 + 1;
	if (val < 1)
		val = 1;

	reg_val = (u8) val;
	if (reg_val * 25000 + 800000 > max_uV)
		return -EINVAL;

	BUG_ON(reg_val * 25000 + 800000 < min_uV);
	BUG_ON(reg_val > AS3722_LDO0_VSEL_MAX);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel,
			AS3722_LDO_VSEL_MASK, reg_val);
}

static int as3722_ldo0_get_current_limit(struct regulator_dev *dev)
{
	u32 val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[rdev_get_id(dev)].reg_vsel,
			&val);
	val &= AS3722_LDO_ILIMIT_MASK;

	/* return ldo specific values */
	if (val)
		return 300000;

	return 150000;
}

static int as3722_ldo0_set_current_limit(struct regulator_dev *dev,
		int min_uA, int max_uA)
{
	u8 val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	/* we check the values in case the constraints are wrong */
	if (min_uA <= 150000 && max_uA >= 150000)
		val = 0;
	else if (min_uA > 150000 && max_uA >= 300000)
		val = AS3722_LDO_ILIMIT_BIT;
	else
		return -EINVAL;

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel,
			AS3722_LDO_ILIMIT_MASK, val);
}

static struct regulator_ops as3722_ldo0_ops = {
	.is_enabled = as3722_ldo0_is_enabled,
	.enable = as3722_ldo0_enable,
	.disable = as3722_ldo0_disable,
	.list_voltage = as3722_ldo0_list_voltage,
	.get_voltage = as3722_ldo0_get_voltage,
	.set_voltage = as3722_ldo0_set_voltage,
	.get_current_limit = as3722_ldo0_get_current_limit,
	.set_current_limit = as3722_ldo0_set_current_limit,
};

/*
 * as3722 ldo3 low output range (0.61V-1.5V)  */
static int as3722_ldo3_is_enabled(struct regulator_dev *dev)
{
	u32 val = 0;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[rdev_get_id(dev)].reg_enable,
			&val);
	return (val & AS3722_LDO3_CTRL_MASK) != 0;
}

static int as3722_ldo3_enable(struct regulator_dev *dev)
{
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_enable,
			AS3722_LDO3_CTRL_MASK, AS3722_LDO3_ON);
}

static int as3722_ldo3_disable(struct regulator_dev *dev)
{
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_enable,
			AS3722_LDO3_CTRL_MASK, 0);
}

static int as3722_ldo3_list_voltage(struct regulator_dev *dev,
		unsigned selector)
{
	if (selector >= AS3722_LDO3_VSEL_MAX)
		return -EINVAL;

	return 600000 + (selector + 1) * 20000;
}

static int as3722_ldo3_get_voltage(struct regulator_dev *dev)
{
	u32 val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel, &val);
	val &= AS3722_LDO3_VSEL_MASK;
	if (val > 0)
		val--;  /* ldo vsel has min value 1, selector starts at
			   0 */

	return as3722_ldo3_list_voltage(dev, val);
}

static int as3722_ldo3_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 reg_val;
	int val;
	struct as3722 *as3722 = rdev_get_drvdata(dev);


	if (min_uV > 1500000 || max_uV < 620000)
		return -EINVAL;

	/* 20mV steps from 0.62V to 1.5V */
	val = (min_uV - 600001) / 20000 + 1;
	if (val < 1)
		val = 1;

	reg_val = (u8) val;
	if (reg_val * 20000 + 600000 > max_uV)
		return -EINVAL;

	BUG_ON(reg_val * 20000 + 600000 < min_uV);
	BUG_ON(reg_val > AS3722_LDO3_VSEL_MAX);

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel,
			AS3722_LDO3_VSEL_MASK, reg_val);
}

static int as3722_ldo3_get_current_limit(struct regulator_dev *dev)
{
	return 150000;
}

static int as3722_ldo3_set_mode(struct regulator_dev *dev, u8 mode)
{
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	if (mode != AS3722_LDO3_MODE_PMOS && mode
		!= AS3722_LDO3_MODE_PMOS_TRACKING && mode
		!= AS3722_LDO3_MODE_NMOS && mode != AS3722_LDO3_MODE_SWITCH)
		return -EINVAL;

	return as3722_set_bits(as3722,
			as3722_reg_lookup[rdev_get_id(dev)].reg_vsel,
			AS3722_LDO3_MODE_MASK, mode);
}

static struct regulator_ops as3722_ldo3_ops = {
	.is_enabled = as3722_ldo3_is_enabled,
	.enable = as3722_ldo3_enable,
	.disable = as3722_ldo3_disable,
	.list_voltage = as3722_ldo3_list_voltage,
	.get_voltage = as3722_ldo3_get_voltage,
	.set_voltage = as3722_ldo3_set_voltage,
	.get_current_limit = as3722_ldo3_get_current_limit,
};

/*
 * as3722 ldo 1-2 and 4-11 (0.8V-3.3V)
 */
static int as3722_ldo_is_enabled(struct regulator_dev *dev)
{
	u32 val = 0;
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[id].reg_enable, &val);
	return (val & as3722_reg_lookup[id].enable_bit) != 0;
}

static int as3722_ldo_enable(struct regulator_dev *dev)
{
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_enable,
			as3722_reg_lookup[id].enable_bit,
			as3722_reg_lookup[id].enable_bit);
}

static int as3722_ldo_disable(struct regulator_dev *dev)
{
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_enable,
			as3722_reg_lookup[id].enable_bit, 0);
}

static int as3722_ldo_list_voltage(struct regulator_dev *dev,
		unsigned selector)
{
	if (selector >= AS3722_LDO_NUM_VOLT)
		return -EINVAL;

	selector++;     /* ldo vsel min value is 1, selector starts at 0. */
	return 800000 + selector * 25000;
}

static int as3722_ldo_get_voltage(struct regulator_dev *dev)
{
	u32 val;
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[id].reg_vsel, &val);
	val &= AS3722_LDO_VSEL_MASK;
	/* ldo vsel has a gap from 0x25 to 0x3F (27 values). */
	if (val > AS3722_LDO_VSEL_DNU_MAX)
		val -= 27;
	/* ldo vsel min value is 1, selector starts at 0. */
	if (val > 0)
		val--;

	return as3722_ldo_list_voltage(dev, val);
}

static int as3722_ldo_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 reg_val;
	int val;
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);


	if (min_uV > 3300000 || max_uV < 825000)
		return -EINVAL;

	if (min_uV <= 1700000) {
		/* 25mV steps from 0.825V to 1.7V */
		val = (min_uV - 800001) / 25000 + 1;
		if (val < 1)
			val = 1;
		reg_val = (u8) val;
		if (reg_val * 25000 + 800000 > max_uV)
			return -EINVAL;
		BUG_ON(reg_val * 25000 + 800000 < min_uV);
	} else {
		/* 25mV steps from 1.725V to 3.3V */
		reg_val = (min_uV - 1700001) / 25000 + 0x40;
		if ((reg_val - 0x40) * 25000 + 1725000 > max_uV)
			return -EINVAL;
		BUG_ON((reg_val - 0x40) * 25000 + 1725000 < min_uV);
	}

	BUG_ON(reg_val > AS3722_LDO_VSEL_MAX);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_vsel,
			AS3722_LDO_VSEL_MASK, reg_val);
}

static int as3722_ldo_get_current_limit(struct regulator_dev *dev)
{
	u32 val;
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[id].reg_vsel, &val);
	val &= AS3722_LDO_ILIMIT_MASK;

	/* return ldo specific values */
	if (val)
		return 300000;

	return 150000;
}

static int as3722_ldo_set_current_limit(struct regulator_dev *dev,
		int min_uA, int max_uA)
{
	u8 val;
	int loweruA = 150000;
	int id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	/* we check the values in case the constraints are wrong */
	if (min_uA <= loweruA && max_uA >= loweruA)
		val = 0;
	else if (min_uA > loweruA && max_uA >= 300000)
		val = AS3722_LDO_ILIMIT_BIT;
	else
		return -EINVAL;

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_vsel,
			AS3722_LDO_ILIMIT_MASK, val);
}

static int as3722_ldo_enable_time(struct regulator_dev *dev)
{
	return 2000;
}

static struct regulator_ops as3722_ldo_ops = {
	.is_enabled = as3722_ldo_is_enabled,
	.enable = as3722_ldo_enable,
	.disable = as3722_ldo_disable,
	.list_voltage = as3722_ldo_list_voltage,
	.get_voltage = as3722_ldo_get_voltage,
	.set_voltage = as3722_ldo_set_voltage,
	.get_current_limit = as3722_ldo_get_current_limit,
	.set_current_limit = as3722_ldo_set_current_limit,
	.enable_time = as3722_ldo_enable_time,
};

/*
 * as3722 step down
 */
static int as3722_sd_is_enabled(struct regulator_dev *dev)
{
	u32 val;
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[id].reg_enable, &val);

	return (val & as3722_reg_lookup[id].enable_bit) != 0;
}

static int as3722_sd_enable(struct regulator_dev *dev)
{
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_enable,
			as3722_reg_lookup[id].enable_bit,
			as3722_reg_lookup[id].enable_bit);
}

static int as3722_sd_disable(struct regulator_dev *dev)
{
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_enable,
			as3722_reg_lookup[id].enable_bit, 0);
}

static unsigned int as3722_sd_get_mode(struct regulator_dev *dev)
{
	u32 val;
	u8 reg_id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, AS3722_SD_CONTROL_REG, &val);

	switch (rdev_get_id(dev)) {
	case AS3722_SD0:
		as3722_reg_read(as3722, AS3722_SD0_CONTROL_REG, &val);
		if ((val & AS3722_SD0_MODE_MASK) == AS3722_SD0_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD1:
		as3722_reg_read(as3722, AS3722_SD1_CONTROL_REG, &val);
		if ((val & AS3722_SD1_MODE_MASK) == AS3722_SD1_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD2:
		as3722_reg_read(as3722, AS3722_SD23_CONTROL_REG, &val);
		if ((val & AS3722_SD2_MODE_MASK) == AS3722_SD2_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD3:
		as3722_reg_read(as3722, AS3722_SD23_CONTROL_REG, &val);
		if ((val & AS3722_SD3_MODE_MASK) == AS3722_SD3_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD4:
		as3722_reg_read(as3722, AS3722_SD4_CONTROL_REG, &val);
		if ((val & AS3722_SD1_MODE_MASK) == AS3722_SD1_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD5:
		as3722_reg_read(as3722, AS3722_SD5_CONTROL_REG, &val);
		if ((val & AS3722_SD1_MODE_MASK) == AS3722_SD1_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	case AS3722_SD6:
		as3722_reg_read(as3722, AS3722_SD6_CONTROL_REG, &val);
		if ((val & AS3722_SD1_MODE_MASK) == AS3722_SD1_MODE_FAST)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	default:
		dev_err(as3722->dev, "regulator id %d invalid.\n",
				reg_id);
	}

	return -ERANGE;
}

static int as3722_sd_set_mode(struct regulator_dev *dev,
		unsigned int mode)
{
	u8 val, mask, reg;
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	if (mode != REGULATOR_MODE_FAST && mode != REGULATOR_MODE_NORMAL)
		return -EINVAL;

	switch (id) {
	case AS3722_SD0:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD0_MODE_FAST;
		else
			val = AS3722_SD0_MODE_NORMAL;

		reg = AS3722_SD0_CONTROL_REG;
		mask = AS3722_SD0_MODE_MASK;
		break;
	case AS3722_SD1:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD1_MODE_FAST;
		else
			val = AS3722_SD1_MODE_NORMAL;

		reg = AS3722_SD1_CONTROL_REG;
		mask = AS3722_SD1_MODE_MASK;
		break;
	case AS3722_SD2:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD2_MODE_FAST;
		else
			val = AS3722_SD2_MODE_NORMAL;

		reg = AS3722_SD23_CONTROL_REG;
		mask = AS3722_SD2_MODE_MASK;
		break;
	case AS3722_SD3:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD3_MODE_FAST;
		else
			val = AS3722_SD3_MODE_NORMAL;

		reg = AS3722_SD23_CONTROL_REG;
		mask = AS3722_SD3_MODE_MASK;
		break;
	case AS3722_SD4:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD4_MODE_FAST;
		else
			val = AS3722_SD4_MODE_NORMAL;

		reg = AS3722_SD4_CONTROL_REG;
		mask = AS3722_SD4_MODE_MASK;
		break;
	case AS3722_SD5:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD5_MODE_FAST;
		else
			val = AS3722_SD5_MODE_NORMAL;

		reg = AS3722_SD5_CONTROL_REG;
		mask = AS3722_SD5_MODE_MASK;
		break;
	case AS3722_SD6:
		if (mode == REGULATOR_MODE_FAST)
			val = AS3722_SD6_MODE_FAST;
		else
			val = AS3722_SD6_MODE_NORMAL;

		reg = AS3722_SD6_CONTROL_REG;
		mask = AS3722_SD6_MODE_MASK;
		break;
	default:
		dev_err(as3722->dev, "regulator id %d invalid.\n",
				id);
		return -EINVAL;
	}

	return as3722_set_bits(as3722, reg, mask, val);
}

static int as3722_sd_list_voltage(struct regulator_dev *dev, unsigned
		selector) {
	u8 id = rdev_get_id(dev);

	if (id == AS3722_SD0 || id == AS3722_SD1 || id == AS3722_SD6) {
		if (selector >= AS3722_SD0_VSEL_MAX)
			return -EINVAL;

		return 600000 + (selector + 1) * 10000;
	} else {
		if (selector > AS3722_SD2_VSEL_MAX)
			return -EINVAL;

		/* ldo vsel min value is 1, selector starts at 0. */
		selector++;
		if (selector <= 0x40)
			return 600000 + selector * 12500;
		if (selector <= 0x70)
			return 1400000 + (selector - 0x40) * 25000;
		if (selector <= 0x7F)
			return 2600000 + (selector - 0x70) * 50000;

		return -ERANGE;
	}
	return -EINVAL;
}

static int as3722_sd_get_voltage(struct regulator_dev *dev)
{
	u32 val;
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	as3722_reg_read(as3722, as3722_reg_lookup[id].reg_vsel, &val);
	val &= AS3722_SD_VSEL_MASK;
	if (val > 0)
		val--;  /* ldo vsel min value is 1, selector starts at
			   0. */

	return as3722_sd_list_voltage(dev, val);
}

static int as3722_sd_lowpower_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 reg_val;
	int val;
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	/*       0 ... 0        0x00 : not allowed as voltage setting
	 *  610000 ... 1500000: 0x01 - 0x40, 10mV steps */

	if (min_uV > 1500000 || max_uV < 610000)
		return -EINVAL;

	val = (min_uV - 600001) / 10000 + 1;
	if (val < 1)
		val = 1;

	reg_val = (u8) val;
	if (reg_val * 10000 + 600000 > max_uV)
		return -EINVAL;
	BUG_ON(reg_val * 10000 + 600000 < min_uV);

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_vsel,
			AS3722_SD_VSEL_MASK, reg_val);
}

static int as3722_sd_nom_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 reg_val;
	int val;
	u8 id = rdev_get_id(dev);
	struct as3722 *as3722 = rdev_get_drvdata(dev);

	/*       0 ... 0        0x00 : not allowed as voltage setting
	 *  612500 ... 1400000: 0x01 - 0x40, 12.5mV steps
	 * 1425000 ... 2600000: 0x41 - 0x70, 25mV steps
	 * 2650000 ... 3350000: 0x41 - 0x70, 50mV steps */

	if (min_uV > 3350000 || max_uV < 612500)
		return -EINVAL;

	if (min_uV <= 1400000) {
		val = (min_uV - 600001) / 12500 + 1;
		if (val < 1)
			val = 1;

		reg_val = (u8) val;
		if ((reg_val * 12500) + 600000 > max_uV)
			return -EINVAL;

		BUG_ON((reg_val * 12500) + 600000 < min_uV);

	} else if (min_uV <= 2600000) {
		reg_val = (min_uV - 1400001) / 25000 + 1;

		if ((reg_val * 25000) + 1400000 > max_uV)
			return -EINVAL;

		BUG_ON((reg_val * 25000) + 1400000 < min_uV);

		reg_val += 0x40;

	} else {

		reg_val = (min_uV - 2600001) / 50000 + 1;

		if ((reg_val * 50000) + 2600000 > max_uV)
			return -EINVAL;

		BUG_ON((reg_val * 50000) + 2600000 < min_uV);

		reg_val += 0x70;
	}

	return as3722_set_bits(as3722, as3722_reg_lookup[id].reg_vsel,
			AS3722_SD_VSEL_MASK, reg_val);
}

static int as3722_sd_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
{
	u8 id = rdev_get_id(dev);

	if (id == AS3722_SD0 || id == AS3722_SD1 || id == AS3722_SD6)
		return as3722_sd_lowpower_set_voltage(dev, min_uV, max_uV,
							selector);
	else
		return as3722_sd_nom_set_voltage(dev, min_uV, max_uV,
							selector);
}

static int as3722_sd_enable_time(struct regulator_dev *dev)
{
	return 2000;
}

static struct regulator_ops as3722_sd_ops = {
	.is_enabled = as3722_sd_is_enabled,
	.enable = as3722_sd_enable,
	.disable = as3722_sd_disable,
	.list_voltage = as3722_sd_list_voltage,
	.get_voltage = as3722_sd_get_voltage,
	.set_voltage = as3722_sd_set_voltage,
	.get_mode = as3722_sd_get_mode,
	.set_mode = as3722_sd_set_mode,
	.enable_time = as3722_sd_enable_time,
};

static struct regulator_desc regulators[] = {
	{
		.name = "as3722-sd0",
		.id = AS3722_SD0,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD0_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd1",
		.id = AS3722_SD1,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD0_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd2",
		.id = AS3722_SD2,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD2_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd3",
		.id = AS3722_SD3,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD2_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd4",
		.id = AS3722_SD4,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD2_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd5",
		.id = AS3722_SD5,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD2_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-sd6",
		.id = AS3722_SD6,
		.ops = &as3722_sd_ops,
		.n_voltages = AS3722_SD0_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo0",
		.id = AS3722_LDO0,
		.ops = &as3722_ldo0_ops,
		.n_voltages = AS3722_LDO0_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo1",
		.id = AS3722_LDO1,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo2",
		.id = AS3722_LDO2,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo3",
		.id = AS3722_LDO3,
		.ops = &as3722_ldo3_ops,
		.n_voltages = AS3722_LDO3_VSEL_MAX,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo4",
		.id = AS3722_LDO4,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo5",
		.id = AS3722_LDO5,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo6",
		.id = AS3722_LDO6,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo7",
		.id = AS3722_LDO7,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo9",
		.id = AS3722_LDO9,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo10",
		.id = AS3722_LDO10,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "as3722-ldo11",
		.id = AS3722_LDO11,
		.ops = &as3722_ldo_ops,
		.n_voltages = AS3722_LDO_NUM_VOLT,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int as3722_extreg_init(struct as3722 *as3722, int id, int ext_pwr_ctrl)
{
	int ret;
	u32 sleep_ctrl_mask = 0;
	u32 shift_count = 0;

	if ((ext_pwr_ctrl < AS3722_EXT_CONTROL_ENABLE1) ||
		(ext_pwr_ctrl > AS3722_EXT_CONTROL_ENABLE3))
		return -EINVAL;

	sleep_ctrl_mask = as3722_reg_lookup[id].sleep_ctrl_bit_mask_mask;
	while (!(sleep_ctrl_mask & (0x1 << shift_count))) {
		shift_count++;
	}

	ret = as3722_set_bits(as3722, as3722_reg_lookup[id].sleep_ctrl_reg,
			as3722_reg_lookup[id].sleep_ctrl_bit_mask_mask,
			ext_pwr_ctrl << shift_count);
	if (ret < 0)
		return ret;

	return 0;
}

static int oc_alarm_table[] = {0, 1600, 1800, 2000, 2200, 2400, 2600, 2800};

static int as3722_overcurrent_init(struct as3722 *as3722, int id,
		struct as3722_regulator_platform_data *rpdata)
{
	int ret;
	int oc_trip = rpdata->oc_trip_thres_perphase;
	int oc_alarm = rpdata->oc_alarm_thres_perphase;
	int mask;
	int reg;
	int trip_val;
	int alarm_val;
	int i;
	int val;

	if (oc_trip <= 2500)
		trip_val = 0;
	else if (oc_trip <= 3000)
		trip_val = 1;
	else
		trip_val = 2;

	for (i = 0; i < ARRAY_SIZE(oc_alarm_table); ++i) {
		if (oc_alarm <=  oc_alarm_table[i])
			break;
	}
	alarm_val = i;

	switch (id) {
	case AS3722_SD0:
		mask = AS3722_OVCURRENT_SD0_ALARM_MASK |
				AS3722_OVCURRENT_SD0_TRIP_MASK;
		val = (trip_val << AS3722_OVCURRENT_SD0_TRIP_SHIFT) |
				(alarm_val << AS3722_OVCURRENT_SD0_ALARM_SHIFT);
		reg = AS3722_OVCURRENT;
		break;

	case AS3722_SD1:
		mask = AS3722_OVCURRENT_SD1_TRIP_MASK;
		val = (trip_val << AS3722_OVCURRENT_SD1_TRIP_SHIFT);
		reg = AS3722_OVCURRENT;
		break;

	case AS3722_SD6:
		mask = AS3722_OVCURRENT_SD6_ALARM_MASK |
				AS3722_OVCURRENT_SD6_TRIP_MASK;
		val = (trip_val << AS3722_OVCURRENT_SD6_TRIP_SHIFT) |
				(alarm_val << AS3722_OVCURRENT_SD6_ALARM_SHIFT);
		reg = AS3722_OVCURRENT_DEB;
		break;
	default:
		return 0;
	}
	ret = as3722_set_bits(as3722, reg, mask, val);
	if (ret < 0)
		dev_err(as3722->dev, "Reg 0x%02x update failed %d\n", reg, ret);
	return ret;
}

static int as3722_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	struct as3722_platform_data *pdata = as3722->dev->platform_data;
	int regulator;
	int ret;
	int ext_control_num;

	if (WARN_ON(pdev->id < 0 || pdev->id >= AS3722_NUM_REGULATORS))
		return -EINVAL;

	config.dev = pdev->dev.parent;
	config.driver_data = as3722;
	config.regmap = as3722->regmap;

	for (regulator = 0; regulator < AS3722_NUM_REGULATORS; regulator++) {
		if (!(pdata->reg_pdata[regulator] &&
			pdata->reg_pdata[regulator]->reg_init))
			continue;

		if (pdata->reg_pdata[regulator]->oc_configure_enable) {
			ret = as3722_overcurrent_init(as3722, regulator,
					pdata->reg_pdata[regulator]);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"OC init for regulator %d failed %d\n",
					regulator, ret);
				return ret;
			}
		}
		config.init_data = pdata->reg_pdata[regulator]->reg_init;
		rdev = regulator_register(&regulators[regulator],
				&config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "as3722 register"
				"regulator nr %d err\n", regulator);
			return PTR_ERR(rdev);
		}
		as3722->rdevs[regulator] = rdev;
		ext_control_num = pdata->reg_pdata[regulator]->ext_control;
		if (ext_control_num) {
			ret = as3722_extreg_init(as3722, regulator, ext_control_num);
			if (ret < 0) {
				dev_err(&pdev->dev, "as3722 external reg init "
					"failed for regulator nr %d err\n", regulator);
				return ret;
			}
		}
	}

	/* Check if LDO3 need to work in tracking mode or not */
	rdev = as3722->rdevs[AS3722_LDO3];
	if (pdata->enable_ldo3_tracking) {
		ret = as3722_ldo3_set_mode(rdev,
		AS3722_LDO3_MODE_PMOS_TRACKING);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"as3722 track enable failed for LDO3 err\n");
			return ret;
		}
	}
	return 0;
}

static int as3722_regulator_remove(struct platform_device *pdev)
{
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	int regulator;

	if (WARN_ON(pdev->id < 0 || pdev->id >= AS3722_NUM_REGULATORS))
		return -EINVAL;

	for (regulator = 0; regulator < AS3722_NUM_REGULATORS; regulator++) {
		if (as3722->rdevs[regulator]) {
			regulator_unregister(as3722->rdevs[regulator]);
			as3722->rdevs[regulator] = NULL;
		}
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int as3722_regulator_suspend(struct device *dev)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	struct as3722_platform_data *pdata = as3722->dev->platform_data;
	struct regulator_dev *rdev;
	int ret;
	/* Check and set LDO3 working mode in LP0 */
	rdev = as3722->rdevs[AS3722_LDO3];
	if (pdata->disabe_ldo3_tracking_suspend) {
		ret = as3722_ldo3_set_mode(rdev,
		AS3722_LDO3_MODE_NMOS);
		if (ret < 0) {
			dev_err(as3722->dev,
			"as3722 ldo3 tracking disable failed err\n");
			return ret;
		}
	}
	return 0;
}

static int as3722_regulator_resume(struct device *dev)
{
	struct as3722 *as3722 = dev_get_drvdata(dev->parent);
	struct as3722_platform_data *pdata = as3722->dev->platform_data;
	struct regulator_dev *rdev;
	int ret;

	/* Check if LDO3 need to be set to tracking mode*/
	rdev = as3722->rdevs[AS3722_LDO3];
	if (pdata->enable_ldo3_tracking) {
		ret = as3722_ldo3_set_mode(rdev,
			AS3722_LDO3_MODE_PMOS_TRACKING);
		if (ret < 0) {
			dev_err(as3722->dev,
				"as3722 ldo3 tracking disable failed err\n");
			return ret;
		}
	}
	return 0;
}

static const struct dev_pm_ops as3722_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(as3722_regulator_suspend,
	as3722_regulator_resume)
};
#endif

static struct platform_driver as3722_regulator_driver = {
	.driver = {
		.name = "as3722-regulator",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &as3722_pm_ops,
#endif
	},
	.probe = as3722_regulator_probe,
	.remove = as3722_regulator_remove,
};

static int __init as3722_regulator_init(void)
{
	return platform_driver_register(&as3722_regulator_driver);
}

subsys_initcall(as3722_regulator_init);

static void __exit as3722_regulator_exit(void)
{
	platform_driver_unregister(&as3722_regulator_driver);
}

module_exit(as3722_regulator_exit);

MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
MODULE_DESCRIPTION("AS3722 regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:as3722-regulator");
