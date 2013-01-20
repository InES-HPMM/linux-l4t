/*
 * drivers/mfd/max77660-core.c
 * Max77660 mfd driver (I2C bus access)
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#include <linux/mfd/max77660/max77660-core.h>


struct max77660_irq_data {
	int mask_reg;
	u16 mask;
	u8 top_mask;
	u8 top_shift;
	int cache_idx;
	bool is_rtc;
	bool is_unmask;
	u8 trigger_type;
};

struct max77660_chip *max77660_chip;

static struct resource gpio_resources[] = {
	{
		.start	= MAX77660_IRQ_INT_TOP_GPIO,
		.end	= MAX77660_IRQ_INT_TOP_GPIO,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource rtc_resources[] = {
	{
		.start	= MAX77660_IRQ_RTC,
		.end	= MAX77660_IRQ_RTC,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource fg_resources[] = {
	{
		.start	= MAX77660_IRQ_FG,
		.end	= MAX77660_IRQ_FG,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource chg_resources[] = {
	{
		.start	= MAX77660_IRQ_CHG,
		.end	= MAX77660_IRQ_CHG,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource haptic_resources[] = {
	{
		.start	= MAX77660_IRQ_HAPTIC,
		.end	= MAX77660_IRQ_HAPTIC,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource max77660_sys_wdt_resources[] = {
	{
		.start	= MAX77660_IRQ_GLBL_WDTWRN_SYS,
		.end	= MAX77660_IRQ_GLBL_WDTWRN_SYS,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct mfd_cell max77660_cells[] = {
	{
		.name = "max77660-gpio",
		.num_resources	= ARRAY_SIZE(gpio_resources),
		.resources	= &gpio_resources[0],
	},
	{
		.name = "max77660-pmic",
	},
	{
		.name = "max77660-rtc",
		.num_resources	= ARRAY_SIZE(rtc_resources),
		.resources	= &rtc_resources[0],
	},
	{
		.name = "max77660-fg",
		.num_resources	= ARRAY_SIZE(fg_resources),
		.resources	= &fg_resources[0],
	},
	{
		.name = "max77660-chg",
		.num_resources	= ARRAY_SIZE(chg_resources),
		.resources	= &chg_resources[0],
	},
	{
		.name = "max77660-vibrator",
		.num_resources	= ARRAY_SIZE(haptic_resources),
		.resources	= &haptic_resources[0],
	},
	{
		.name = "max77660-sys-wdt",
		.num_resources	= ARRAY_SIZE(max77660_sys_wdt_resources),
		.resources	= &max77660_sys_wdt_resources[0],
	},
};

static struct max77660_irq_data max77660_irqs[MAX77660_IRQ_NR] = {
	[MAX77660_IRQ_INT_TOP_GPIO] = {
			.top_mask = IRQ_TOP1_GPIO_MASK,
			.top_shift = IRQ_TOP1_GPIO_SHIFT,
			.cache_idx = -1,
	},
	[MAX77660_IRQ_INT_TOP_OVF] = {
			.top_mask = IRQ_TOP1_OVF_MASK,
			.top_shift = IRQ_TOP1_OVF_SHIFT,
			.cache_idx = -1,
	},
	[MAX77660_IRQ_INT_TOP_SYSINT] = {
			.top_mask = IRQ_TOP1_TOPSYS_MASK,
			.top_shift = IRQ_TOP1_TOPSYS_SHIFT,
			.cache_idx = -1,
	},
	[MAX77660_IRQ_FG] = {
		.top_mask = IRQ_TOP1_FUELG_MASK,
		.top_shift = IRQ_TOP1_FUELG_SHIFT,
		.cache_idx = -1,
		.is_rtc = 1,
	},
		[MAX77660_IRQ_CHG] = {
		.top_mask = IRQ_TOP1_CHARGER_MASK,
		.top_shift = IRQ_TOP1_CHARGER_SHIFT,
		.cache_idx = -1,
		.is_rtc = 1,
	},
	[MAX77660_IRQ_RTC] = {
		.top_mask = IRQ_TOP1_RTC_MASK,
		.top_shift = IRQ_TOP1_RTC_SHIFT,
		.cache_idx = -1,
		.is_rtc = 1,
	},
	[MAX77660_IRQ_BUCK_PF] = {
		.top_mask = IRQ_TOP2_BUCK_MASK,
		.top_shift = IRQ_TOP2_BUCK_SHIFT,
		.cache_idx = CACHE_IRQ_BUCK,
		.mask_reg = MAX77660_REG_IRQ_BUCKINT_MASK,
		.mask = 0x7F,
	},
	[MAX77660_IRQ_LDO_PF] = {
		.top_mask = IRQ_TOP2_LDO_MASK,
		.top_shift = IRQ_TOP2_LDO_SHIFT,
		.cache_idx = CACHE_IRQ_LDO,
		.mask_reg = MAX77660_REG_IRQ_LDOINT1_MASK,
		.mask = 0x3FF,
	},
	[MAX77660_IRQ_SIM] = {
		.top_mask = IRQ_TOP1_SIM_MASK,
		.top_shift = IRQ_TOP1_SIM_SHIFT,
		.cache_idx = -1,
	},
	[MAX77660_IRQ_ADC] = {
		.top_mask = IRQ_TOP1_ADC_MASK,
		.top_shift = IRQ_TOP1_ADC_SHIFT,
		.cache_idx = -1,
	},
};

#if 0
/* MAX77660 PMU doesn't allow PWR_OFF and SFT_RST setting in ONOFF_CFG1
 * at the same time. So if it try to set PWR_OFF and SFT_RST to ONOFF_CFG1
 * simultaneously, handle only SFT_RST and ignore PWR_OFF.
 */
#define CHECK_ONOFF_CFG1_MASK	(ONOFF_SFT_RST_MASK | ONOFF_PWR_OFF_MASK)
#define CHECK_ONOFF_CFG1(_addr, _val)			\
	unlikely((_addr == MAX77660_REG_ONOFF_CFG1) &&	\
		 ((_val & CHECK_ONOFF_CFG1_MASK) == CHECK_ONOFF_CFG1_MASK))
#endif
static inline int max77660_i2c_write(struct max77660_chip *chip, u8 addr,
				void *src, u32 bytes,
				enum max77660_i2c_slave slave)
{
	int ret = 0;
	struct regmap *regmap = chip->regmap_power;

	dev_dbg(chip->dev, "i2c_write: addr=0x%02x, src=0x%02x, bytes=%u\n",
		addr, *((u8 *)src), bytes);

	switch (slave) {
	case MAX77660_I2C_CORE:
	case MAX77660_I2C_GPIO:
	case MAX77660_I2C_PMIC:
		regmap = chip->regmap_power;
		break;
	case MAX77660_I2C_RTC:
		regmap = chip->regmap_rtc;
		break;
	case MAX77660_I2C_CHG:
		   regmap = chip->regmap_chg;
		break;
	case MAX77660_I2C_FG:
		regmap = chip->regmap_fg;
		break;
	case MAX77660_I2C_HAPTIC:
		regmap = chip->regmap_haptic;
		break;
	}

	if (MAX77660_I2C_RTC == slave) {
		/* RTC registers support sequential writing */
		ret = regmap_bulk_write(regmap, addr, src, bytes);
	} else {
		/* Power registers support register-data pair writing */
		u8 *src8 = (u8 *)src;
		unsigned int val;
		int i;

		for (i = 0; i < bytes; i++) {
			#if 0
			if (CHECK_ONOFF_CFG1(addr, *src8))
				val = *src8++ & ~ONOFF_PWR_OFF_MASK;
			else
			#endif
			val = *src8++;
			ret = regmap_write(regmap, addr, val);
			if (ret < 0)
				break;
			addr++;
		}
	}
	if (ret < 0)
		dev_err(chip->dev, "%s() failed, e %d\n", __func__, ret);
	return ret;
}

static inline int max77660_i2c_read(struct max77660_chip *chip, u8 addr,
			void *dest, u32 bytes, enum max77660_i2c_slave slave)
{
	int ret = 0;
	struct regmap *regmap = chip->regmap_power;

	switch (slave) {
	case MAX77660_I2C_CORE:
	case MAX77660_I2C_GPIO:
	case MAX77660_I2C_PMIC:
		regmap = chip->regmap_power;
		break;
	case MAX77660_I2C_RTC:
		regmap = chip->regmap_rtc;
		break;
	case MAX77660_I2C_CHG:
		regmap = chip->regmap_chg;
		break;
	case MAX77660_I2C_FG:
		regmap = chip->regmap_fg;
		break;
	case MAX77660_I2C_HAPTIC:
		regmap = chip->regmap_haptic;
		break;
	}

	ret = regmap_bulk_read(regmap, addr, dest, bytes);
	if (ret < 0) {
		dev_err(chip->dev, "%s() failed, e %d\n", __func__, ret);
		return ret;
	}

	dev_dbg(chip->dev, "i2c_read: addr=0x%02x, dest=0x%02x, bytes=%u\n",
		addr, *((u8 *)dest), bytes);
	return ret;
}

int max77660_read(struct device *dev, u8 addr, void *values, u32 len,
		  enum max77660_i2c_slave slave)
{
	struct max77660_chip *chip;
	int ret;

	if (slave == MAX77660_I2C_CORE)
		chip = dev_get_drvdata(dev);
	else
		chip = dev_get_drvdata(dev->parent);


	mutex_lock(&chip->io_lock);
	ret = max77660_i2c_read(chip, addr, values, len, slave);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77660_read);

int max77660_write(struct device *dev, u8 addr, void *values, u32 len,
		   enum max77660_i2c_slave slave)
{
	struct max77660_chip *chip;
	int ret;

	if (slave == MAX77660_I2C_CORE)
		chip = dev_get_drvdata(dev);
	else
		chip = dev_get_drvdata(dev->parent);

	mutex_lock(&chip->io_lock);
	ret = max77660_i2c_write(chip, addr, values, len, slave);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77660_write);

int max77660_set_bits(struct device *dev, u8 addr, u8 mask, u8 value,
		      enum max77660_i2c_slave slave)
{
	struct max77660_chip *chip;
	struct regmap *regmap;
	int ret;

	if (slave == MAX77660_I2C_CORE)
		chip = dev_get_drvdata(dev);
	else
		chip = dev_get_drvdata(dev->parent);

	switch (slave) {
	case MAX77660_I2C_CORE:
	case MAX77660_I2C_GPIO:
	case MAX77660_I2C_PMIC:
		regmap = chip->regmap_power;
		break;
	case MAX77660_I2C_RTC:
		 regmap = chip->regmap_rtc;
		 break;
	case MAX77660_I2C_CHG:
		regmap = chip->regmap_chg;
		break;
	case MAX77660_I2C_FG:
		regmap = chip->regmap_fg;
		break;
	case MAX77660_I2C_HAPTIC:
		regmap = chip->regmap_haptic;
		break;
	}

	mutex_lock(&chip->io_lock);
	ret = regmap_update_bits(regmap, addr, mask, value);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77660_set_bits);

static void max77660_power_off(void)
{
	struct max77660_chip *chip = max77660_chip;

	if (!chip)
		return;

	dev_info(chip->dev, "%s: Global shutdown\n", __func__);
	max77660_set_bits(chip->dev, MAX77660_REG_GLOBAL_CFG1,
			GLBLCNFG0_SFT_OFF_OFFRST_MASK,
			GLBLCNFG0_SFT_OFF_OFFRST_MASK,
			MAX77660_I2C_CORE);
}

static int max77660_sleep(struct max77660_chip *chip, bool on)
{
	int ret = 0;

	if (chip->pdata->flags & SLP_LPM_ENABLE) {
		/* Put the power rails into Low-Power mode during sleep mode,
		 * if the power rail's power mode is GLPM. */
		ret = max77660_set_bits(chip->dev, MAX77660_REG_GLOBAL_CFG1,
					GLBLCNFG1_GLBL_LPM_MASK,
					on ? GLBLCNFG1_GLBL_LPM_MASK : 0,
					MAX77660_I2C_CORE);
		if (ret < 0)
			return ret;
	}

	/* Enable sleep that AP can be placed into sleep mode
	 * by pulling EN1 low */
	return max77660_set_bits(chip->dev, MAX77660_REG_GLOBAL_CFG5,
				 GLBLCNFG5_EN1234_MASK_MASK,
				 on ? GLBLCNFG5_EN1234_MASK_MASK : 0,
				 MAX77660_I2C_CORE);
}

static inline int max77660_cache_write(struct device *dev, u8 addr, u8 mask,
				       u8 val, u8 *cache)
{
	u8 new_val;
	int ret;

	new_val = (*cache & ~mask) | (val & mask);
	if (*cache != new_val) {
		ret = max77660_write(dev, addr, &new_val, 1, MAX77660_I2C_CORE);
		if (ret < 0)
			return ret;
		*cache = new_val;
	}
	return 0;
}

static void max77660_irq_mask(struct irq_data *data)
{
	struct max77660_chip *chip = irq_data_get_irq_chip_data(data);

	max77660_irqs[data->irq - chip->irq_base].is_unmask = 0;
}

static void max77660_irq_unmask(struct irq_data *data)
{
	struct max77660_chip *chip = irq_data_get_irq_chip_data(data);

	max77660_irqs[data->irq - chip->irq_base].is_unmask = 1;
}

static void max77660_irq_lock(struct irq_data *data)
{
	struct max77660_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->irq_lock);
}

static void max77660_irq_sync_unlock(struct irq_data *data)
{
	struct max77660_chip *chip = irq_data_get_irq_chip_data(data);
	struct max77660_irq_data *irq_data =
			&max77660_irqs[data->irq - chip->irq_base];
	int idx = irq_data->cache_idx;
	u8 irq_top_mask = chip->cache_irq_top_mask;
	u16 irq_mask = chip->cache_irq_mask[idx];
	int update_irq_top = 0;
	u32 len = 1;
	int ret;

	if (irq_data->is_unmask) {
		if (chip->irq_top_count[irq_data->top_shift] == 0)
			update_irq_top = 1;
		chip->irq_top_count[irq_data->top_shift]++;

		if (irq_data->top_mask != IRQ_TOP1_TOPSYS_MASK)
			irq_top_mask &= ~irq_data->top_mask;

		if (idx != -1)
			irq_mask &= ~irq_data->mask;
	} else {
		if (chip->irq_top_count[irq_data->top_shift] == 1)
			update_irq_top = 1;

		if (--chip->irq_top_count[irq_data->top_shift] < 0)
			chip->irq_top_count[irq_data->top_shift] = 0;

		if (irq_data->top_mask != IRQ_TOP1_TOPSYS_MASK)
			irq_top_mask |= irq_data->top_mask;

		if (idx != -1)
			irq_mask |= irq_data->mask;
	}

	if ((idx != -1) && (irq_mask != chip->cache_irq_mask[idx])) {
		if (irq_data->top_mask == IRQ_TOP2_LDO_MASK)
			len = 3;

		ret = max77660_write(chip->dev, irq_data->mask_reg,
				     &irq_mask, len, MAX77660_I2C_CORE);
		if (ret < 0)
			goto out;

		chip->cache_irq_mask[idx] = irq_mask;
	}

	if (update_irq_top && (irq_top_mask != chip->cache_irq_top_mask)) {
		ret = max77660_cache_write(chip->dev,
					MAX77660_REG_IRQ_TOP1_MASK,
					irq_data->top_mask, irq_top_mask,
					&chip->cache_irq_top_mask);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&chip->irq_lock);
}

static inline int max77660_do_irq(struct max77660_chip *chip, u8 addr,
				  int irq_base, int irq_end)
{
	struct max77660_irq_data *irq_data = NULL;
	int irqs_to_handle[irq_end - irq_base + 1];
	int handled = 0;
	u16 val;
	u32 len = 1;
	int i;
	int ret;

	ret = max77660_read(chip->dev, addr, &val, len, MAX77660_I2C_CORE);
	if (ret < 0)
		return ret;

	for (i = irq_base; i <= irq_end; i++) {
		irq_data = &max77660_irqs[i];
		if (val & irq_data->mask) {
			irqs_to_handle[handled] = i + chip->irq_base;
			handled++;
		}
	}

	for (i = 0; i < handled; i++)
		handle_nested_irq(irqs_to_handle[i]);

	return 0;
}

static irqreturn_t max77660_irq(int irq, void *data)
{
	struct max77660_chip *chip = data;
	u8 irq_top;
	int ret;

	ret = max77660_read(chip->dev, MAX77660_REG_IRQ_TOP1, &irq_top,
			1, MAX77660_I2C_CORE);
	if (ret < 0) {
		dev_err(chip->dev, "irq: Failed to get irq top status\n");
		return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP1_TOPSYS_MASK) {
		ret = max77660_do_irq(chip, MAX77660_REG_IRQ_GLBINT1,
			MAX77660_IRQ_GLBL_EN0_R,
			MAX77660_IRQ_GLBL_WDTWRN_CHG);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP1_GPIO_MASK)
		handle_nested_irq(MAX77660_IRQ_INT_TOP_GPIO + chip->irq_base);

	if (irq_top & IRQ_TOP1_RTC_MASK)
		handle_nested_irq(MAX77660_IRQ_RTC + chip->irq_base);

	if (irq_top & IRQ_TOP1_FUELG_MASK)
		handle_nested_irq(MAX77660_IRQ_FG + chip->irq_base);

	if (irq_top & IRQ_TOP1_CHARGER_MASK)
		handle_nested_irq(MAX77660_IRQ_CHG + chip->irq_base);

	if (irq_top & IRQ_TOP1_ADC_MASK)
		handle_nested_irq(MAX77660_IRQ_ADC + chip->irq_base);

	if (irq_top & IRQ_TOP1_SIM_MASK)
		handle_nested_irq(MAX77660_IRQ_SIM + chip->irq_base);

	if (irq_top & IRQ_TOP1_OVF_MASK) {
		ret = max77660_read(chip->dev, MAX77660_REG_IRQ_TOP2, &irq_top,
			1, MAX77660_I2C_CORE);

		if (ret < 0)
			return IRQ_NONE;

		if (irq_top & IRQ_TOP2_BUCK_MASK)
			handle_nested_irq(MAX77660_IRQ_BUCK_PF
						+ chip->irq_base);

		if (irq_top & IRQ_TOP2_LDO_MASK)
			handle_nested_irq(MAX77660_IRQ_LDO_PF + chip->irq_base);
	}


	return IRQ_HANDLED;
}

static struct irq_chip max77660_irq_chip = {
	.name = "max77660-irq",
	.irq_mask = max77660_irq_mask,
	.irq_unmask = max77660_irq_unmask,
	.irq_bus_lock = max77660_irq_lock,
	.irq_bus_sync_unlock = max77660_irq_sync_unlock,
};

static int max77660_irq_init(struct max77660_chip *chip)
{
	u32 temp;
	int i, ret = 0;

	mutex_init(&chip->irq_lock);

	/* Mask all interrupts */
	chip->cache_irq_top_mask = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_GLBLINT1] = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_GLBLINT2] = 0xF;
	chip->cache_irq_mask[CACHE_IRQ_BUCK] = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_LDO] = 0xFFFF;

	max77660_write(chip->dev, MAX77660_REG_IRQ_TOP1_MASK,
		       &chip->cache_irq_top_mask, 2, MAX77660_I2C_CORE);

	max77660_write(chip->dev, MAX77660_REG_IRQ_GLBINT1_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_GLBLINT1], 2,
		       MAX77660_I2C_CORE);
	max77660_write(chip->dev, MAX77660_REG_IRQ_BUCKINT_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_BUCK], 1,
		       MAX77660_I2C_CORE);
	max77660_write(chip->dev, MAX77660_REG_IRQ_LDOINT1_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LDO], 3,
		       MAX77660_I2C_CORE);

	/* Clear all interrups */
	max77660_read(chip->dev, MAX77660_REG_IRQ_GLBINT1, &temp, 2,
				MAX77660_I2C_CORE);

	max77660_read(chip->dev, MAX77660_REG_IRQ_BUCKINT, &temp, 1,
				MAX77660_I2C_CORE);
	max77660_read(chip->dev, MAX77660_REG_IRQ_LDOINT1, &temp, 3,
				MAX77660_I2C_CORE);
	max77660_read(chip->dev, MAX77660_REG_GPIO_IRQ1, &temp, 2,
				MAX77660_I2C_CORE);

	for (i = chip->irq_base; i < (MAX77660_IRQ_NR + chip->irq_base); i++) {
		int irq = i - chip->irq_base;
		if (i >= NR_IRQS) {
			dev_err(chip->dev,
				"irq_init: Can't set irq chip for irq %d\n", i);
			continue;
		}
		if ((irq >= MAX77660_IRQ_GPIO0) && (irq <= MAX77660_IRQ_GPIO9))
			continue;

		irq_set_chip_data(i, chip);

		irq_set_chip_and_handler(i, &max77660_irq_chip,
				handle_edge_irq);
#ifdef CONFIG_ARM
		set_irq_flags(i, IRQF_VALID);
#else
		irq_set_noprobe(i);
#endif
		irq_set_nested_thread(i, 1);
	}

	ret = request_threaded_irq(chip->i2c_power->irq, NULL, max77660_irq,
				   IRQF_ONESHOT, "max77660", chip);
	if (ret) {
		dev_err(chip->dev, "irq_init: Failed to request irq %d\n",
			chip->i2c_power->irq);
		return ret;
	}

	device_init_wakeup(chip->dev, 1);
	enable_irq_wake(chip->i2c_power->irq);

	chip->cache_irq_top_mask &= ~IRQ_TOP1_TOPSYS_MASK;
	max77660_write(chip->dev, MAX77660_REG_IRQ_TOP1_MASK,
		       &chip->cache_irq_top_mask, 1, MAX77660_I2C_CORE);

	chip->cache_irq_mask[CACHE_IRQ_GLBLINT1] = 0;
	max77660_write(chip->dev, MAX77660_REG_IRQ_GLBINT1,
			&chip->cache_irq_mask[CACHE_IRQ_GLBLINT1],
			2,
			MAX77660_I2C_CORE);

	/*chip->cache_irq_mask[CACHE_IRQ_ONOFF] &= ~ONOFF_IRQ_EN0_RISING;
	max77660_write(chip->dev, MAX77660_REG_ONOFF_IRQ_MASK,
	&chip->cache_irq_mask[CACHE_IRQ_ONOFF], 1, MAX77660_I2C_CORE); */

	return 0;
}

static void max77660_irq_exit(struct max77660_chip *chip)
{
	if (chip->i2c_power->irq)
		free_irq(chip->i2c_power->irq, chip);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *max77660_dentry_regs;

static int max77660_debugfs_dump_regs(struct max77660_chip *chip, char *label,
			u8 *addrs, int num_addrs, char *buf,
			ssize_t *len, enum max77660_i2c_slave slave)
{
	ssize_t count = *len;
	u8 val;
	int ret = 0;
	int i;

	count += sprintf(buf + count, "%s\n", label);
	if (count >= PAGE_SIZE - 1)
		return -ERANGE;

	for (i = 0; i < num_addrs; i++) {
		count += sprintf(buf + count, "0x%02x: ", addrs[i]);
		if (count >= PAGE_SIZE - 1)
			return -ERANGE;

		ret = max77660_read(chip->dev, addrs[i], &val, 1, slave);
		if (ret == 0)
			count += sprintf(buf + count, "0x%02x\n", val);
		else
			count += sprintf(buf + count, "<read fail: %d>\n", ret);

		if (count >= PAGE_SIZE - 1)
			return -ERANGE;
	}

	*len = count;
	return 0;
}

static int max77660_debugfs_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t max77660_debugfs_regs_read(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77660_chip *chip = file->private_data;
	char *buf;
	size_t len = 0;
	ssize_t ret;

	/* Excluded interrupt status register to prevent register clear */
	u8 global_regs[] = {
		0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x0F, 0x10, 0x11, 0x12
	};
	u8 sd_regs[] = {
		0x09, 0x13, 0x17,
		0x37, 0x38, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
		0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53
	};
	u8 ldo_regs[] = {
		0x14, 0x15, 0x16, 0x18, 0x19, 0x1A, 0x54, 0x55, 0x56, 0x57,
		0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61,
		0x61, 0x62, 0x63, 0x64, 0x65
	};
	u8 gpio_regs[] = {
		0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73,
		0x74, 0x75, 0x76, 0x77, 0x78, 0x79
	};
	u8 rtc_regs[] = {
		0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
		0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B
	};
	u8 osc_32k_regs[] = { 0xA0, 0xA1 };
	u8 fps_regs[] = {
		0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
		0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
		0x35, 0x36
	};
	u8 cid_regs[] = { 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F };

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += sprintf(buf + len, "MAX77660 Registers\n");
	max77660_debugfs_dump_regs(chip, "[Global]", global_regs,
		ARRAY_SIZE(global_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[Step-Down]", sd_regs,
		ARRAY_SIZE(sd_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[LDO]", ldo_regs,
		ARRAY_SIZE(ldo_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[GPIO]", gpio_regs,
		ARRAY_SIZE(gpio_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[RTC]", rtc_regs,
		ARRAY_SIZE(rtc_regs), buf, &len, MAX77660_I2C_RTC);
	max77660_debugfs_dump_regs(chip, "[32kHz Oscillator]", osc_32k_regs,
		ARRAY_SIZE(osc_32k_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[Flexible Power Sequencer]", fps_regs,
		ARRAY_SIZE(fps_regs), buf, &len, MAX77660_I2C_CORE);
	max77660_debugfs_dump_regs(chip, "[Chip Identification]", cid_regs,
		ARRAY_SIZE(cid_regs), buf, &len, MAX77660_I2C_CORE);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static const struct file_operations max77660_debugfs_regs_fops = {
	.open = max77660_debugfs_regs_open,
	.read = max77660_debugfs_regs_read,
};

static void max77660_debugfs_init(struct max77660_chip *chip)
{
	max77660_dentry_regs = debugfs_create_file(chip->i2c_power->name,
						   0444, 0, chip,
						   &max77660_debugfs_regs_fops);
	if (!max77660_dentry_regs)
		dev_warn(chip->dev,
			 "debugfs_init: Failed to create debugfs file\n");
}

static void max77660_debugfs_exit(struct max77660_chip *chip)
{
	debugfs_remove(max77660_dentry_regs);
}
#else
static inline void max77660_debugfs_init(struct max77660_chip *chip)
{
}

static inline void max77660_debugfs_exit(struct max77660_chip *chip)
{
}
#endif /* CONFIG_DEBUG_FS */

static bool rd_wr_reg_power(struct device *dev, unsigned int reg)
{
	if (reg < 0xF2)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static bool rd_wr_reg_rtc(struct device *dev, unsigned int reg)
{
	if (reg < 0x1C)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static bool rd_wr_reg_fg(struct device *dev, unsigned int reg)
{
	if (reg <= 0xFF)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static bool rd_wr_reg_chg(struct device *dev, unsigned int reg)
{
	if (reg <= 0xFF)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static bool rd_wr_reg_haptic(struct device *dev, unsigned int reg)
{
	if (reg <= 0xFF)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static const struct regmap_config max77660_regmap_config[] = {
	{
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x9F,
		.writeable_reg = rd_wr_reg_power,
		.readable_reg = rd_wr_reg_power,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xF2,
		.writeable_reg = rd_wr_reg_rtc,
		.readable_reg = rd_wr_reg_rtc,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xFF,
		.writeable_reg = rd_wr_reg_fg,
		.readable_reg = rd_wr_reg_fg,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xFF,
		.writeable_reg = rd_wr_reg_chg,
		.readable_reg = rd_wr_reg_chg,
	}, {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0xFF,
		.writeable_reg = rd_wr_reg_haptic,
		.readable_reg = rd_wr_reg_haptic,
	},
};

static int max77660_slave_address[MAX77660_NUM_SLAVES] = {
	MAX77660_PWR_I2C_ADDR,
	MAX77660_RTC_I2C_ADDR,
	MAX77660_CHG_I2C_ADDR,
	MAX77660_FG_I2C_ADDR,
	MAX77660_HAPTIC_I2C_ADDR,
};

static int max77660_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max77660_platform_data *pdata = client->dev.platform_data;
	struct max77660_chip *chip;
	int ret = 0;
	u8 val;
	int i;

	if (!pdata) {
		dev_err(&client->dev, "probe: Invalid platform_data\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "Memory alloc for chip failed\n");
		return -ENOMEM;
	}

	chip->i2c_power = client;
	i2c_set_clientdata(client, chip);

	for (i = 0; i < MAX77660_NUM_SLAVES; i++) {
		if (max77660_slave_address[i] == client->addr)
			chip->clients[i] = client;
		else
			chip->clients[i] = i2c_new_dummy(client->adapter,
						max77660_slave_address[i]);
		if (!chip->clients[i]) {
			dev_err(&client->dev, "can't attach client %d\n", i);
			ret = -ENOMEM;
			goto fail_client_reg;
		}

		i2c_set_clientdata(chip->clients[i], chip);
		chip->rmap[i] = devm_regmap_init_i2c(chip->clients[i],
					&max77660_regmap_config[i]);
		if (IS_ERR(chip->rmap[i])) {
			ret = PTR_ERR(chip->rmap[i]);
			dev_err(&client->dev,
				"regmap %d init failed, err %d\n", i, ret);
			goto fail_client_reg;
		}
	}
	chip->regmap_power = chip->rmap[MAX77660_PWR_SLAVE];
	chip->regmap_rtc = chip->rmap[MAX77660_RTC_SLAVE];
	chip->regmap_fg = chip->rmap[MAX77660_FG_SLAVE];
	chip->regmap_chg = chip->rmap[MAX77660_CHG_SLAVE];
	chip->regmap_haptic = chip->rmap[MAX77660_HAPTIC_SLAVE];

	chip->dev = &client->dev;
	chip->pdata = pdata;
	chip->irq_base = pdata->irq_base;
	mutex_init(&chip->io_lock);

	/* Dummy read to see if chip is present or not*/
	ret = max77660_read(chip->dev, MAX77660_REG_CID5, &val, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "preinit: Failed to get register 0x%x\n",
				MAX77660_REG_CID5);
		return ret;
	}

	max77660_irq_init(chip);
	max77660_debugfs_init(chip);
	ret = max77660_sleep(chip, false);
	if (ret < 0) {
		dev_err(&client->dev, "probe: Failed to disable sleep\n");
		goto out_exit;
	}

	if (pdata->use_power_off && !pm_power_off) {
		max77660_chip = chip;
		pm_power_off = max77660_power_off;
	}

	ret =  mfd_add_devices(&client->dev, -1, max77660_cells,
			ARRAY_SIZE(max77660_cells), NULL, chip->irq_base, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "mfd add dev failed, e = %d\n", ret);
		goto out_exit;
	}

	ret = mfd_add_devices(&client->dev, 0, pdata->sub_devices,
			      pdata->num_subdevs, NULL, 0, NULL);
	if (ret != 0) {
		dev_err(&client->dev, "probe: Failed to add subdev: %d\n", ret);
		goto out_mfd_clean;
	}

	return 0;

out_mfd_clean:
	mfd_remove_devices(chip->dev);
out_exit:
	max77660_debugfs_exit(chip);
	max77660_irq_exit(chip);
	mutex_destroy(&chip->io_lock);

fail_client_reg:
	for (i = 0; i < MAX77660_NUM_SLAVES; i++) {
		if (chip->clients[i]  && (chip->clients[i] != client))
			i2c_unregister_device(chip->clients[i]);
	}

	max77660_chip = NULL;
	return ret;
}

static int __devexit max77660_remove(struct i2c_client *client)
{
	struct max77660_chip *chip = i2c_get_clientdata(client);
	int i;

	mfd_remove_devices(chip->dev);
	max77660_debugfs_exit(chip);
	max77660_irq_exit(chip);
	mutex_destroy(&chip->io_lock);
	for (i = 0; i < MAX77660_NUM_SLAVES; i++) {
		if (chip->clients[i] != client)
			i2c_unregister_device(chip->clients[i]);
	}
	max77660_chip = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77660_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77660_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (client->irq)
		disable_irq(client->irq);

	ret = max77660_sleep(chip, true);
	if (ret < 0)
		dev_err(dev, "suspend: Failed to enable sleep\n");

	return ret;
}

static int max77660_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77660_chip *chip = i2c_get_clientdata(client);
	int ret;

	ret = max77660_sleep(chip, false);
	if (ret < 0) {
		dev_err(dev, "resume: Failed to disable sleep\n");
		return ret;
	}

	if (client->irq)
		enable_irq(client->irq);

	return 0;
}
#else
#define max77660_suspend      NULL
#define max77660_resume       NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id max77660_id[] = {
	{"max77660", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max77660_id);

static const struct dev_pm_ops max77660_pm = {
	.suspend = max77660_suspend,
	.resume = max77660_resume,
};

static struct i2c_driver max77660_driver = {
	.driver = {
		.name = "max77660",
		.owner = THIS_MODULE,
		.pm = &max77660_pm,
	},
	.probe = max77660_probe,
	.remove = __devexit_p(max77660_remove),
	.id_table = max77660_id,
};

static int __init max77660_init(void)
{
	return i2c_add_driver(&max77660_driver);
}
subsys_initcall(max77660_init);

static void __exit max77660_exit(void)
{
	i2c_del_driver(&max77660_driver);
}
module_exit(max77660_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77660 Multi Function Device Core Driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Maxim Integrated");
MODULE_ALIAS("platform:max77660-core");
