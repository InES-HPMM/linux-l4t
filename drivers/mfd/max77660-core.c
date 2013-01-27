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
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#include <linux/mfd/max77660/max77660-core.h>


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

static const struct regmap_irq max77660_top_irqs[] = {
	[MAX77660_IRQ_FG] = {
		.mask = MAX77660_IRQ_TOP1_FUELG_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_CHG] = {
		.mask = MAX77660_IRQ_TOP1_CHARGER_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_CHG] = {
		.mask = MAX77660_IRQ_TOP1_CHARGER_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_RTC] = {
		.mask = MAX77660_IRQ_TOP1_RTC_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_INT_TOP_GPIO] = {
		.mask = MAX77660_IRQ_TOP1_GPIO_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_SIM] = {
		.mask = MAX77660_IRQ_TOP1_SIM_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_ADC] = {
		.mask = MAX77660_IRQ_TOP1_ADC_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_TOPSYSINT] = {
		.mask = MAX77660_IRQ_TOP1_TOPSYS_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_LDOINT] = {
		.mask = MAX77660_IRQ_TOP2_LDO_MASK,
		.reg_offset = 1,
	},
	[MAX77660_IRQ_BUCKINT] = {
		.mask = MAX77660_IRQ_TOP2_BUCK_MASK,
		.reg_offset = 1,
	},
};

static const struct regmap_irq max77660_global_irqs[] = {
	[MAX77660_IRQ_GLBL_TJALRM2 - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_TJALRM2_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_TJALRM1 - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_TJALRM1_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_SYSLOW - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_SYSLOW_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_I2C_WDT - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_I2CWDT_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_EN0_1SEC - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_EN0_1SEC_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_EN0_F - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_EN0_F_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_EN0_R - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT1_EN0_R_MASK,
		.reg_offset = 0,
	},
	[MAX77660_IRQ_GLBL_WDTWRN_CHG - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT2_WDTWRN_CHG_MASK,
		.reg_offset = 1,
	},
	[MAX77660_IRQ_GLBL_WDTWRN_SYS - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT2_WDTWRN_SYS_MASK,
		.reg_offset = 1,
	},
	[MAX77660_IRQ_GLBL_MR_F - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT2_MR_F_MASK,
		.reg_offset = 1,
	},
	[MAX77660_IRQ_GLBL_MR_R - MAX77660_IRQ_GLBL_BASE] = {
		.mask = MAX77660_IRQ_GLBLINT2_MR_R_MASK,
		.reg_offset = 1,
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
					MAX77660_GLBLCNFG1_GLBL_LPM,
					on ? MAX77660_GLBLCNFG1_GLBL_LPM : 0,
					MAX77660_I2C_CORE);
		if (ret < 0)
			return ret;
	}

	/* Enable sleep that AP can be placed into sleep mode
	 * by pulling EN1 low */
	ret = max77660_set_bits(chip->dev, MAX77660_REG_GLOBAL_CFG5,
			GLBLCNFG5_EN1_MASK_MASK,
			on ? 0 : GLBLCNFG5_EN1_MASK_MASK,
			MAX77660_I2C_CORE);
	if (ret < 0)
		return ret;
	if (chip->pdata->en_buck2_ext_ctrl)
		ret = max77660_set_bits(chip->dev, MAX77660_REG_GLOBAL_CFG7,
				BIT(0),
				on ? 0 : GLBLCNFG7_EN2_MASK_MASK,
				MAX77660_I2C_CORE);

	return ret;
}

static int max77660_32kclk_init(struct max77660_chip *chip,
		struct max77660_platform_data *pdata)
{
	u8 mask = 0;
	u8 val = 0;
	int ret;

	val |= (pdata->en_clk32out1? 1 : 0) << OUT1_EN_32KCLK_SHIFT;
	val |= (pdata->en_clk32out2? 1 : 0) << OUT2_EN_32KCLK_SHIFT;
	mask = OUT1_EN_32KCLK_MASK | OUT2_EN_32KCLK_MASK;

	ret = max77660_reg_update(chip->dev, MAX77660_PWR_SLAVE,
					MAX77660_REG_CNFG32K1,
					val, mask);
	return ret;

}
static struct regmap_irq_chip max77660_top_irq_chip = {
	.name = "max77660-top",
	.irqs = max77660_top_irqs,
	.num_irqs = ARRAY_SIZE(max77660_top_irqs),
	.num_regs = 2,
	.irq_reg_stride = 1,
	.status_base = MAX77660_REG_IRQ_TOP1,
	.mask_base = MAX77660_REG_IRQ_TOP1_MASK,
};

static struct regmap_irq_chip max77660_global_irq_chip = {
	.name = "max77660-global",
	.irqs = max77660_global_irqs,
	.num_irqs = ARRAY_SIZE(max77660_global_irqs),
	.num_regs = 2,
	.irq_reg_stride = 1,
	.status_base = MAX77660_REG_IRQ_GLBINT1,
	.mask_base = MAX77660_REG_IRQ_GLBINT1_MASK,
};

static int max77660_init_irqs(struct max77660_chip *chip,
		struct max77660_platform_data *pdata)
{
	int ret;

	/* Unmask the IQR_M */
	ret = max77660_reg_clr_bits(chip->dev, MAX77660_PWR_SLAVE,
			MAX77660_REG_IRQ_GLBINT1_MASK,
			MAX77660_IRQ_GLBLINT1_IRQ_M_MASK);
	if (ret < 0) {
		dev_err(chip->dev, "Clear IRQM failed %d\n", ret);
		return ret;
	}

	ret = regmap_add_irq_chip(chip->rmap[MAX77660_PWR_SLAVE],
		chip->chip_irq, IRQF_ONESHOT, pdata->irq_base,
		&max77660_top_irq_chip, &chip->top_irq_data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add top irq_chip %d\n", ret);
		return ret;
	}

	ret = regmap_add_irq_chip(chip->rmap[MAX77660_PWR_SLAVE],
		pdata->irq_base + MAX77660_IRQ_TOPSYSINT,
		IRQF_ONESHOT, pdata->irq_base + MAX77660_IRQ_GLBL_BASE,
		&max77660_global_irq_chip, &chip->global_irq_data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add global irq_chip %d\n", ret);
		goto fail_global_irq;
	}
	return 0;

fail_global_irq:
	regmap_del_irq_chip(chip->chip_irq, chip->top_irq_data);
	chip->top_irq_data = NULL;
	return ret;
}

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
		.max_register = 0xC0,
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
	chip->chip_irq = client->irq;
	mutex_init(&chip->io_lock);

	/* Dummy read to see if chip is present or not*/
	ret = max77660_read(chip->dev, MAX77660_REG_CID5, &val, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "preinit: Failed to get register 0x%x\n",
				MAX77660_REG_CID5);
		return ret;
	}

	ret = max77660_init_irqs(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "Irq initialisation failed: %d\n", ret);
		goto fail_irq;
	}

	ret = max77660_sleep(chip, false);
	if (ret < 0) {
		dev_err(&client->dev, "probe: Failed to disable sleep\n");
		goto out_exit;
	}

	if (pdata->use_power_off && !pm_power_off) {
		max77660_chip = chip;
		pm_power_off = max77660_power_off;
	}

	ret = max77660_32kclk_init(chip, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "probe: Failed to initialize 32k clk\n");
		goto out_exit;
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
	regmap_del_irq_chip(pdata->irq_base + MAX77660_IRQ_GLBL_BASE,
		chip->global_irq_data);
	regmap_del_irq_chip(chip->chip_irq, chip->top_irq_data);

fail_irq:
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
	struct max77660_platform_data *pdata = client->dev.platform_data;
	int i;

	mfd_remove_devices(chip->dev);
	mutex_destroy(&chip->io_lock);
	regmap_del_irq_chip(pdata->irq_base + MAX77660_IRQ_GLBL_BASE,
		chip->global_irq_data);
	regmap_del_irq_chip(chip->chip_irq, chip->top_irq_data);
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

	if (chip->pdata->en_buck2_ext_ctrl) {
		ret = max77660_set_bits(chip->dev, MAX77660_REG_BUCK_PWR_MODE1,
				MAX77660_BUCK2_PWR_MODE_MASK,
				0,
				MAX77660_I2C_CORE);
		if (ret < 0)
			dev_err(dev, "Failed to disable buck2 ext ctrl\n");
	}

	return ret;
}

static int max77660_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77660_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (chip->pdata->en_buck2_ext_ctrl) {
		ret = max77660_set_bits(chip->dev, MAX77660_REG_BUCK_PWR_MODE1,
				MAX77660_BUCK2_PWR_MODE_MASK,
				MAX77660_BUCK2_PWR_MODE_MASK,
				MAX77660_I2C_CORE);
		if (ret < 0)
			dev_err(dev, "Failed to disable buck2 ext ctrl\n");
	}

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
