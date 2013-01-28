/*
 * MAXIM MAX77660 GPIO driver
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 * Author: Laxman dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/regmap.h>

/* GPIO control registers */
#define MAX77660_REG_GPIO_CTRL0		0x6A
#define MAX77660_REG_GPIO_CTRL1		0x6B
#define MAX77660_REG_GPIO_CTRL2		0x6C
#define MAX77660_REG_GPIO_CTRL3		0x6D
#define MAX77660_REG_GPIO_CTRL4		0x6E
#define MAX77660_REG_GPIO_CTRL5		0x6F
#define MAX77660_REG_GPIO_CTRL6		0x70
#define MAX77660_REG_GPIO_CTRL7		0x71
#define MAX77660_REG_GPIO_CTRL8		0x72
#define MAX77660_REG_GPIO_CTRL9		0x73

#define MAX77660_REG_GPIO_PUE1		0x74
#define MAX77660_REG_GPIO_PUE2		0x75
#define MAX77660_REG_GPIO_PDE1		0x76
#define MAX77660_REG_GPIO_PDE2		0x77
#define MAX77660_REG_GPIO_AME1		0x78
#define MAX77660_REG_GPIO_AME2		0x79


#define GPIO_REG_ADDR(offset) (MAX77660_REG_GPIO_CTRL0 + offset)

#define GPIO_CTRL_DBNC_MASK			(BIT(7)|BIT(6))
#define GPIO_CTRL_DBNC_SHIFT		6
#define GPIO_CTRL_REFE_IRQ_MASK		(BIT(5)|BIT(4))
#define GPIO_CTRL_REFE_IRQ_SHIFT	4
#define GPIO_CTRL_DOUT_MASK			BIT(3)
#define GPIO_CTRL_DOUT_SHIFT		3
#define GPIO_CTRL_DIN_MASK			BIT(2)
#define GPIO_CTRL_DIN_SHIFT			2
#define GPIO_CTRL_DIR_MASK			BIT(1)
#define GPIO_CTRL_DIR_SHIFT			1
#define GPIO_CTRL_OUT_DRV_MASK		BIT(0)
#define GPIO_CTRL_OUT_DRV_SHIFT		0
#define GPIO_DBNC_NONE			0
#define GPIO_DBNC_8MS			1
#define GPIO_DBNC_16MS			2
#define GPIO_DBNC_32MS			3

struct max77660_gpio {
	struct gpio_chip	gpio_chip;
	struct device		*parent;
	struct device		*dev;
	int			gpio_irq;
	int			irq_base;
	int			gpio_base;
};

static const struct regmap_irq max77660_gpio_irqs[] = {
	[MAX77660_IRQ_GPIO0 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE0,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 0,
	},
	[MAX77660_IRQ_GPIO1 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE1,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 1,
	},
	[MAX77660_IRQ_GPIO2 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE2,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 2,
	},
	[MAX77660_IRQ_GPIO3 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE3,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 3,
	},
	[MAX77660_IRQ_GPIO4 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE4,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 4,
	},
	[MAX77660_IRQ_GPIO5 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE5,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 5,
	},
	[MAX77660_IRQ_GPIO6 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE6,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 6,
	},
	[MAX77660_IRQ_GPIO7 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ1_LVL2_GPIO_EDGE7,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 0,
		.type_reg_offset = 7,
	},
	[MAX77660_IRQ_GPIO8 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ2_LVL2_GPIO_EDGE8,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 1,
		.type_reg_offset = 8,
	},
	[MAX77660_IRQ_GPIO9 - MAX77660_IRQ_GPIO0] = {
		.mask = MAX77660_IRQ2_LVL2_GPIO_EDGE9,
		.type_rising_mask = MAX77660_CNFG_GPIO_INT_RISING,
		.type_falling_mask = MAX77660_CNFG_GPIO_INT_FALLING,
		.reg_offset = 1,
		.type_reg_offset = 9,
	},
};

static struct regmap_irq_chip max77660_gpio_irq_chip = {
	.name = "max77660-gpio",
	.irqs = max77660_gpio_irqs,
	.num_irqs = ARRAY_SIZE(max77660_gpio_irqs),
	.num_regs = 2,
	.num_type_reg = 10,
	.irq_reg_stride = 1,
	.type_reg_stride = 1,
	.status_base = MAX77660_REG_IRQ1_LVL2_GPIO,
	.type_base = MAX77660_REG_CNFG_GPIO0,
};

static struct max77660_gpio *max77660_gpio_chip;

static inline struct max77660_gpio *to_max77660_gpio(struct gpio_chip *gpio)
{
	return container_of(gpio, struct max77660_gpio, gpio_chip);
}

static int max77660_gpio_set_pull_up(struct max77660_gpio *max77660_gpio,
		int offset, int pull_up)
{
	struct device *parent = max77660_gpio->parent;
	u8 val = 0;
	u8 reg = MAX77660_REG_GPIO_PUE1;

	if ((offset < MAX77660_GPIO0) || (MAX77660_GPIO_NR <= offset))
		return -EINVAL;

	if (pull_up == GPIO_PU_ENABLE)
		val = (1 << (offset%8));

	reg += offset/8;

	return max77660_reg_update(parent, MAX77660_PWR_SLAVE, reg,
					val, (1 << (offset%8)));
}

static int max77660_gpio_set_pull_down(struct max77660_gpio *max77660_gpio,
		int offset, int pull_down)
{
	struct device *parent = max77660_gpio->parent;
	u8 val = 0;
	u8 reg = MAX77660_REG_GPIO_PDE1;

	if ((offset < MAX77660_GPIO0) || (MAX77660_GPIO_NR <= offset))
		return -EINVAL;

	if (pull_down == GPIO_PD_ENABLE)
			val = (1 << (offset%8));

	reg += offset/8;

	return max77660_reg_update(parent, MAX77660_PWR_SLAVE, reg,
					val, (1 << (offset%8)));
}

static inline int max77660_gpio_is_alternate(
		struct max77660_gpio *max77660_gpio, int offset)
{
	struct device *parent = max77660_gpio->parent;
	int ret = 0;
	u8 reg = MAX77660_REG_GPIO_AME1 + offset/8;
	u8 val;

	ret = max77660_reg_read(parent, MAX77660_PWR_SLAVE, reg, &val);
	if (ret < 0)
		return 0;
	return val & (1 << (offset%8)) ? 1 : 0;
}

static int max77660_gpio_config_alternate(int gpio, int alternate)
{
	struct max77660_gpio *max77660_gpio = max77660_gpio_chip;
	struct device *parent = max77660_gpio->parent;
	u8 reg = MAX77660_REG_GPIO_AME1;
	u8 val = 0;
	int ret = 0;
	int offset;

	if (!max77660_gpio)
		return -ENXIO;

	offset = gpio - max77660_gpio->gpio_base;
	if ((offset < MAX77660_GPIO0) || (MAX77660_GPIO_NR <= offset))
		return -EINVAL;

	reg += offset/8;
	if (alternate == GPIO_ALT_ENABLE) {
		val = (1 << (offset%8));
		if (offset == MAX77660_GPIO7) {
			ret = max77660_gpio_set_pull_up(max77660_gpio,
								offset, 0);
			if (ret < 0)
				return ret;

			ret = max77660_gpio_set_pull_down(max77660_gpio,
						offset, 0);
			if (ret < 0)
				return ret;
		}
	}

	return max77660_reg_update(parent, MAX77660_PWR_SLAVE, reg,
					val, (1 << (offset%8)));
}

static int max77660_gpio_dir_input(struct gpio_chip *gpio, unsigned offset)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);
	struct device *parent = max77660_gpio->parent;

	if (max77660_gpio_is_alternate(max77660_gpio, offset)) {
		dev_err(max77660_gpio->dev,
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	return max77660_reg_update(parent, MAX77660_PWR_SLAVE,
					GPIO_REG_ADDR(offset),
					GPIO_CTRL_DIR_MASK, GPIO_CTRL_DIR_MASK);
}

static int max77660_gpio_get(struct gpio_chip *gpio, unsigned offset)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);
	struct device *parent = max77660_gpio->parent;
	u8 val;
	int ret;

	if (max77660_gpio_is_alternate(max77660_gpio, offset)) {
		dev_err(max77660_gpio->dev,
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	ret = max77660_reg_read(parent, MAX77660_PWR_SLAVE,
				GPIO_REG_ADDR(offset), &val);
	if (ret < 0)
		return ret;

	if (GPIO_CTRL_DIR_MASK & val)
		return (val & GPIO_CTRL_DIN_MASK) >> GPIO_CTRL_DIN_SHIFT;
	else
		return (val & GPIO_CTRL_DOUT_MASK) >> GPIO_CTRL_DOUT_SHIFT;
}

static int max77660_gpio_dir_output(struct gpio_chip *gpio, unsigned offset,
				int value)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);
	struct device *parent = max77660_gpio->parent;
	u8 val = (value ? (1 << GPIO_CTRL_DOUT_SHIFT) : 0);
	int ret = 0;
	u8 mask = GPIO_CTRL_DIR_MASK | GPIO_CTRL_DOUT_MASK;
	if (max77660_gpio_is_alternate(max77660_gpio, offset)) {
		dev_warn(max77660_gpio->dev,
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	ret = max77660_reg_update(parent, MAX77660_PWR_SLAVE,
					GPIO_REG_ADDR(offset),
					val, mask);
	return ret;

}

static int max77660_gpio_set_debounce(struct gpio_chip *gpio,
		unsigned offset, unsigned debounce)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);
	struct device *parent = max77660_gpio->parent;
	u8 shift = GPIO_CTRL_DBNC_SHIFT;
	u8 val = 0;

	if (max77660_gpio_is_alternate(max77660_gpio, offset)) {
		dev_warn(max77660_gpio->dev,
			"gpio%u is used as alternate mode\n", offset);
		return 0;
	}

	if (debounce == 0)
		val = 0;
	else if ((0 < debounce) && (debounce <= 8))
		val = (GPIO_DBNC_8MS << shift);
	else if ((8 < debounce) && (debounce <= 16))
		val = (GPIO_DBNC_16MS << shift);
	else if ((16 < debounce) && (debounce <= 32))
		val = (GPIO_DBNC_32MS << shift);
	else
		return -EINVAL;

	return max77660_reg_update(parent, MAX77660_PWR_SLAVE,
			GPIO_REG_ADDR(offset), val,
			GPIO_CTRL_DBNC_MASK);
}

static void max77660_gpio_set(struct gpio_chip *gpio, unsigned offset,
			int value)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);
	struct device *parent = max77660_gpio->parent;
	u8 val = (value ? 1 : 0) << GPIO_CTRL_DOUT_SHIFT;

	if (max77660_gpio_is_alternate(max77660_gpio, offset)) {
		dev_err(max77660_gpio->dev,
			"gpio%u is used as alternate mode\n", offset);
		return;
	}

	max77660_reg_update(parent, MAX77660_PWR_SLAVE, GPIO_REG_ADDR(offset),
					val, GPIO_CTRL_DOUT_MASK);
	return;

}

static int max77660_gpio_to_irq(struct gpio_chip *gpio, unsigned offset)
{
	struct max77660_gpio *max77660_gpio = to_max77660_gpio(gpio);

	return max77660_gpio->irq_base + offset;
}

static int max77660_gpio_set_config(struct max77660_gpio *max77660_gpio,
				struct max77660_gpio_config *gpio_cfg)
{
	struct device *parent = max77660_gpio->parent;
	int gpio = gpio_cfg->gpio;
	u8 val = 0, mask = 0;
	int ret = 0;

	if ((gpio < MAX77660_GPIO0) || (MAX77660_GPIO_NR <= gpio))
		return -EINVAL;

	if (gpio_cfg->pull_up != GPIO_PU_DEF) {
		ret = max77660_gpio_set_pull_up(max77660_gpio, gpio,
					gpio_cfg->pull_up);
		if (ret < 0) {
			dev_err(max77660_gpio->dev,
				"Failed to set gpio%d pull-up\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->pull_down != GPIO_PD_DEF) {
		ret = max77660_gpio_set_pull_down(max77660_gpio, gpio,
					gpio_cfg->pull_down);
		if (ret < 0) {
			dev_err(max77660_gpio->dev,
				"Failed to set gpio%d pull-down\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->dir != GPIO_DIR_DEF) {
		mask = GPIO_CTRL_DIR_MASK;
		if (gpio_cfg->dir == GPIO_DIR_IN) {
			val |= GPIO_CTRL_DIR_MASK;
		} else {
			if (gpio_cfg->dout != GPIO_DOUT_DEF) {
				mask |= GPIO_CTRL_DOUT_MASK;
				if (gpio_cfg->dout == GPIO_DOUT_HIGH)
					val |= GPIO_CTRL_DOUT_MASK;
			}

			if (gpio_cfg->out_drv != GPIO_OUT_DRV_DEF) {
				mask |= GPIO_CTRL_OUT_DRV_MASK;
				if (gpio_cfg->out_drv == GPIO_OUT_DRV_PUSH_PULL)
					val |= GPIO_CTRL_OUT_DRV_MASK;
			}
		}

		ret = max77660_reg_write(parent, MAX77660_PWR_SLAVE,
						GPIO_REG_ADDR(gpio), val);

		if (ret < 0) {
			dev_err(max77660_gpio->dev,
				"Failed to set gpio%d control\n", gpio);
			return ret;
		}
	}

	if (gpio_cfg->alternate != GPIO_ALT_DEF) {
		ret = max77660_gpio_config_alternate(
				gpio + max77660_gpio->gpio_base,
				gpio_cfg->alternate);
		if (ret < 0) {
			dev_err(max77660_gpio->dev,
				"Failed to set gpio%d alternate\n", gpio);
			return ret;
		}
	}

	return 0;
}

static int max77660_gpio_irq_init(struct max77660_gpio *max77660_gpio,
		struct max77660_platform_data *pdata)
{
	struct device *parent = max77660_gpio->parent;
	struct max77660_chip *chip = dev_get_drvdata(parent);
	int ret;

	max77660_gpio->gpio_irq = pdata->irq_base + MAX77660_IRQ_INT_TOP_GPIO;
	max77660_gpio->irq_base = pdata->irq_base + MAX77660_IRQ_GPIO0;
	ret = regmap_add_irq_chip(chip->rmap[MAX77660_PWR_SLAVE],
		max77660_gpio->gpio_irq, IRQF_ONESHOT,
		max77660_gpio->irq_base,
		&max77660_gpio_irq_chip, &chip->gpio_irq_data);
	if (ret < 0) {
		dev_err(max77660_gpio->dev,
			"Failed to add gpio irq_chip %d\n", ret);
		return ret;
	}
	return 0;
}

static void max77660_gpio_irq_remove(struct max77660_gpio *max77660_gpio)
{
	struct max77660_chip *chip;

	chip = dev_get_drvdata(max77660_gpio->dev->parent);
	regmap_del_irq_chip(max77660_gpio->gpio_irq,
			chip->gpio_irq_data);
	chip->gpio_irq_data = NULL;
}

static int max77660_gpio_init_regs(struct max77660_gpio *max77660_gpio,
		struct max77660_platform_data *pdata)
{
	int ret;
	int i;

	for (i = 0; i < pdata->num_gpio_cfgs; i++) {
		ret = max77660_gpio_set_config(max77660_gpio,
					&pdata->gpio_cfgs[i]);
		if (ret < 0) {
			dev_err(max77660_gpio->dev,
				"Failed to set gpio config\n");
			return ret;
		}
	}

	return 0;
}

static int __devinit max77660_gpio_probe(struct platform_device *pdev)
{
	struct max77660_platform_data *pdata;
	struct max77660_gpio *max77660_gpio;
	int ret;
	int gpio_irq;

	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data not found\n");
		return -ENODEV;
	}

	gpio_irq = platform_get_irq(pdev, 0);
	if (gpio_irq <= 0) {
		dev_err(&pdev->dev, "Gpio interrupt is not available\n");
		return -ENODEV;
	}

	max77660_gpio = devm_kzalloc(&pdev->dev,
				sizeof(*max77660_gpio), GFP_KERNEL);
	if (!max77660_gpio) {
		dev_err(&pdev->dev, "Could not allocate max77660_gpio\n");
		return -ENOMEM;
	}

	max77660_gpio->parent = pdev->dev.parent;
	max77660_gpio->dev = &pdev->dev;
	max77660_gpio->gpio_irq = gpio_irq;

	max77660_gpio->gpio_chip.owner = THIS_MODULE;
	max77660_gpio->gpio_chip.label = pdev->name;
	max77660_gpio->gpio_chip.dev = &pdev->dev;
	max77660_gpio->gpio_chip.direction_input = max77660_gpio_dir_input;
	max77660_gpio->gpio_chip.get = max77660_gpio_get;
	max77660_gpio->gpio_chip.direction_output = max77660_gpio_dir_output;
	max77660_gpio->gpio_chip.set_debounce = max77660_gpio_set_debounce;
	max77660_gpio->gpio_chip.set = max77660_gpio_set;
	max77660_gpio->gpio_chip.to_irq = max77660_gpio_to_irq;
	max77660_gpio->gpio_chip.ngpio = MAX77660_GPIO_NR;
	max77660_gpio->gpio_chip.can_sleep = 1;
	if (pdata->gpio_base)
		max77660_gpio->gpio_chip.base = pdata->gpio_base;
	else
		max77660_gpio->gpio_chip.base = -1;

	max77660_gpio_chip = max77660_gpio;
	ret = max77660_gpio_init_regs(max77660_gpio, pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio_init regs failed\n");
		return ret;
	}

	ret = gpiochip_add(&max77660_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio_init: Failed to add gpiomax77660_gpio\n");
		return ret;
	}
	max77660_gpio->gpio_base = max77660_gpio->gpio_chip.base;

	ret = max77660_gpio_irq_init(max77660_gpio, pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio irq init failed: %d\n", ret);
		goto fail;
	}

	platform_set_drvdata(pdev, max77660_gpio);
	return 0;

fail:
	ret = gpiochip_remove(&max77660_gpio->gpio_chip);
	return ret;
}

static int __devexit max77660_gpio_remove(struct platform_device *pdev)
{
	struct max77660_gpio *max77660_gpio = platform_get_drvdata(pdev);

	max77660_gpio_irq_remove(max77660_gpio);
	max77660_gpio_chip = 0;

	return gpiochip_remove(&max77660_gpio->gpio_chip);
}

static struct platform_driver max77660_gpio_driver = {
	.driver.name	= "max77660-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= max77660_gpio_probe,
	.remove		= __devexit_p(max77660_gpio_remove),
};

static int __init max77660_gpio_init(void)
{
	return platform_driver_register(&max77660_gpio_driver);
}
subsys_initcall(max77660_gpio_init);

static void __exit max77660_gpio_exit(void)
{
	platform_driver_unregister(&max77660_gpio_driver);
}
module_exit(max77660_gpio_exit);

MODULE_DESCRIPTION("GPIO interface for MAX77660 PMIC");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maxim Integrated");
MODULE_ALIAS("platform:max77660-gpio");
