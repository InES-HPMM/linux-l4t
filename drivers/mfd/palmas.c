/*
 * TI Palmas MFD Driver
 *
 * Copyright 2011-2012 Texas Instruments Inc.
 *
 * Author: Graeme Gregory <gg@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/mfd/core.h>
#include <linux/mfd/palmas.h>
#include <linux/of_platform.h>

#define EXT_PWR_REQ (PALMAS_EXT_CONTROL_ENABLE1 |	\
			PALMAS_EXT_CONTROL_ENABLE2 |	\
			PALMAS_EXT_CONTROL_NSLEEP)

enum palmas_ids {
	PALMAS_PMIC_ID,
	PALMAS_GPIO_ID,
	PALMAS_LEDS_ID,
	PALMAS_WDT_ID,
	PALMAS_RTC_ID,
	PALMAS_PWRBUTTON_ID,
	PALMAS_GPADC_ID,
	PALMAS_RESOURCE_ID,
	PALMAS_CLK_ID,
	PALMAS_PWM_ID,
	PALMAS_USB_ID,
	PALMAS_EXTCON_ID,
};

static struct resource palmas_rtc_resources[] = {
	{
		.start  = PALMAS_RTC_ALARM_IRQ,
		.end    = PALMAS_RTC_ALARM_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell palmas_children[] = {
	{
		.name = "palmas-pmic",
		.id = PALMAS_PMIC_ID,
	},
	{
		.name = "palmas-gpio",
		.id = PALMAS_GPIO_ID,
	},
	{
		.name = "palmas-leds",
		.id = PALMAS_LEDS_ID,
	},
	{
		.name = "palmas-wdt",
		.id = PALMAS_WDT_ID,
	},
	{
		.name = "palmas-rtc",
		.id = PALMAS_RTC_ID,
		.resources = &palmas_rtc_resources[0],
		.num_resources = ARRAY_SIZE(palmas_rtc_resources),
	},
	{
		.name = "palmas-pwrbutton",
		.id = PALMAS_PWRBUTTON_ID,
	},
	{
		.name = "palmas-gpadc",
		.id = PALMAS_GPADC_ID,
	},
	{
		.name = "palmas-resource",
		.id = PALMAS_RESOURCE_ID,
	},
	{
		.name = "palmas-clk",
		.id = PALMAS_CLK_ID,
	},
	{
		.name = "palmas-pwm",
		.id = PALMAS_PWM_ID,
	},
	{
		.name = "palmas-usb",
		.id = PALMAS_USB_ID,
	},
	{
		.name = "palmas-extcon",
		.id = PALMAS_EXTCON_ID,
	}
};

static const struct regmap_config palmas_regmap_config[PALMAS_NUM_CLIENTS] = {
	{
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = PALMAS_BASE_TO_REG(PALMAS_PU_PD_OD_BASE,
					PALMAS_PRIMARY_SECONDARY_PAD3),
	},
	{
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = PALMAS_BASE_TO_REG(PALMAS_GPADC_BASE,
					PALMAS_GPADC_SMPS_VSEL_MONITORING),
	},
	{
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = PALMAS_BASE_TO_REG(PALMAS_TRIM_GPADC_BASE,
					PALMAS_GPADC_TRIM16),
	},
};

static const struct regmap_irq palmas_irqs[] = {
	/* INT1 IRQs */
	[PALMAS_CHARG_DET_N_VBUS_OVV_IRQ] = {
		.mask = PALMAS_INT1_STATUS_CHARG_DET_N_VBUS_OVV,
	},
	[PALMAS_PWRON_IRQ] = {
		.mask = PALMAS_INT1_STATUS_PWRON,
	},
	[PALMAS_LONG_PRESS_KEY_IRQ] = {
		.mask = PALMAS_INT1_STATUS_LONG_PRESS_KEY,
	},
	[PALMAS_RPWRON_IRQ] = {
		.mask = PALMAS_INT1_STATUS_RPWRON,
	},
	[PALMAS_PWRDOWN_IRQ] = {
		.mask = PALMAS_INT1_STATUS_PWRDOWN,
	},
	[PALMAS_HOTDIE_IRQ] = {
		.mask = PALMAS_INT1_STATUS_HOTDIE,
	},
	[PALMAS_VSYS_MON_IRQ] = {
		.mask = PALMAS_INT1_STATUS_VSYS_MON,
	},
	[PALMAS_VBAT_MON_IRQ] = {
		.mask = PALMAS_INT1_STATUS_VBAT_MON,
	},
	/* INT2 IRQs*/
	[PALMAS_RTC_ALARM_IRQ] = {
		.mask = PALMAS_INT2_STATUS_RTC_ALARM,
		.reg_offset = 1,
	},
	[PALMAS_RTC_TIMER_IRQ] = {
		.mask = PALMAS_INT2_STATUS_RTC_TIMER,
		.reg_offset = 1,
	},
	[PALMAS_WDT_IRQ] = {
		.mask = PALMAS_INT2_STATUS_WDT,
		.reg_offset = 1,
	},
	[PALMAS_BATREMOVAL_IRQ] = {
		.mask = PALMAS_INT2_STATUS_BATREMOVAL,
		.reg_offset = 1,
	},
	[PALMAS_RESET_IN_IRQ] = {
		.mask = PALMAS_INT2_STATUS_RESET_IN,
		.reg_offset = 1,
	},
	[PALMAS_FBI_BB_IRQ] = {
		.mask = PALMAS_INT2_STATUS_FBI_BB,
		.reg_offset = 1,
	},
	[PALMAS_SHORT_IRQ] = {
		.mask = PALMAS_INT2_STATUS_SHORT,
		.reg_offset = 1,
	},
	[PALMAS_VAC_ACOK_IRQ] = {
		.mask = PALMAS_INT2_STATUS_VAC_ACOK,
		.reg_offset = 1,
	},
	/* INT3 IRQs */
	[PALMAS_GPADC_AUTO_0_IRQ] = {
		.mask = PALMAS_INT3_STATUS_GPADC_AUTO_0,
		.reg_offset = 2,
	},
	[PALMAS_GPADC_AUTO_1_IRQ] = {
		.mask = PALMAS_INT3_STATUS_GPADC_AUTO_1,
		.reg_offset = 2,
	},
	[PALMAS_GPADC_EOC_SW_IRQ] = {
		.mask = PALMAS_INT3_STATUS_GPADC_EOC_SW,
		.reg_offset = 2,
	},
	[PALMAS_GPADC_EOC_RT_IRQ] = {
		.mask = PALMAS_INT3_STATUS_GPADC_EOC_RT,
		.reg_offset = 2,
	},
	[PALMAS_ID_OTG_IRQ] = {
		.mask = PALMAS_INT3_STATUS_ID_OTG,
		.reg_offset = 2,
	},
	[PALMAS_ID_IRQ] = {
		.mask = PALMAS_INT3_STATUS_ID,
		.reg_offset = 2,
	},
	[PALMAS_VBUS_OTG_IRQ] = {
		.mask = PALMAS_INT3_STATUS_VBUS_OTG,
		.reg_offset = 2,
	},
	[PALMAS_VBUS_IRQ] = {
		.mask = PALMAS_INT3_STATUS_VBUS,
		.reg_offset = 2,
	},
	/* INT4 IRQs */
	[PALMAS_GPIO_0_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_0,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_1_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_1,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_2_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_2,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_3_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_3,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_4_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_4,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_5_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_5,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_6_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_6,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_7_IRQ] = {
		.mask = PALMAS_INT4_STATUS_GPIO_7,
		.reg_offset = 3,
	},
	[PALMAS_GPIO_8_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_8,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_9_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_9,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_10_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_10,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_11_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_11,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_12_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_12,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_13_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_13,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_14_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_14,
		.reg_offset = 4,
	},
	[PALMAS_GPIO_15_IRQ] = {
		.mask = PALMAS_INT5_STATUS_GPIO_15,
		.reg_offset = 4,
	},
};

static struct regmap_irq_chip palmas_irq_chip = {
	.name = "palmas",
	.irqs = palmas_irqs,
	.num_irqs = ARRAY_SIZE(palmas_irqs),

	.num_regs = 5,
	.irq_reg_stride = 5,
	.status_base = PALMAS_BASE_TO_REG(PALMAS_INTERRUPT_BASE,
			PALMAS_INT1_STATUS),
	.mask_base = PALMAS_BASE_TO_REG(PALMAS_INTERRUPT_BASE,
			PALMAS_INT1_MASK),
	.wake_base = 1,
};

struct palmas_sleep_requestor_info {
	int id;
	int reg_offset;
	int bit_pos;
};

#define SLEEP_REQUESTOR(_id, _offset, _pos)		\
	[PALMAS_SLEEP_REQSTR_ID_##_id] = {		\
		.id = PALMAS_SLEEP_REQSTR_ID_##_id,	\
		.reg_offset = _offset,			\
		.bit_pos = _pos,			\
	}

static struct palmas_sleep_requestor_info sleep_reqt_info[] = {
	SLEEP_REQUESTOR(REGEN1, 0, 0),
	SLEEP_REQUESTOR(REGEN2, 0, 1),
	SLEEP_REQUESTOR(SYSEN1, 0, 2),
	SLEEP_REQUESTOR(SYSEN2, 0, 3),
	SLEEP_REQUESTOR(CLK32KG, 0, 4),
	SLEEP_REQUESTOR(CLK32KGAUDIO, 0, 5),
	SLEEP_REQUESTOR(REGEN3, 0, 6),
	SLEEP_REQUESTOR(SMPS12, 1, 0),
	SLEEP_REQUESTOR(SMPS3, 1, 1),
	SLEEP_REQUESTOR(SMPS45, 1, 2),
	SLEEP_REQUESTOR(SMPS6, 1, 3),
	SLEEP_REQUESTOR(SMPS7, 1, 4),
	SLEEP_REQUESTOR(SMPS8, 1, 5),
	SLEEP_REQUESTOR(SMPS9, 1, 6),
	SLEEP_REQUESTOR(SMPS10, 1, 7),
	SLEEP_REQUESTOR(LDO1, 2, 0),
	SLEEP_REQUESTOR(LDO2, 2, 1),
	SLEEP_REQUESTOR(LDO3, 2, 2),
	SLEEP_REQUESTOR(LDO4, 2, 3),
	SLEEP_REQUESTOR(LDO5, 2, 4),
	SLEEP_REQUESTOR(LDO6, 2, 5),
	SLEEP_REQUESTOR(LDO7, 2, 6),
	SLEEP_REQUESTOR(LDO8, 2, 7),
	SLEEP_REQUESTOR(LDO9, 3, 0),
	SLEEP_REQUESTOR(LDOLN, 3, 1),
	SLEEP_REQUESTOR(LDOUSB, 3, 2),
	SLEEP_REQUESTOR(LDO10, 3, 3),
	SLEEP_REQUESTOR(LDO11, 3, 4),
	SLEEP_REQUESTOR(LDO12, 3, 5),
	SLEEP_REQUESTOR(LDO13, 3, 6),
	SLEEP_REQUESTOR(LDO14, 3, 7),
	SLEEP_REQUESTOR(REGEN4, 0, 7),
	SLEEP_REQUESTOR(REGEN5, 18, 0),
	SLEEP_REQUESTOR(REGEN7, 18, 2),
};

struct palmas_clk32k_info {
	unsigned int control_reg;
	unsigned int sleep_reqstr_id;
};

static struct palmas_clk32k_info palmas_clk32k_info[] = {
	{
		.control_reg = PALMAS_CLK32KG_CTRL,
		.sleep_reqstr_id = PALMAS_SLEEP_REQSTR_ID_CLK32KG,
	}, {
		.control_reg = PALMAS_CLK32KGAUDIO_CTRL,
		.sleep_reqstr_id = PALMAS_SLEEP_REQSTR_ID_CLK32KGAUDIO,
	},
};

static int palmas_resource_write(struct palmas *palmas, unsigned int reg,
	unsigned int value)
{
	unsigned int addr = PALMAS_BASE_TO_REG(PALMAS_RESOURCE_BASE, reg);

	return regmap_write(palmas->regmap[0], addr, value);
}

static int palmas_resource_update(struct palmas *palmas, unsigned int reg,
	unsigned int mask, unsigned int value)
{
	unsigned int addr = PALMAS_BASE_TO_REG(PALMAS_RESOURCE_BASE, reg);

	return regmap_update_bits(palmas->regmap[0], addr, mask, value);
}

static int palmas_control_update(struct palmas *palmas, unsigned int reg,
	unsigned int mask, unsigned int value)
{
	unsigned int addr = PALMAS_BASE_TO_REG(PALMAS_PMU_CONTROL_BASE, reg);

	return regmap_update_bits(palmas->regmap[0], addr, mask, value);
}

int palmas_ext_power_req_config(struct palmas *palmas,
		int id, int ext_pwr_ctrl, bool enable)
{
	int preq_mask_bit = 0;
	int ret;
	int base_reg = 0;
	int bit_pos;

	if (!(ext_pwr_ctrl & EXT_PWR_REQ))
		return 0;

	if (id >= PALMAS_SLEEP_REQSTR_ID_MAX)
		return 0;

	if (palmas->id == TPS80036 && id == PALMAS_SLEEP_REQSTR_ID_REGEN3)
		return 0;

	if (palmas->id != TPS80036 && id > PALMAS_SLEEP_REQSTR_ID_LDO14)
		return 0;

	if (ext_pwr_ctrl & PALMAS_EXT_CONTROL_NSLEEP) {
		if (id <= PALMAS_SLEEP_REQSTR_ID_REGEN4)
			base_reg = PALMAS_NSLEEP_RES_ASSIGN;
		else
			base_reg = PALMAS_NSLEEP_RES_ASSIGN2;
		preq_mask_bit = 0;
	} else if (ext_pwr_ctrl & PALMAS_EXT_CONTROL_ENABLE1) {
		if (id <= PALMAS_SLEEP_REQSTR_ID_REGEN4)
			base_reg = PALMAS_ENABLE1_RES_ASSIGN;
		else
			base_reg = PALMAS_ENABLE1_RES_ASSIGN2;
		preq_mask_bit = 1;
	} else if (ext_pwr_ctrl & PALMAS_EXT_CONTROL_ENABLE2) {
		if (id <= PALMAS_SLEEP_REQSTR_ID_REGEN4)
			base_reg = PALMAS_ENABLE2_RES_ASSIGN;
		else
			base_reg = PALMAS_ENABLE2_RES_ASSIGN2;
		preq_mask_bit = 2;
	}

	bit_pos = sleep_reqt_info[id].bit_pos;
	base_reg += sleep_reqt_info[id].reg_offset;
	if (enable)
		ret = palmas_resource_update(palmas, base_reg,
				BIT(bit_pos), BIT(bit_pos));
	else
		ret = palmas_resource_update(palmas, base_reg,
				BIT(bit_pos), 0);
	if (ret < 0) {
		dev_err(palmas->dev, "Update on resource reg failed\n");
		return ret;
	}

	/* Unmask the PREQ */
	ret = palmas_control_update(palmas, PALMAS_POWER_CTRL,
				BIT(preq_mask_bit), 0);
	if (ret < 0) {
		dev_err(palmas->dev, "Power control register update fails\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(palmas_ext_power_req_config);

static void palmas_init_ext_control(struct palmas *palmas)
{
	int ret;
	int i;

	/* Clear all external control for this rail */
	for (i = 0; i < 12; ++i) {
		ret = palmas_resource_write(palmas,
				PALMAS_NSLEEP_RES_ASSIGN + i, 0);
		if (ret < 0)
			dev_err(palmas->dev,
				"Error in clearing res assign register\n");
	}

	/* Mask the PREQ */
	ret = palmas_control_update(palmas, PALMAS_POWER_CTRL, 0x7, 0x7);
	if (ret < 0)
		dev_err(palmas->dev, "Power control reg write failed\n");
}

static void palmas_clk32k_init(struct palmas *palmas,
	struct palmas_platform_data *pdata)
{
	int ret;
	struct palmas_clk32k_init_data *clk32_idata = pdata->clk32k_init_data;
	int data_size = pdata->clk32k_init_data_size;
	unsigned int reg;
	int i;
	int id;

	if (!clk32_idata || !data_size)
		return;

	for (i = 0; i < data_size; ++i) {
		struct palmas_clk32k_init_data *clk32_pd =  &clk32_idata[i];

		reg = palmas_clk32k_info[clk32_pd->clk32k_id].control_reg;
		if (clk32_pd->enable)
			ret = palmas_resource_update(palmas, reg,
					PALMAS_CLK32KG_CTRL_MODE_ACTIVE,
					PALMAS_CLK32KG_CTRL_MODE_ACTIVE);
		else
			ret = palmas_resource_update(palmas, reg,
					PALMAS_CLK32KG_CTRL_MODE_ACTIVE, 0);
		if (ret < 0) {
			dev_err(palmas->dev, "Error in updating clk reg\n");
			return;
		}

		/* Sleep control */
		id = palmas_clk32k_info[clk32_pd->clk32k_id].sleep_reqstr_id;
		if (clk32_pd->sleep_control) {
			ret = palmas_ext_power_req_config(palmas, id,
					clk32_pd->sleep_control, true);
			if (ret < 0) {
				dev_err(palmas->dev,
					"Error in ext power control reg\n");
				return;
			}

			ret = palmas_resource_update(palmas, reg,
					PALMAS_CLK32KG_CTRL_MODE_SLEEP,
					PALMAS_CLK32KG_CTRL_MODE_SLEEP);
			if (ret < 0) {
				dev_err(palmas->dev,
					"Error in updating clk reg\n");
				return;
			}
		} else {

			ret = palmas_resource_update(palmas, reg,
					PALMAS_CLK32KG_CTRL_MODE_SLEEP, 0);
			if (ret < 0) {
				dev_err(palmas->dev,
					"Error in updating clk reg\n");
				return;
			}
		}
	}
}

static int palmas_set_pdata_irq_flag(struct i2c_client *i2c,
		struct palmas_platform_data *pdata)
{
	struct irq_data *irq_data = irq_get_irq_data(i2c->irq);
	if (!irq_data) {
		dev_err(&i2c->dev, "Invalid IRQ: %d\n", i2c->irq);
		return -EINVAL;
	}

	pdata->irq_flags = irqd_get_trigger_type(irq_data);
	dev_info(&i2c->dev, "Irq flag is 0x%08x\n", pdata->irq_flags);
	return 0;
}

static void palmas_dt_to_pdata(struct i2c_client *i2c,
		struct palmas_platform_data *pdata)
{
	struct device_node *node = i2c->dev.of_node;
	int ret;
	u32 prop;

	ret = of_property_read_u32(node, "ti,mux-pad1", &prop);
	if (!ret) {
		pdata->mux_from_pdata = 1;
		pdata->pad1 = prop;
	}

	ret = of_property_read_u32(node, "ti,mux-pad2", &prop);
	if (!ret) {
		pdata->mux_from_pdata = 1;
		pdata->pad2 = prop;
	}

	/* The default for this register is all masked */
	ret = of_property_read_u32(node, "ti,power-ctrl", &prop);
	if (!ret)
		pdata->power_ctrl = prop;
	else
		pdata->power_ctrl = PALMAS_POWER_CTRL_NSLEEP_MASK |
					PALMAS_POWER_CTRL_ENABLE1_MASK |
					PALMAS_POWER_CTRL_ENABLE2_MASK;
	if (i2c->irq)
		palmas_set_pdata_irq_flag(i2c, pdata);
}

static struct palmas *palmas_dev;
static void palmas_power_off(void)
{
	if (!palmas_dev)
		return;

	palmas_control_update(palmas_dev, PALMAS_DEV_CTRL, 1, 0);
}

static int palmas_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct palmas *palmas;
	struct palmas_platform_data *pdata;
	struct device_node *node = i2c->dev.of_node;
	int ret = 0, i;
	unsigned int reg, addr;
	int slave;
	struct mfd_cell *children;

	pdata = dev_get_platdata(&i2c->dev);

	if (node && !pdata) {
		pdata = devm_kzalloc(&i2c->dev, sizeof(*pdata), GFP_KERNEL);

		if (!pdata)
			return -ENOMEM;

		palmas_dt_to_pdata(i2c, pdata);
	}

	if (!pdata)
		return -EINVAL;

	palmas = devm_kzalloc(&i2c->dev, sizeof(struct palmas), GFP_KERNEL);
	if (palmas == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, palmas);
	palmas->dev = &i2c->dev;
	palmas->id = id->driver_data;
	palmas->irq = i2c->irq;

	for (i = 0; i < PALMAS_NUM_CLIENTS; i++) {
		if (i == 0)
			palmas->i2c_clients[i] = i2c;
		else {
			palmas->i2c_clients[i] =
					i2c_new_dummy(i2c->adapter,
							i2c->addr + i);
			if (!palmas->i2c_clients[i]) {
				dev_err(palmas->dev,
					"can't attach client %d\n", i);
				ret = -ENOMEM;
				goto err;
			}
			palmas->i2c_clients[i]->dev.of_node = of_node_get(node);
		}
		palmas->regmap[i] = devm_regmap_init_i2c(palmas->i2c_clients[i],
				&palmas_regmap_config[i]);
		if (IS_ERR(palmas->regmap[i])) {
			ret = PTR_ERR(palmas->regmap[i]);
			dev_err(palmas->dev,
				"Failed to allocate regmap %d, err: %d\n",
				i, ret);
			goto err;
		}
	}

	/* Change interrupt line output polarity */
	if (pdata->irq_flags & IRQ_TYPE_LEVEL_HIGH)
		reg = PALMAS_POLARITY_CTRL_INT_POLARITY;
	else
		reg = 0;
	ret = palmas_update_bits(palmas, PALMAS_PU_PD_OD_BASE,
			PALMAS_POLARITY_CTRL, PALMAS_POLARITY_CTRL_INT_POLARITY,
			reg);
	if (ret < 0) {
		dev_err(palmas->dev, "POLARITY_CTRL updat failed: %d\n", ret);
		goto err;
	}

	/* Change IRQ into clear on read mode for efficiency */
	slave = PALMAS_BASE_TO_SLAVE(PALMAS_INTERRUPT_BASE);
	addr = PALMAS_BASE_TO_REG(PALMAS_INTERRUPT_BASE, PALMAS_INT_CTRL);
	reg = PALMAS_INT_CTRL_INT_CLEAR;

	regmap_write(palmas->regmap[slave], addr, reg);

	if (palmas->id != TPS80036) {
		/* INT5 is only supported for TPS80036 */
		palmas_irq_chip.num_irqs = ARRAY_SIZE(palmas_irqs) - 8;
		palmas_irq_chip.num_regs = 4;
	}
	ret = regmap_add_irq_chip(palmas->regmap[slave], palmas->irq,
			IRQF_ONESHOT | pdata->irq_flags, pdata->irq_base,
			&palmas_irq_chip, &palmas->irq_data);
	if (ret < 0)
		goto err;

	slave = PALMAS_BASE_TO_SLAVE(PALMAS_PU_PD_OD_BASE);
	addr = PALMAS_BASE_TO_REG(PALMAS_PU_PD_OD_BASE,
			PALMAS_PRIMARY_SECONDARY_PAD1);

	if (pdata->mux_from_pdata) {
		reg = pdata->pad1;
		ret = regmap_write(palmas->regmap[slave], addr, reg);
		if (ret)
			goto err_irq;
	} else {
		ret = regmap_read(palmas->regmap[slave], addr, &reg);
		if (ret)
			goto err_irq;
	}

	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_0))
		palmas->gpio_muxed |= PALMAS_GPIO_0_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_1_MASK))
		palmas->gpio_muxed |= PALMAS_GPIO_1_MUXED;
	else if ((reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_1_MASK) ==
			(2 << PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_1_SHIFT))
		palmas->led_muxed |= PALMAS_LED1_MUXED;
	else if ((reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_1_MASK) ==
			(3 << PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_1_SHIFT))
		palmas->pwm_muxed |= PALMAS_PWM1_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_2_MASK))
		palmas->gpio_muxed |= PALMAS_GPIO_2_MUXED;
	else if ((reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_2_MASK) ==
			(2 << PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_2_SHIFT))
		palmas->led_muxed |= PALMAS_LED2_MUXED;
	else if ((reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_2_MASK) ==
			(3 << PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_2_SHIFT))
		palmas->pwm_muxed |= PALMAS_PWM2_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD1_GPIO_3))
		palmas->gpio_muxed |= PALMAS_GPIO_3_MUXED;

	addr = PALMAS_BASE_TO_REG(PALMAS_PU_PD_OD_BASE,
			PALMAS_PRIMARY_SECONDARY_PAD2);

	if (pdata->mux_from_pdata) {
		reg = pdata->pad2;
		ret = regmap_write(palmas->regmap[slave], addr, reg);
		if (ret)
			goto err_irq;
	} else {
		ret = regmap_read(palmas->regmap[slave], addr, &reg);
		if (ret)
			goto err_irq;
	}

	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD2_GPIO_4))
		palmas->gpio_muxed |= PALMAS_GPIO_4_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD2_GPIO_5_MASK))
		palmas->gpio_muxed |= PALMAS_GPIO_5_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD2_GPIO_6))
		palmas->gpio_muxed |= PALMAS_GPIO_6_MUXED;
	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD2_GPIO_7_MASK))
		palmas->gpio_muxed |= PALMAS_GPIO_7_MUXED;

	addr = PALMAS_BASE_TO_REG(PALMAS_PU_PD_OD_BASE,
			PALMAS_PRIMARY_SECONDARY_PAD3);

	if (pdata->mux_from_pdata) {
		reg = pdata->pad3;
		ret = regmap_write(palmas->regmap[slave], addr, reg);
		if (ret)
			goto err;
	} else {
		ret = regmap_read(palmas->regmap[slave], addr, &reg);
		if (ret)
			goto err;
	}

	if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD3_DVFS2))
		palmas->gpio_muxed |= PALMAS_GPIO_6_MUXED;


	if (palmas->id != TPS80036) {
		palmas->ngpio = 8;
	} else {
		palmas->ngpio = 16;

		addr = PALMAS_BASE_TO_REG(PALMAS_PU_PD_OD_BASE,
				PALMAS_PRIMARY_SECONDARY_PAD4);

		if (pdata->mux_from_pdata) {
			reg = pdata->pad4;
			ret = regmap_write(palmas->regmap[slave], addr, reg);
			if (ret)
				goto err;
		} else {
			ret = regmap_read(palmas->regmap[slave], addr, &reg);
			if (ret)
				goto err;
		}
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_8_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_8_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_9_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_9_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_10_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_10_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_11_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_11_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_12_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_12_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_13_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_13_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_14_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_14_MUXED;
		if (!(reg & PALMAS_PRIMARY_SECONDARY_PAD4_GPIO_15_MASK))
			palmas->gpio_muxed |= PALMAS_GPIO_15_MUXED;
	}

	dev_info(palmas->dev, "Muxing GPIO %x, PWM %x, LED %x\n",
			palmas->gpio_muxed, palmas->pwm_muxed,
			palmas->led_muxed);

	reg = pdata->power_ctrl;

	slave = PALMAS_BASE_TO_SLAVE(PALMAS_PMU_CONTROL_BASE);
	addr = PALMAS_BASE_TO_REG(PALMAS_PMU_CONTROL_BASE, PALMAS_POWER_CTRL);

	ret = regmap_write(palmas->regmap[slave], addr, reg);
	if (ret)
		goto err_irq;

	/*
	 * If we are probing with DT do this the DT way and return here
	 * otherwise continue and add devices using mfd helpers.
	 */
	if (node) {
		ret = of_platform_populate(node, NULL, NULL, &i2c->dev);
		if (ret < 0)
			goto err_irq;
		else
			return ret;
	}


	palmas_init_ext_control(palmas);

	palmas_clk32k_init(palmas, pdata);

	children = kmemdup(palmas_children, sizeof(palmas_children),
			   GFP_KERNEL);
	if (!children) {
		ret = -ENOMEM;
		goto err_irq;
	}

	children[PALMAS_PMIC_ID].platform_data = pdata->pmic_pdata;
	children[PALMAS_PMIC_ID].pdata_size = sizeof(*pdata->pmic_pdata);

	children[PALMAS_GPADC_ID].platform_data = pdata->gpadc_pdata;
	children[PALMAS_GPADC_ID].pdata_size = sizeof(*pdata->gpadc_pdata);

	children[PALMAS_RESOURCE_ID].platform_data = pdata->resource_pdata;
	children[PALMAS_RESOURCE_ID].pdata_size =
			sizeof(*pdata->resource_pdata);

	children[PALMAS_USB_ID].platform_data = pdata->usb_pdata;
	children[PALMAS_USB_ID].pdata_size = sizeof(*pdata->usb_pdata);

	children[PALMAS_CLK_ID].platform_data = pdata->clk_pdata;
	children[PALMAS_CLK_ID].pdata_size = sizeof(*pdata->clk_pdata);

	ret = mfd_add_devices(palmas->dev, -1,
			      children, ARRAY_SIZE(palmas_children),
			      NULL, 0,
			      regmap_irq_get_domain(palmas->irq_data));
	kfree(children);

	if (ret < 0)
		goto err_devices;

	if (pdata->use_power_off && !pm_power_off)
		pm_power_off = palmas_power_off;

	palmas_dev = palmas;
	return ret;

err_devices:
	mfd_remove_devices(palmas->dev);
err_irq:
	regmap_del_irq_chip(palmas->irq, palmas->irq_data);
err:
	return ret;
}

static int palmas_i2c_remove(struct i2c_client *i2c)
{
	struct palmas *palmas = i2c_get_clientdata(i2c);

	mfd_remove_devices(palmas->dev);
	regmap_del_irq_chip(palmas->irq, palmas->irq_data);

	return 0;
}
#ifdef CONFIG_PM_SLEEP
static int palmas_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq)
		disable_irq(client->irq);
	return 0;
}

static int palmas_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq)
		enable_irq(client->irq);
	return 0;
}
#endif

static const struct i2c_device_id palmas_i2c_id[] = {
	{ "palmas", PALMAS},
	{ "twl6035", TWL6035},
	{ "twl6037", TWL6037},
	{ "tps65913", TPS65913},
	{ "tps80036", TPS80036},
	{ /* end */ }
};
MODULE_DEVICE_TABLE(i2c, palmas_i2c_id);

static struct of_device_id of_palmas_match_tbl[] = {
	{ .compatible = "ti,palmas", },
	{ /* end */ }
};

static const struct dev_pm_ops palmas_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(palmas_suspend, palmas_resume)
};

static struct i2c_driver palmas_i2c_driver = {
	.driver = {
		   .name = "palmas",
		   .of_match_table = of_palmas_match_tbl,
		   .owner = THIS_MODULE,
		   .pm = &palmas_pm_ops,
	},
	.probe = palmas_i2c_probe,
	.remove = palmas_i2c_remove,
	.id_table = palmas_i2c_id,
};

static int __init palmas_i2c_init(void)
{
	return i2c_add_driver(&palmas_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(palmas_i2c_init);

static void __exit palmas_i2c_exit(void)
{
	i2c_del_driver(&palmas_i2c_driver);
}
module_exit(palmas_i2c_exit);

MODULE_AUTHOR("Graeme Gregory <gg@slimlogic.co.uk>");
MODULE_DESCRIPTION("Palmas chip family multi-function driver");
MODULE_LICENSE("GPL");
