/*
 * as3722.h definitions
 *
 * Copyright (C) 2013 ams
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
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

#ifndef __LINUX_MFD_AS3722_PLAT_H
#define __LINUX_MFD_AS3722_PLAT_H

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/as3722-reg.h>

struct as3722_reg_init {
	u32 reg;
	u32 val;
};

extern const struct regmap_config as3722_regmap_config;


#define AS3722_EXT_CONTROL_ENABLE1		0x1
#define AS3722_EXT_CONTROL_ENABLE2		0x2
#define AS3722_EXT_CONTROL_ENABLE3		0x3

struct as3722_rtc {
	struct rtc_device *rtc;
	int alarm_enabled;      /* used for suspend/resume */
};

struct as3722 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;
	struct regulator_dev *rdevs[AS3722_NUM_REGULATORS];
	struct as3722_rtc rtc;
	int chip_irq;
};

enum {
	AS3722_GPIO_CFG_NO_INVERT = 0,
	AS3722_GPIO_CFG_INVERT = 1,
};

enum {
	AS3722_GPIO_CFG_OUTPUT_DISABLED = 0,
	AS3722_GPIO_CFG_OUTPUT_ENABLED = 1, };

struct as3722_gpio_config {
	int gpio;
	int mode;
	int invert;
	int iosf;
	int output_state;
};

struct as3722_pinctrl_init_data {
	int pin_id;
	int usage;
	int mode;
};

/*
 * as3722_regulator_platform_data: Regulator platform data.
 * @ext_control: External control.
 * @oc_configure_enable: Enable overcurrent configuration.
 * @oc_trip_thres_perphase: Overcurrent trip threshold current in mA per pahse.
 *	This should be 2500, 3000, 3500.
 * @oc_alarm_thres_perphase: Overcurrent alarm threshold current in mA per
 *	pahse. This should be 0 (for disable), 1600, 1800, 2000, 2200, 2400,
 *	2600, 2800.
 */
struct as3722_regulator_platform_data {
	struct regulator_init_data *reg_init;
	int ext_control;
	bool oc_configure_enable;
	int oc_trip_thres_perphase;
	int oc_alarm_thres_perphase;
};

/*
 * as3722_adc_extcon_platform_data: ADC platform data.
 * @connection_name: Extcon connection name.
 */
struct as3722_adc_extcon_platform_data {
	const char *connection_name;
	bool enable_adc1_continuous_mode;
	bool enable_low_voltage_range;
	int adc_channel;
	int hi_threshold;
	int low_threshold;
};

struct as3722_platform_data {
	struct as3722_regulator_platform_data *reg_pdata[AS3722_NUM_REGULATORS];

	/* register initialisation */
	struct as3722_reg_init *core_init_data;
	int gpio_base;
	int irq_base;
	int irq_type;
	int use_internal_int_pullup;
	int use_internal_i2c_pullup;
	int num_gpio_cfgs;
	bool use_power_off;
	bool use_power_reset;
	struct as3722_gpio_config *gpio_cfgs;
	struct as3722_pinctrl_init_data *pinctrl_pdata;
	struct as3722_adc_extcon_platform_data *extcon_pdata;
	int pinctrl_init_data_size;

	bool enable_ldo3_tracking;
	bool disabe_ldo3_tracking_suspend;
	bool enable_clk32k_out;
};

static inline int as3722_reg_read(struct as3722 *as3722, u32 reg, u32 *dest)
{
	return regmap_read(as3722->regmap, reg, dest);
}

static inline int as3722_reg_write(struct as3722 *as3722, u32 reg, u32 value)
{
	return regmap_write(as3722->regmap, reg, value);
}

static inline int as3722_block_read(struct as3722 *as3722, u32 reg,
		int count, u8 *buf)
{
	return regmap_bulk_read(as3722->regmap, reg, buf, count);
}

static inline int as3722_block_write(struct as3722 *as3722, u32 reg,
		int count, u8 *data)
{
	return regmap_bulk_write(as3722->regmap, reg, data, count);
}

static inline int as3722_set_bits(struct as3722 *as3722, u32 reg,
		u32 mask, u8 val)
{
	return regmap_update_bits(as3722->regmap, reg, mask, val);
}

/* ADC */
enum as3722_adc_source {
	AS3722_ADC_SD0 = 0,
	AS3722_ADC_SD1 = 1,
	AS3722_ADC_SD6 = 2,
	AS3722_ADC_TEMP_SENSOR = 3,
	AS3722_ADC_VSUP = 4,
	AS3722_ADC_GPIO1 = 5,
	AS3722_ADC_GPIO2 = 6,
	AS3722_ADC_GPIO3 = 7,
	AS3722_ADC_GPIO4 = 8,
	AS3722_ADC_GPIO6 = 9,
	AS3722_ADC_GPIO7 = 10,
	AS3722_ADC_VBAT = 11,
	AS3722_ADC_PWM_CLK2 = 12,
	AS3722_ADC_PWM_DAT2 = 13,
	AS3722_ADC_TEMP1_SD0 = 16,
	AS3722_ADC_TEMP2_SD0 = 17,
	AS3722_ADC_TEMP3_SD0 = 18,
	AS3722_ADC_TEMP4_SD0 = 19,
	AS3722_ADC_TEMP_SD1 = 20,
	AS3722_ADC_TEMP1_SD6 = 21,
	AS3722_ADC_TEMP2_SD6 = 22,
};

enum as3722_adc_channel {
	AS3722_ADC0 = 0,
	AS3722_ADC1 = 1,
};

int as3722_adc_read(struct as3722 *as3722,
			enum as3722_adc_channel channel,
			enum as3722_adc_source source);

#endif






