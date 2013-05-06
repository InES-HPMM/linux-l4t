/*
 * as3722-regmap.c - regmap for AS3722 PMICs
 *
 * Copyright (C) 2013 ams AG
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

#include <linux/mfd/as3722-reg.h>

/* Default Register Values (for caching)
 * Please make sure to update (or update cache at startup)
 * after device is OTP programmed! */
static struct reg_default as3722_defaults[] = {
	{ 0x0000, 0x0000 }, /* SD0 Voltage */
	{ 0x0001, 0x0000 }, /* SD1 Voltage */
	{ 0x0002, 0x0000 }, /* SD2 Voltage */
	{ 0x0003, 0x0000 }, /* SD3 Voltage */
	{ 0x0004, 0x0000 }, /* SD4 Voltage */
	{ 0x0005, 0x0000 }, /* SD5 Voltage */
	{ 0x0006, 0x0000 }, /* SD6 Voltage */
	{ 0x0008, 0x0003 }, /* GPIO0 Control */
	{ 0x0009, 0x0003 }, /* GPIO1 Control */
	{ 0x000a, 0x0003 }, /* GPIO2 Control */
	{ 0x000b, 0x0003 }, /* GPIO3 Control */
	{ 0x000c, 0x0003 }, /* GPIO4 Control */
	{ 0x000d, 0x0003 }, /* GPIO5 Control */
	{ 0x000e, 0x0003 }, /* GPIO6 Control */
	{ 0x000f, 0x0003 }, /* GPIO7 Control */
	{ 0x0010, 0x0000 }, /* LDO0 Voltage */
	{ 0x0011, 0x0000 }, /* LDO1 Voltage */
	{ 0x0012, 0x0000 }, /* LDO2 Voltage */
	{ 0x0013, 0x0000 }, /* LDO3 Voltage */
	{ 0x0014, 0x0000 }, /* LDO4 Voltage */
	{ 0x0015, 0x0000 }, /* LDO5 Voltage */
	{ 0x0016, 0x0000 }, /* LDO6 Voltage */
	{ 0x0017, 0x0000 }, /* LDO7 Voltage */
	{ 0x0019, 0x0000 }, /* LDO9 Voltage */
	{ 0x001a, 0x0000 }, /* LDO10 Voltage */
	{ 0x001b, 0x0000 }, /* LDO11 Voltage */
	{ 0x001d, 0x0000 }, /* LDO3 Settings */
	{ 0x001e, 0x0000 }, /* GPIO deb1 */
	{ 0x001f, 0x0000 }, /* GPIO deb2 */
	{ 0x0020, 0x0000 }, /* GPIO Signal Out */
	{ 0x0021, 0x0000 }, /* GPIO Signal In */
	{ 0x0022, 0x0000 }, /* Reg_sequ_mod1 */
	{ 0x0023, 0x0000 }, /* Reg_sequ_mod2 */
	{ 0x0024, 0x0000 }, /* Reg_sequ_mod3 */
	{ 0x0027, 0x0000 }, /* SD_phsw_ctrl */
	{ 0x0028, 0x0000 }, /* SD_phsw_status */
	{ 0x0029, 0x0000 }, /* SD0 Control */
	{ 0x002a, 0x0001 }, /* SD1 Control */
	{ 0x002b, 0x0000 }, /* SDmph Control */
	{ 0x002c, 0x0000 }, /* SD23 Control */
	{ 0x002d, 0x0000 }, /* SD4 Control */
	{ 0x002e, 0x0000 }, /* SD5 Control */
	{ 0x002f, 0x0001 }, /* SD6 Control */
	{ 0x0030, 0x0000 }, /* SD_dvm */
	{ 0x0031, 0x0000 }, /* Resetreason */
	{ 0x0032, 0x0000 }, /* Battery Voltage Monitor */
	{ 0x0033, 0x0000 }, /* Startup Control */
	{ 0x0034, 0x0008 }, /* RestTimer */
	{ 0x0035, 0x0000 }, /* ReferenceControl */
	{ 0x0036, 0x0000 }, /* ResetControl */
	{ 0x0037, 0x0001 }, /* OvertemperatureControl */
	{ 0x0038, 0x0000 }, /* WatchdogControl */
	{ 0x0039, 0x0000 }, /* Reg_standby_mod1 */
	{ 0x003a, 0x0000 }, /* Reg_standby_mod2 */
	{ 0x003b, 0x0000 }, /* Reg_standby_mod3 */
	{ 0x003c, 0x0000 }, /* Enable Control 1 */
	{ 0x003d, 0x0000 }, /* Enable Control 2 */
	{ 0x003e, 0x0000 }, /* Enable Control 3 */
	{ 0x003f, 0x0000 }, /* Enable Control 4 */
	{ 0x0040, 0x0000 }, /* Enable Control 5 */
	{ 0x0041, 0x0000 }, /* PWM Control low */
	{ 0x0042, 0x0000 }, /* PWM Control high */
	{ 0x0046, 0x0000 }, /* Watchdog Timer */
	{ 0x0048, 0x0000 }, /* Watchdog Software Signal */
	{ 0x0049, 0x0000 }, /* IO Voltage */
	{ 0x004a, 0x0000 }, /* Battery_voltage_monitor2 */
	{ 0x004d, 0x007f }, /* SDcontrol */
	{ 0x004e, 0x00ff }, /* LDOcontrol0 */
	{ 0x004f, 0x000e }, /* LDOcontrol1 */
	{ 0x0050, 0x0000 }, /* SD0_protect */
	{ 0x0051, 0x0000 }, /* SD6_protect */
	{ 0x0052, 0x0000 }, /* PWM_vcontrol1 */
	{ 0x0053, 0x0000 }, /* PWM_vcontrol2 */
	{ 0x0054, 0x0000 }, /* PWM_vcontrol3 */
	{ 0x0055, 0x0000 }, /* PWM_vcontrol4 */
	{ 0x0057, 0x0040 }, /* BBcharger */
	{ 0x0058, 0x0000 }, /* CTRLsequ1 */
	{ 0x0059, 0x0000 }, /* CTRLsequ2 */
	{ 0x005a, 0x0000 }, /* OVcurrent */
	{ 0x005b, 0x0000 }, /* OVcurrent_deb */
	{ 0x005c, 0x0000 }, /* SDlv_deb */
	{ 0x005d, 0x0000 }, /* OC_pg_ctrl */
	{ 0x005e, 0x0000 }, /* OC_pg_ctrl2 */
	{ 0x005f, 0x0000 }, /* CTRLstatus */
	{ 0x0060, 0x0020 }, /* RTC Control */
	{ 0x0061, 0x0000 }, /* RTCsecond */
	{ 0x0062, 0x0000 }, /* RTCminute */
	{ 0x0063, 0x0000 }, /* RTChour */
	{ 0x0064, 0x0001 }, /* RTCday */
	{ 0x0065, 0x0001 }, /* RTCmonth */
	{ 0x0066, 0x0000 }, /* RTCyear */
	{ 0x0067, 0x0000 }, /* RTCAlarmsecond */
	{ 0x0068, 0x0000 }, /* RTCAlarmminute */
	{ 0x0069, 0x0000 }, /* RTCAlarmhour */
	{ 0x006a, 0x003f }, /* RTCAlarmday */
	{ 0x006b, 0x001f }, /* RTCAlarmmonth */
	{ 0x006c, 0x007f }, /* RTCAlarmyear */
	{ 0x006d, 0x0000 }, /* SRAM */
	{ 0x006f, 0x0000 }, /* RTC_Access */
	{ 0x0073, 0x0000 }, /* RegStatus */
	{ 0x0074, 0x00ff }, /* InterruptMask1 */
	{ 0x0075, 0x00ff }, /* InterruptMask2 */
	{ 0x0076, 0x00ff }, /* InterruptMask3 */
	{ 0x0077, 0x00ff }, /* InterruptMask4 */
	{ 0x0080, 0x0000 }, /* ADC0 Control */
	{ 0x0081, 0x0000 }, /* ADC1 Control */
	{ 0x0086, 0x007f }, /* ADC1 threshold hi MSB */
	{ 0x0087, 0x0007 }, /* ADC1 threshold hi LSB */
	{ 0x0088, 0x0000 }, /* ADC1 threshold lo MSB */
	{ 0x0089, 0x0000 }, /* ADC1 threshold lo LSB */
	{ 0x008a, 0x0000 }, /* ADC Configuration */
};

/*
 * Access masks.
 */
static bool as3722_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AS3722_SD0_VOLTAGE_REG:
	case AS3722_SD1_VOLTAGE_REG:
	case AS3722_SD2_VOLTAGE_REG:
	case AS3722_SD3_VOLTAGE_REG:
	case AS3722_SD4_VOLTAGE_REG:
	case AS3722_SD5_VOLTAGE_REG:
	case AS3722_SD6_VOLTAGE_REG:
	case AS3722_GPIO0_CONTROL_REG:
	case AS3722_GPIO1_CONTROL_REG:
	case AS3722_GPIO2_CONTROL_REG:
	case AS3722_GPIO3_CONTROL_REG:
	case AS3722_GPIO4_CONTROL_REG:
	case AS3722_GPIO5_CONTROL_REG:
	case AS3722_GPIO6_CONTROL_REG:
	case AS3722_GPIO7_CONTROL_REG:
	case AS3722_LDO0_VOLTAGE_REG:
	case AS3722_LDO1_VOLTAGE_REG:
	case AS3722_LDO2_VOLTAGE_REG:
	case AS3722_LDO3_VOLTAGE_REG:
	case AS3722_LDO4_VOLTAGE_REG:
	case AS3722_LDO5_VOLTAGE_REG:
	case AS3722_LDO6_VOLTAGE_REG:
	case AS3722_LDO7_VOLTAGE_REG:
	case AS3722_LDO9_VOLTAGE_REG:
	case AS3722_LDO10_VOLTAGE_REG:
	case AS3722_LDO11_VOLTAGE_REG:
	case 0x1d:
	case 0x1e:
	case 0x1f:
	case AS3722_GPIO_SIGNAL_OUT_REG:
	case AS3722_GPIO_SIGNAL_IN_REG:
	case 0x22:
	case 0x23:
	case 0x24:
	case 0x27:
	case 0x28:
	case AS3722_SD0_CONTROL_REG:
	case AS3722_SD1_CONTROL_REG:
	case AS3722_SDmph_CONTROL_REG:
	case AS3722_SD23_CONTROL_REG:
	case AS3722_SD4_CONTROL_REG:
	case AS3722_SD5_CONTROL_REG:
	case AS3722_SD6_CONTROL_REG:
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x33:
	case 0x34:
	case 0x35:
	case 0x36:
	case 0x37:
	case AS3722_WATCHDOG_CONTROL_REG:
	case 0x39:
	case 0x3a:
	case 0x3b:
	case 0x3c:
	case 0x3d:
	case 0x3e:
	case 0x3f:
	case 0x40:
	case 0x41:
	case 0x42:
	case AS3722_WATCHDOG_TIMER_REG:
	case AS3722_WATCHDOG_SOFTWARE_SIGNAL_REG:
	case AS3722_IOVOLTAGE_REG:
	case 0x4a:
	case AS3722_SD_CONTROL_REG:
	case AS3722_LDOCONTROL0_REG:
	case AS3722_LDOCONTROL1_REG:
	case 0x50:
	case 0x51:
	case 0x52:
	case 0x53:
	case 0x54:
	case 0x55:
	case 0x57:
	case AS3722_CTRL1_REG:
	case AS3722_CTRL2_REG:
	case 0x5a:
	case 0x5b:
	case 0x5c:
	case 0x5d:
	case 0x5e:
	case 0x5f:
	case AS3722_RTC_CONTROL_REG:
	case AS3722_RTC_SECOND_REG:
	case AS3722_RTC_MINUTE_REG:
	case AS3722_RTC_HOUR_REG:
	case AS3722_RTC_DAY_REG:
	case AS3722_RTC_MONTH_REG:
	case AS3722_RTC_YEAR_REG:
	case AS3722_RTC_ALARM_SECOND_REG:
	case AS3722_RTC_ALARM_MINUTE_REG:
	case AS3722_RTC_ALARM_HOUR_REG:
	case AS3722_RTC_ALARM_DAY_REG:
	case AS3722_RTC_ALARM_MONTH_REG:
	case AS3722_RTC_ALARM_YEAR_REG:
	case 0x6d:
	case 0x6f:
	case 0x73:
	case AS3722_INTERRUPTMASK1_REG:
	case AS3722_INTERRUPTMASK2_REG:
	case AS3722_INTERRUPTMASK3_REG:
	case AS3722_INTERRUPTMASK4_REG:
	case AS3722_INTERRUPTSTATUS1_REG:
	case AS3722_INTERRUPTSTATUS2_REG:
	case AS3722_INTERRUPTSTATUS3_REG:
	case AS3722_INTERRUPTSTATUS4_REG:
	case 0x7d:
	case AS3722_ADC0_CONTROL_REG:
	case AS3722_ADC1_CONTROL_REG:
	case AS3722_ADC0_MSB_RESULT_REG:
	case AS3722_ADC0_LSB_RESULT_REG:
	case AS3722_ADC1_MSB_RESULT_REG:
	case AS3722_ADC1_LSB_RESULT_REG:
	case AS3722_ADC1_THRESHOLD_HI_MSB_REG:
	case AS3722_ADC1_THRESHOLD_HI_LSB_REG:
	case AS3722_ADC1_THRESHOLD_LO_MSB_REG:
	case AS3722_ADC1_THRESHOLD_LO_LSB_REG:
	case AS3722_ADC_CONFIG_REG:
	case AS3722_ADDR_ASIC_ID1:
	case AS3722_ADDR_ASIC_ID2:
		return true;
	default:
		return false;
	}
}

static bool as3722_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AS3722_SD0_VOLTAGE_REG:
	case AS3722_SD1_VOLTAGE_REG:
	case AS3722_SD2_VOLTAGE_REG:
	case AS3722_SD3_VOLTAGE_REG:
	case AS3722_SD4_VOLTAGE_REG:
	case AS3722_SD5_VOLTAGE_REG:
	case AS3722_SD6_VOLTAGE_REG:
	case AS3722_GPIO0_CONTROL_REG:
	case AS3722_GPIO1_CONTROL_REG:
	case AS3722_GPIO2_CONTROL_REG:
	case AS3722_GPIO3_CONTROL_REG:
	case AS3722_GPIO4_CONTROL_REG:
	case AS3722_GPIO5_CONTROL_REG:
	case AS3722_GPIO6_CONTROL_REG:
	case AS3722_GPIO7_CONTROL_REG:
	case AS3722_LDO0_VOLTAGE_REG:
	case AS3722_LDO1_VOLTAGE_REG:
	case AS3722_LDO2_VOLTAGE_REG:
	case AS3722_LDO3_VOLTAGE_REG:
	case AS3722_LDO4_VOLTAGE_REG:
	case AS3722_LDO5_VOLTAGE_REG:
	case AS3722_LDO6_VOLTAGE_REG:
	case AS3722_LDO7_VOLTAGE_REG:
	case AS3722_LDO9_VOLTAGE_REG:
	case AS3722_LDO10_VOLTAGE_REG:
	case AS3722_LDO11_VOLTAGE_REG:
	case 0x1d:
	case 0x1e:
	case 0x1f:
	case AS3722_GPIO_SIGNAL_OUT_REG:
	case 0x22:
	case 0x23:
	case 0x24:
	case 0x27:
	case 0x28:
	case AS3722_SD0_CONTROL_REG:
	case AS3722_SD1_CONTROL_REG:
	case AS3722_SDmph_CONTROL_REG:
	case AS3722_SD23_CONTROL_REG:
	case AS3722_SD4_CONTROL_REG:
	case AS3722_SD5_CONTROL_REG:
	case AS3722_SD6_CONTROL_REG:
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x33:
	case 0x34:
	case 0x35:
	case 0x36:
	case 0x37:
	case AS3722_WATCHDOG_CONTROL_REG:
	case 0x39:
	case 0x3a:
	case 0x3b:
	case 0x3c:
	case 0x3d:
	case 0x3e:
	case 0x3f:
	case 0x40:
	case 0x41:
	case 0x42:
	case AS3722_WATCHDOG_TIMER_REG:
	case AS3722_WATCHDOG_SOFTWARE_SIGNAL_REG:
	case AS3722_IOVOLTAGE_REG:
	case 0x4a:
	case AS3722_SD_CONTROL_REG:
	case AS3722_LDOCONTROL0_REG:
	case AS3722_LDOCONTROL1_REG:
	case 0x50:
	case 0x51:
	case 0x52:
	case 0x53:
	case 0x54:
	case 0x55:
	case 0x57:
	case AS3722_CTRL1_REG:
	case AS3722_CTRL2_REG:
	case 0x5a:
	case 0x5b:
	case 0x5c:
	case 0x5d:
	case 0x5e:
	case AS3722_RTC_CONTROL_REG:
	case AS3722_RTC_SECOND_REG:
	case AS3722_RTC_MINUTE_REG:
	case AS3722_RTC_HOUR_REG:
	case AS3722_RTC_DAY_REG:
	case AS3722_RTC_MONTH_REG:
	case AS3722_RTC_YEAR_REG:
	case AS3722_RTC_ALARM_SECOND_REG:
	case AS3722_RTC_ALARM_MINUTE_REG:
	case AS3722_RTC_ALARM_HOUR_REG:
	case AS3722_RTC_ALARM_DAY_REG:
	case AS3722_RTC_ALARM_MONTH_REG:
	case AS3722_RTC_ALARM_YEAR_REG:
	case 0x6d:
	case 0x6f:
	case AS3722_INTERRUPTMASK1_REG:
	case AS3722_INTERRUPTMASK2_REG:
	case AS3722_INTERRUPTMASK3_REG:
	case AS3722_INTERRUPTMASK4_REG:
	case AS3722_INTERRUPTSTATUS1_REG:
	case AS3722_INTERRUPTSTATUS2_REG:
	case AS3722_INTERRUPTSTATUS3_REG:
	case AS3722_INTERRUPTSTATUS4_REG:
	case 0x7d:
	case AS3722_ADC0_CONTROL_REG:
	case AS3722_ADC1_CONTROL_REG:
	case AS3722_ADC1_THRESHOLD_HI_MSB_REG:
	case AS3722_ADC1_THRESHOLD_HI_LSB_REG:
	case AS3722_ADC1_THRESHOLD_LO_MSB_REG:
	case AS3722_ADC1_THRESHOLD_LO_LSB_REG:
	case AS3722_ADC_CONFIG_REG:
		return true;
	default:
		return false;
	}
}

static bool as3722_volatile(struct device *dev, unsigned int reg)
{
	return false;
}

const struct regmap_config as3722_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,

	.max_register = AS3722_REGISTER_COUNT,
	.readable_reg = as3722_readable,
	.writeable_reg = as3722_writeable,
	.volatile_reg = as3722_volatile,

	.reg_defaults = as3722_defaults,
	.num_reg_defaults = ARRAY_SIZE(as3722_defaults),
};
