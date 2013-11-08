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

#ifndef __LINUX_MFD_AS3722_REG_H
#define __LINUX_MFD_AS3722_REG_H

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define AS3722_DEVICE_ID       0x0C
#define AS3722_MAX_REGISTER                  0xf3
#define AS3722_NUM_REGULATORS                  18
#define AS3722_NUM_STEPDOWN_REGULATORS         7

/* defines for register init */
#define AS3722_REG_INIT(reg_offset, reg_value)  \
{                                              \
	.reg     = (reg_offset),                \
	.val     = (reg_value),                 \
}

#define AS3722_REG_INIT_TERMINATE              0xFF

/* regulator IDs */
enum as3722_regulators{
AS3722_SD0,
AS3722_SD1,
AS3722_SD2,
AS3722_SD3,
AS3722_SD4,
AS3722_SD5,
AS3722_SD6,
AS3722_LDO0,
AS3722_LDO1,
AS3722_LDO2,
AS3722_LDO3,
AS3722_LDO4,
AS3722_LDO5,
AS3722_LDO6,
AS3722_LDO7,
AS3722_LDO9,
AS3722_LDO10,
AS3722_LDO11,
};

/* AS3722 ASIC ID */
#define AS3722_ADDR_ASIC_ID1                   0x90
#define AS3722_ADDR_ASIC_ID2                   0x91

/* GPIO IDs */
#define AS3722_GPIO0                           0
#define AS3722_GPIO1                           1
#define AS3722_GPIO2                           2
#define AS3722_GPIO3                           3
#define AS3722_GPIO4                           4
#define AS3722_GPIO5                           5
#define AS3722_GPIO6                           6
#define AS3722_GPIO7                           7

#define AS3722_NUM_GPIO                        8
#define AS3722_GPIO_IRQ_BASE                   0

/* GPIO modes */
#define AS3722_GPIO_MODE_MASK                  0x07
#define AS3722_GPIO_MODE_INPUT                 0
#define AS3722_GPIO_MODE_OUTPUT_VDDH           1
#define AS3722_GPIO_MODE_IO_OPEN_DRAIN         2
#define AS3722_GPIO_MODE_ADC_IN                3
#define AS3722_GPIO_MODE_INPUT_W_PULLUP        4
#define AS3722_GPIO_MODE_INPUT_W_PULLDOWN      5
#define AS3722_GPIO_MODE_IO_OPEN_DRAIN_PULLUP  6
#define AS3722_GPIO_MODE_OUTPUT_VDDL           7

/* GPIO usage */
#define AS3722_GPIO_USAGE_MASK                          0x78
#define AS3722_GPIO_USAGE_IO_OPERATION                  0
#define AS3722_GPIO_USAGE_INTERRUPT_OUTPUT              1
#define AS3722_GPIO_USAGE_VSUB_VBAT_LOW_UNDERBOUNCED    2
#define AS3722_GPIO_USAGE_GPIO_INTERRUPT_INPUT          3
#define AS3722_GPIO_USAGE_PWM_INPUT                     4
#define AS3722_GPIO_USAGE_VOLTAGE_STBY                  5
#define AS3722_GPIO_USAGE_OC_PG_SD0                     6
#define AS3722_GPIO_USAGE_PWR_GOOD                      7
#define AS3722_GPIO_USAGE_32K_OUTPUT                    8
#define AS3722_GPIO_USAGE_WATCHDOG_INPUT                9
#define AS3722_GPIO_USAGE_SOFT_RESET_INPUT              0xB
#define AS3722_GPIO_USAGE_PWM_OUTPUT                    0xC
#define AS3722_GPIO_USAGE_VSUP_VBAT_LOW_DEBOUNCED       0xD
#define AS3722_GPIO_USAGE_OC_PG_SD6                     0xE

/* Interrupt IDs */
#define AS3722_IRQ_MAX_HANDLER                 18
#define AS3722_IRQ_LID                         0
#define AS3722_IRQ_ACOK                        1
#define AS3722_IRQ_CORE_PWRREQ                 2
#define AS3722_IRQ_OCURR_ACOK                  3
#define AS3722_IRQ_ONKEY_LONG                  4
#define AS3722_IRQ_ONKEY                       5
#define AS3722_IRQ_OVTMP                       6
#define AS3722_IRQ_LOWBAT                      7
#define AS3722_IRQ_RTC_REP                     8
#define AS3722_IRQ_RTC_ALARM                   9
#define AS3722_IRQ_SD0                         10
#define AS3722_IRQ_RTC_GPIO1                   11
#define AS3722_IRQ_RTC_GPIO2                   12
#define AS3722_IRQ_RTC_GPIO3                   13
#define AS3722_IRQ_RTC_GPIO4                   14
#define AS3722_IRQ_RTC_GPIO5                   15
#define AS3722_IRQ_WATCHDOG                    16
#define AS3722_IRQ_ADC                         17

/* AS3722 registers */
#define AS3722_SD0_VOLTAGE_REG                 0x00
#define AS3722_SD1_VOLTAGE_REG                 0x01
#define AS3722_SD2_VOLTAGE_REG                 0x02
#define AS3722_SD3_VOLTAGE_REG                 0x03
#define AS3722_SD4_VOLTAGE_REG                 0x04
#define AS3722_SD5_VOLTAGE_REG                 0x05
#define AS3722_SD6_VOLTAGE_REG                 0x06
#define AS3722_GPIO0_CONTROL_REG               0x08
#define AS3722_GPIO1_CONTROL_REG               0x09
#define AS3722_GPIO2_CONTROL_REG               0x0A
#define AS3722_GPIO3_CONTROL_REG               0x0B
#define AS3722_GPIO4_CONTROL_REG               0x0C
#define AS3722_GPIO5_CONTROL_REG               0x0D
#define AS3722_GPIO6_CONTROL_REG               0x0E
#define AS3722_GPIO7_CONTROL_REG               0x0F
#define AS3722_LDO0_VOLTAGE_REG                0x10
#define AS3722_LDO1_VOLTAGE_REG                0x11
#define AS3722_LDO2_VOLTAGE_REG                0x12
#define AS3722_LDO3_VOLTAGE_REG                0x13
#define AS3722_LDO4_VOLTAGE_REG                0x14
#define AS3722_LDO5_VOLTAGE_REG                0x15
#define AS3722_LDO6_VOLTAGE_REG                0x16
#define AS3722_LDO7_VOLTAGE_REG                0x17
#define AS3722_LDO9_VOLTAGE_REG                0x19
#define AS3722_LDO10_VOLTAGE_REG               0x1A
#define AS3722_LDO11_VOLTAGE_REG               0x1B

#define AS3722_GPIO_SIGNAL_OUT_REG             0x20
#define AS3722_GPIO_SIGNAL_IN_REG              0x21

#define AS3722_SD0_CONTROL_REG                 0x29
#define AS3722_SD1_CONTROL_REG                 0x2A
#define AS3722_SDmph_CONTROL_REG               0x2B
#define AS3722_SD23_CONTROL_REG                0x2C
#define AS3722_SD4_CONTROL_REG                 0x2D
#define AS3722_SD5_CONTROL_REG                 0x2E
#define AS3722_SD6_CONTROL_REG                 0x2F

#define AS3722_RESET_CONTROL_REG               0x36
#define AS3722_WATCHDOG_CONTROL_REG            0x38
#define AS3722_WATCHDOG_TIMER_REG              0x46
#define AS3722_WATCHDOG_SOFTWARE_SIGNAL_REG    0x48
#define AS3722_IOVOLTAGE_REG                   0x49
#define AS3722_SD_CONTROL_REG                  0x4D
#define AS3722_LDOCONTROL0_REG                 0x4E
#define AS3722_LDOCONTROL1_REG                 0x4F

#define AS3722_CTRL1_REG                       0x58
#define AS3722_CTRL2_REG                       0x59
#define AS3722_OVCURRENT                       0x5A
#define AS3722_OVCURRENT_DEB                   0x5B
#define AS3722_RTC_CONTROL_REG                 0x60
#define AS3722_RTC_SECOND_REG                  0x61
#define AS3722_RTC_MINUTE_REG                  0x62
#define AS3722_RTC_HOUR_REG                    0x63
#define AS3722_RTC_DAY_REG                     0x64
#define AS3722_RTC_MONTH_REG                   0x65
#define AS3722_RTC_YEAR_REG                    0x66
#define AS3722_RTC_ALARM_SECOND_REG            0x67
#define AS3722_RTC_ALARM_MINUTE_REG            0x68
#define AS3722_RTC_ALARM_HOUR_REG              0x69
#define AS3722_RTC_ALARM_DAY_REG               0x6A
#define AS3722_RTC_ALARM_MONTH_REG             0x6B
#define AS3722_RTC_ALARM_YEAR_REG              0x6C

#define AS3722_INTERRUPTMASK1_REG              0x74
#define AS3722_INTERRUPTMASK2_REG              0x75
#define AS3722_INTERRUPTMASK3_REG              0x76
#define AS3722_INTERRUPTMASK4_REG              0x77
#define AS3722_INTERRUPTSTATUS1_REG            0x78
#define AS3722_INTERRUPTSTATUS2_REG            0x79
#define AS3722_INTERRUPTSTATUS3_REG            0x7A
#define AS3722_INTERRUPTSTATUS4_REG            0x7B

#define AS3722_EXTERNAL_ENABLE_CTRL1           0x3C
#define AS3722_EXTERNAL_ENABLE_CTRL2           0x3D
#define AS3722_EXTERNAL_ENABLE_CTRL3           0x3E
#define AS3722_EXTERNAL_ENABLE_CTRL4           0x3F
#define AS3722_EXTERNAL_ENABLE_CTRL5           0x40

#define AS3722_LDO0_EXTERNAL_ENABLE_MASK       0x3
#define AS3722_LDO1_EXTERNAL_ENABLE_MASK       0xC
#define AS3722_LDO2_EXTERNAL_ENABLE_MASK       0x30
#define AS3722_LDO3_EXTERNAL_ENABLE_MASK       0xC0
#define AS3722_LDO4_EXTERNAL_ENABLE_MASK       0x3
#define AS3722_LDO5_EXTERNAL_ENABLE_MASK       0xC
#define AS3722_LDO6_EXTERNAL_ENABLE_MASK       0x30
#define AS3722_LDO7_EXTERNAL_ENABLE_MASK       0xC0
#define AS3722_LDO9_EXTERNAL_ENABLE_MASK       0xC
#define AS3722_LDO10_EXTERNAL_ENABLE_MASK      0x30
#define AS3722_LDO11_EXTERNAL_ENABLE_MASK      0xC0
#define AS3722_SD0_EXTERNAL_ENABLE_MASK        0x3
#define AS3722_SD1_EXTERNAL_ENABLE_MASK        0xC
#define AS3722_SD2_EXTERNAL_ENABLE_MASK        0x30
#define AS3722_SD3_EXTERNAL_ENABLE_MASK        0xC0
#define AS3722_SD4_EXTERNAL_ENABLE_MASK        0x3
#define AS3722_SD5_EXTERNAL_ENABLE_MASK        0xC
#define AS3722_SD6_EXTERNAL_ENABLE_MASK        0x30

#define AS3722_OVCURRENT_SD0_ALARM_MASK        0x07
#define AS3722_OVCURRENT_SD0_ALARM_SHIFT       0x01
#define AS3722_OVCURRENT_SD0_TRIP_MASK         0x18
#define AS3722_OVCURRENT_SD0_TRIP_SHIFT        0x03
#define AS3722_OVCURRENT_SD1_TRIP_MASK         0x60
#define AS3722_OVCURRENT_SD1_TRIP_SHIFT        0x05

#define AS3722_OVCURRENT_SD6_ALARM_MASK        0x07
#define AS3722_OVCURRENT_SD6_ALARM_SHIFT       0x01
#define AS3722_OVCURRENT_SD6_TRIP_MASK         0x18
#define AS3722_OVCURRENT_SD6_TRIP_SHIFT        0x03

#define AS3722_ADC0_CONTROL_REG                0x80
#define AS3722_ADC1_CONTROL_REG                0x81
#define AS3722_ADC0_MSB_RESULT_REG             0x82
#define AS3722_ADC0_LSB_RESULT_REG             0x83
#define AS3722_ADC1_MSB_RESULT_REG             0x84
#define AS3722_ADC1_LSB_RESULT_REG             0x85
#define AS3722_ADC1_THRESHOLD_HI_MSB_REG       0x86
#define AS3722_ADC1_THRESHOLD_HI_LSB_REG       0x87
#define AS3722_ADC1_THRESHOLD_LO_MSB_REG       0x88
#define AS3722_ADC1_THRESHOLD_LO_LSB_REG       0x89
#define AS3722_ADC_CONFIG_REG                  0x8A

/* AS3722 register bits and bit masks */
#define AS3722_LDO_ILIMIT_MASK                 (1 << 7)
#define AS3722_LDO_ILIMIT_BIT                  (1 << 7)
#define AS3722_LDO0_VSEL_MASK                  0x1F
#define AS3722_LDO0_VSEL_MIN                   0x01
#define AS3722_LDO0_VSEL_MAX                   0x12
#define AS3722_LDO3_VSEL_MASK                  0x3F
#define AS3722_LDO3_VSEL_MIN                   0x01
#define AS3722_LDO3_VSEL_MAX                   0x45
#define AS3722_LDO_VSEL_MASK                   0x7F
#define AS3722_LDO_VSEL_MIN                    0x01
#define AS3722_LDO_VSEL_MAX                    0x7F
#define AS3722_LDO_VSEL_DNU_MIN                0x25
#define AS3722_LDO_VSEL_DNU_MAX                0x3F
#define AS3722_LDO_NUM_VOLT                    100

#define AS3722_LDO0_ON                         (1 << 0)
#define AS3722_LDO0_OFF                        (0 << 0)
#define AS3722_LDO0_CTRL_MASK                  (1 << 0)
#define AS3722_LDO1_ON                         (1 << 1)
#define AS3722_LDO1_OFF                        (0 << 1)
#define AS3722_LDO1_CTRL_MASK                  (1 << 1)
#define AS3722_LDO2_ON                         (1 << 2)
#define AS3722_LDO2_OFF                        (0 << 2)
#define AS3722_LDO2_CTRL_MASK                  (1 << 2)
#define AS3722_LDO3_ON                         (1 << 3)
#define AS3722_LDO3_OFF                        (0 << 3)
#define AS3722_LDO3_CTRL_MASK                  (1 << 3)
#define AS3722_LDO3_MODE_MASK                  (3 << 6)
#define AS3722_LDO3_MODE_PMOS                  (0 << 6)
#define AS3722_LDO3_MODE_PMOS_TRACKING         (1 << 6)
#define AS3722_LDO3_MODE_NMOS                  (2 << 6)
#define AS3722_LDO3_MODE_SWITCH                (3 << 6)
#define AS3722_LDO4_ON                         (1 << 4)
#define AS3722_LDO4_OFF                        (0 << 4)
#define AS3722_LDO4_CTRL_MASK                  (1 << 4)
#define AS3722_LDO5_ON                         (1 << 5)
#define AS3722_LDO5_OFF                        (0 << 5)
#define AS3722_LDO5_CTRL_MASK                  (1 << 5)
#define AS3722_LDO6_ON                         (1 << 6)
#define AS3722_LDO6_OFF                        (0 << 6)
#define AS3722_LDO6_CTRL_MASK                  (1 << 6)
#define AS3722_LDO7_ON                         (1 << 7)
#define AS3722_LDO7_OFF                        (0 << 7)
#define AS3722_LDO7_CTRL_MASK                  (1 << 7)
#define AS3722_LDO9_ON                         (1 << 1)
#define AS3722_LDO9_OFF                        (0 << 1)
#define AS3722_LDO9_CTRL_MASK                  (1 << 1)
#define AS3722_LDO10_ON                        (1 << 2)
#define AS3722_LDO10_OFF                       (0 << 2)
#define AS3722_LDO10_CTRL_MASK                 (1 << 2)
#define AS3722_LDO11_ON                        (1 << 3)
#define AS3722_LDO11_OFF                       (0 << 3)
#define AS3722_LDO11_CTRL_MASK                 (1 << 3)

#define AS3722_SD_VSEL_MASK                    0x7F
#define AS3722_SD0_VSEL_MIN                    0x01
#define AS3722_SD0_VSEL_MAX                    0x5A
#define AS3722_SD2_VSEL_MIN                    0x01
#define AS3722_SD2_VSEL_MAX                    0x7F
#define AS3722_SD0_ON                          (1 << 0)
#define AS3722_SD0_OFF                         (0 << 0)
#define AS3722_SD0_CTRL_MASK                   (1 << 0)
#define AS3722_SD1_ON                          (1 << 1)
#define AS3722_SD1_OFF                         (0 << 1)
#define AS3722_SD1_CTRL_MASK                   (1 << 1)
#define AS3722_SD2_ON                          (1 << 2)
#define AS3722_SD2_OFF                         (0 << 2)
#define AS3722_SD2_CTRL_MASK                   (1 << 2)
#define AS3722_SD3_ON                          (1 << 3)
#define AS3722_SD3_OFF                         (0 << 3)
#define AS3722_SD3_CTRL_MASK                   (1 << 3)
#define AS3722_SD4_ON                          (1 << 4)
#define AS3722_SD4_OFF                         (0 << 4)
#define AS3722_SD4_CTRL_MASK                   (1 << 4)
#define AS3722_SD5_ON                          (1 << 5)
#define AS3722_SD5_OFF                         (0 << 5)
#define AS3722_SD5_CTRL_MASK                   (1 << 5)
#define AS3722_SD6_ON                          (1 << 6)
#define AS3722_SD6_OFF                         (0 << 6)
#define AS3722_SD6_CTRL_MASK                   (1 << 6)

#define AS3722_SD0_MODE_FAST                   (1 << 4)
#define AS3722_SD0_MODE_NORMAL                 (0 << 4)
#define AS3722_SD0_MODE_MASK                   (1 << 4)
#define AS3722_SD1_MODE_FAST                   (1 << 4)
#define AS3722_SD1_MODE_NORMAL                 (0 << 4)
#define AS3722_SD1_MODE_MASK                   (1 << 4)
#define AS3722_SD2_MODE_FAST                   (1 << 2)
#define AS3722_SD2_MODE_NORMAL                 (0 << 2)
#define AS3722_SD2_MODE_MASK                   (1 << 2)
#define AS3722_SD3_MODE_FAST                   (1 << 6)
#define AS3722_SD3_MODE_NORMAL                 (0 << 6)
#define AS3722_SD3_MODE_MASK                   (1 << 6)
#define AS3722_SD4_MODE_FAST                   (1 << 2)
#define AS3722_SD4_MODE_NORMAL                 (0 << 2)
#define AS3722_SD4_MODE_MASK                   (1 << 2)
#define AS3722_SD5_MODE_FAST                   (1 << 2)
#define AS3722_SD5_MODE_NORMAL                 (0 << 2)
#define AS3722_SD5_MODE_MASK                   (1 << 2)
#define AS3722_SD6_MODE_FAST                   (1 << 4)
#define AS3722_SD6_MODE_NORMAL                 (0 << 4)
#define AS3722_SD6_MODE_MASK                   (1 << 4)

#define AS3722_POWER_OFF_MASK                  (1 << 1)
#define AS3722_IRQ_MASK_LID                    (1 << 0)
#define AS3722_IRQ_MASK_ACOK                   (1 << 1)
#define AS3722_IRQ_MASK_CORE_PWRREQ            (1 << 2)
#define AS3722_IRQ_MASK_OCURR_ACOK             (1 << 3)
#define AS3722_IRQ_MASK_ONKEY_LONG             (1 << 4)
#define AS3722_IRQ_MASK_ONKEY                  (1 << 5)
#define AS3722_IRQ_MASK_OVTMP                  (1 << 6)
#define AS3722_IRQ_MASK_LOWBAT                 (1 << 7)

#define AS3722_IRQ_MASK_SD0                    (1 << 0)

#define AS3722_IRQ_MASK_RTC_REP                (1 << 7)
#define AS3722_IRQ_MASK_RTC_ALARM              (1 << 0)
#define AS3722_IRQ_MASK_GPIO_EDGE1             (1 << 1)
#define AS3722_IRQ_MASK_GPIO_EDGE2             (1 << 2)
#define AS3722_IRQ_MASK_GPIO_EDGE3             (1 << 3)
#define AS3722_IRQ_MASK_GPIO_EDGE4             (1 << 4)
#define AS3722_IRQ_MASK_GPIO_EDGE5             (1 << 5)
#define AS3722_IRQ_MASK_WATCHDOG               (1 << 6)
#define AS3722_IRQ_MASK_ADC                    (1 << 7)

#define AS3722_IRQ_BIT_LID                     (1 << 0)
#define AS3722_IRQ_BIT_ACOK                    (1 << 1)
#define AS3722_IRQ_BIT_CORE_PWRREQ             (1 << 2)
#define AS3722_IRQ_BIT_SD0                     (1 << 3)
#define AS3722_IRQ_BIT_ONKEY_LONG              (1 << 4)
#define AS3722_IRQ_BIT_ONKEY                   (1 << 5)
#define AS3722_IRQ_BIT_OVTMP                   (1 << 6)
#define AS3722_IRQ_BIT_LOWBAT                  (1 << 7)

#define AS3722_IRQ_BIT_RTC_REP                 (1 << 7)
#define AS3722_IRQ_BIT_RTC_ALARM               (1 << 0)

#define AS3722_IRQ_BIT_WATCHDOG                (1 << 6)

#define AS3722_ADC_MASK_BUF_ON                 (1 << 2)
#define AS3722_ADC_BIT_BUF_ON                  (1 << 2)
#define AS3722_ADC1_MASK_INT_MODE_ON           (1 << 1)
#define AS3722_ADC1_BIT_INT_MODE_ON            (1 << 1)
#define AS3722_ADC1_MASK_INTERVAL_TIME         (1 << 0)
#define AS3722_ADC1_BIT_INTERVAL_TIME          (1 << 0)

#define AS3722_ADC_MASK_MSB_VAL                0x7F
#define AS3722_ADC1_INT_MASK                   (1 << 7)
#define AS3722_ADC1_UNMASK_VALUE               (0 << 7)

#define AS3722_ADC_MASK_LSB_VAL                0x07

#define AS3722_ADC0_MASK_CONV_START            (1 << 7)
#define AS3722_ADC0_BIT_CONV_START             (1 << 7)
#define AS3722_ADC0_MASK_CONV_NOTREADY         (1 << 7)
#define AS3722_ADC0_BIT_CONV_NOTREADY          (1 << 7)
#define AS3722_ADC0_MASK_SOURCE_SELECT         0x1F

#define AS3722_ADC1_MASK_CONV_START            (1 << 7)
#define AS3722_ADC1_BIT_CONV_START             (1 << 7)
#define AS3722_ADC1_MASK_CONV_NOTREADY         (1 << 7)
#define AS3722_ADC1_BIT_CONV_NOTREADY          (1 << 7)
#define AS3722_ADC1_MASK_SOURCE_SELECT         0x1F

#define AS3722_GPIO_INV_MASK                   0x80
#define AS3722_GPIO_INV                        0x80
#define AS3722_GPIO_IOSF_MASK                  0x78
#define AS3722_GPIO_IOSF_NORMAL                0
#define AS3722_GPIO_IOSF_INTERRUPT_OUT         (1 << 3)
#define AS3722_GPIO_IOSF_VSUP_LOW_OUT          (2 << 3)
#define AS3722_GPIO_IOSF_GPIO_INTERRUPT_IN     (3 << 3)
#define AS3722_GPIO_IOSF_ISINK_PWM_IN          (4 << 3)
#define AS3722_GPIO_IOSF_VOLTAGE_STBY          (5 << 3)
#define AS3722_GPIO_IOSF_PWR_GOOD_OUT          (7 << 3)
#define AS3722_GPIO_IOSF_Q32K_OUT              (8 << 3)
#define AS3722_GPIO_IOSF_WATCHDOG_IN           (9 << 3)
#define AS3722_GPIO_IOSF_SOFT_RESET_IN         (11 << 3)
#define AS3722_GPIO_IOSF_PWM_OUT               (12 << 3)
#define AS3722_GPIO_IOSF_VSUP_LOW_DEB_OUT      (13 << 3)
#define AS3722_GPIO_IOSF_SD6_LOW_VOLT_LOW      (14 << 3)

#define AS3722_GPIO0_SIGNAL_MASK               (1 << 0)
#define AS3722_GPIO1_SIGNAL_MASK               (1 << 1)
#define AS3722_GPIO2_SIGNAL_MASK               (1 << 2)
#define AS3722_GPIO3_SIGNAL_MASK               (1 << 3)
#define AS3722_GPIO4_SIGNAL_MASK               (1 << 4)
#define AS3722_GPIO5_SIGNAL_MASK               (1 << 5)
#define AS3722_GPIO6_SIGNAL_MASK               (1 << 6)
#define AS3722_GPIO7_SIGNAL_MASK               (1 << 7)

#define AS3722_ADC1_LOW_VOLTAGE_RANGE_MASK     (1 << 5)
#define AS3722_ADC1_INTEVAL_SCAN_MASK          (1 << 6)
#define AS3722_ADC1_CONVERSION_START_MASK      (1 << 7)

#define AS3722_INT_PULLUP_MASK                 (1<<5)
#define AS3722_INT_PULLUP_ON                   (1<<5)
#define AS3722_INT_PULLUP_OFF                  (0<<5)
#define AS3722_I2C_PULLUP_MASK                 (1<<4)
#define AS3722_I2C_PULLUP_ON                   (1<<4)
#define AS3722_I2C_PULLUP_OFF                  (0<<4)

#define AS3722_RTC_REP_WAKEUP_EN_MASK          (1 << 0)
#define AS3722_RTC_ALARM_WAKEUP_EN_MASK        (1 << 1)
#define AS3722_RTC_ON_MASK                     (1 << 2)
#define AS3722_RTC_IRQMODE_MASK                (3 << 3)
#define AS3722_RTC_32KCLK_MASK                 (1 << 5)
#define AS3722_RTC_32KCLK_EN_VAL               (1 << 5)

#define AS3722_WATCHDOG_TIMER_MAX              127
#define AS3722_WATCHDOG_ON_MASK                (1<<0)
#define AS3722_WATCHDOG_ON                     (1<<0)
#define AS3722_WATCHDOG_SW_SIG_MASK            (1<<0)
#define AS3722_WATCHDOG_SW_SIG                 (1<<0)

#define AS3722_FORCE_RESET			BIT(0)
#define AS3722_POWER_OFF			BIT(1)
#endif
