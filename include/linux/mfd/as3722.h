/*
 * as3722.h definitions
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __LINUX_MFD_AS3722_H__
#define __LINUX_MFD_AS3722_H__

#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>

#define AS3722_RTC_REP_WAKEUP_EN			BIT(0)
#define AS3722_RTC_ALARM_WAKEUP_EN			BIT(1)
#define AS3722_RTC_ON					BIT(2)
#define AS3722_RTC_IRQMODE				BIT(3)
#define AS3722_RTC_CLK32K_OUT_EN			BIT(5)


#define AS3722_GPIOn_SIGNAL(n)                          BIT(n)
#define AS3722_GPIOn_CONTROL_REG(n)             (AS3722_GPIO0_CONTROL_REG + n)

#define AS3722_GPIO_MODE_INPUT_PULL_UP                  0x04
#define AS3722_GPIO_MODE_INPUT_PULL_DOWN                0x05
#define AS3722_GPIO_MODE_IO_OPEN_DRAIN_PULL_UP          0x06

#define AS3722_GPIO_IOSF_VAL(n)                         (((n) & 0xF) << 3)




#define AS3722_SDn_CTRL(n)                              BIT(n)

#define AS3722_ENABLE_CTRL1_REG                         0x3C
#define AS3722_ENABLE_CTRL2_REG                         0x3D
#define AS3722_ENABLE_CTRL3_REG                         0x3E
#define AS3722_ENABLE_CTRL4_REG                         0x3F
#define AS3722_ENABLE_CTRL5_REG                         0x40

#define AS3722_OVCURRENT_REG                            0x5A
#define AS3722_OVCURRENT_DEB_REG                        0x5B

#define AS3722_SD0_EXT_ENABLE_MASK                      0x03
#define AS3722_SD1_EXT_ENABLE_MASK                      0x0C
#define AS3722_SD2_EXT_ENABLE_MASK                      0x30
#define AS3722_SD3_EXT_ENABLE_MASK                      0xC0
#define AS3722_SD4_EXT_ENABLE_MASK                      0x03
#define AS3722_SD5_EXT_ENABLE_MASK                      0x0C
#define AS3722_SD6_EXT_ENABLE_MASK                      0x30
#define AS3722_LDO0_EXT_ENABLE_MASK                     0x03
#define AS3722_LDO1_EXT_ENABLE_MASK                     0x0C
#define AS3722_LDO2_EXT_ENABLE_MASK                     0x30
#define AS3722_LDO3_EXT_ENABLE_MASK                     0xC0
#define AS3722_LDO4_EXT_ENABLE_MASK                     0x03
#define AS3722_LDO5_EXT_ENABLE_MASK                     0x0C
#define AS3722_LDO6_EXT_ENABLE_MASK                     0x30
#define AS3722_LDO7_EXT_ENABLE_MASK                     0xC0
#define AS3722_LDO9_EXT_ENABLE_MASK                     0x0C
#define AS3722_LDO10_EXT_ENABLE_MASK                    0x30
#define AS3722_LDO11_EXT_ENABLE_MASK                    0xC0


/* AS3722 register bits and bit masks */
#define AS3722_LDO0_VSEL_MASK                           0x1F
#define AS3722_LDO0_VSEL_MIN                            0x01
#define AS3722_LDO0_VSEL_MAX                            0x12
#define AS3722_LDO0_NUM_VOLT                            0x12
#define AS3722_LDO3_VSEL_MASK                           0x3F
#define AS3722_LDO3_VSEL_MIN                            0x01
#define AS3722_LDO3_NUM_VOLT                            0x2D
#define AS3722_LDO_VSEL_MASK                            0x7F
#define AS3722_LDO_VSEL_MIN                             0x01
#define AS3722_LDO_VSEL_MAX                             0x7F
#define AS3722_LDO_VSEL_DNU_MIN                         0x25
#define AS3722_LDO_VSEL_DNU_MAX                         0x3F

#define AS3722_LDO0_CTRL                                BIT(0)
#define AS3722_LDO1_CTRL                                BIT(1)
#define AS3722_LDO2_CTRL                                BIT(2)
#define AS3722_LDO3_CTRL                                BIT(3)
#define AS3722_LDO4_CTRL                                BIT(4)
#define AS3722_LDO5_CTRL                                BIT(5)
#define AS3722_LDO6_CTRL                                BIT(6)
#define AS3722_LDO7_CTRL                                BIT(7)
#define AS3722_LDO9_CTRL                                BIT(1)
#define AS3722_LDO10_CTRL                               BIT(2)
#define AS3722_LDO11_CTRL                               BIT(3)







static inline int as3722_read(struct as3722 *as3722, u32 reg, u32 *dest)
{
	return regmap_read(as3722->regmap, reg, dest);
}

static inline int as3722_write(struct as3722 *as3722, u32 reg, u32 value)
{
	return regmap_write(as3722->regmap, reg, value);
}

static inline int as3722_update_bits(struct as3722 *as3722, u32 reg,
		u32 mask, u8 val)
{
	return regmap_update_bits(as3722->regmap, reg, mask, val);
}

static inline int as3722_irq_get_virq(struct as3722 *as3722, int irq)
{
	return regmap_irq_get_virq(as3722->irq_data, irq);
}

#endif
