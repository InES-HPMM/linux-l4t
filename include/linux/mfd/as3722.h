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

static inline int as3722_update_bits(struct as3722 *as3722, u32 reg,
                u32 mask, u8 val)
{
        return regmap_update_bits(as3722->regmap, reg, mask, val);
}

#endif
