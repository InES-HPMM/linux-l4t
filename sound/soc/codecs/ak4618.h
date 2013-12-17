/*
 * ak4618.h - AK4618 Audio Codec driver supporting AK4618
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __AK4618_H__
#define __AK4618_H__

#define AK4618_POWER_MANAGEMENT_1	0x00
#define AK4618_POWER_MANAGEMENT_2	0x01
#define AK4618_SYSTEM_CLOCK		0x02
#define AK4618_FILTER_SETTING_1		0x03
#define AK4618_FILTER_SETTING_2		0x04
#define AK4618_AUDIO_INTERFACE_FORMAT	0x05
#define AK4618_SOFT_MUTE		0x06
#define AK4618_DAC1L_VOLUME		0x07
#define AK4618_DAC1R_VOLUME		0x08
#define AK4618_DAC2L_VOLUME		0x09
#define AK4618_DAC2R_VOLUME		0x0A
#define AK4618_DAC3L_VOLUME		0x0B
#define AK4618_DAC3R_VOLUME		0x0C
#define AK4618_DAC4L_VOLUME		0x0D
#define AK4618_DAC4R_VOLUME		0x0E
#define AK4618_DAC5L_VOLUME		0x0F
#define AK4618_DAC5R_VOLUME		0x10
#define AK4618_DAC6L_VOLUME		0x11
#define AK4618_DAC6R_VOLUME		0x12
#define AK4618_INPUT_CONTROL		0x13
#define AK4618_MICROPHONE_GAIN_0	0x14
#define AK4618_MICROPHONE_GAIN_1	0x15
#define AK4618_MICROPHONE_GAIN_2	0x16

#define AK4618_MASTER_MODE		(1 << 7)
#define AK4618_MODE_SELECT_MASK		(1 << 7)
#define AK4618_SLAVE_MODE		0

#define AK4618_PM_ADC_ON_MASK		(0xe << 0)
#define AK4618_PM_ADC12_ON		(1 << 1)
#define AK4618_PM_ADC34_ON		(1 << 2)
#define AK4618_PM_ADC56_ON		(1 << 3)

#define AK4618_SOFT_MUTE_MASK		(1 << 0)
#define AK4618_SOFT_MUTE_ENABLE		0
#define AK4618_SOFT_UNMUTE_ENABLE	(1 << 0)

#define AK4618_PM_DAC_ON_MASK		0x3f
#define AK4618_PM_DAC1_ON		(1 << 0)
#define AK4618_PM_DAC2_ON		(1 << 1)
#define AK4618_PM_DAC3_ON		(1 << 2)
#define AK4618_PM_DAC4_ON		(1 << 3)
#define AK4618_PM_DAC5_ON		(1 << 4)
#define AK4618_PM_DAC6_ON		(1 << 5)

#define AK4618_FMT_FORMAT_MASK		0x04
#define AK4618_FMT_I2S			0x04
#define AK4618_FMT_TDM_FORMAT_MASK	(0x03 << 4)
#define AK4618_FMT_TDM_128		(0x03 << 4)
#define AK4618_FMT_TDM_256		(0x02 << 4)
#define AK4618_FMT_TDM_512		(0x01 << 4)

#define AK4618_SYS_CLK_AUTO_MASK	(1 << 0)
#define AK4618_SYS_CLK_AUTO		(1 << 0)
#define AK4618_SYS_CLK_MANUAL		0
#define AK4618_SYS_CLK_MASK		(0x0f << 4)
#define AK4618_SYS_CLK_MCLK_SEL_256	(0 << 6)
#define AK4618_SYS_CLK_MCLK_SEL_384	(1 << 6)
#define AK4618_SYS_CLK_MCLK_SEL_512	(2 << 6)
/* NORMAL : 8kHz ~ 48kHz */
#define AK4618_SYS_CLK_SAMPLING_RATE_NORMAL	(0 << 4)
/* DOUBLE : 64kHz ~ 96kHz */
#define AK4618_SYS_CLK_SAMPLING_RATE_DOUBLE	(1 << 4)
/* QUAD : 128kHz ~ 192kHz */
#define AK4618_SYS_CLK_SAMPLING_RATE_QUAD	(2 << 4)

#define AK4618_NUM_REGS			0x17

#endif
