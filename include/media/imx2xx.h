/**
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#ifndef __IMX2XX_H__
#define __IMX2XX_H__

/* Group hold */
#define IMX2XX_GROUP_HOLD_ADDR			0x0104

/* Gain */
#define IMX2XX_GAIN_ADDR_MSB			0x0204
#define IMX2XX_GAIN_ADDR_LSB			0x0205
#define IMX2XX_GAIN_SHORT_ADDR_MSB		0x0216
#define IMX2XX_GAIN_SHORT_ADDR_LSB		0x0217

/* Coarse time */
#define IMX2XX_COARSE_TIME_ADDR_MSB		0x0202
#define IMX2XX_COARSE_TIME_ADDR_LSB		0x0203
#define IMX2XX_COARSE_TIME_SHORT_ADDR_MSB	0x0224
#define IMX2XX_COARSE_TIME_SHORT_ADDR_LSB	0x0225

/* Frame length */
#define IMX2XX_FRAME_LENGTH_ADDR_MSB		0x0340
#define IMX2XX_FRAME_LENGTH_ADDR_LSB		0x0341

/* OTP */
#define IMX2XX_OTP_CTRL_ADDR			0x0A00
#define IMX2XX_OTP_STATUS_ADDR			0x0A01
#define IMX2XX_OTP_PAGE_NUM_ADDR		0x0A02
#define IMX2XX_OTP_PAGE_START_ADDR		0x0A04
#define IMX2XX_OTP_PAGE_END_ADDR		0x0A43
#define IMX2XX_OTP_PAGE_SIZE \
	 (IMX2XX_OTP_PAGE_END_ADDR - IMX2XX_OTP_PAGE_START_ADDR + 1)
#define IMX2XX_OTP_STATUS_IN_PROGRESS		0
#define IMX2XX_OTP_STATUS_READ_COMPLETE		1
#define IMX2XX_OTP_STATUS_READ_FAIL		5

/* Min, Max values */
#define IMX2XX_GAIN_SHIFT			8
#define IMX2XX_MIN_GAIN				(1 << IMX2XX_GAIN_SHIFT)
#define IMX2XX_MAX_GAIN				(16 << IMX2XX_GAIN_SHIFT)
#define IMX2XX_MIN_FRAME_LENGTH			(0x0)
#define IMX2XX_MAX_FRAME_LENGTH			(0xffff)
#define IMX2XX_MAX_COARSE_DIFF			10
#define IMX2XX_MIN_EXPOSURE_COARSE		(0x0001)
#define IMX2XX_MAX_EXPOSURE_COARSE		(IMX2XX_MAX_FRAME_LENGTH-\
						 IMX2XX_MAX_COARSE_DIFF)

/* 16-bit reg offset, 8-bit register */
#define imx2xx_reg				struct reg_8

/* Special values in mode tables */
#define IMX2XX_TABLE_WAIT_MS			0
#define IMX2XX_TABLE_END			1

/* Crop Registers */

#define IMX2XX_NUM_CROP_REGS	8

#define IMX2XX_MASK_CORRUPT_FRAME_ADDR		0x0105
#define IMX2XX_MASK_CORRUPT_FRAME_DISABLE	0

#define IMX2XX_CROP_X_START_ADDR_MSB	0x0344
#define IMX2XX_CROP_X_START_ADDR_LSB	0x0345
#define IMX2XX_CROP_Y_START_ADDR_MSB	0x0346
#define IMX2XX_CROP_Y_START_ADDR_LSB	0x0347
#define IMX2XX_CROP_X_END_ADDR_MSB	0x0348
#define IMX2XX_CROP_X_END_ADDR_LSB	0x0349
#define IMX2XX_CROP_Y_END_ADDR_MSB	0x034A
#define IMX2XX_CROP_Y_END_ADDR_LSB	0x034B

#define IMX2XX_CROP_X_OUTPUT_SIZE_MSB	0x034C
#define IMX2XX_CROP_X_OUTPUT_SIZE_LSB	0x034D
#define IMX2XX_CROP_Y_OUTPUT_SIZE_MSB	0x034E
#define IMX2XX_CROP_Y_OUTPUT_SIZE_LSB	0x034F

/* Mode table starting entries */
enum {
	IMX2XX_MODE_COMMON = 0,
	IMX2XX_MODE_START_STREAM,
	IMX2XX_MODE_STOP_STREAM,
	IMX2XX_MODE_TEST_PATTERN,
	IMX2XX_SENSOR_MODE_BEGIN,
};

#endif  /* __IMX2XX_H__ */
