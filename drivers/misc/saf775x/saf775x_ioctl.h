 /*
 * saf775x_ioctl.h  --  SAF775X Soc Audio driver IO control
 *
 * Copyright (c) 2014-2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHIN
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SAF775X_IOCTL_H__
#define __SAF775X_IOCTL_H__

#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#define BYTEPOS_IN_WORD(i)  (BITS_PER_BYTE * i)
#define CHAR_BIT_MASK(i)    (0xFF << BYTEPOS_IN_WORD(i))


struct saf775x_cmd {
	unsigned int reg;
	unsigned int reg_len;
	unsigned int val;
	unsigned int val_len;
};

enum {
	SAF775X_CONTROL_SET_IOCTL = _IOW(0xF4, 0x01, struct saf775x_cmd),
	SAF775x_CODEC_RESET_IOCTL = _IO(0xF4, 0x02),
	SAF775X_CONTROL_GET_IOCTL = _IOR(0xF4, 0x03, struct saf775x_cmd),
};

struct saf775x_ioctl_ops {
	int (*codec_write)(struct i2c_client *codec,
		unsigned int reg, unsigned int val,
		unsigned int reg_len, unsigned int val_len);
	int (*codec_reset)(void);
	int (*codec_read)(struct i2c_client *codec,
		unsigned char *val, unsigned int val_len);
};

int saf775x_hwdep_create(struct i2c_client *codec);
int saf775x_hwdep_cleanup(void);
struct saf775x_ioctl_ops *saf775x_get_ioctl_ops(void);

#endif /* __SAF775X_IOCTL_H__ */
