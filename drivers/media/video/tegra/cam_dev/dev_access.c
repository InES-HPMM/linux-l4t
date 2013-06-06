/*
 * dev_access.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CAMERA_DEVICE_INTERNAL

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>

#include <media/nvc.h>
#include <media/camera.h>

/*#define DEBUG_I2C_TRAFFIC*/
#ifdef DEBUG_I2C_TRAFFIC
static unsigned char dump_buf[32 + 3 * 16];
static DEFINE_MUTEX(dump_mutex);

static void camera_dev_dump(
	struct camera_device *cdev, u32 reg, u8 *buf, u8 num)
{
	int len;
	int i;

	mutex_lock(&dump_mutex);
	len = sprintf(dump_buf, "%s %04x =", __func__, reg);
	for (i = 0; i < num; i++)
		len += sprintf(dump_buf + len, " %02x", buf[i]);
	dump_buf[len] = 0;
	dev_info(cdev->dev, "%s\n", dump_buf);
	mutex_unlock(&dump_mutex);
}
#else
static void camera_dev_dump(
	struct camera_device *cdev, u32 reg, u8 *buf, u8 num)
{
	if (num == 1) {
		dev_dbg(cdev->dev, "%s %04x = %02x\n", __func__, reg, *buf);
		return;
	}
	dev_dbg(cdev->dev, "%s %04x = %02x %02x ...\n",
		__func__, reg, buf[0], buf[1]);
}
#endif

static int camera_dev_rd(struct camera_device *cdev, u32 reg, u32 *val)
{
	int ret = -ENODEV;

	mutex_lock(&cdev->mutex);
	if (cdev->is_power_on)
		ret = regmap_read(cdev->regmap, reg, val);
	else
		dev_err(cdev->dev, "%s: power is off.\n", __func__);
	mutex_unlock(&cdev->mutex);
	camera_dev_dump(cdev, reg, (u8 *)val, sizeof(*val));
	return ret;
}

int camera_dev_rd_table(struct camera_device *cdev, struct camera_reg *table)
{
	struct camera_reg *p_table = table;
	u32 val;
	int err = 0;

	dev_dbg(cdev->dev, "%s", __func__);
	while (p_table->addr != CAMERA_TABLE_END) {
		err = camera_dev_rd(cdev, p_table->addr, &val);
		if (err)
			goto dev_rd_tbl_done;

		p_table->val = (u16)val;
		p_table++;
	}

dev_rd_tbl_done:
	return err;
}

static int camera_dev_wr_blk(
	struct camera_device *cdev, u32 reg, u8 *buf, int len)
{
	int ret = -ENODEV;

	dev_dbg(cdev->dev, "%s %d\n", __func__, len);
	camera_dev_dump(cdev, reg, buf, len);
	mutex_lock(&cdev->mutex);
	if (cdev->is_power_on)
		ret = regmap_raw_write(cdev->regmap, reg, buf, len);
	else
		dev_err(cdev->dev, "%s: power is off.\n", __func__);
	mutex_unlock(&cdev->mutex);
	return ret;
}

int camera_dev_parser(
	struct camera_device *cdev, u32 addr, u32 val)
{
	u8 flag = 0;

	switch (addr) {
	case CAMERA_TABLE_PWR:
	{
		struct nvc_regulator *preg;
		flag = val & CAMERA_TABLE_PWR_FLAG_ON ? 0xff : 0;
		val &= ~CAMERA_TABLE_PWR_FLAG_MASK;
		if (val >= cdev->num_reg) {
			dev_err(cdev->dev,
				"reg index %d out of range.\n", val);
			return -ENODEV;
		}

		preg = &cdev->regs[val];
		if (preg->vreg) {
			int err;
			if (flag) {
				err = regulator_enable(preg->vreg);
				dev_dbg(cdev->dev, "enable %s\n",
					preg->vreg_name);
			} else {
				err = regulator_disable(preg->vreg);
				dev_dbg(cdev->dev, "disable %s\n",
					preg->vreg_name);
			}
			if (err) {
				dev_err(cdev->dev, "%s %s err\n",
					__func__, preg->vreg_name);
				return -EFAULT;
			}
		} else
			dev_dbg(cdev->dev, "%s not available\n",
				preg->vreg_name);
		break;
	}
	case CAMERA_TABLE_GPIO_INX_ACT:
	case CAMERA_TABLE_GPIO_INX_DEACT:
	{
		struct nvc_gpio *gpio;
		if (val >= cdev->num_gpio) {
			dev_err(cdev->dev,
				"gpio index %d out of range.\n", val);
			return -ENODEV;
		}

		gpio = &cdev->gpios[val];
		if (gpio->valid) {
			flag = gpio->active_high ? 0xff : 0;
			if (addr != CAMERA_TABLE_GPIO_INX_ACT)
				flag = !flag;
			gpio_set_value(gpio->gpio, flag & 0x01);
			dev_dbg(cdev->dev, "IDX %d(%d) %d\n", val,
				gpio->gpio, flag & 0x01);
		}
		break;
	}
	case CAMERA_TABLE_GPIO_ACT:
	case CAMERA_TABLE_GPIO_DEACT:
		if (val >= ARCH_NR_GPIOS) {
			dev_err(cdev->dev,
				"gpio index %d out of range.\n", val);
			return -ENODEV;
		}

		flag = 0xff;
		if (addr != CAMERA_TABLE_GPIO_ACT)
			flag = !flag;
		gpio_set_value(val, flag & 0x01);
		dev_dbg(cdev->dev,
			"GPIO %d %d\n", val, flag & 0x01);
		break;
	case CAMERA_TABLE_PINMUX:
		if (!cdev->pinmux_num) {
			dev_dbg(cdev->dev,
				"%s PINMUX TABLE\n", "no");
			break;
		}
		flag = val & CAMERA_TABLE_PINMUX_FLAG_ON ? 0xff : 0;
		if (flag) {
			if (cdev->mclk_enable_idx == CAMDEV_INVALID) {
				dev_dbg(cdev->dev,
					"%s enable PINMUX\n", "no");
				break;
			}
			val = cdev->mclk_enable_idx;
		} else {
			if (cdev->mclk_disable_idx == CAMDEV_INVALID) {
				dev_dbg(cdev->dev,
					"%s disable PINMUX\n", "no");
				break;
			}
			val = cdev->mclk_disable_idx;
		}
		tegra_pinmux_config_table(
			cdev->pinmux_tbl[val], 1);
		dev_dbg(cdev->dev, "PINMUX %d\n", flag & 0x01);
		break;
	case CAMERA_TABLE_INX_PINMUX:
		if (val >= cdev->pinmux_num) {
			dev_err(cdev->dev,
				"pinmux idx %d out of range.\n", val);
				break;
		}
		tegra_pinmux_config_table(cdev->pinmux_tbl[val], 1);
		dev_dbg(cdev->dev, "PINMUX %d done.\n", val);
		break;
	case CAMERA_TABLE_REG_NEW_POWER:
		break;
	case CAMERA_TABLE_INX_POWER:
		break;
	case CAMERA_TABLE_WAIT_MS:
		val *= 1000;
	case CAMERA_TABLE_WAIT_US:
		dev_dbg(cdev->dev, "Sleep %d uS\n", val);
		usleep_range(val, val + 20);
		break;
	default:
		dev_err(cdev->dev, "unrecognized cmd %x.\n", addr);
		return -ENODEV;
		break;
	}

	return 1;
}

int camera_dev_wr_table(struct camera_device *cdev, struct camera_reg *table)
{
	const struct camera_reg *next;
	u8 *b_ptr = cdev->i2c_buf;
	u8 byte_num;
	u16 buf_count = 0;
	u32 addr = 0;
	int err = 0;

	dev_dbg(cdev->dev, "%s\n", __func__);
	if (!cdev->chip) {
		dev_err(cdev->dev, "%s chip?\n", "EMPTY");
		return -EFAULT;
	}

	byte_num = cdev->chip->regmap_cfg.val_bits / 8;
	if (byte_num != 1 && byte_num != 2) {
		dev_err(cdev->dev,
			"unsupported byte length %d.\n", byte_num);
		return -EFAULT;
	}

	for (next = table; next->addr != CAMERA_TABLE_END; next++) {
		dev_dbg(cdev->dev, "%x - %x\n", next->addr, next->val);
		if (next->addr & CAMERA_INT_MASK) {
			err = camera_dev_parser(cdev, next->addr, next->val);
			if (err > 0) { /* special cmd executed */
				err = 0;
				continue;
			}
			if (err < 0) /* this is a real error */
				break;
		}

		if (!buf_count) {
			b_ptr = cdev->i2c_buf;
			addr = next->addr;
		}
		switch (byte_num) {
		case 2:
			*b_ptr++ = (u8)(next->val >> 8);
			buf_count++;
		case 1:
			*b_ptr++ = (u8)next->val;
			buf_count++;
			break;
		}
		{
			const struct camera_reg *n_next = next + 1;
			if (n_next->addr == next->addr + 1 &&
				(buf_count + byte_num <= SIZEOF_I2C_BUF) &&
				!(n_next->addr & CAMERA_INT_MASK))
				continue;
		}

		err = camera_dev_wr_blk(cdev, addr, cdev->i2c_buf, buf_count);
		if (err)
			break;

		buf_count = 0;
	}

	return err;
}

int camera_regulator_get(struct device *dev,
	struct nvc_regulator *nvc_reg, char *vreg_name)
{
	struct regulator *reg = NULL;
	int err = 0;

	dev_dbg(dev, "%s %s", __func__, vreg_name);
	if (vreg_name == NULL) {
		dev_err(dev, "%s NULL regulator name.\n", __func__);
		return -EFAULT;
	}
	reg = regulator_get(dev, vreg_name);
	if (unlikely(IS_ERR_OR_NULL(reg))) {
		dev_err(dev, "%s %s ERR: %d\n", __func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		nvc_reg->vreg = NULL;
	} else {
		dev_dbg(dev, "%s: %s\n", __func__, vreg_name);
		nvc_reg->vreg = reg;
		nvc_reg->vreg_name = vreg_name;
		nvc_reg->vreg_flag = false;
	}

	return err;
}
