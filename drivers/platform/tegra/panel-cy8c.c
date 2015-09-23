/*
 * CY8C4014-based display power sequence driver
 *
 * Copyright (C) 2015 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform/tegra/panel-cy8c.h>

/* register definitions */
#define CY8C_REG_CMD			0x00
#define		CY8C_CMD_GOTO_BOOT	0x01
#define		CY8C_CMD_RESET		0x02
#define		CY8C_CMD_WRITE_DAT	0x05
#define		CY8C_CMD_SLEEP		0x06
#define		CY8C_CMD_LCD_ON		0x07
#define		CY8C_CMD_LCD_OFF	0x08
#define CY8C_REG_CMD_DAT		0x01
#define CY8C_REG_CMD_STATUS		0x02
#define		CY8C_STATUS_GOOD	0x00
#define		CY8C_STATUS_BUSY	0x01
#define		CY8C_STATUS_ERR		0x02
#define CY8C_REG_APP_MINOR_REV	0x03
#define CY8C_REG_APP_MAJOR_REV	0x04
#define CY8C_REG_PWR_SEQ_STATE	0x05
#define		CY8C_LCD_STATE_PWRING_ON	0x01
#define		CY8C_LCD_STATE_PWRING_OFF	0x02
#define		CY8C_LCD_STATE_ON	0x03
#define		CY8C_LCD_STATE_OFF	0x04
#define CY8C_REG_1V8_ON_DLY_LSB		0x06
#define CY8C_REG_1V8_ON_DLY_MSB		0x07
#define CY8C_REG_3V3_ON_DLY_LSB		0x08
#define CY8C_REG_3V3_ON_DLY_MSB		0x09
#define CY8C_REG_3V0_ON_DLY_LSB		0x0A
#define CY8C_REG_3V0_ON_DLY_MSB		0x0B
#define CY8C_REG_VPP_ON_DLY_LSB		0x0C
#define CY8C_REG_VPP_ON_DLY_MSB		0x0D
#define CY8C_REG_VMM_ON_DLY_LSB		0x0E
#define CY8C_REG_VMM_ON_DLY_MSB		0x0F
#define CY8C_REG_RST_ON_DLY_LSB		0x10
#define CY8C_REG_RST_ON_DLY_MSB		0x11
#define CY8C_REG_1V8_OFF_DLY_LSB	0x12
#define CY8C_REG_1V8_OFF_DLY_MSB	0x13
#define CY8C_REG_3V3_OFF_DLY_LSB	0x14
#define CY8C_REG_3V3_OFF_DLY_MSB	0x15
#define CY8C_REG_3V0_OFF_DLY_LSB	0x16
#define CY8C_REG_3V0_OFF_DLY_MSB	0x17
#define CY8C_REG_VPP_OFF_DLY_LSB	0x18
#define CY8C_REG_VPP_OFF_DLY_MSB	0x19
#define CY8C_REG_VMM_OFF_DLY_LSB	0x1A
#define CY8C_REG_VMM_OFF_DLY_MSB	0x1B
#define CY8C_REG_RST_OFF_DLY_LSB	0x1C
#define CY8C_REG_RST_OFF_DLY_MSB	0x1D
#define CY8C_REG_GPIO_STATE			0x1E
#define		CY8C_REG_GPIO_LO		0x00
#define		CY8C_REG_GPIO_HI		0x01
#define CY8C_REG_MAX	0x1F

/* boot device mode address */
#define CY8C_BOOT_DEV_ADDR			0x08

#define MAX_COMMAND_SIZE			512
#define BASE_CMD_SIZE				0x07
#define CMD_START					0x01
#define CMD_ENTER_BOOTLOADER		0x38
#define CMD_EXIT_BOOTLOADER			0x3B
#define CMD_STOP					0x17
#define COMMAND_DATA_SIZE			0x01
#define RESET						0x00
#define COMMAND_SIZE (BASE_CMD_SIZE + COMMAND_DATA_SIZE)

#define DEVICE_MODE_INVALID		0
#define DEVICE_MODE_APP			1
#define DEVICE_MODE_BOOT		2

/* Expected amount of time that the panel
 * may be in use by userspace for flashing*/
#define PANEL_LOCK_POLL_MS 1000
#define PANEL_LOCK_POLL_TIMEOUT_MS 30000

/* Expected amount of time for panel state change */
#define PANEL_STATE_POLL_MS 50
#define PANEL_STATE_POLL_TIMEOUT_MS 1000

#define CY8C_BOOT_DEV_NAME "cy8c_panel_boot"
#define CY8C_APP_DEV_NAME "cy8c_panel_app"

struct cy8c_data {
	/* app device */
	struct i2c_client *client;
	struct regmap *regmap;
	struct miscdevice miscdev_app;

	/* boot device */
	struct miscdevice miscdev_boot;
	struct i2c_client *client_boot;
	struct i2c_adapter *adap;
	struct i2c_board_info brd_boot;

	struct mutex lock;
	struct dentry *debugdir;
	int device_mode;
	int en_gpio;
};

static struct cy8c_data *drvdata;

typedef unsigned short (*cy8c_crc_algo_t)(
	unsigned char *buf, unsigned long size);

static int of_cy8c_panel_parse_pdata(struct i2c_client *client,
	struct cy8c_data *data)
{
	struct device_node *np = client->dev.of_node;
	int en_gpio;
	int ret;

	en_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (en_gpio < 0) {
		dev_err(&data->client->dev,
			"enable-gpio property not found\n");
		return en_gpio;
	}

	if (gpio_is_valid(en_gpio)) {
		ret = devm_gpio_request_one(&client->dev, en_gpio,
			GPIOF_OUT_INIT_HIGH, CY8C_APP_DEV_NAME);
		if (ret) {
			dev_err(&data->client->dev, "enable-gpio is unavailable\n");
			return ret;
		}
		data->en_gpio = en_gpio;
	} else {
		dev_err(&data->client->dev, "enable-gpio is invalid\n");
		return -EINVAL;
	}

	return 0;
}

static int _cy8c_poll_device(struct cy8c_data *data, u32 reg, u32 mask,
		u32 exp_val, u32 poll_interval_ms, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	struct regmap *regmap = drvdata->regmap;
	u32 reg_val = 0;
	int ret;

	do {
		ret = regmap_read(regmap, reg, &reg_val);
		if (unlikely(ret)) {
			dev_err(&data->client->dev,
				"failed to read device register\n");
			return ret;
		}

		if ((reg_val & mask) != exp_val)
			msleep(poll_interval_ms);
		else
			return 0;

	} while (time_after(timeout_jf, jiffies));

	dev_err(&data->client->dev, "device poll timeout\n");

	return -ETIMEDOUT;
}

static int _cy8c_wait_for_pwr_state(struct cy8c_data *data, int state)
{
	return _cy8c_poll_device(data, CY8C_REG_PWR_SEQ_STATE, 0xff,
		state, PANEL_STATE_POLL_MS, PANEL_STATE_POLL_TIMEOUT_MS);
}

static int _cy8c_panel_set_state(struct cy8c_data *data, bool enable)
{
	struct regmap *regmap = data->regmap;
	int ret;
	int reg;
	int target_state = enable ? CY8C_LCD_STATE_ON : CY8C_LCD_STATE_OFF;
	int target_cmd = enable ? CY8C_CMD_LCD_ON : CY8C_CMD_LCD_OFF;

	if (data->device_mode != DEVICE_MODE_APP) {
		dev_err(&data->client->dev,
			"can't set panel state, device not in app mode\n");
		ret = -EBUSY;
		goto end;
	}

	ret = regmap_read(regmap, CY8C_REG_PWR_SEQ_STATE, &reg);
	if (ret) {
		dev_err(&data->client->dev,
			"failed to read panel power state\n");
		goto end;
	}

	/* Wait for ongoing state transitions to finish */
	if (reg == CY8C_LCD_STATE_PWRING_ON) {
		ret = _cy8c_wait_for_pwr_state(data, CY8C_LCD_STATE_ON);
		if (ret)
			goto end;

		reg = CY8C_LCD_STATE_ON;
	} else if (reg == CY8C_LCD_STATE_PWRING_OFF) {
		ret = _cy8c_wait_for_pwr_state(data, CY8C_LCD_STATE_OFF);
		if (ret)
			goto end;

		reg = CY8C_LCD_STATE_OFF;
	}

	if (reg == target_state)
		goto end;

	/* en_gpio must be asserted while the panel is on */
	if (enable) {
		ret = gpio_direction_output(data->en_gpio, 1);
		if (ret)
			goto end;
	}

	ret = regmap_write(regmap, CY8C_REG_CMD, target_cmd);
	if (ret)
		goto end;

	/* Wait until transition is complete */
	ret = _cy8c_wait_for_pwr_state(data, target_state);
	if (ret) {
		dev_err(&data->client->dev,
			"timedout while setting panel state\n");
		goto end;
	}

	if (!enable) {
		ret = gpio_direction_output(data->en_gpio, 0);
		if (ret)
			goto end;
	}

end:
	/* If panel disable failed, deassert en_gpio anyway; if cy8c app
	 * is still responsive, it should still disable the panel */
	if (!enable && ret)
		gpio_direction_output(data->en_gpio, 0);
	return ret;
}

int cy8c_panel_set_state(bool enable)
{
	int ret = 0;
	struct cy8c_data *data;

	/* drvdata will be null if device isn't present on this platform
	 * (no compatible node found), or -ENODATA if probe ran but failed. */
	if (!drvdata) {
		pr_debug("cy8c not present\n");
		return -ENODEV;
	} else if (IS_ERR(drvdata)) {
		pr_err("cy8c uninitialized\n");
		return PTR_ERR(drvdata);
	}

	data = drvdata;

	mutex_lock(&data->lock);

	ret = _cy8c_panel_set_state(data, enable);
	if (ret)
		dev_err(&data->client->dev,
			"failed to set panel state, error %d\n", ret);

	mutex_unlock(&data->lock);
	return ret;
}
EXPORT_SYMBOL(cy8c_panel_set_state);

static int cy8c_panel_pwr_state_get(void *vdata, u64 *val)
{
	struct cy8c_data *data = vdata;
	int ret;
	u32 reg_val;

	if (!data) {
		dev_err(&data->client->dev, "no device\n");
		return -ENODEV;
	}

	mutex_lock(&data->lock);

	if (data->device_mode != DEVICE_MODE_APP) {
		dev_err(&data->client->dev,
			"can't get panel state, device in boot mode\n");
		ret = -EBUSY;
		goto unlock;
	}

	ret = regmap_read(data->regmap, CY8C_REG_PWR_SEQ_STATE,
		&reg_val);
	if (ret) {
		dev_err(&data->client->dev,
			"failed to read device register\n");
		goto unlock;
	}

	*val = reg_val;

unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int cy8c_panel_pwr_state_set(void *vdata, u64 val)
{
	struct cy8c_data *data = vdata;

	/* val must be 1 (on) or 0 (off) */
	if (val > 1) {
		dev_err(&data->client->dev,
			"invalid panel power state requested\n");
		return -EINVAL;
	}

	return cy8c_panel_set_state(val);
}
DEFINE_SIMPLE_ATTRIBUTE(cy8c_panel_pwr_state_fops,
	cy8c_panel_pwr_state_get, cy8c_panel_pwr_state_set, "%llu\n");

static unsigned short cy8c_crc_algo_sum(
	unsigned char *buf, unsigned long size)
{
	unsigned short sum = 0;
	while (size-- > 0)
		sum += *buf++;

	return 1 + ~sum;
}

static unsigned short cy8c_crc_algo_crc(
	unsigned char *buf, unsigned long size)
{
	unsigned short crc = 0xffff;
	unsigned short tmp;
	int i;

	if (size == 0)
		return ~crc;

	do {
		for (i = 0, tmp = 0x00ff & *buf++;
			i < 8;
			i++, tmp >>= 1) {
			if ((crc & 0x0001) ^ (tmp & 0x0001))
				crc = (crc >> 1) ^ 0x8408;
			else
				crc >>= 1;
		}
	} while (--size);

	crc = ~crc;
	tmp = crc;
	crc = (crc << 8) | (tmp >> 8 & 0xFF);

	return crc;
}

static cy8c_crc_algo_t crc_algos[] = {
	&cy8c_crc_algo_sum,
	&cy8c_crc_algo_crc,
};

static int _cy8c_send_app_mode(struct cy8c_data *data,
		cy8c_crc_algo_t crc_func)
{
	unsigned char cmd_buf[COMMAND_SIZE] = {0, };
	unsigned short checksum;
	unsigned long res_size;
	unsigned long cmd_size;
	int ret;

	if (COMMAND_SIZE < 3)
		return -EINVAL;

	res_size = BASE_CMD_SIZE;
	cmd_size = COMMAND_SIZE;
	cmd_buf[0] = CMD_START;
	cmd_buf[1] = CMD_EXIT_BOOTLOADER;
	cmd_buf[2] = (unsigned char)COMMAND_DATA_SIZE;
	cmd_buf[3] = (unsigned char)(COMMAND_DATA_SIZE >> 8);
	cmd_buf[4] = RESET;
	checksum = (*crc_func)(cmd_buf, COMMAND_SIZE - 3);
	cmd_buf[5] = (unsigned char)checksum;
	cmd_buf[6] = (unsigned char)(checksum >> 8);
	cmd_buf[7] = CMD_STOP;

	ret = i2c_master_send(data->client_boot, cmd_buf, cmd_size);

	return ret;
}

/* Called from probe path only, so left unlocked */
static int cy8c_boot_mode_enter(struct cy8c_data *data,
		cy8c_crc_algo_t crc_func)
{
	const unsigned long RESULT_DATA_SIZE = 8;
	unsigned short checksum;
	unsigned char cmd_buf[BASE_CMD_SIZE];
	unsigned char res_buf[BASE_CMD_SIZE + RESULT_DATA_SIZE];
	int res_size, cmd_size;
	int ret;

	if (COMMAND_SIZE < 3)
		return -EINVAL;

	res_size = BASE_CMD_SIZE + RESULT_DATA_SIZE;
	cmd_size = BASE_CMD_SIZE;
	cmd_buf[0] = CMD_START;
	cmd_buf[1] = CMD_ENTER_BOOTLOADER;
	cmd_buf[2] = 0;
	cmd_buf[3] = 0;
	checksum = (*crc_func)(cmd_buf, BASE_CMD_SIZE - 3);
	cmd_buf[4] = (unsigned char)checksum;
	cmd_buf[5] = (unsigned char)(checksum >> 8);
	cmd_buf[6] = CMD_STOP;

	ret = i2c_master_send(data->client_boot, cmd_buf, cmd_size);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(data->client_boot, res_buf, res_size);

	return ret;
}

/* Called from probe path only, so left unlocked */
static int cy8c_boot_mode_test(struct cy8c_data *data)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(crc_algos); i++) {
		ret = cy8c_boot_mode_enter(data, crc_algos[i]);
		if (ret > 0)
			ret = 0;
		else
			dev_err(&data->client->dev,
				"device not in boot mode\n");

		if (ret == 0)
			break;
	}

	return ret;
}

static int _cy8c_app_mode_test(struct cy8c_data *data)
{
	int ret, reg;
	ret = regmap_read(data->regmap, CY8C_REG_APP_MINOR_REV, &reg);
	return ret;
}

/*
	app mode
	called from user space

	The packet sent to device in boot mode needs a crc value
	as parameter. There are 2 algorithms to compute the crc
	according to the content of the fw file in user space code
	implementation at vendor/bin/vendor/nvidia/loki/utils/cyload.

	In the current design the 'sum' algo is used to compute
	crc, but we will support both of the ALGOes just in case.
*/
static int _cy8c_boot_to_app(struct cy8c_data *data)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(crc_algos); i++) {
		ret = _cy8c_send_app_mode(data, crc_algos[i]);
		if (ret > 0)
			ret = 0;
		else
			dev_err(&data->client->dev,
				"cannot put device to app mode\n");

		if (ret == 0) {

			/* 100 ms is enough in test */
			msleep(100);

			ret = _cy8c_app_mode_test(data);
			if (unlikely(ret))
				dev_err(&data->client->dev,
					"device not in app mode %d\n", i);
		}

		if (ret == 0)
			break;
	}

	return ret;
}

static ssize_t cy8c_boot_mode_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct miscdevice *miscdev_app_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	char *name = "unknown";

	data = container_of(miscdev_app_cdev, struct cy8c_data, miscdev_app);

	/* Lock to make sure device_mode isn't undergoing a state transition */
	mutex_lock(&data->lock);

	if (data->device_mode == DEVICE_MODE_APP)
		name = "app";
	else if (data->device_mode == DEVICE_MODE_BOOT)
		name = "boot";

	mutex_unlock(&data->lock);

	return sprintf(buf, "%s\n", name);
}

int _cy8c_mode_set(struct cy8c_data *data, int mode)
{
	int ret;

	switch (mode) {
	case DEVICE_MODE_APP:
		/* cy8c is already be in app mode if this is called
		 * from cyload */
		ret = _cy8c_app_mode_test(data);
		if (ret) {
			dev_err(&data->client->dev, "trying to jump to app\n");
			ret = _cy8c_boot_to_app(data);
			if (ret) {
				dev_err(&data->client->dev, "failed jumping to app\n");
				return ret;
			}
		}

		data->device_mode = DEVICE_MODE_APP;
		break;
	case DEVICE_MODE_BOOT:
		/* Try disabling the panel before resetting the uC.
		 * If this command fails then either 1) uC is already
		 * in boot mode, or 2) uC will take care of disabling the
		 * panel before reset */
		ret = _cy8c_panel_set_state(data, false);
		if (ret)
			dev_err(&data->client->dev,
				"can't disable panel before entering boot\n");

		ret = regmap_write(data->regmap, CY8C_REG_CMD,
			CY8C_CMD_GOTO_BOOT);
		if (ret) {
			dev_err(&data->client->dev,
				"cannot put dev to boot mode\n");
			return ret;
		}

		data->device_mode = DEVICE_MODE_BOOT;
		break;
	default:
		dev_err(&data->client->dev,
			"unknown mode %d requested\n", mode);
		ret = -EINVAL;
	}

	return ret;
}

int cy8c_mode_set(struct cy8c_data *data, int mode)
{
	int ret;

	mutex_lock(&data->lock);
	ret = _cy8c_mode_set(data, mode);
	mutex_unlock(&data->lock);

	return ret;
}

/*
	0 -> APP mode
	1 -> Boot mode
*/
static ssize_t cy8c_boot_mode_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct miscdevice *miscdev_app_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	ssize_t ret = -EINVAL;
	int action = -1;

	data = container_of(miscdev_app_cdev, struct cy8c_data, miscdev_app);

	if (buf == NULL || buf[0] == 0) {
		dev_err(&data->client->dev, "input buf invalid\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &action) != 1) {
		dev_err(&data->client->dev, "input data format invalid\n");
		return -EINVAL;
	}

	if (action != 0 && action != 1) {
		dev_err(&data->client->dev, "input data format value\n");
		return -EINVAL;
	}

	ret = cy8c_mode_set(data, action ? DEVICE_MODE_BOOT :
		DEVICE_MODE_APP);

	return ret == 0 ? size : ret;
}

static ssize_t cy8c_version_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct miscdevice *miscdev_app_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	ssize_t ret = 0;
	ssize_t count = 0;
	int version = -1;
	data = container_of(miscdev_app_cdev, struct cy8c_data, miscdev_app);

	mutex_lock(&data->lock);

	if (data->device_mode != DEVICE_MODE_APP) {
		dev_err(&data->client->dev, "device not in app mode %s\n",
			__func__);
		ret = -EAGAIN;
		goto end;
	}

	ret = regmap_read(data->regmap,
		CY8C_REG_APP_MINOR_REV,
		&version);
	if (unlikely(ret)) {
		dev_err(&data->client->dev, "device in boot mode?\n");
		goto end;
	}
	count += sprintf(buf, "%02x: %02x\n", CY8C_REG_APP_MINOR_REV, version);

	ret = regmap_read(data->regmap,
		CY8C_REG_APP_MAJOR_REV,
		&version);
	if (unlikely(ret)) {
		dev_err(&data->client->dev, "device in boot mode?\n");
		goto end;
	}
	count += sprintf(buf + count, "%02x: %02x\n",
		CY8C_REG_APP_MAJOR_REV, version);

end:
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t cy8c_version_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct miscdevice *miscdev_app_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	data = container_of(miscdev_app_cdev, struct cy8c_data, miscdev_app);
	dev_err(&data->client->dev, "not implemented\n");
	return -ENOSYS;
}

static DEVICE_ATTR(boot_mode, S_IRUGO|S_IWUSR,
		cy8c_boot_mode_show, cy8c_boot_mode_set);
static DEVICE_ATTR(version, S_IRUGO|S_IWUSR,
		cy8c_version_show, cy8c_version_set);

static int cy8c_boot_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev_boot =
		(struct miscdevice *)file->private_data;
	struct cy8c_data *data = NULL;
	data = container_of(miscdev_boot, struct cy8c_data, miscdev_boot);

	if (!mutex_trylock(&data->lock)) {
		dev_err(&data->client->dev, "already opened\n");
		return -EBUSY;
	}

	if (data->device_mode != DEVICE_MODE_BOOT) {
		dev_err(&data->client->dev,
			"wait for device in boot mode\n");

		mutex_unlock(&data->lock);
		return -EIO;
	}

	file->private_data = data;
	dev_err(&data->client->dev, "opened\n");
	/* mutex is unlocked in cy8c_boot_release */
	return 0;
}

static int cy8c_boot_release(struct inode *inode, struct file *file)
{
	int ret;
	struct cy8c_data *data = file->private_data;
	file->private_data = NULL;

	/* cyload doesn't restore uC to app state, driver is responsible
	 * for that. */
	ret = _cy8c_mode_set(data, DEVICE_MODE_APP);

	/* acquired in cy8c_boot_open */
	mutex_unlock(&data->lock);

	dev_err(&data->client->dev, "released\n");

	return ret;
}

static ssize_t _cy8c_boot_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct cy8c_data *data = filp->private_data;
	unsigned char *p = NULL;
	int ret;

	if (data->device_mode != DEVICE_MODE_BOOT) {
		dev_err(&data->client->dev, "device not in boot mode\n");
		ret = -EINVAL;
		goto end;
	}

	if (count == 0) {
		dev_err(&data->client->dev, "no data\n");
		ret = -EINVAL;
		goto end;
	}

	p = kzalloc(count, GFP_KERNEL);

	if (p == NULL) {
		dev_err(&data->client->dev, "no memory\n");
		ret = -ENOMEM;
		goto end;
	}

	/* Read data */
	ret = i2c_master_recv(data->client_boot, p, count);
	if (ret != count) {
		dev_err(&data->client->dev,
			"failed to read %d\n", ret);
		ret = -EIO;
		goto end;
	}

	ret = copy_to_user(buf, p, count);
	if (ret) {
		dev_err(&data->client->dev,
			"failed to copy from user space\n");
	}

end:

	/* kfree(NULL) is safe */
	kfree(p);

	return ret == 0 ? count : ret;
}

static ssize_t _cy8c_boot_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct cy8c_data *data = filp->private_data;
	unsigned char *p = NULL;
	int ret;

	if (data->device_mode != DEVICE_MODE_BOOT) {
		dev_err(&data->client->dev, "device not in boot mode\n");
		ret = -EINVAL;
		goto end;
	}

	if (count == 0) {
		dev_err(&data->client->dev, "no data\n");
		ret = -EINVAL;
		goto end;
	}

	p = kzalloc(count, GFP_KERNEL);

	if (p == NULL) {
		dev_err(&data->client->dev, "no memory\n");
		ret = -ENOMEM;
		goto end;
	}

	if (copy_from_user(p, buf, count)) {
		dev_err(&data->client->dev,
			"failed to copy from user space\n");
		ret = -EFAULT;
		goto end;
	}

	/* Write data */
	ret = i2c_master_send(data->client_boot, p, count);
	if (ret != count) {
		dev_err(&data->client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
	}

end:

	/* kfree(NULL) is safe */
	kfree(p);

	return ret;
}

/* Read and write ops must be done in locked context to avoid clashes with
 * cy8c state changes triggered by other sources. Acquiring/releasing
 * the mutex is done by the open and release ops.
 *
 * This interface is to be used for sideloading firmware only.
 */
static const struct file_operations cy8c_boot_fileops = {
	.owner = THIS_MODULE,
	.open = cy8c_boot_open,
	.release = cy8c_boot_release,
	.read = _cy8c_boot_read,
	.write = _cy8c_boot_write,
};

static int cy8c_panel_create_sysfs(struct cy8c_data *data)
{
	int ret;
	struct i2c_client *client = data->client;

	ret = device_create_file(data->miscdev_app.this_device,
		&dev_attr_boot_mode);
	if (ret) {
		dev_err(&client->dev, "Failed to register boot sys node\n");
		goto err1;
	}

	ret = device_create_file(data->miscdev_app.this_device,
		&dev_attr_version);
	if (ret) {
		dev_err(&client->dev, "Failed to register version sys node\n");
		goto err2;
	}

	return ret;

err2:
	device_remove_file(data->miscdev_app.this_device, &dev_attr_boot_mode);

err1:
	return ret;
}

static void cy8c_panel_remove_sysfs(struct cy8c_data *data)
{
	device_remove_file(data->miscdev_app.this_device, &dev_attr_boot_mode);
	device_remove_file(data->miscdev_app.this_device, &dev_attr_version);
}

#ifdef CONFIG_DEBUG_FS
static void cy8c_panel_remove_debugfs(struct cy8c_data *data)
{
	/* debugfs_remove_recursive(NULL) is safe */
	debugfs_remove_recursive(data->debugdir);
	data->debugdir = NULL;
}

static void cy8c_panel_create_debugfs(struct cy8c_data *data)
{
	struct dentry *ret;

	data->debugdir = debugfs_create_dir(CY8C_APP_DEV_NAME, NULL);
	if (!data->debugdir)
		goto err;

	ret = debugfs_create_file("panel_pwr_state", S_IRUGO,
		data->debugdir, data, &cy8c_panel_pwr_state_fops);
	if (!ret)
		goto err;

	return;

err:
	cy8c_panel_remove_debugfs(data);
	dev_err(&data->client->dev, "Failed to create debugfs\n");
	return;
}
#else
static void cy8c_panel_create_debugfs(struct cy8c_data *data) { }
static void cy8c_panel_remove_debugfs(struct cy8c_data *data) { }
#endif

static int cy8c_panel_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct cy8c_data *data;
	struct regmap_config rconfig;
	int ret, reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		ret = -ENODEV;
		goto err1;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err1;
	}

	/* enable print in show/get */
	data->client = client;

	ret = of_cy8c_panel_parse_pdata(client, data);
	if (ret) {
		dev_err(&client->dev, "of data parse failed\n");
		goto err2;
	}

	memset(&rconfig, 0, sizeof(rconfig));
	rconfig.reg_bits = 8;
	rconfig.val_bits = 8;
	rconfig.cache_type = REGCACHE_NONE;
	rconfig.max_register = CY8C_REG_MAX-1;

	/*This should happen before set clientdata*/
	data->regmap = regmap_init_i2c(client, &rconfig);
	if (!data->regmap) {
		devm_kfree(&client->dev, data);
		dev_err(&client->dev, "Failed to allocate register map\n");
		ret = -ENOMEM;
		goto err2;
	}

	/* boot device I2C client */
	data->adap = i2c_get_adapter(client->adapter->nr);
	memset(&data->brd_boot, 0, sizeof(data->brd_boot));
	strncpy(data->brd_boot.type, CY8C_BOOT_DEV_NAME,
		sizeof(data->brd_boot.type));
	data->brd_boot.addr = CY8C_BOOT_DEV_ADDR;
	data->client_boot = i2c_new_device(data->adap, &data->brd_boot);

	i2c_set_clientdata(client, data);

	data->device_mode = DEVICE_MODE_INVALID;

	/*
		Sometimes the app fw is broken or out of integration,
		so we will detect app mode first then boot mode
	*/
	ret = regmap_read(data->regmap, CY8C_REG_APP_MAJOR_REV, &reg);
	if (ret == 0) {
		dev_dbg(&client->dev, "rev: 0x%02x ", reg);

		ret = regmap_read(data->regmap, CY8C_REG_APP_MINOR_REV, &reg);
		if (ret) {
			dev_err(&client->dev, "Failed to read revision-minor\n");
			goto err3;
		}

		dev_dbg(&client->dev, "0x%02x\n", reg);
		data->device_mode = DEVICE_MODE_APP;
	} else {
		dev_dbg(&client->dev, "not detect app device\n");
		dev_dbg(&client->dev, "continue to detect boot device\n");

		/* Detect whether boot device is available */
		ret = cy8c_boot_mode_test(data);
		if (ret == 0) {
			dev_dbg(&client->dev, "boot device detected\n");
			data->device_mode = DEVICE_MODE_BOOT;
		} else {
			dev_dbg(&client->dev, "boot device not detected\n");
			goto err3;
		}
	}

	/* boot device */
	data->miscdev_boot.name = CY8C_BOOT_DEV_NAME;
	data->miscdev_boot.fops = &cy8c_boot_fileops;
	data->miscdev_boot.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&data->miscdev_boot);
	if (ret) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
			__func__, CY8C_BOOT_DEV_NAME);
		goto err3;
	}

	/* app device */
	data->miscdev_app.name = CY8C_APP_DEV_NAME;
	data->miscdev_app.minor = MISC_DYNAMIC_MINOR;
	data->miscdev_app.parent = &client->dev;
	ret = misc_register(&data->miscdev_app);
	if (ret) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
			__func__, CY8C_APP_DEV_NAME);
		goto err4;
	}

	ret = cy8c_panel_create_sysfs(data);
	if (ret)
		goto err5;

	cy8c_panel_create_debugfs(data);

	mutex_init(&data->lock);

	/* cache for calls from other modules */
	drvdata = data;

	return ret;

err5:
	misc_deregister(&data->miscdev_app);

err4:
	misc_deregister(&data->miscdev_boot);

err3:
	if (data->client_boot)
		i2c_unregister_device(data->client_boot);
	if (data->adap)
		i2c_put_adapter(data->adap);

err2:
	devm_kfree(&client->dev, data);
err1:
	drvdata = (struct cy8c_data *)-ENODATA;
	return ret;
}

static int cy8c_panel_remove(struct i2c_client *client)
{
	struct cy8c_data *data = i2c_get_clientdata(client);

	devm_gpio_free(&client->dev, data->en_gpio);

	cy8c_panel_remove_sysfs(data);
	cy8c_panel_remove_debugfs(data);

	if (data->miscdev_boot.this_device)
		misc_deregister(&data->miscdev_app);

	if (data->miscdev_boot.this_device)
		misc_deregister(&data->miscdev_boot);

	if (data->client_boot)
		i2c_unregister_device(data->client_boot);

	if (data->adap)
		i2c_put_adapter(data->adap);

	mutex_destroy(&data->lock);
	drvdata = NULL;

	return 0;
}

static const struct i2c_device_id cy8c_panel_id[] = {
	{"cy8c_panel", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8c_panel_id);

#ifdef CONFIG_OF
static struct of_device_id cy8c_of_match[] = {
	{.compatible = "nvidia,cy8c_panel", },
	{ },
};
#endif

static struct i2c_driver cy8c_panel_driver = {
	.driver = {
		.name   = "cy8c_panel",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(cy8c_of_match),
#endif
	},
	.probe	  = cy8c_panel_probe,
	.remove	 = cy8c_panel_remove,
	.id_table   = cy8c_panel_id,
};

module_i2c_driver(cy8c_panel_driver);

MODULE_AUTHOR("Daniel Solomon <daniels@nvidia.com>");
MODULE_DESCRIPTION("CY8C I2C based display power sequence driver");
MODULE_LICENSE("GPL");
