/*
 * tegra_cpc.c - Access CPC storage blocks through i2c bus
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

/* misc device */
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/tegra_cpc.h>

/* local data structures */
struct cpc_i2c_host {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct i2c_msg readback;

	int irq;
	struct mutex lock;
	struct completion complete;
};

static atomic_t cpc_in_use;
static struct cpc_i2c_host *cpc_host;

static int cpc_read_data(struct i2c_client *client, u8 cmd,
					void *data, u16 length)
{
	int err;
	struct i2c_msg msg[2];

	if (!client->adapter)
		return -ENODEV;

	/*
	   First put header of CPC read command. Start with the write mode flag,
	   to communicate which command we are sending. Then read the response
	   on the bus
	*/
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = data;


	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		pr_warn("%s: i2c transfer failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int cpc_write_data(struct i2c_client *client, u8 cmd,
				void *data, u8 length, void *hmac)
{
	int err;
	struct i2c_msg msg[4];
	u8 command = cmd;

	if (!client->adapter)
		return -ENODEV;

	/* first put header of CPC write command */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &command;

	msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].len = sizeof(length);
	msg[1].buf = &length;

	msg[2].addr = client->addr;
	msg[2].flags = 0;
	msg[2].len = length;
	msg[2].buf = data;

	msg[3].addr = client->addr;
	msg[3].flags = 0;
	msg[3].len = CPC_HMAC_SIZE;
	msg[3].buf = hmac;


	err = i2c_transfer(client->adapter, msg, 4);
	if (err != 4) {
		pr_err("cpc: i2c transfer failed, for cmd %x\n", cmd);
		return -EINVAL;
	}

	return err;
}

static int cpc_write_key(struct i2c_client *client, u8 cmd,
						void *hmac, u8 length)
{
	int err;
	struct i2c_msg msg[2];
	u8 command = cmd;

	if (!client->adapter)
		return -ENODEV;

	/* first put header of CPC write command */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &command;

	msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].len = length;
	msg[1].buf = hmac;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		pr_err("cpc: i2c transfer failed, for cmd %x\n", cmd);
		return -EINVAL;
	}

	return err;
}

static void _cpc_do_data(struct cpc_i2c_host *cpc, struct cpc_frame_t *fr)
{
	__u32 write_counter = 0;
	u8 status = 0;

	mutex_lock(&cpc->lock);
	switch (fr->req_or_resp) {
	case CPC_READ_M_COUNT:
		/* mcounter size = 4 */
		cpc_read_data(cpc->i2c_client, fr->req_or_resp,
						&write_counter, 4);
		fr->write_counter = write_counter;

		cpc_read_data(cpc->i2c_client, CPC_GET_RESULT, &status, 1);
		fr->result = status;
		break;

	case CPC_READ_FRAME:
		cpc_read_data(cpc->i2c_client, fr->req_or_resp,
				fr->data, fr->len);

		cpc_read_data(cpc->i2c_client, CPC_GET_RESULT, &status, 1);
		fr->result = status;
		break;

	case CPC_WRITE_FRAME:
		init_completion(&cpc->complete);
		cpc_write_data(cpc->i2c_client, fr->req_or_resp, fr->data,
				fr->len, fr->hmac);

		if (!wait_for_completion_timeout(&cpc->complete,
						msecs_to_jiffies(200))) {
			pr_err("%s timeout on write complete\n", __func__);
			fr->result = CPC_WR_FAIL;
			/*
			   TODO: break
			 */
		}

		cpc_read_data(cpc->i2c_client, CPC_GET_RESULT, &status, 1);
		fr->result = status;
		break;

	case CPC_PROGRAM_KEY:
		init_completion(&cpc->complete);
		cpc_write_key(cpc->i2c_client, fr->req_or_resp,
						fr->hmac, CPC_HMAC_SIZE);

		if (!wait_for_completion_timeout(&cpc->complete,
						msecs_to_jiffies(200))) {
			pr_err("%s timeout on write complete\n", __func__);
			fr->result = CPC_KEY_FAIL;
			/*
			   TODO: break
			 */
		}

		cpc_read_data(cpc->i2c_client, CPC_GET_RESULT, &status, 1);
		fr->result = status;
		break;

	default:
		pr_warn("unsupported CPC command\n");
	}
	fr->req_or_resp = fr->req_or_resp<<8;
	mutex_unlock(&cpc->lock);
}

static irqreturn_t cpc_irq(int irq, void *data)
{
	struct cpc_i2c_host *cpc = data;

	complete(&cpc->complete);
	return IRQ_HANDLED;
}

static int cpc_open(struct inode *inode, struct file *filp)
{
	if (atomic_xchg(&cpc_in_use, 1))
		return -EBUSY;

	if (!cpc_host) {
		pr_info("CPC is not ready\n");
		return -EBADF;
	}

	filp->private_data = cpc_host;

	return nonseekable_open(inode, filp);
}

static int cpc_release(struct inode *inode, struct file *filp)
{
	WARN_ON(!atomic_xchg(&cpc_in_use, 0));
	return 0;
}

static long cpc_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct cpc_i2c_host *cpc = file->private_data;
	struct cpc_frame_t *cpc_fr;

	if (_IOC_TYPE(cmd) != NVCPC_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > NVCPC_IOCTL_DO_IO)
		return -ENOTTY;

	switch (cmd) {
	case NVCPC_IOCTL_DO_IO:
		cpc_fr = kmalloc(sizeof(struct cpc_frame_t), GFP_KERNEL);
		if (cpc_fr == NULL) {
			pr_err("%s: failed to allocate mem\n", __func__);
			return -ENOMEM;
		}

		if (copy_from_user(cpc_fr, (const void __user *)arg,
					sizeof(struct cpc_frame_t))) {
			kfree(cpc_fr);
			return -EFAULT;
		}

		_cpc_do_data(cpc, cpc_fr);

		if (copy_to_user((void __user *)arg, cpc_fr,
					sizeof(struct cpc_frame_t))) {
			kfree(cpc_fr);
			return -EFAULT;
		}
		kfree(cpc_fr);
		break;
	default:
		pr_err("CPC: unsupported ioctl\n");
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations cpc_dev_fops = {
	.owner = THIS_MODULE,
	.open = cpc_open,
	.release = cpc_release,
	.unlocked_ioctl = cpc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cpc_ioctl,
#endif
};

static struct miscdevice cpc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tegra_cpc",
	.fops = &cpc_dev_fops,
};

static int cpc_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int status;

	cpc_host = kzalloc(sizeof(struct cpc_i2c_host), GFP_KERNEL);
	if (!cpc_host) {
		pr_err("unable to allocate memory\n");
		return -ENOMEM;
	}

	cpc_host->i2c_client = client;
	i2c_set_clientdata(client, cpc_host);
	cpc_host->irq = client->irq;
	mutex_init(&cpc_host->lock);

	status = request_threaded_irq(cpc_host->irq, cpc_irq, NULL,
				IRQF_TRIGGER_RISING, "cpc_irq", cpc_host);
	if (status) {
		pr_err("%s: request irq failed %d\n", __func__, status);
		goto fail;
	}

	/* TODO: check for CP DRM controller status. Make sure it's ready
	 */

	/* setup misc device for userspace io*/
	if (misc_register(&cpc_dev)) {
		pr_err("%s: misc device register failed\n", __func__);
		status = -ENOMEM;
		goto fail;
	}

	dev_info(&client->dev, "host driver for CPC\n");
	return 0;
fail:
	dev_set_drvdata(&client->dev, NULL);
	mutex_destroy(&cpc_host->lock);
	kfree(cpc_host);
	return status;
}

static int cpc_i2c_remove(struct i2c_client *client)
{
	struct cpc_i2c_host *cpc_host = dev_get_drvdata(&client->dev);

	mutex_destroy(&cpc_host->lock);
	kfree(cpc_host);
	cpc_host = NULL;
	return 0;
}

static struct of_device_id cpc_i2c_of_match_table[] = {
	{ .compatible = "nvidia,cpc", },
	{},
};

static const struct i2c_device_id cpc_i2c_id[] = {
	{ "cpc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mmc_i2c_id);

static struct i2c_driver cpc_i2c_driver = {
	.driver = {
		.name = "cpc",
		.owner = THIS_MODULE,
		.of_match_table = cpc_i2c_of_match_table,
	},
	.probe = cpc_i2c_probe,
	.remove = cpc_i2c_remove,
	.id_table = cpc_i2c_id,
};

module_i2c_driver(cpc_i2c_driver);

MODULE_AUTHOR("Vinayak Pane");
MODULE_DESCRIPTION("Content protection misc driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:cpc_i2c");
