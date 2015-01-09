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

#define CPC_I2C_DEADLINE_MS 200

#define TEGRA_CPC_DEBUG 0

/* local data structures */
struct cpc_i2c_host {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct i2c_msg readback;

	int irq;
	struct mutex lock;
	struct completion complete;
};

/*
   There will be at most one uC instance per board
 */
static atomic_t cpc_in_use;
static struct cpc_i2c_host *cpc_host;

/*
   Given a buffer, construct a CPC packet. Returns the actually occupied size of
   the buffer. Would BUG if the given buffer is not long enough
 */
static int cpc_build_packet(struct i2c_client *client,
				u8 buffer[], unsigned int buf_size_byte,
				u8 cmd_id, cpc_data_len length,
				const void *data, const void *hmac)
{
	unsigned int cur_data_index = CPC_DATA_INDEX;

	unsigned int required_buf_size = CPC_HEADER_SIZE;
	required_buf_size += data ? length : 0;
	required_buf_size += hmac ? CPC_HMAC_SIZE : 0;

	dev_dbg(&client->dev,
		"%s cmd %08x len %u buffer size %u reqd %u\n",
		__func__, cmd_id, length, buf_size_byte, required_buf_size);

	if (unlikely(buf_size_byte < required_buf_size)) {
		dev_err(&client->dev,
			"Buffer (%u) too small for data (%u)\n",
			buf_size_byte, required_buf_size);
		return -EINVAL;
	}

	buffer[CPC_CMD_ID_INDEX] = cmd_id;
	buffer[CPC_DATALEN_INDEX] = length;

	if (data) {
		memcpy(&buffer[cur_data_index], data, length);
		cur_data_index += length;
	}

	if (hmac)
		memcpy(&buffer[cur_data_index], hmac, CPC_HMAC_SIZE);

	return required_buf_size;
}

static int cpc_send_i2c_msg(struct i2c_client *client, struct i2c_msg *msgs,
				unsigned int num_msg)
{
	int err;
	err = i2c_transfer(client->adapter, msgs, num_msg);
	if (unlikely(err != num_msg)) {
		dev_err(&client->dev,
			"%s: i2c transfer failed. Transferred %u out of %u\n",
			__func__, err, num_msg);
		return -EINVAL;
	}
	return 0;
}

/*
   Issues a given read command and copies the resulting data
   to the data_buf parameter.

   data_len indicates how many bytes of real data uC would return upon the
   request

   total_len indicates how many bytes of all data uC would return upon the
   request. data_len and total_len would be different if security data such as
   HMAC is expected to be returned. sizeof(data_buf) must be >= total_len
 */
static int cpc_read_data(struct i2c_client *client, u8 cmd,
			cpc_data_len data_len, void *data_buf,
			unsigned int total_len)
{
	struct i2c_msg msg[2];
	int packet_len;

	/*
	   The read command will not send any data. The uC may send more data
	   then length bytes, if HMAC is involved

	   The buffer for reading the data back has to be big enough to hold
	   the maximum value which can be expressed by the size of cpc_data_len
	*/
	u8 buffer_req[CPC_HEADER_SIZE];
	u8 *buffer_resp = data_buf;

	if (unlikely(!client->adapter))
		return -ENODEV;

	if ( unlikely(
		(!data_len) ||
		(!total_len) ||
		(data_len > CPC_MAX_DATA_SIZE) ||
		(total_len > data_len + CPC_HMAC_SIZE))) {
		dev_err(&client->dev,
			"%s: invalid range. Expected: 0 < data_len %d <= %d\n",
			__func__, data_len, CPC_MAX_DATA_SIZE);
		dev_err(&client->dev,
			"%s: invalid range. Expected: 0 < total_len %d <= %d\n",
			__func__, total_len, data_len + CPC_MAX_DATA_SIZE);
		return -EINVAL;
	}

	/*
	   First put header of CPC read command. Start with the write mode flag,
	   to communicate which command we are sending. Then read the response
	   on the bus
	 */

	packet_len = cpc_build_packet(client, buffer_req,
				sizeof(buffer_req),
				cmd, data_len, NULL, NULL);
	if (packet_len <= 0)
		return -EINVAL;

	msg[0].addr = client->addr;
	msg[0].len = (__u16) packet_len;
	msg[0].flags = 0;
	msg[0].buf = buffer_req;

	/*
	   The second i2c packet is empty, as it is only a buffer space for
	   reading the requested data back
	*/
	msg[1].addr = client->addr;
	msg[1].len = total_len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buffer_resp;
	return cpc_send_i2c_msg(client, msg,
		sizeof(msg) / sizeof(struct i2c_msg));
}

/*
 * Executes a write command. length is the size of data only, not including hmac
 */
static int cpc_write_data(struct i2c_client *client, u8 cmd,
				const void *data, cpc_data_len length,
				const void *hmac)
{
	struct i2c_msg msg[1];
	int packet_len;
	u8 buffer_req[CPC_HEADER_SIZE + CPC_MAX_DATA_SIZE + CPC_HMAC_SIZE];

	if (!client->adapter)
		return -ENODEV;

	/* first put header of CPC write command */
	packet_len = cpc_build_packet(client, buffer_req,
				sizeof(buffer_req), cmd,
				length, data, hmac);
	if (packet_len <= 0)
		return -EINVAL;

	msg[0].addr = client->addr;
	msg[0].len = (__u16) packet_len;
	msg[0].flags = 0;
	msg[0].buf = buffer_req;

	return cpc_send_i2c_msg(client, msg,
		sizeof(msg) / sizeof(struct i2c_msg));
}

static int _cpc_do_data(struct cpc_i2c_host *cpc, struct cpc_frame_t *fr)
{
	int err = 0;
	u8 buffer[CPC_MAX_DATA_SIZE + CPC_HMAC_SIZE];
	cpc_data_len data_len = fr->len;
	unsigned int total_len  = fr->len;

	if (unlikely(
		(data_len == 0) ||
		(data_len > CPC_MAX_DATA_SIZE) ||
		(data_len + CPC_HMAC_SIZE > sizeof(buffer)))) {
		dev_dbg(&cpc->i2c_client->dev,
			"The requested size is invalid:%u", data_len);
		return -EINVAL;
	}

	memset(buffer, 0, sizeof(buffer));
	fr->result = CPC_RESULT_UNKNOWN_REQUEST;
	mutex_lock(&cpc->lock);
	switch (fr->req_or_resp) {
	case CPC_READ_M_COUNT:
		err = cpc_read_data(cpc->i2c_client, fr->req_or_resp, data_len,
						buffer, total_len);
		if (err)
			goto fail;

		memcpy(&fr->write_counter, &buffer[0], CPC_COUNTER_SIZE);
		break;

	case CPC_READ_FRAME:
		total_len += CPC_HMAC_SIZE;
		err = cpc_read_data(cpc->i2c_client, fr->req_or_resp, data_len,
				buffer, total_len);
		if (err)
			goto fail;

		memcpy(&fr->data, &buffer[0], data_len);
		memcpy(&fr->hmac, &buffer[data_len], CPC_HMAC_SIZE);
		break;

	case CPC_WRITE_FRAME:
		init_completion(&cpc->complete);
		err = cpc_write_data(cpc->i2c_client, fr->req_or_resp, fr->data,
				data_len, fr->hmac);
		if (err)
			goto fail;

		if (!wait_for_completion_timeout(&cpc->complete,
				msecs_to_jiffies(CPC_I2C_DEADLINE_MS))) {
			pr_err("%s timeout on write complete\n", __func__);
			fr->result = CPC_RESULT_WRITE_FAILURE;
		}
		break;
	default:
		err = -EINVAL;
		pr_warn("unsupported CPC command id %08x\n", fr->req_or_resp);
		goto fail;
	}
	err = cpc_read_data(cpc->i2c_client, CPC_GET_RESULT,
			sizeof(fr->result), &fr->result,
			sizeof(fr->result));
fail:
	mutex_unlock(&cpc->lock);

	dev_dbg(&cpc->i2c_client->dev,
			"%s execution result:cmd %08x len %u total_len %u\n"
			"\tresult %08x counter %u\n", __func__, fr->req_or_resp,
			data_len, total_len, fr->result, fr->write_counter);
	dev_dbg(&cpc->i2c_client->dev,
			"data %02x %02x %02x %02x\n",
			fr->data[3], fr->data[2], fr->data[1], fr->data[0]);
	return err;
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
	struct cpc_frame_t *cpc_fr = 0;
	long err = 0;

	if (_IOC_TYPE(cmd) != NVCPC_IOC_MAGIC) {
		err = -ENOTTY;
		goto fail;
	}

	if (_IOC_NR(cmd) > NVCPC_IOCTL_DO_IO) {
		err = -ENOTTY;
		goto fail;
	}

	switch (cmd) {
	case NVCPC_IOCTL_DO_IO:
		cpc_fr = kmalloc(sizeof(struct cpc_frame_t), GFP_KERNEL);
		if (!cpc_fr) {
			pr_err("%s: failed to allocate memory\n", __func__);
			err = -ENOMEM;
			goto fail;
		}

		if (copy_from_user(cpc_fr, (const void __user *)arg,
					sizeof(struct cpc_frame_t))) {
			err = -EFAULT;
			goto fail;
		}

		if (cpc_fr->len >
			(sizeof(cpc_fr->hmac) + sizeof(cpc_fr->data))) {
			err = -EINVAL;
			goto fail;
		}

		if (!cpc_fr->len) {
			err = -EINVAL;
			goto fail;
		}

		err = _cpc_do_data(cpc, cpc_fr);

		if (copy_to_user((void __user *)arg, cpc_fr,
					sizeof(struct cpc_frame_t))) {
			err = -EFAULT;
			goto fail;
		}
		break;
	default:
		pr_err("CPC: unsupported ioctl\n");
		err =  -EINVAL;
		goto fail;
	}

fail:
	kfree(cpc_fr);
	return err;
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

	/* Initialize stack variables and structures */
	cpc_host = kzalloc(sizeof(struct cpc_i2c_host), GFP_KERNEL);
	if (!cpc_host) {
		pr_err("unable to allocate memory\n");
		return -ENOMEM;
	}

	/* Setup I2C */
	cpc_host->i2c_client = client;
	i2c_set_clientdata(client, cpc_host);
	cpc_host->irq = client->irq;
	mutex_init(&cpc_host->lock);

	/* Setup IRQ */
	status = request_threaded_irq(cpc_host->irq, cpc_irq, NULL,
				IRQF_TRIGGER_RISING, "cpc_irq", cpc_host);
	if (status) {
		pr_err("%s: request irq failed %d\n", __func__, status);
		goto fail;
	}

	/* TODO: check for CP DRM controller status. Make sure it's ready
	 */

	/* setup misc device for userspace io */
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
