/*
 * saf775x_ioctl.c -- SAF775X Soc Audio driver IO control
 *
 * Copyright (c) 2014-2015 NVIDIA CORPORATION.  All rights reserved.
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
#include <asm/mach-types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "saf775x_ioctl.h"

static struct saf775x_ioctl_ops saf775x_ops;
struct i2c_client *codec_priv;

static int saf775x_hwdep_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct saf775x_cmd __user *_saf775x = (struct saf775x_cmd *)arg;
	struct saf775x_cmd saf775x;
	int ret = 0;
	unsigned char *buf;


	switch (cmd) {
	case SAF775X_CONTROL_SET_IOCTL:

		if (arg && copy_from_user(&saf775x, _saf775x, sizeof(saf775x)))
			return -EFAULT;

		if (saf775x_ops.codec_write)
			ret = saf775x_ops.codec_write(codec_priv,
				saf775x.reg, saf775x.val,
				saf775x.reg_len, saf775x.val_len);
		else
			ret = -EFAULT;
		break;

	case SAF775x_CODEC_RESET_IOCTL:
		saf775x_ops.codec_reset();
		break;

	case SAF775X_CONTROL_GET_IOCTL:

		if (arg && copy_from_user(&saf775x, _saf775x, sizeof(saf775x)))
			return -EFAULT;

		buf = devm_kzalloc(&codec_priv->dev,
		 sizeof(*buf) * saf775x.val_len, GFP_KERNEL);

		if (saf775x_ops.codec_read) {
			ret = saf775x_ops.codec_read(codec_priv,
				buf, saf775x.val_len);

		copy_to_user((unsigned char *)saf775x.val, buf,
				sizeof(*buf) * saf775x.val_len);
		devm_kfree(&codec_priv->dev, buf);
		} else
			ret = -EFAULT;
		break;

	default:
		return -EFAULT;
	}

	return ret;
}

static int saf775x_ioctl_open(struct inode *inp, struct file *filep)
{
	return 0;
}


static int saf775x_ioctl_release(struct inode *inp, struct file *filep)
{
	return 0;
}


static int saf775x_ioctl_major;
static struct cdev saf775x_ioctl_cdev;
static struct class *saf775x_ioctl_class;

static const struct file_operations saf775x_ioctl_fops = {
	.owner = THIS_MODULE,
	.open = saf775x_ioctl_open,
	.release = saf775x_ioctl_release,
	.unlocked_ioctl = saf775x_hwdep_ioctl,
};

static void saf775x_ioctl_cleanup(void)
{
	cdev_del(&saf775x_ioctl_cdev);
	device_destroy(saf775x_ioctl_class, MKDEV(saf775x_ioctl_major, 0));

	if (saf775x_ioctl_class)
		class_destroy(saf775x_ioctl_class);

	unregister_chrdev_region(MKDEV(saf775x_ioctl_major, 0),
							1);
}

int saf775x_hwdep_create(struct i2c_client *client)
{
	int result;
	int ret = -ENODEV;
	dev_t saf775x_ioctl_dev;
	codec_priv = client;
	result = alloc_chrdev_region(&saf775x_ioctl_dev, 0,
			1, "saf775x_hwdep");
	if (result < 0)
		goto fail_err;

	saf775x_ioctl_major = MAJOR(saf775x_ioctl_dev);
	cdev_init(&saf775x_ioctl_cdev, &saf775x_ioctl_fops);

	saf775x_ioctl_cdev.owner = THIS_MODULE;
	saf775x_ioctl_cdev.ops = &saf775x_ioctl_fops;
	result = cdev_add(&saf775x_ioctl_cdev, saf775x_ioctl_dev, 1);
	if (result < 0)
		goto fail_chrdev;

	saf775x_ioctl_class = class_create(THIS_MODULE, "saf775x_hwdep");

	if (IS_ERR(saf775x_ioctl_class)) {
		pr_err(KERN_ERR "saf775x_hwdep: device class file already in use.\n");
		saf775x_ioctl_cleanup();
		return PTR_ERR(saf775x_ioctl_class);
	}

	device_create(saf775x_ioctl_class, NULL,
			MKDEV(saf775x_ioctl_major, 0),
				NULL, "%s", "saf775x_hwdep");
	return 0;

fail_chrdev:
	unregister_chrdev_region(saf775x_ioctl_dev, 1);

fail_err:
	return ret;

}
EXPORT_SYMBOL_GPL(saf775x_hwdep_create);

int saf775x_hwdep_cleanup(void)
{
	saf775x_ioctl_cleanup();
	return 0;
}
EXPORT_SYMBOL_GPL(saf775x_hwdep_cleanup);

struct saf775x_ioctl_ops *saf775x_get_ioctl_ops(void)
{
	return &saf775x_ops;
}
EXPORT_SYMBOL_GPL(saf775x_get_ioctl_ops);

MODULE_AUTHOR("Arun S L <aruns@nvidia.com>");
MODULE_DESCRIPTION("SAF775X Soc Audio driver IO control");
MODULE_LICENSE("GPL");
