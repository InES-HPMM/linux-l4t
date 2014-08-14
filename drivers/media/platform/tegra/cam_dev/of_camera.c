/*
 * debugfs.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.

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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <media/nvc.h>
#include <media/camera.h>

static struct camera_platform_info *cam_desc;

#if defined(DEBUG)
static int of_camera_get_strings(char *s, int length)
{
	int len, sn = 0;
	char ch;

	for (len = 0; len < length; len++) {
		ch = s[len];
		/* if \0 and not the 1st char, new string */
		if (len && !ch) {
			sn++;
			continue;
		}
		/* check \r \n */
		if (ch == 0x0D || ch == 0x0A)
			continue;
		/* check charactors */
		if (ch >= 0x20 && ch <= 0x7E)
			continue;
		return 0;
	}

	/* last charactor has to be a '\0' */
	if (ch)
		return 0;

	return sn;
}

static void of_camera_dump_strings(
	struct device *dev, const char *t, const char *s, int len)
{
	char *ptr, *buf;

	if (!len || !s)
		return;

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return;

	memcpy(buf, s, len);
	ptr = buf;
	while (--len) {
		if (!*ptr)
			*ptr = ',';
		ptr++;
	}
	dev_info(dev, "     %s = %s\n", t, buf);
	kfree(buf);
}

static void of_camera_dump_array(
	struct device *dev, const char *t, const u8 *s, int len)
{
	char *ptr, *buf;
	int i;

	if (!len || !s)
		return;

	buf = kzalloc(len * 4 + 20, GFP_KERNEL);
	if (!buf)
		return;

	ptr = buf;
	for (i = 0; i < len; i++, s++) {
		sprintf(ptr, " %02x", *s);
		ptr += 3;
		if (i % 16 == 15) {
			strcat(ptr, "\n");
			ptr += strlen(ptr);
		} else if (i % 4 == 3) {
			strcat(ptr, ", ");
			ptr += strlen(ptr);
		}
	}
	dev_info(dev, "     %s(%d) = %s\n", t, len, buf);
	kfree(buf);
}

static void of_camera_dump_property(struct device *dev, struct property *pp)
{
	if (of_camera_get_strings(pp->value, pp->length))
		of_camera_dump_strings(dev, pp->name, pp->value, pp->length);
	else
		of_camera_dump_array(dev, pp->name, pp->value, pp->length);
}
#else
static inline void of_camera_dump_array(
	struct device *d, const char *t, const u8 *n, int l) {}
static inline void of_camera_dump_property(
	struct device *d, struct property *p) {}
#endif

static struct device_node *of_camera_child_by_index(
	const struct device_node *np, int index)
{
	struct device_node *child;
	int i = 0;

	for_each_child_of_node(np, child) {
		if (index == i)
			break;
		i++;
	}

	return child;
}

static uint of_camera_scan_maxlen(
	struct platform_device *dev, struct device_node *np)
{
	struct device_node *child;
	struct property *pp;
	uint max_s = 0, mc;

	for_each_child_of_node(np, child) {
		dev_dbg(&dev->dev, " child %s\n", child->name);
		for (pp = child->properties; pp; pp = pp->next) {
			if (max_s < pp->length)
				max_s = pp->length;
			of_camera_dump_property(&dev->dev, pp);
		}
		mc = of_camera_scan_maxlen(dev, child);
		if (mc > max_s)
			max_s = mc;
	}

	return max_s;
}

static uint of_camera_get_child_count(
	struct platform_device *dev, struct device_node *np)
{
	struct device_node *child;
	uint cnt = 0;

	for_each_child_of_node(np, child) {
		cnt++;
		dev_dbg(&dev->dev, " child %s\n", child->name);
	}

	return cnt;
}

static uint of_camera_profile_statistic(
	struct platform_device *dev, struct device_node **prof, uint *num_prof)
{
	dev_dbg(&dev->dev, "%s\n", __func__);

	if (prof) {
		*prof = of_find_node_by_name(dev->dev.of_node, "profiles");
		if (*prof && num_prof) {
			*num_prof = of_camera_get_child_count(dev, *prof);
			dev_dbg(&dev->dev, "        profiles %d\n", *num_prof);
		}
	}

	return of_camera_scan_maxlen(dev, dev->dev.of_node);
}

int of_camera_get_property(struct camera_info *cam, unsigned long arg)
{
	struct camera_platform_data *pdata = cam_desc->pdata;
	struct device_node *np = NULL;
	struct nvc_param param;
	char *ref_name = NULL;
	const void *prop;
	void *pcvt, *pbuf = NULL;
	int len, i, err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	if (copy_from_user(&param, (const void __user *)arg, sizeof(param))) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if ((param.param & CAMERA_DT_TYPE_MASK) == CAMERA_DT_QUERY) {
		param.sizeofvalue = cam_desc->pdata->max_blob_size;
		param.variant = cam_desc->pdata->prof_num;
		param.param = cam_desc->pdata->mod_num;
		if (copy_to_user((void __user *)arg, &param, sizeof(param))) {
			dev_err(cam->dev, "%s copy_to_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}

	/* sanity check */
	if (!param.sizeofvalue) {
		dev_err(cam->dev, "%s invalid property name length %d\n",
			__func__, param.sizeofvalue);
		return -EBADF;
	}

	dev_dbg(cam->dev, "%s params: %x, %x, %x, %d\n", __func__,
		param.param, param.variant, param.p_value, param.sizeofvalue);
	/* use bit mask to determine if it's a profile or a module query */
	switch (param.param & CAMERA_DT_HANDLE_MASK) {
	case CAMERA_DT_HANDLE_PROFILE:
		/* looking for a profile */
		if (!pdata->of_profiles) {
			dev_dbg(cam->dev, "%s DT has no profile node.\n",
				__func__);
			return -EEXIST;
		}
		np = of_camera_child_by_index(
			pdata->of_profiles, param.variant);
		break;
	case CAMERA_DT_HANDLE_PHANDLE:
		np = of_find_node_by_phandle(param.variant);
		break;
	case CAMERA_DT_HANDLE_MODULE:
		/* module of_node */
		if (param.variant >= pdata->mod_num) {
			dev_err(cam->dev, "%s has no sub module node %d\n",
				__func__, param.variant);
			return -EEXIST;
		}
		np = pdata->modules[param.variant].of_node;
		break;
	case CAMERA_DT_HANDLE_SENSOR:
		/* sensor of_node */
		if (param.variant >= pdata->mod_num) {
			dev_err(cam->dev, "%s has no sub module node %d\n",
				__func__, param.variant);
			return -EEXIST;
		}
		if (pdata->modules[param.variant].sensor.bi)
			np = pdata->modules[param.variant].sensor.bi->of_node;
		break;
	case CAMERA_DT_HANDLE_FOCUSER:
		/* focuser of_node */
		if (param.variant >= pdata->mod_num) {
			dev_err(cam->dev, "%s has no sub module node %d\n",
				__func__, param.variant);
			return -EEXIST;
		}
		if (pdata->modules[param.variant].focuser.bi)
			np = pdata->modules[param.variant].focuser.bi->of_node;
		break;
	case CAMERA_DT_HANDLE_FLASH:
		/* flash of_node */
		if (param.variant >= pdata->mod_num) {
			dev_err(cam->dev, "%s has no sub module node %d\n",
				__func__, param.variant);
			return -EEXIST;
		}
		if (pdata->modules[param.variant].flash.bi)
			np = pdata->modules[param.variant].flash.bi->of_node;
		break;
	default:
		dev_err(cam->dev, "%s unsupported handle type %x\n",
			__func__, param.param);
		return -EINVAL;
	}
	if (!np) {
		dev_dbg(cam->dev, "%s module %d has no such DT node\n",
			__func__, param.variant);
		return -ECHILD;
	}

	dev_dbg(cam->dev, "%s now on %s of [%d]\n",
		__func__, np->full_name, param.variant);

	pbuf = kzalloc(param.sizeofvalue + CAMERA_MAX_NAME_LENGTH, GFP_KERNEL);
	if (!pbuf) {
		dev_err(cam->dev, "%s cannot alloc memory\n", __func__);
		return -ENOMEM;
	}
	ref_name = pbuf;
	pcvt = pbuf + CAMERA_MAX_NAME_LENGTH;

	len = param.sizeofvalue < CAMERA_MAX_NAME_LENGTH - 1 ?
		param.sizeofvalue : CAMERA_MAX_NAME_LENGTH - 1;
	if (copy_from_user(ref_name, MAKE_CONSTUSER_PTR(param.p_value), len)) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		kfree(pbuf);
		return -EFAULT;
	}

	dev_dbg(cam->dev, "%s looking for %s\n", __func__, ref_name);
	prop = of_get_property(np, ref_name, &len);
	if (len > param.sizeofvalue) {
		dev_err(cam->dev, "%s return buffer size %d too small %d\n",
			__func__, param.sizeofvalue, len);
		kfree(pbuf);
		return -E2BIG;
	}
	if (!prop) {
		dev_dbg(cam->dev, "%s property %s not exists\n",
			__func__, ref_name);
		param.sizeofvalue = 0;
		goto get_property_end;
	}
	param.sizeofvalue = len;
	dev_dbg(cam->dev, "%s %p size %d\n", __func__, prop, len);

	switch (param.param & CAMERA_DT_TYPE_MASK) {
	case CAMERA_DT_STRING:
		if (len)
			param.variant = of_property_count_strings(
				np, ref_name);
		else
			param.variant = 0;
		pcvt = (void *)prop;
		break;
	case CAMERA_DT_DATA_U8:
		if (len != sizeof(u8)) {
			dev_err(cam->dev, "%s data_u8: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		pcvt = (void *)prop;
		break;
	case CAMERA_DT_DATA_U16:
		if (len != sizeof(u16)) {
			dev_err(cam->dev, "%s data_u8: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		*(u16 *)pcvt = be16_to_cpup(prop);
		break;
	case CAMERA_DT_DATA_U32:
		if (len != sizeof(u32)) {
			dev_err(cam->dev, "%s data_u8: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		*(u32 *)pcvt = be32_to_cpup(prop);
		break;
	case CAMERA_DT_DATA_U64:
		if (len != sizeof(u64)) {
			dev_err(cam->dev, "%s data_u8: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		*(u64 *)pcvt = be64_to_cpup(prop);
		break;
	case CAMERA_DT_ARRAY_U8:
		pcvt = (void *)prop;
		break;
	case CAMERA_DT_ARRAY_U16:
		if (len % sizeof(__be16)) {
			dev_err(cam->dev, "%s array_u16: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		for (i = 0; i < len / sizeof(__be16); i++) {
			((u16 *)pcvt)[i] = be16_to_cpup(prop);
			prop += sizeof(__be16);
		}
		break;
	case CAMERA_DT_ARRAY_U32:
		if (len % sizeof(__be32)) {
			dev_err(cam->dev, "%s array_u16: size mismatch %d\n",
				__func__, len);
			err = -EINVAL;
			break;
		}
		for (i = 0; i < len / sizeof(__be32); i++) {
			((u32 *)pcvt)[i] = be32_to_cpup(prop);
			prop += sizeof(__be32);
		}
		break;
	default:
		dev_err(cam->dev, "%s unsupported request %x\n",
			__func__, param.param);
		err = -ENOENT;
	}

get_property_end:
	if (copy_to_user((void __user *)arg, &param, sizeof(param))) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}
	of_camera_dump_array(cam->dev, ref_name, pcvt, len);
	if (!err && len && copy_to_user(MAKE_USER_PTR(param.p_value),
		pcvt, len)) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

	kfree(pbuf);
	dev_dbg(cam->dev, "%s %d\n", __func__, err);
	return err;
}

static struct device_node *of_camera_node_lookup(
	struct camera_board *cb, struct i2c_board_info *bi, int bus_num)
{
	if (!cb->bi || !cb->bi->of_node)
		return NULL;

	if ((bus_num == cb->busnum) &&
		(bi->addr == cb->bi->addr) &&
		!strcmp(bi->type, cb->chipname))
		return cb->bi->of_node;

	return NULL;
}

int of_camera_find_node(
	struct camera_info *cam, int bus_num, struct i2c_board_info *bi)
{
	struct camera_platform_data *pdata = cam_desc->pdata;
	struct camera_module *md;
	int i;

	dev_dbg(cam->dev, "%s: %s %x %x @ %d\n", __func__,
		bi->type, bi->addr, bi->flags, bus_num);
	if (!pdata || !pdata->modules || !bi)
		return 0;

	if (bi->of_node)
		goto node_validation;

	md = pdata->modules;
	for (i = 0; i < pdata->mod_num; i++, md++) {
		bi->of_node = of_camera_node_lookup(&md->sensor, bi, bus_num);
		if (bi->of_node)
			break;

		bi->of_node = of_camera_node_lookup(&md->focuser, bi, bus_num);
		if (bi->of_node)
			break;

		bi->of_node = of_camera_node_lookup(&md->flash, bi, bus_num);
		if (bi->of_node)
			break;
	}

node_validation:
	/* only attach device of_node when keyword compatible presents */
	if (bi->of_node &&
		!of_get_property(bi->of_node, "use_of_node", NULL))
		bi->of_node = NULL;

	return 0;
}

static struct device_node *of_camera_get_brdinfo(
	struct platform_device *dev,
	struct device_node *np,
	struct i2c_board_info *bi,
	int *busnum)
{
	struct device_node *prfdev;
	phandle ph;
	const char *sname;
	int err;

	if (!np || !bi)
		return (void *)-EFAULT;

	dev_dbg(&dev->dev, "%s - %s\n", __func__, np->name);
	err = of_property_read_u32(np, "profile", &ph);
	if (err) {
		dev_err(&dev->dev, "%s no phandle in %s\n",
			__func__, np->name);
		return ERR_PTR(err);
	}
	prfdev = of_find_node_by_phandle(ph);
	if (!prfdev) {
		dev_err(&dev->dev, "%s can't find profile %d\n",
			__func__, ph);
		return (void *)-ENOENT;
	}
	sname = of_get_property(prfdev, "drivername", NULL);
	if (!sname) {
		dev_err(&dev->dev, "%s no devicename in %s\n",
			__func__, prfdev->name);
		return (void *)-ENOENT;
	}
	snprintf(bi->type, sizeof(bi->type), "%s", sname);
	if (of_property_read_u32(prfdev, "addr", (void *)&bi->addr)) {
		dev_err(&dev->dev, "%s missing addr in %s\n",
			__func__, prfdev->name);
		return (void *)-ENOENT;
	}
	if (busnum && of_property_read_u32(prfdev, "busnum", (void *)busnum)) {
		dev_err(&dev->dev, "%s missing busnum in %s\n",
			__func__, prfdev->name);
		return (void *)-ENOENT;
	}

	return prfdev;
}

struct camera_platform_data *of_camera_create_pdata(
	struct platform_device *dev)
{
	struct device_node *modules, *submod, *subdev, *pdev;
	struct camera_platform_data *pd = NULL;
	struct i2c_board_info *bi = NULL;
	struct camera_data_blob *cb;
	int num, nr;
	const char *sname;

	dev_dbg(&dev->dev, "%s\n", __func__);
	modules = of_find_node_by_name(dev->dev.of_node, "modules");
	if (!modules) {
		dev_info(&dev->dev, "module node not found\n");
		return (void *)-ENODEV;
	}

	num = of_camera_get_child_count(dev, modules);

	pd = kzalloc(sizeof(*pd) + (num + 1) * (sizeof(*pd->modules) +
		sizeof(struct i2c_board_info) * 3),
		GFP_KERNEL);
	if (!pd) {
		dev_err(&dev->dev, "%s allocate memory failed!\n", __func__);
		return (void *)-ENOMEM;
	}
	/* layout */
	pd->modules = (void *)&pd[1];
	pd->mod_num = num;
	bi = (void *)&pd->modules[num + 1];
	num = 0;
	for_each_child_of_node(modules, submod) {
		for_each_child_of_node(submod, subdev) {
			pdev = of_camera_get_brdinfo(dev, subdev, bi, &nr);
			if (IS_ERR(pdev))
				break;
			/* looking for matching pdata in lut */
			if (dev->dev.platform_data) {
				cb = dev->dev.platform_data;
				sname = of_get_property(
					subdev, "platformdata", NULL);
				/* searching for match signature */
				while (sname && cb && cb->name) {
					if (!strcmp(sname, cb->name)) {
						bi->platform_data = cb->data;
						break;
					}
					cb++;
				}
			}
			bi->of_node = pdev;

			sname = of_get_property(pdev, "chipname", NULL);
			if (!sname) {
				dev_err(&dev->dev, "%s no chipname in %s\n",
					__func__, pdev->name);
				break;
			}

			dev_dbg(&dev->dev,
				"    device %s, subdev %s @ %d-%04x %p, %s\n",
				subdev->name, bi->type, nr,
				bi->addr, bi->platform_data, sname);
			if (!strcmp(subdev->name, "sensor")) {
				pd->modules[num].sensor.bi = bi;
				pd->modules[num].sensor.chipname = sname;
				pd->modules[num].sensor.busnum = nr;
			} else if (!strcmp(subdev->name, "focuser")) {
				pd->modules[num].focuser.bi = bi;
				pd->modules[num].focuser.chipname = sname;
				pd->modules[num].focuser.busnum = nr;
			} else if (!strcmp(subdev->name, "flash")) {
				pd->modules[num].flash.bi = bi;
				pd->modules[num].flash.chipname = sname;
				pd->modules[num].flash.busnum = nr;
			}
			bi++;
		}
		pd->modules[num].of_node = submod;
		num++;
	}

	/* generic info */
	of_property_read_u32(dev->dev.of_node, "configuration", &pd->cfg);
	pd->max_blob_size = of_camera_profile_statistic(
		dev, &pd->of_profiles, &pd->prof_num);
	pd->lut = dev->dev.platform_data;
	pd->freeable = true;

	return pd;
}

int of_camera_init(struct camera_platform_info *info)
{
	cam_desc = info;
	return 0;
}

int of_camera_remove(void)
{
	return 0;
}
