/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license as published by
 * the free software foundation; either version 2 of the license, or
 * (at your option) any later version.
 *
 * this program is distributed in the hope that it will be useful, but without
 * any warranty; without even the implied warranty of merchantability or
 * fitness for a particular purpose.  see the gnu general public license for
 * more details.
 *
 * you should have received a copy of the gnu general public license along
 * with this program; if not, write to the free software foundation, inc.,
 * 51 franklin street, fifth floor, boston, ma  02110-1301, usa.
 */

#include <linux/tegra-soc.h>
#include "fuse.h"

#ifndef __TEGRA21x_FUSE_OFFSETS_H
#define __TEGRA21x_FUSE_OFFSETS_H

/* private_key4 */
#define DEVKEY_START_OFFSET			0x2A
#define DEVKEY_START_BIT			12

/* arm_debug_dis */
#define JTAG_START_OFFSET		0x0
#define JTAG_START_BIT			3

/* security_mode */
#define ODM_PROD_START_OFFSET		0x0
#define ODM_PROD_START_BIT		11

/* boot_device_info */
#define SB_DEVCFG_START_OFFSET		0x2C
#define SB_DEVCFG_START_BIT		20

/* reserved_sw[2:0] */
#define SB_DEVSEL_START_OFFSET		0x2C
#define SB_DEVSEL_START_BIT		28

/* private_key0 -> private_key3 (SBK) */
#define SBK_START_OFFSET	0x22
#define SBK_START_BIT		20

/* reserved_sw[7:4] */
#define SW_RESERVED_START_OFFSET	0x2E
#define SW_RESERVED_START_BIT		4
#define SW_RESERVED_SIZE_BITS		12

/* reserved_sw[3] */
#define IGNORE_DEVSEL_START_OFFSET	0x2E
#define IGNORE_DEVSEL_START_BIT	7

/* public key */
#define PUBLIC_KEY_START_OFFSET	0xC
#define PUBLIC_KEY_START_BIT		6

/* pkc_disable */
#define PKC_DISABLE_START_OFFSET	0x52
#define PKC_DISABLE_START_BIT		7

/* video vp8 enable */
#define VP8_ENABLE_START_OFFSET	0x2E
#define VP8_ENABLE_START_BIT		4

/* odm lock */
#define ODM_LOCK_START_OFFSET		0x0
#define ODM_LOCK_START_BIT		6

/* reserved_odm0 -> reserved_odm7 */
#define ODM_RESERVED_DEVSEL_START_OFFSET	0x2E
#define ODM_RESERVED_START_BIT			17

#define FUSE_VENDOR_CODE		0x200
#define FUSE_VENDOR_CODE_MASK		0xf
#define FUSE_FAB_CODE			0x204
#define FUSE_FAB_CODE_MASK		0x3f
#define FUSE_LOT_CODE_0		0x208
#define FUSE_LOT_CODE_1		0x20c
#define FUSE_WAFER_ID			0x210
#define FUSE_WAFER_ID_MASK		0x3f
#define FUSE_X_COORDINATE		0x214
#define FUSE_X_COORDINATE_MASK		0x1ff
#define FUSE_Y_COORDINATE		0x218
#define FUSE_Y_COORDINATE_MASK		0x1ff
#define FUSE_GPU_INFO			0x390
#define FUSE_GPU_INFO_MASK		(1<<2)
#define FUSE_SPARE_BIT			0x380

/* fuse registers used in public fuse data read API */
#define FUSE_FT_REV			0x128
#define FUSE_CP_REV			0x190
/* fuse spare bits are used to get Tj-ADT values */
#define NUM_TSENSOR_SPARE_BITS		28
/* tsensor calibration register */
#define FUSE_TSENSOR_CALIB_0		0x198
/* sparse realignment register */
#define FUSE_SPARE_REALIGNMENT_REG_0	0x37c
/* tsensor8_calib */
#define FUSE_TSENSOR_CALIB_8		0x280

#define FUSE_BASE_CP_SHIFT		0
#define FUSE_BASE_CP_MASK		0x3ff
#define FUSE_BASE_FT_SHIFT		10
#define FUSE_BASE_FT_MASK		0x7ff
#define FUSE_SHIFT_CP_SHIFT		0
#define FUSE_SHIFT_CP_MASK		0x3f
#define FUSE_SHIFT_CP_BITS		6
#define FUSE_SHIFT_FT_SHIFT		21
#define FUSE_SHIFT_FT_MASK		0x1f
#define FUSE_SHIFT_FT_BITS		5

#define TEGRA_FUSE_SUPPLY		"dummy"
#define PGM_TIME_US		12

DEVICE_ATTR(public_key, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(pkc_disable, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(vp8_enable, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(odm_lock, 0440, tegra_fuse_show, tegra_fuse_store);

int tegra_fuse_add_sysfs_variables(struct platform_device *pdev,
					bool odm_security_mode)
{
	dev_attr_odm_lock.attr.mode = 0640;
	if (odm_security_mode) {
		dev_attr_public_key.attr.mode =  0440;
		dev_attr_pkc_disable.attr.mode = 0440;
		dev_attr_vp8_enable.attr.mode = 0440;
	} else {
		dev_attr_public_key.attr.mode =  0640;
		dev_attr_pkc_disable.attr.mode = 0640;
		dev_attr_vp8_enable.attr.mode = 0640;
	}
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_public_key.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_pkc_disable.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_vp8_enable.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_odm_lock.attr));

	return 0;
}

int tegra_fuse_rm_sysfs_variables(struct platform_device *pdev)
{
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_public_key.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_pkc_disable.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_vp8_enable.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_odm_lock.attr);

	return 0;
}

int tegra_fuse_ch_sysfs_perm(struct device *dev, struct kobject *kobj)
{
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_public_key.attr, 0440));
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_pkc_disable.attr, 0440));
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_vp8_enable.attr, 0440));

	return 0;
}
#endif /* __TEGRA21x_FUSE_OFFSETS_H */
