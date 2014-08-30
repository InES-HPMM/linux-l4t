/*
 * amc.h
 *
 * A header file for AMC/ARAM
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_NVADSP_AMC_H
#define __TEGRA_NVADSP_AMC_H

#define AMC_CONFIG			0x00
#define	  AMC_CONFIG_ALIASING		(1 << 0)
#define	  AMC_CONFIG_CARVEOUT		(1 << 1)
#define	  AMC_CONFIG_ERR_RESP		(1 << 2)
#define AMC_INT_STATUS			(0x04)
#define   AMC_INT_STATUS_ARAM		(1 << 0)
#define   AMC_INT_STATUS_REG		(1 << 1)
#define AMC_INT_MASK			0x08
#define AMC_INT_SET			0x0C
#define AMC_INT_CLR			0x10
#define	  AMC_INT_INVALID_ARAM_ACCESS	(1 << 0)
#define	  AMC_INT_INVALID_REG_ACCESS	(1 << 1)
#define AMC_ERROR_ADDR			0x14

#define AMC_ERROR_ADDR_IGNORE		SZ_4K

#endif /* __TEGRA_NVADSP_AMC_H */
