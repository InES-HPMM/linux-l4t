/*
* tegra-xotg.h - Nvidia XOTG implementation
*
* Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include<linux/usb/otg.h>

extern struct usb_hcd *tegra_xhci_hcd;

/* nvidia otg controller Structure */
struct xotg {
	u8 hs_otg_port;
	struct usb_phy phy;
	struct device *dev;
	struct platform_device *pdev;
	spinlock_t lock;
	struct mutex vbus_lock;
	int nv_irq;
	int id;

	/* vbus */
	struct regulator *usb_vbus_reg;
	struct work_struct vbus_work;
	bool vbus_enabled;

	/* extcon */
	struct extcon_dev *id_extcon_dev;
	struct notifier_block id_extcon_nb;
	bool id_grounded;
};

extern int xotg_debug_level;
#define LEVEL_DEBUG	4
#define LEVEL_INFO	3
#define LEVEL_WARNING	2
#define LEVEL_ERROR	1

#define xotg_dbg(dev, fmt, args...) { \
	if (xotg_debug_level >= LEVEL_DEBUG) \
		dev_dbg(dev, "%s():%d: " fmt, __func__ , __LINE__, ## args); \
	}
#define xotg_info(dev, fmt, args...) { \
	if (xotg_debug_level >= LEVEL_INFO) \
		dev_info(dev, "%s():%d: " fmt, __func__ , __LINE__, ## args); \
	}
#define xotg_err(dev, fmt, args...) { \
	if (xotg_debug_level >= LEVEL_ERROR) \
		dev_err(dev, fmt, ## args); \
	}
#define xotg_warn(dev, fmt, args...) { \
	if (xotg_debug_level >= LEVEL_WARNING) \
		dev_warn(dev, fmt, ## args); \
	}

#define xotg_method_entry(dev) xotg_dbg(dev, "enter\n")
#define xotg_method_exit(dev) xotg_dbg(dev, "exit\n")
