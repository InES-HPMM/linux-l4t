/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA_USB_CHARGER_H
#define __TEGRA_USB_CHARGER_H

/**
 * Tegra USB charger opaque handle
 */
struct tegra_usb_cd;

/**
 * Tegra USB different connection types
 */
enum tegra_usb_connect_type {
	CONNECT_TYPE_NONE,
	CONNECT_TYPE_SDP,
	CONNECT_TYPE_DCP,
	CONNECT_TYPE_DCP_MAXIM,
	CONNECT_TYPE_DCP_QC2,
	CONNECT_TYPE_CDP,
	CONNECT_TYPE_NV_CHARGER,
	CONNECT_TYPE_NON_STANDARD_CHARGER,
	CONNECT_TYPE_APPLE_500MA,
	CONNECT_TYPE_APPLE_1000MA,
	CONNECT_TYPE_APPLE_2000MA
};

/**
 * Returns USB charger detection handle.
 */
struct tegra_usb_cd *tegra_usb_get_ucd(void);

/**
 * Detects the USB charger and returns the type.
 */
enum tegra_usb_connect_type
	tegra_ucd_detect_cable_and_set_current(struct tegra_usb_cd *ucd);

/**
 * Set USB charging current.
 */
void tegra_ucd_set_current(struct tegra_usb_cd *ucd, int current_ma);

#endif /* __TEGRA_USB_CHARGER_H */
