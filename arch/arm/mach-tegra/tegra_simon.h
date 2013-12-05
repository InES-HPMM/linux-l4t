/*
 * arch/arm/mach-tegra/tegra_simon.h
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _MACH_TEGRA_SIMON_H_
#define _MACH_TEGRA_SIMON_H_

enum tegra_simon_domain {
	TEGRA_SIMON_DOMAIN_NONE = 0,
	TEGRA_SIMON_DOMAIN_CPU,
	TEGRA_SIMON_DOMAIN_GPU,
	TEGRA_SIMON_DOMAIN_CORE,

	TEGRA_SIMON_DOMAIN_NUM,
};

int tegra_register_simon_notifier(struct notifier_block *nb);
void tegra_unregister_simon_notifier(struct notifier_block *nb);

#endif /* _MACH_TEGRA_SIMON_H_ */
