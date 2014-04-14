/*
 * include/linux/tegra-pm.h
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _LINUX_TEGRA_PM_H_
#define _LINUX_TEGRA_PM_H_

#define TEGRA_PM_SUSPEND	0x0001
#define TEGRA_PM_RESUME		0x0002
 
enum tegra_suspend_mode {
	TEGRA_SUSPEND_NONE = 0,
	TEGRA_SUSPEND_LP2,	/* CPU voltage off */
	TEGRA_SUSPEND_LP1,	/* CPU voltage off, DRAM self-refresh */
	TEGRA_SUSPEND_LP0,      /* CPU + core voltage off, DRAM self-refresh */
	TEGRA_MAX_SUSPEND_MODE,
};

int tegra_suspend_dram(enum tegra_suspend_mode mode, unsigned int flags);

int tegra_register_pm_notifier(struct notifier_block *nb);
int tegra_unregister_pm_notifier(struct notifier_block *nb);

#ifdef CONFIG_TEGRA_LP0_IN_IDLE
int tegra_enter_lp0(unsigned long sleep_time);
#else
static inline int tegra_enter_lp0(unsigned long sleep_time)
{ return 0; }
#endif

#endif /* _LINUX_TEGRA_PM_H_ */
