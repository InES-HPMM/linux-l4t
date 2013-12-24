/*
 * arch/arm/mach-tegra/tegra_simon.h
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#define TEGRA_SIMON_GRADING_INTERVAL_SEC	5000000

struct tegra_simon_grader {
	enum tegra_simon_domain		domain;
	const char			*domain_name;
	ktime_t				last_grading;
	int				grade;
	int				grading_mv_limit;
	unsigned long			garding_rate_limit;
	struct work_struct		grade_update_work;
	struct notifier_block		grading_condition_nb;
	struct thermal_zone_device	*tzd;
	struct clk			*clk;

	int (*grade_simon_domain) (int domain, int mv, int temperature);
};

#ifdef CONFIG_TEGRA_USE_SIMON
int tegra_register_simon_notifier(struct notifier_block *nb);
void tegra_unregister_simon_notifier(struct notifier_block *nb);
#else
static inline int tegra_register_simon_notifier(struct notifier_block *nb)
{ return -ENOSYS; }
static inline void tegra_unregister_simon_notifier(struct notifier_block *nb)
{ }
#endif

#endif /* _MACH_TEGRA_SIMON_H_ */
