/*
 * arch/arm/mach-tegra/cpuidle.h
 *
 * Declarations for power state transition code
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_CPUIDLE_H
#define __MACH_TEGRA_CPUIDLE_H

#include <linux/cpuidle.h>

#ifdef CONFIG_PM_SLEEP

extern int tegra_lp2_exit_latency;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
bool tegra2_idle_lp2(struct cpuidle_device *dev, struct cpuidle_state *state);
void tegra2_cpu_idle_stats_lp2_ready(unsigned int cpu);
void tegra2_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us);
bool tegra2_lp2_is_allowed(struct cpuidle_device *dev,
			struct cpuidle_state *state);
#ifdef CONFIG_DEBUG_FS
int tegra2_lp2_debug_show(struct seq_file *s, void *data);
#endif
#else
bool tegra3_idle_lp2(struct cpuidle_device *dev, struct cpuidle_state *state);
void tegra3_cpu_idle_stats_lp2_ready(unsigned int cpu);
void tegra3_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us);
bool tegra3_lp2_is_allowed(struct cpuidle_device *dev,
			   struct cpuidle_state *state);
int tegra3_cpuidle_init_soc(void);

bool tegra11x_idle_lp2(struct cpuidle_device *dev, struct cpuidle_state *state);
bool tegra11x_lp2_is_allowed(struct cpuidle_device *dev,
			   struct cpuidle_state *state);
#ifdef CONFIG_DEBUG_FS
int tegra3_lp2_debug_show(struct seq_file *s, void *data);
#endif
#endif

static inline int tegra_cpuidle_init_soc(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return 0;
#else
	return tegra3_cpuidle_init_soc();
#endif
}

static inline void tegra_cpu_idle_stats_lp2_ready(unsigned int cpu)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_cpu_idle_stats_lp2_ready(cpu);
#else
	tegra3_cpu_idle_stats_lp2_ready(cpu);
#endif
}

static inline void tegra_cpu_idle_stats_lp2_time(unsigned int cpu, s64 us)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tegra2_cpu_idle_stats_lp2_time(cpu, us);
#else
	tegra3_cpu_idle_stats_lp2_time(cpu, us);
#endif
}

static inline bool tegra_idle_lp2(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_idle_lp2(dev, state);
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
	return tegra3_idle_lp2(dev, state);
#else
	return tegra11x_idle_lp2(dev, state);
#endif
}

static inline bool tegra_lp2_is_allowed(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_is_allowed(dev, state);
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
	return tegra3_lp2_is_allowed(dev, state);
#else
	return tegra11x_lp2_is_allowed(dev, state);
#endif
}

static inline void tegra_lp2_set_global_latency(struct cpuidle_state *state)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	state->exit_latency = tegra_lp2_exit_latency;
#endif
	/* Tegra3 does not use global exit latency */
}

void tegra_lp2_update_target_residency(struct cpuidle_state *state);

#ifdef CONFIG_DEBUG_FS
static inline int tegra_lp2_debug_show(struct seq_file *s, void *data)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return tegra2_lp2_debug_show(s, data);
#else
	return tegra3_lp2_debug_show(s, data);
#endif
}
#endif
#endif /* CONFIG_PM_SLEEP */

#if defined(CONFIG_CPU_IDLE) && defined(CONFIG_PM_SLEEP)
void tegra_lp2_in_idle(bool enable);
#else
static inline void tegra_lp2_in_idle(bool enable) {}
#endif

#if defined(CONFIG_ARCH_TEGRA_HAS_SYMMETRIC_CPU_PWR_GATE)
int get_power_gating_partition(void);
#endif

#endif
