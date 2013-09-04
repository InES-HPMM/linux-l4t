/*
 * arch/arm/mach-tegra/tegra12_speedo.c
 *
 * Copyright (C) 2013 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/bug.h>			/* For BUG_ON.  */

#include <mach/tegra_fuse.h>
#include <linux/tegra-soc.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "fuse.h"
#include "iomap.h"

#define TEGRA124_CPU_SPEEDO 2271 /* FIXME: Get Correct Value */

#define GPU_PROCESS_CORNERS_NUM		2

#define FUSE_CPU_SPEEDO_0	0x114
#define FUSE_CPU_SPEEDO_1	0x12c
#define FUSE_CPU_SPEEDO_2	0x130
#define FUSE_SOC_SPEEDO_0	0x134
#define FUSE_SOC_SPEEDO_1	0x138
#define FUSE_SOC_SPEEDO_2	0x13c
#define FUSE_CPU_IDDQ		0x118
#define FUSE_SOC_IDDQ		0x140
#define FUSE_GPU_IDDQ		0x228
#define FUSE_FT_REV		0x128

static int threshold_index;
static int cpu_process_id;
static int core_process_id;
static int gpu_process_id;
static int cpu_speedo_id;
static int cpu_speedo_value;
static int soc_speedo_id;
static int gpu_speedo_id;
static int package_id;
static int cpu_iddq_value;
static int gpu_iddq_value;
static int soc_iddq_value;

static int cpu_speedo_0_value;
static int cpu_speedo_1_value;
static int soc_speedo_0_value;
static int soc_speedo_1_value;
static int soc_speedo_2_value;

static int gpu_speedo_value;

static int enable_app_profiles;

static const u32 gpu_process_speedos[][GPU_PROCESS_CORNERS_NUM] = {
/* proc_id  0,	1 */
	{1950,	UINT_MAX}, /* [0]: threshold_index 0 */
	{0,	UINT_MAX}, /* [1]: threshold_index 0 */
};

void tegra_init_speedo_data(void)
{
	int i;

	if (!tegra_platform_is_silicon()) {
		cpu_process_id  =  0;
		core_process_id =  0;
		gpu_process_id  = 0;
		cpu_speedo_id   = 0;
		soc_speedo_id   = 0;
		gpu_speedo_id   = 0;
		package_id = -1;
		cpu_speedo_value = 1777;
		cpu_speedo_0_value = 0;
		cpu_speedo_1_value = 0;
		soc_speedo_0_value = 0;
		soc_speedo_1_value = 0;
		soc_speedo_2_value = 0;
		soc_iddq_value = 0;
		gpu_iddq_value = 0;
		return;
	}

	cpu_speedo_value = TEGRA124_CPU_SPEEDO;
	cpu_speedo_0_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_0);
	cpu_speedo_1_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_1);

	/* GPU Speedo is stored in CPU_SPEEDO_2 */
	gpu_speedo_value = tegra_fuse_readl(FUSE_CPU_SPEEDO_2);

	soc_speedo_0_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_0);
	soc_speedo_1_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_1);
	soc_speedo_2_value = tegra_fuse_readl(FUSE_SOC_SPEEDO_2);

	cpu_iddq_value = tegra_fuse_readl(FUSE_CPU_IDDQ);
	soc_iddq_value = tegra_fuse_readl(FUSE_SOC_IDDQ);
	gpu_iddq_value = tegra_fuse_readl(FUSE_GPU_IDDQ);

	for (i = 0; i < GPU_PROCESS_CORNERS_NUM; i++) {
		if (gpu_speedo_value <
			gpu_process_speedos[threshold_index][i]) {
			break;
		}
	}

	gpu_process_id = i;
	pr_info("Tegra12: GPU Speedo %d", gpu_speedo_value);

	/* cpu_speedo_value = TEGRA124_CPU_SPEEDO; */
	cpu_speedo_value = cpu_speedo_0_value;

	if (cpu_speedo_value > 2200)
		cpu_process_id = 1;
	else
		cpu_process_id = 0;

	pr_info("Tegra12: CPU Speedo ID %d, Soc Speedo ID %d, Gpu Speedo ID %d",
		cpu_speedo_id, soc_speedo_id, gpu_speedo_id);

	if (cpu_speedo_value == 0) {
		cpu_speedo_value = 1900;
		pr_warn("Tegra12: Warning: Speedo value not fused. PLEASE FIX!!!!!!!!!!!\n");
		pr_warn("Tegra12: Warning: PLEASE USE BOARD WITH FUSED SPEEDO VALUE !!!!\n");
	}
}

int tegra_cpu_process_id(void)
{
	return cpu_process_id;
}

int tegra_core_process_id(void)
{
	return core_process_id;
}

int tegra_gpu_process_id(void)
{
	return gpu_process_id;
}

int tegra_cpu_speedo_id(void)
{
	return cpu_speedo_id;
}

int tegra_soc_speedo_id(void)
{
	return soc_speedo_id;
}

int tegra_gpu_speedo_id(void)
{
	return gpu_speedo_id;
}

int tegra_package_id(void)
{
	return package_id;
}

int tegra_cpu_speedo_value(void)
{
	return cpu_speedo_value;
}

int tegra_cpu_speedo_0_value(void)
{
	return cpu_speedo_0_value;
}

int tegra_cpu_speedo_1_value(void)
{
	return cpu_speedo_1_value;
}

int tegra_gpu_speedo_value(void)
{
	return gpu_speedo_value;
}

int tegra_soc_speedo_0_value(void)
{
	return soc_speedo_0_value;
}

int tegra_soc_speedo_1_value(void)
{
	return soc_speedo_1_value;
}

int tegra_soc_speedo_2_value(void)
{
	return soc_speedo_2_value;
}
/*
 * CPU and core nominal voltage levels as determined by chip SKU and speedo
 * (not final - can be lowered by dvfs tables and rail dependencies; the
 * latter is resolved by the dvfs code)
 */
int tegra_cpu_speedo_mv(void)
{
	/* Not applicable on Tegra12 */
	return -ENOSYS;
}

int tegra_core_speedo_mv(void)
{
	switch (soc_speedo_id) {
	case 0:
		return 1100;
	default:
		BUG();
	}
}

int tegra_get_cpu_iddq_value(void)
{
	return cpu_iddq_value;
}

int tegra_get_soc_iddq_value(void)
{
	return soc_iddq_value;
}

int tegra_get_gpu_iddq_value(void)
{
	return gpu_iddq_value;
}

static int get_enable_app_profiles(char *val, const struct kernel_param *kp)
{
	return param_get_uint(val, kp);
}

static struct kernel_param_ops tegra_profiles_ops = {
	.get = get_enable_app_profiles,
};

module_param_cb(tegra_enable_app_profiles,
	&tegra_profiles_ops, &enable_app_profiles, 0444);
