/*
 * dev.h
 *
 * A header file for Host driver for ADSP and APE
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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

#ifndef __TEGRA_NVADSP_DEV_H
#define __TEGRA_NVADSP_DEV_H

#include <linux/tegra_nvadsp.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/debugfs.h>

#include "hwmailbox.h"
#include "amc.h"
#include "os.h"

/*
 * Note: These enums should be aligned to the regs mentioned in the
 * device tree
*/
enum {
	AMC,
	AMISC,
	ABRIDGE,
	APE_MAX_REG
};

enum {
	ADSP_DRAM1,
	ADSP_DRAM2,
	ADSP_MAX_DRAM_MAP
};

#define AMISC_REGS	0x2000

#define AMISC_ADSP_L2_REGFILEBASE	0x10
#define AMISC_SHRD_SMP_STA		0x14
#define AMISC_SEM_REG_START		0x1c
#define AMISC_SEM_REG_END		0x44
#define AMISC_TSC			0x48
#define AMISC_ACTMON_AVG_CNT		0x81c

#define AMISC_REG_START_OFFSET		0x0
#define AMISC_REG_MBOX_OFFSET		0x64
#define ADSP_ACTMON_REG_START_OFFSET	0x800
#define ADSP_ACTMON_REG_END_OFFSET	0x828


struct nvadsp_pm_state {
	u32 aram[AMC_ARAM_WSIZE];
	uint32_t amc_regs[AMC_REGS];
	uint32_t amisc_regs[AMISC_REGS];
	u32 evp[AMC_EVP_WSIZE];
	void *evp_ptr;
};

struct nvadsp_drv_data {
	void __iomem **base_regs;
	void __iomem **base_regs_saved;
	struct resource *dram_region[ADSP_MAX_DRAM_MAP];
	struct hwmbox_queue hwmbox_send_queue;
	int hwmbox_send_virq;
	int hwmbox_recv_virq;

	struct nvadsp_mbox **mboxes;
	unsigned long *mbox_ids;
	spinlock_t mbox_lock;

#if CONFIG_DEBUG_FS
	struct dentry *adsp_debugfs_root;
#endif
	struct clk *ape_clk;
	struct clk *adsp_clk;
	struct clk *adsp_cpu_clk;
	struct clk *uartape_clk;
	struct clk *ahub_clk;
	long max_adsp_freq;

	struct nvadsp_pm_state state;
	bool adsp_os_running;
	bool adsp_os_suspended;
	void *shared_adsp_os_data;

#ifdef CONFIG_TEGRA_ADSP_DFS
	bool dfs_initialized;
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	bool actmon_initialized;
#endif
};

#define ADSP_CONFIG	0x04
#define MAXCLKLATENCY	(3 << 8)
#define UART_BAUD_RATE	9600

status_t nvadsp_mbox_init(struct platform_device *pdev);
status_t nvadsp_amc_init(struct platform_device *pdev);

#ifdef CONFIG_TEGRA_ADSP_DFS
void adsp_cpu_set_rate(unsigned long freq);
int adsp_dfs_core_init(struct platform_device *pdev);
int adsp_dfs_core_exit(struct platform_device *pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
int ape_actmon_probe(struct platform_device *pdev);
#endif

#ifdef CONFIG_TEGRA_EMC_APE_DFS
status_t emc_dfs_init(struct platform_device *pdev);
void emc_dfs_exit(void);
#endif

#endif /* __TEGRA_NVADSP_DEV_H */
