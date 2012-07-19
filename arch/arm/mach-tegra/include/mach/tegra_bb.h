/*
 * arch/arm/mach-tegra/include/mach/tegra_bb.h
 *
 * Copyright (C) 2012 NVIDIA Corporation.
 *
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

#ifndef __MACH_TEGRA_BB_H
#define __MACH_TEGRA_BB_H

/* define to enable simulated shared memory on test platform */

struct tegra_bb_platform_data {
	phys_addr_t ipc_base;
	size_t ipc_size;
	phys_addr_t priv_base;
	size_t priv_size;
	/* Baseband->AP IRQ */
	unsigned int bb_irq;
	void *bb_handle;
};

/*
 * tegra_bb_register_ipc: register callback for IPC notification
 * @param struct tegra_bb device pointer
 * @param void (*cb)(voif *data) callback function
 * @param void * callback data
 * @return none
 */
void tegra_bb_register_ipc(struct platform_device *dev,
			   void (*cb)(void *data), void *cb_data);
/*
 * tegra_bb_generate_ipc: generate IPC
 * @param struct tegra_bb device pointer
 * @return none
 */
void tegra_bb_generate_ipc(struct platform_device *dev);
#endif
