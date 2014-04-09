/*
 * os.h
 *
 * A header file containing data structures shared with ADSP OS
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
#ifndef __TEGRA_NVADSP_OS_H
#define __TEGRA_NVADSP_OS_H

int nvadsp_os_probe(struct platform_device *);
int adsp_add_load_mappings(phys_addr_t, void *, int);
void *get_mailbox_shared_region(void);

#endif /* __TEGRA_NVADSP_OS_H */
