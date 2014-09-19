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
#include <linux/firmware.h>

#define CONFIG_ADSP_DRAM_LOG_WITH_TAG	1
#define CONFIG_USE_STATIC_APP_LOAD	0
#define CONFIG_SYSTEM_FPGA		1

#define SYM_NAME_SZ 128

#define APE_FPGA_MISC_RST_DEVICES 0x702dc800 /*1882048512*/
#define APE_RESET (1 << 6)

#define ADSP_SMMU_LOAD_ADDR	0x80300000
#define ADSP_APP_MEM_SMMU_ADDR	(ADSP_SMMU_LOAD_ADDR + SZ_8M)
#define ADSP_APP_MEM_SIZE	SZ_8M
#define ADSP_SMMU_SIZE		SZ_16M

#define AMC_EVP_RESET_VEC_0		0x700
#define AMC_EVP_UNDEF_VEC_0		0x704
#define AMC_EVP_SWI_VEC_0		0x708
#define AMC_EVP_PREFETCH_ABORT_VEC_0	0x70c
#define AMC_EVP_DATA_ABORT_VEC_0	0x710
#define AMC_EVP_RSVD_VEC_0		0x714
#define AMC_EVP_IRQ_VEC_0		0x718
#define AMC_EVP_FIQ_VEC_0		0x71c
#define AMC_EVP_RESET_ADDR_0		0x720
#define AMC_EVP_UNDEF_ADDR_0		0x724
#define AMC_EVP_SWI_ADDR_0		0x728
#define AMC_EVP_PREFETCH_ABORT_ADDR_0	0x72c
#define AMC_EVP_DATA_ABORT_ADDR_0	0x730
#define AMC_EVP_RSVD_ADDR_0		0x734
#define AMC_EVP_IRQ_ADDR_0		0x738
#define AMC_EVP_FIQ_ADDR_0		0x73c

#define AMC_EVP_SIZE (AMC_EVP_FIQ_ADDR_0 - AMC_EVP_RESET_VEC_0 + 4)
#define AMC_EVP_WSIZE (AMC_EVP_SIZE >> 2)

#define OS_LOAD_TIMEOUT		5000 /* ms */
#define ADSP_COM_MBOX_ID	2

enum adsp_os_cmd {
	ADSP_OS_SUSPEND,
};

/**
 * struct global_sym_info - Global Symbol information required by app loader.
 * @name:	Name of the symbol
 * @addr:	Address of the symbol
 * @info:	Type and binding attributes
 */
struct global_sym_info {
	char name[SYM_NAME_SZ];
	uint32_t addr;
	unsigned char info;
};

struct app_mem_size {
	uint64_t dram;
	uint64_t dram_shared;
	uint64_t dram_shared_wc;
	uint64_t aram;
	uint64_t aram_x;
};

struct adsp_module {
	const char			*name;
	void				*handle;
	void				*module_ptr;
	uint32_t			adsp_module_ptr;
	size_t				size;
	const struct app_mem_size	mem_size;
};

int nvadsp_os_probe(struct platform_device *);
int nvadsp_app_module_probe(struct platform_device *);
int adsp_add_load_mappings(phys_addr_t, void *, int);
void *get_mailbox_shared_region(void);
struct elf32_shdr *nvadsp_get_section(const struct firmware *, char *);
struct global_sym_info *find_global_symbol(const char *);
void update_nvadsp_app_shared_ptr(void *);

struct adsp_module
*load_adsp_module(const char *, const char *, struct device *);
void unload_adsp_module(struct adsp_module *);

int allocate_memory_from_adsp(void **, unsigned int);
bool is_adsp_dram_addr(u64);
int wait_for_adsp_os_load_complete(void);
#endif /* __TEGRA_NVADSP_OS_H */
