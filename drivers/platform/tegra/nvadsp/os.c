/*
 * os.c
 *
 * ADSP OS management
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/tegra_nvadsp.h>
#include <linux/elf.h>

#include "os.h"

#define APE_FPGA_MISC_RST_DEVICES 0x702dc800 /*1882048512*/
#define APE_RESET (1 << 6)

#define ADSP_SMMU_LOAD_ADDR	0x80300000
#define ADSP_SMMU_SIZE		SZ_8M

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

#define NVADSP_ELF "adsp.elf"
#define NVADSP_FIRMWARE NVADSP_ELF

#define MAILBOX_REGION ".mbox_shared_data"

/* Maximum number of LOAD MAPPINGS supported */
#define NM_LOAD_MAPPINGS 20

struct nvadsp_os_data {
	void __iomem *reset_reg;
	struct platform_device *pdev;
};

static struct nvadsp_os_data priv;

struct nvadsp_mappings {
	phys_addr_t da;
	void *va;
	int len;
};

static struct nvadsp_mappings adsp_map[NM_LOAD_MAPPINGS];
static int map_idx;

int adsp_add_load_mappings(phys_addr_t pa, void *mapping, int len)
{
	if (map_idx >= NM_LOAD_MAPPINGS)
		return -EINVAL;

	adsp_map[map_idx].da = pa;
	adsp_map[map_idx].va = mapping;
	adsp_map[map_idx].len = len;
	map_idx++;
	return 0;
}

void *adsp_da_to_va_mappings(u64 da, int len)
{
	void *ptr = NULL;
	int i;

	for (i = 0; i < map_idx; i++) {
		int offset = da - adsp_map[i].da;

		/* try next carveout if da is too small */
		if (offset < 0)
			continue;

		/* try next carveout if da is too large */
		if (offset + len > adsp_map[i].len)
			continue;

		ptr = adsp_map[i].va + offset;
		break;
	}
	return ptr;
}

static struct elf32_shdr *
get_section(const struct firmware *fw, char *sec_name)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	const u8 *elf_data = fw->data;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)elf_data;
	struct elf32_shdr *shdr;
	const char *name_table;

	/* look for the resource table and handle it */
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	name_table = elf_data + shdr[ehdr->e_shstrndx].sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++, shdr++)
		if (!strcmp(name_table + shdr->sh_name, sec_name)) {
			dev_dbg(dev, "found the section %s\n",
					name_table + shdr->sh_name);
			return shdr;
		}
	return NULL;
}

void *get_mailbox_shared_region(void)
{
	const struct firmware *fw;
	struct device *dev = &priv.pdev->dev;
	struct elf32_shdr *shdr;
	int addr;
	int size;
	int ret;

	if (!dev) {
		pr_info("ADSP Driver is not initialized\n");
		return ERR_PTR(-EINVAL);
	}

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_info(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		return ERR_PTR(ret);
	}

	shdr = get_section(fw, MAILBOX_REGION);
	if (!shdr) {
		dev_info(dev, "section %s not found\n", MAILBOX_REGION);
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev,
		"the section is present at 0x%x\n", shdr->sh_addr);
	addr = shdr->sh_addr;
	size = shdr->sh_size;
	return adsp_da_to_va_mappings(addr, size);
}

int nvadsp_os_elf_load(const struct firmware *fw)
{
	struct device *dev = &priv.pdev->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		void *va;
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;

		if (phdr->p_type != PT_LOAD)
			continue;


		dev_info(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
					phdr->p_type, da, memsz, filesz);

		va = adsp_da_to_va_mappings(da, filesz);
		if (!va) {
			dev_err(dev, "no va for da 0x%x filesz 0x%x\n",
							da, filesz);
			ret = -EINVAL;
			break;
		}

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
							filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
					offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (filesz)
			memcpy(va, elf_data + offset, filesz);
	}

	return ret;
}

static int allocate_memeory_for_adsp_os(void)
{
	struct platform_device *pdev = priv.pdev;
	struct device *dev = &pdev->dev;
	void *dram_va;
	size_t size;
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	dma_addr_t addr = ADSP_SMMU_LOAD_ADDR;

	size = ADSP_SMMU_SIZE;
	dram_va = dma_alloc_at_coherent(dev, size, &addr, GFP_KERNEL);
	if (!dram_va) {
		dev_info(dev, "unable to allocate SMMU pages\n");
		return PTR_ERR(dram_va);
	}
#else
	phys_addr_t addr;
	struct nvadsp_platform_data *plat_data = pdev->dev.platform_data;

	if (IS_ERR_OR_NULL(plat_data)) {
		dev_info(dev, "carvout is NULL\n");
		return PTR_ERR(plat_data);
	}

	addr = plat_data->co_pa;
	size = plat_data->co_size;
	dram_va = ioremap_nocache(addr, plat_data->co_size);
	if (!dram_va) {
		dev_info(dev, "remap failed for addr %lx\n",
					(long)plat_data->co_pa);
		return PTR_ERR(dram_va);
	}
#endif
	adsp_add_load_mappings(addr, dram_va, size);
	return 0;
}

int nvadsp_os_load(void)
{
	const struct firmware *fw;
	int ret;
	struct device *dev = &priv.pdev->dev;

	if (!dev) {
		pr_info("ADSP Driver is not initialized\n");
		ret = -EINVAL;
		goto end;
	}

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_info(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		goto end;
	}

	ret = allocate_memeory_for_adsp_os();
	if (ret < 0) {
		dev_info(dev,
			"unable to allocate memory for adsp os\n");
		goto end;
	}

	dev_info(dev, "Loading ADSP OS firmware %s\n",
						NVADSP_FIRMWARE);

	ret = nvadsp_os_elf_load(fw);
	if (ret)
		dev_info(dev, "failed to load %s\n", NVADSP_FIRMWARE);
end:
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_load);

int nvadsp_os_start(void)
{
	struct device *dev = &priv.pdev->dev;
	if (!dev) {
		pr_info("ADSP Driver is not initialized\n");
		return -EINVAL;
	}

	dev_info(dev, "starting ADSP OS ....\n");
	writel(APE_RESET, priv.reset_reg);
	return 0;
}
EXPORT_SYMBOL(nvadsp_os_start);

int nvadsp_os_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	priv.reset_reg = ioremap(APE_FPGA_MISC_RST_DEVICES, 1);
	if (!priv.reset_reg) {
		dev_info(dev, "unable to map reset addr\n");
		return -EINVAL;
	}

	priv.pdev = pdev;

	return 0;
}
