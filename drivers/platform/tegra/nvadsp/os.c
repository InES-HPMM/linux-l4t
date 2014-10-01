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
#include <linux/tegra-soc.h>
#include <linux/elf.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <asm-generic/uaccess.h>

#include "ape_actmon.h"
#include "os.h"
#include "dev.h"
#include "dram_app_mem_manager.h"

#define NVADSP_ELF "adsp.elf"
#define NVADSP_FIRMWARE NVADSP_ELF

#define MAILBOX_REGION		".mbox_shared_data"
#define DEBUG_RAM_REGION	".debug_mem_logs"

/* Maximum number of LOAD MAPPINGS supported */
#define NM_LOAD_MAPPINGS 20

#define EOT	0x04 /* End of Transmission */
#define SOH	0x01 /* Start of Header */

#define ADSP_TAG	"\n[ADSP OS]"

#define UART_BAUD_RATE	9600

/* Intiialize with FIXED rate, once OS boots up DFS will set required freq */
#define ADSP_TO_APE_CLK_RATIO	2
/* 13.5 MHz, should be changed at bringup time */
#define APE_CLK_FIX_RATE	13500
/*
 * ADSP CLK = APE_CLK * ADSP_TO_APE_CLK_RATIO
 * or
 * ADSP CLK = APE_CLK >> ADSP_TO_APE_CLK_RATIO
 */
#define ADSP_CLK_FIX_RATE (APE_CLK_FIX_RATE * ADSP_TO_APE_CLK_RATIO)

/* total number of crashes allowed on adsp */
#define ALLOWED_CRASHES	2

#define DISABLE_MBOX2_EMPTY_INT	0xFFFFFFFF
#define ENABLE_MBOX2_EMPTY_INT	0x0

#define LOGGER_TIMEOUT	20 /* in ms */

#define LOAD_ADSP_FREQ 51200000lu /* in Hz */

struct nvadsp_debug_log {
	struct device *dev;
	char *debug_ram_rdr;
	int debug_ram_sz;
	int ram_iter;
};

struct nvadsp_os_data {
#if !CONFIG_SYSTEM_FPGA
	void __iomem		*reset_reg;
#endif
	const struct firmware	*os_firmware;
	struct platform_device	*pdev;
	struct global_sym_info	*adsp_glo_sym_tbl;
	void __iomem		*misc_base;
	struct resource		**dram_region;
	struct nvadsp_debug_log	logger;
	struct work_struct	restart_os_work;
	int			adsp_num_crashes;
};

static struct nvadsp_os_data priv;

struct nvadsp_mappings {
	phys_addr_t da;
	void *va;
	int len;
};

static struct nvadsp_mappings adsp_map[NM_LOAD_MAPPINGS];
static int map_idx;
static struct nvadsp_mbox adsp_com_mbox;
#ifdef CONFIG_TEGRA_ADSP_DFS
static bool is_dfs_initialized;
#endif
#ifdef CONFIG_TEGRA_ADSP_ACTMON
static bool is_actmon_initialized;
#endif

DECLARE_COMPLETION(entered_wfe);
#ifdef CONFIG_DEBUG_FS

static int adsp_logger_open(struct inode *inode, struct file *file)
{
	char *start;
	struct nvadsp_debug_log *logger = inode->i_private;

	/* loop till writer is initilized with SOH */
	do {
		msleep(20);
		if (!IS_ERR_OR_NULL(logger->debug_ram_rdr))
			start = strchr(logger->debug_ram_rdr, SOH);
	} while (!start);

	/* maxdiff can be 0, therefore valid */
	logger->ram_iter = start - logger->debug_ram_rdr;

	file->private_data = logger;

	return 0;
}

static int adsp_logger_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t adsp_logger_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	char last_char;
	ssize_t return_char = 1;
	struct nvadsp_debug_log *logger = file->private_data;
	struct device *dev = logger->dev;
loop:
	last_char = logger->debug_ram_rdr[logger->ram_iter];

	if ((last_char != EOT) && (last_char != 0)) {
#if CONFIG_ADSP_DRAM_LOG_WITH_TAG
		if ((last_char == '\n') || (last_char == '\r')) {

			if (copy_to_user(buf, ADSP_TAG, sizeof(ADSP_TAG) - 1)) {
				dev_err(dev, "%s failed\n", __func__);
				return -EFAULT;
			}
			return_char = sizeof(ADSP_TAG) - 1;

		} else
#endif
		if (copy_to_user(buf, &last_char, 1)) {
			dev_err(dev, "%s failed\n", __func__);
			return -EFAULT;
		}

		logger->ram_iter =
			(logger->ram_iter + 1) % logger->debug_ram_sz;
		return return_char;
	}

	schedule_timeout_interruptible(msecs_to_jiffies(LOGGER_TIMEOUT));
	goto loop;
}

static const struct file_operations adsp_logger_operations = {
	.read		= adsp_logger_read,
	.open		= adsp_logger_open,
	.release	= adsp_logger_release,
	.llseek		= generic_file_llseek,
};

static int adsp_create_debug_logger(struct dentry *adsp_debugfs_root)
{
	int ret = 0;
	struct device *dev = &priv.pdev->dev;

	if (IS_ERR_OR_NULL(adsp_debugfs_root)) {
		ret = -ENOENT;
		goto err_out;
	}

	if (!debugfs_create_file("adsp_logger", S_IRUGO,
					adsp_debugfs_root, &priv.logger,
					&adsp_logger_operations)) {
		dev_err(dev,
		"unable to create adsp logger debug fs file\n");
		ret = -ENOENT;
	}

err_out:
	return ret;
}
#endif

bool is_adsp_dram_addr(u64 addr)
{
	int i;
	struct resource **dram = priv.dram_region;

	for (i = 0; i < ADSP_MAX_DRAM_MAP; i++) {
		if ((addr >= dram[i]->start) &&
				(addr <= dram[i]->end)) {
			return true;
		}
	}
	return false;
}

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

void *nvadsp_da_to_va_mappings(u64 da, int len)
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
EXPORT_SYMBOL(nvadsp_da_to_va_mappings);

void *nvadsp_alloc_coherent(size_t size, dma_addr_t *da, gfp_t flags)
{
	struct device *dev;
	void *va = NULL;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		goto end;
	}

	dev = &priv.pdev->dev;
	va = dma_alloc_coherent(dev, size, da, flags);
	if (!va) {
		dev_err(dev,
			"unable to allocate the memory for size %u\n",
							(u32)size);
		goto end;
	}
	WARN(!is_adsp_dram_addr(*da),
			"bus addr %llx beyond %x\n", *da, UINT_MAX);
end:
	return va;
}
EXPORT_SYMBOL(nvadsp_alloc_coherent);

void nvadsp_free_coherent(size_t size, void *va, dma_addr_t da)
{
	struct device *dev;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}
	dev = &priv.pdev->dev;
	dma_free_coherent(dev, size, va, da);
}
EXPORT_SYMBOL(nvadsp_free_coherent);

struct elf32_shdr *
nvadsp_get_section(const struct firmware *fw, char *sec_name)
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

static inline void dump_global_symbol_table(void)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (!table) {
		dev_err(dev, "no table not created\n");
		return;
	}
	num_ent = table[0].addr;
	dev_info(dev,
	    "total number of entries in global symbol table %d\n", num_ent);

	pr_info("NAME ADDRESS TYPE\n");
	for (i = 1; i < num_ent; i++)
		pr_info("%s %x %s\n", table[i].name, table[i].addr,
				ELF32_ST_TYPE(table[i].info) == STT_FUNC ?
						    "STT_FUNC" : "STT_OBJECT");
}

static int
create_global_symbol_table(const struct firmware *fw)
{
	int i;
	struct device *dev = &priv.pdev->dev;
	struct elf32_shdr *sym_shdr =
				nvadsp_get_section(fw, ".symtab");
	struct elf32_shdr *str_shdr =
				nvadsp_get_section(fw, ".strtab");
	const u8 *elf_data = fw->data;
	const char *name_table;
	/* The first entry stores the number of entries in the array */
	int num_ent = 1;
	struct elf32_sym *sym;
	struct elf32_sym *last_sym;

	sym = (struct elf32_sym *)(elf_data + sym_shdr->sh_offset);
	name_table = elf_data + str_shdr->sh_offset;

	num_ent += sym_shdr->sh_size / sizeof(struct elf32_sym);
	priv.adsp_glo_sym_tbl =
		devm_kzalloc(dev,
			sizeof(struct global_sym_info) * num_ent,
			GFP_KERNEL);
	if (!priv.adsp_glo_sym_tbl)
		return -ENOMEM;

	last_sym = sym + num_ent;

	for (i = 1; sym < last_sym; sym++) {
		unsigned char info = sym->st_info;
		unsigned char type = ELF32_ST_TYPE(info);
		if ((ELF32_ST_BIND(sym->st_info) == STB_GLOBAL) &&
		((type == STT_OBJECT) || (type == STT_FUNC))) {
			char *name = priv.adsp_glo_sym_tbl[i].name;
			strncpy(name, name_table + sym->st_name, SYM_NAME_SZ);
			priv.adsp_glo_sym_tbl[i].addr = sym->st_value;
			priv.adsp_glo_sym_tbl[i].info = info;
			i++;
		}
	}
	priv.adsp_glo_sym_tbl[0].addr = i;
	return 0;
}

struct global_sym_info *find_global_symbol(const char *sym_name)
{
	struct device *dev = &priv.pdev->dev;
	struct global_sym_info *table = priv.adsp_glo_sym_tbl;
	int num_ent;
	int i;

	if (unlikely(!table)) {
		dev_info(dev, "symbol table not present\n");
		return 0;
	}
	num_ent = table[0].addr;

	for (i = 1; i < num_ent; i++) {
		if (!strncmp(table[i].name, sym_name, SYM_NAME_SZ))
			return &table[i];
	}
	return NULL;
}

static void *get_debug_ram(const struct firmware *fw, int *size)
{
	struct device *dev = &priv.pdev->dev;
	struct elf32_shdr *shdr;
	int addr;

	shdr = nvadsp_get_section(fw, DEBUG_RAM_REGION);
	if (!shdr) {
		dev_info(dev, "section %s not found\n", DEBUG_RAM_REGION);
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev,
		"the %s is present at 0x%x\n",
				DEBUG_RAM_REGION, shdr->sh_addr);
	addr = shdr->sh_addr;
	*size = shdr->sh_size;
	return nvadsp_da_to_va_mappings(addr, *size);
}

void *get_mailbox_shared_region(void)
{
	const struct firmware *fw;
	struct device *dev;
	struct elf32_shdr *shdr;
	int addr;
	int size;
	int ret;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return ERR_PTR(-EINVAL);
	}

	dev = &priv.pdev->dev;

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_info(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		return ERR_PTR(ret);
	}

	shdr = nvadsp_get_section(fw, MAILBOX_REGION);
	if (!shdr) {
		dev_info(dev, "section %s not found\n", MAILBOX_REGION);
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev,
		"the shared section is present at 0x%x\n",
						shdr->sh_addr);
	addr = shdr->sh_addr;
	size = shdr->sh_size;
	return nvadsp_da_to_va_mappings(addr, size);
}

static void copy_io_in_l(void *to, const void *from, int sz)
{
	int i;
	for (i = 0; i <= sz; i += 4) {
		int val = *(int *)(from + i);
		writel(val, to + i);
	}
}

static int nvadsp_os_elf_load(const struct firmware *fw)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(priv.pdev);
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

		dev_dbg(dev,
		"phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
				phdr->p_type, da, memsz, filesz);

		va = nvadsp_da_to_va_mappings(da, filesz);
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
		if (filesz) {
			if (!is_adsp_dram_addr(da)) {
				drv_data->state.evp_ptr = va;
				memcpy(drv_data->state.evp,
				       elf_data + offset, filesz);
			} else
				memcpy(va, elf_data + offset, filesz);
		}
	}

	return ret;
}

static int allocate_memory_for_adsp_os(void)
{
	struct platform_device *pdev = priv.pdev;
	struct device *dev = &pdev->dev;
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	dma_addr_t addr;
#else
	phys_addr_t addr;
#endif
	void *dram_va;
	size_t size;
	int ret = 0;

#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	addr = ADSP_SMMU_LOAD_ADDR;
	size = ADSP_SMMU_SIZE;
	dram_va = dma_alloc_at_coherent(dev, size, &addr, GFP_KERNEL);
	if (!dram_va) {
		dev_err(dev, "unable to allocate SMMU pages\n");
		ret = -ENOMEM;
		goto end;
	}
#else
	struct nvadsp_platform_data *plat_data = pdev->dev.platform_data;

	if (IS_ERR_OR_NULL(plat_data)) {
		dev_err(dev, "carvout is NULL\n");
		ret = -ENOMEM;
		goto end;
	}

	addr = plat_data->co_pa;
	size = plat_data->co_size;
	dram_va = ioremap_nocache(addr, plat_data->co_size);
	if (!dram_va) {
		dev_err(dev, "remap failed for addr %lx\n",
					(long)plat_data->co_pa);
		ret = -ENOMEM;
		goto end;
	}
#endif
	adsp_add_load_mappings(addr, dram_va, size);
end:
	return ret;
}

static void deallocate_memory_for_adsp_os(struct device *dev)
{
#if defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	void *va = nvadsp_da_to_va_mappings(ADSP_SMMU_LOAD_ADDR,
			ADSP_SMMU_SIZE);
	dma_free_coherent(dev, ADSP_SMMU_SIZE, va, ADSP_SMMU_LOAD_ADDR);
#endif
}

int nvadsp_os_load(void)
{
	struct nvadsp_drv_data *drv_data;
	const struct firmware *fw;
	struct device *dev;
	int ret;
	void *ptr;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		ret = -EINVAL;
		goto end;
	}

	dev = &priv.pdev->dev;

	drv_data = platform_get_drvdata(priv.pdev);

	ret = request_firmware(&fw, NVADSP_FIRMWARE, dev);
	if (ret < 0) {
		dev_err(dev,
			"reqest firmware for %s failed with %d\n",
					NVADSP_FIRMWARE, ret);
		goto end;
	}

	ret = create_global_symbol_table(fw);
	if (ret) {
		dev_err(dev,
			"unable to create global symbol table\n");
		goto release_firmware;
	}

	ret = allocate_memory_for_adsp_os();
	if (ret) {
		dev_err(dev,
			"unable to allocate memory for adsp os\n");
		goto release_firmware;
	}

	priv.logger.debug_ram_rdr =
		get_debug_ram(fw, &priv.logger.debug_ram_sz);
	if (IS_ERR_OR_NULL(priv.logger.debug_ram_rdr))
		dev_err(dev,
			"Ram debug logging facility not available\n");

	/* hold the pointer to the device */
	priv.logger.dev = dev;


	dev_info(dev, "Loading ADSP OS firmware %s\n", NVADSP_FIRMWARE);

	ret = nvadsp_os_elf_load(fw);
	if (ret) {
		dev_err(dev, "failed to load %s\n", NVADSP_FIRMWARE);
		goto deallocate_os_memory;
	}

	ret = dram_app_mem_init(ADSP_APP_MEM_SMMU_ADDR, ADSP_APP_MEM_SIZE);
	if (ret) {
		dev_err(dev,
			"unable to allocate memory for allocating dynamic apps\n");
		goto deallocate_os_memory;
	}
	ptr = get_mailbox_shared_region();
	update_nvadsp_app_shared_ptr(ptr);
	drv_data->shared_adsp_os_data = ptr;
	priv.os_firmware = fw;

	return 0;

deallocate_os_memory:
	deallocate_memory_for_adsp_os(dev);
release_firmware:
	release_firmware(fw);
end:
	return ret;
}
EXPORT_SYMBOL(nvadsp_os_load);

int __nvadsp_os_start(void)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
#if !CONFIG_SYSTEM_FPGA
	u32 val;
#endif
	int ret;

	dev = &priv.pdev->dev;
	drv_data = platform_get_drvdata(priv.pdev);

	dev_info(dev, "Copying EVP...\n");
	copy_io_in_l(drv_data->state.evp_ptr,
		     drv_data->state.evp,
		     AMC_EVP_SIZE);

	if (drv_data->adsp_cpu_clk) {
		dev_info(dev, "setting adsp cpu to %lu...\n", LOAD_ADSP_FREQ);
		clk_set_rate(drv_data->adsp_cpu_clk, LOAD_ADSP_FREQ);
	}

	dev_info(dev, "Starting ADSP OS...\n");
	if (drv_data->adsp_clk) {
		dev_info(dev, "deasserting adsp...\n");
		tegra_periph_reset_deassert(drv_data->adsp_clk);
		udelay(200);
	}

#if !CONFIG_SYSTEM_FPGA
	writel(APE_RESET, priv.reset_reg);
#endif

	dev_info(dev, "waiting for ADSP OS to boot up...\n");
	ret = wait_for_adsp_os_load_complete();
	dev_info(dev, "waiting for ADSP OS to boot up...Done.\n");

#ifdef CONFIG_TEGRA_ADSP_DFS
	if (!is_dfs_initialized && adsp_dfs_core_init(priv.pdev)) {
		dev_err(dev, "adsp dfs initialization failed\n");
		return -EINVAL;
	}
	is_dfs_initialized = true;
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	if (!is_actmon_initialized && ape_actmon_init(priv.pdev)) {
		dev_err(dev, "ape actmon initialization failed\n");
		return -EINVAL;
	}
	is_actmon_initialized = true;
#endif

	drv_data->adsp_os_loaded = true;
	return 0;
}

int nvadsp_os_start(void)
{
	int ret;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(&priv.pdev->dev);

	return __nvadsp_os_start();
}
EXPORT_SYMBOL(nvadsp_os_start);

static int __nvadsp_os_suspend(void)
{
	uint16_t com_mid = ADSP_COM_MBOX_ID;
	int ret;

#ifdef CONFIG_TEGRA_ADSP_DFS
	adsp_dfs_core_exit(priv.pdev);
	is_dfs_initialized = false;
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ape_actmon_exit(priv.pdev);
	is_actmon_initialized = false;
#endif

	ret = nvadsp_mbox_open(&adsp_com_mbox, &com_mid,
			       "adsp_com_mbox",
			       NULL, NULL);
	if (ret) {
		pr_err("failed to open adsp com mbox\n");
		goto out;
	}

	ret = nvadsp_mbox_send(&adsp_com_mbox, ADSP_OS_SUSPEND,
			       NVADSP_MBOX_SMSG, true, UINT_MAX);
	if (ret) {
		pr_err("failed to send with adsp com mbox\n");
		goto out;
	}

	/* TODO: remove delay */
	msleep(300);

	ret = nvadsp_mbox_close(&adsp_com_mbox);
	if (ret) {
		pr_err("failed to close adsp com mbox\n");
		goto out;
	}

	ret = pm_runtime_put_sync(&priv.pdev->dev);
	if (ret) {
		pr_err("failed in pm_runtime_put_sync\n");
		goto out;
	}
 out:
	return ret;
}

static void __nvadsp_os_stop(bool reload)
{
	const struct firmware *fw = priv.os_firmware;
	struct nvadsp_drv_data *drv_data;
	struct device *dev;
	int err = 0;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	dev = &priv.pdev->dev;
	drv_data = platform_get_drvdata(priv.pdev);

#ifdef CONFIG_TEGRA_ADSP_DFS
	adsp_dfs_core_exit(priv.pdev);
	is_dfs_initialized = false;
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ape_actmon_exit(priv.pdev);
	is_actmon_initialized = false;
#endif

	writel(ENABLE_MBOX2_EMPTY_INT, priv.misc_base + HWMBOX2_REG);
	wait_for_completion(&entered_wfe);
	writel(DISABLE_MBOX2_EMPTY_INT, priv.misc_base + HWMBOX2_REG);

	tegra_periph_reset_assert(drv_data->adsp_clk);

	if (reload) {
		/*
		 * move ram iterator to 0, since after restart the iterator
		 * will be pointing to initial position of start.
		 */
		priv.logger.debug_ram_rdr[0] = EOT;
		priv.logger.ram_iter = 0;
		/* load a fresh copy of adsp.elf */
		if (nvadsp_os_elf_load(fw))
			dev_err(dev, "failed to reload %s\n", NVADSP_FIRMWARE);
	}

	err = pm_runtime_put_sync(dev);
	if (err)
		pr_err("failed in pm_runtime_put_sync\n");

	return;
}


void nvadsp_os_stop(void)
{
	__nvadsp_os_stop(true);
}
EXPORT_SYMBOL(nvadsp_os_stop);

void nvadsp_os_suspend(void)
{
	/*
	 * No os suspend/stop on linsim as
	 * APE can be reset only once.
	 */
	if (tegra_platform_is_linsim())
		return;

	__nvadsp_os_suspend();
}
EXPORT_SYMBOL(nvadsp_os_suspend);

static void nvadsp_os_restart(struct work_struct *work)
{
	struct nvadsp_os_data *data =
		container_of(work, struct nvadsp_os_data, restart_os_work);
	int wdt_virq = tegra_agic_irq_get_virq(INT_ADSP_WDT);
	struct device *dev = &data->pdev->dev;

	disable_irq(wdt_virq);
	nvadsp_os_stop();

	if (tegra_agic_irq_is_active(INT_ADSP_WDT)) {
		dev_info(dev, "wdt interrupt is active hence clearing\n");
		tegra_agic_clear_active(INT_ADSP_WDT);
	}

	if (tegra_agic_irq_is_pending(INT_ADSP_WDT)) {
		dev_info(dev, "wdt interrupt is pending hence clearing\n");
		tegra_agic_clear_pending(INT_ADSP_WDT);
	}

	dev_info(dev, "wdt interrupt is not pending or active...enabling\n");
	enable_irq(wdt_virq);

	data->adsp_num_crashes++;
	if (data->adsp_num_crashes >= ALLOWED_CRASHES) {
		/* making pdev NULL so that externally start is not called */
		priv.pdev = NULL;
		dev_crit(dev, "ADSP has crashed too many times\n");
		return;
	}

	if (nvadsp_os_start())
		dev_crit(dev, "Unable to restart ADSP OS\n");
}

static  irqreturn_t wfe_handler(int irq, void *arg)
{
	struct nvadsp_os_data *data = arg;
	struct device *dev = &data->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
	complete(&entered_wfe);

	return 0;
}

static irqreturn_t adsp_wdt_handler(int irq, void *arg)
{
	struct nvadsp_os_data *data = arg;
	struct device *dev = &data->pdev->dev;

#if CONFIG_SYSTEM_FPGA
	dev_crit(dev, "ADSP OS Hanged or Crashed! Restarting...\n");
	schedule_work(&data->restart_os_work);
#else
	dev_crit(dev, "ADSP OS Hanged or Crashed!\n");
#endif
	return IRQ_HANDLED;
}

int nvadsp_os_probe(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int wdt_virq = tegra_agic_irq_get_virq(INT_ADSP_WDT);
	int wfe_virq = tegra_agic_irq_get_virq(INT_WFE);
	struct device *dev = &pdev->dev;
	int ret = 0;

#if !CONFIG_SYSTEM_FPGA
	priv.reset_reg = ioremap(APE_FPGA_MISC_RST_DEVICES, 1);
	if (!priv.reset_reg) {
		dev_info(dev, "unable to map reset addr\n");
		ret = -EINVAL;
		goto end;
	}
#endif
	priv.pdev = pdev;
	priv.misc_base = drv_data->base_regs[AMISC];
	priv.dram_region = drv_data->dram_region;

#ifdef CONFIG_DEBUG_FS
	if (adsp_create_debug_logger(drv_data->adsp_debugfs_root))
		dev_err(dev,
			"unable to create adsp debug logger file\n");
#endif

	ret = devm_request_irq(dev, wdt_virq, adsp_wdt_handler,
			IRQF_TRIGGER_RISING, "adsp watchdog", &priv);
	if (ret) {
		dev_err(dev, "failed to get adsp watchdog interrupt\n");
		goto end;
	}

	ret = devm_request_irq(dev, wfe_virq, wfe_handler,
			IRQF_TRIGGER_RISING, "wfe handler", &priv);
	if (ret) {
		dev_err(dev, "cannot request for wfe interrupt\n");
		goto end;
	}

	ret = tegra_agic_route_interrupt(INT_AMISC_MBOX_EMPTY2,
			TEGRA_AGIC_ADSP);
	if (ret) {
		dev_err(dev, "failed to route fiq interrupt\n");
		goto end;
	}

	writel(DISABLE_MBOX2_EMPTY_INT, priv.misc_base + HWMBOX2_REG);

	INIT_WORK(&priv.restart_os_work, nvadsp_os_restart);

end:
	return ret;
}
