/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/mmc/sd.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <asm/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <mach/hardware.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>
#include <mach/io_dpd.h>
#include <mach/pinmux.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20

#define SDHCI_VENDOR_CLOCK_CNTRL	0x100
#define SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK	0x1
#define SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VENDOR_CLOCK_CNTRL_INPUT_IO_CLK		0x2
#define SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT	8
#define SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT	16
#define SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT	24
#define SDHCI_VENDOR_CLOCK_CNTRL_SDR50_TUNING		0x20

#define SDHCI_VENDOR_MISC_CNTRL		0x120
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR104_SUPPORT	0x8
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR50_SUPPORT	0x10
#define SDHCI_VENDOR_MISC_CNTRL_ENABLE_DDR50_SUPPORT	0x200
#define SDHCI_VENDOR_MISC_CNTRL_INFINITE_ERASE_TIMEOUT	0x1

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START	0x80000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET	0x70
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET	0x62

#define SDMMC_AUTO_CAL_STATUS	0x1EC
#define SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE	0x80000000
#define SDMMC_AUTO_CAL_STATUS_PULLDOWN_OFFSET	24
#define PULLUP_ADJUSTMENT_OFFSET	20

#define SDHOST_1V8_OCR_MASK	0x8
#define SDHOST_HIGH_VOLT_MIN	2700000
#define SDHOST_HIGH_VOLT_MAX	3600000
#define SDHOST_HIGH_VOLT_2V8	2800000
#define SDHOST_LOW_VOLT_MIN	1800000
#define SDHOST_LOW_VOLT_MAX	1800000
#define SDHOST_HIGH_VOLT_3V2	3200000

#define TEGRA_SDHOST_MIN_FREQ	50000000
#define TEGRA2_SDHOST_STD_FREQ	50000000
#define TEGRA3_SDHOST_STD_FREQ	104000000

#define MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_8	128
#define MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_4	64
#define MAX_TAP_VALUES	256

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void tegra_3x_sdhci_set_card_clock(struct sdhci_host *sdhci, unsigned int clock);
static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci);
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void tegra11x_sdhci_post_reset_init(struct sdhci_host *sdhci);
#endif

static unsigned int tegra_sdhost_min_freq;
static unsigned int tegra_sdhost_std_freq;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static unsigned int tegra3_sdhost_max_clk[4] = {
	208000000,	104000000,	208000000,	104000000 };
#endif

struct tegra_sdhci_hw_ops{
	/* Set the internal clk and card clk.*/
	void	(*set_card_clock)(struct sdhci_host *sdhci, unsigned int clock);
	/* Post reset vendor registers configuration */
	void	(*sdhost_init)(struct sdhci_host *sdhci);
};

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
static struct tegra_sdhci_hw_ops tegra_2x_sdhci_ops = {
};
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
static struct tegra_sdhci_hw_ops tegra_3x_sdhci_ops = {
	.set_card_clock = tegra_3x_sdhci_set_card_clock,
	.sdhost_init = tegra3_sdhci_post_reset_init,
};
#else
static struct tegra_sdhci_hw_ops tegra_11x_sdhci_ops = {
        .sdhost_init = tegra11x_sdhci_post_reset_init,
};
#endif

#define NVQUIRK_FORCE_SDHCI_SPEC_200		BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET		BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300		BIT(2)
#define NVQUIRK_DISABLE_AUTO_CALIBRATION	BIT(3)
#define NVQUIRK_SET_CALIBRATION_OFFSETS		BIT(4)
#define NVQUIRK_SET_DRIVE_STRENGTH		BIT(5)
#define NVQUIRK_DISABLE_SDMMC4_CALIB		BIT(6)

struct sdhci_tegra_soc_data {
	struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
};

struct sdhci_tegra_sd_stats {
	unsigned int data_crc_count;
	unsigned int cmd_crc_count;
	unsigned int data_to_count;
	unsigned int cmd_to_count;
};

struct sdhci_tegra {
	const struct tegra_sdhci_platform_data *plat;
	const struct sdhci_tegra_soc_data *soc_data;
	bool	clk_enabled;
	struct regulator *vdd_io_reg;
	struct regulator *vdd_slot_reg;
	/* Pointer to the chip specific HW ops */
	struct tegra_sdhci_hw_ops *hw_ops;
	/* Host controller instance */
	unsigned int instance;
	/* vddio_min */
	unsigned int vddio_min_uv;
	/* vddio_max */
	unsigned int vddio_max_uv;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	/* max ddr clk supported by the platform */
	unsigned int ddr_clk_limit;
	struct tegra_io_dpd *dpd;
	bool card_present;
	bool is_rail_enabled;
	struct clk *emc_clk;
	unsigned int emc_max_clk;
	struct sdhci_tegra_sd_stats *sd_stat_head;
};

static int show_register_dump(struct seq_file *s, void *data)
{
	struct sdhci_host *host = s->private;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct sdhci_tegra_sd_stats *head;

	seq_printf(s, "ErrorStatistics:\n");
	seq_printf(s, "DataCRC\tCmdCRC\tDataTimeout\tCmdTimeout\n");
	head = tegra_host->sd_stat_head;
	if (head != NULL)
		seq_printf(s, "%d\t%d\t%d\t%d\n", head->data_crc_count,
			head->cmd_crc_count, head->data_to_count,
			head->cmd_to_count);
	return 0;
}

static int sdhci_register_dump(struct inode *inode, struct file *file)
{
	return single_open(file, show_register_dump, inode->i_private);
}

static const struct file_operations sdhci_host_fops = {
	.open		= sdhci_register_dump,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static u32 tegra_sdhci_readl(struct sdhci_host *host, int reg)
{
#ifndef CONFIG_ARCH_TEGRA_11x_SOC
	u32 val;

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		/* Use wp_gpio here instead? */
		val = readl(host->ioaddr + reg);
		return val | SDHCI_WRITE_PROTECT;
	}
#endif
	return readl(host->ioaddr + reg);
}

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}
#endif
	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
#endif

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	if (unlikely((soc_data->nvquirks & NVQUIRK_ENABLE_BLOCK_GAP_DET) &&
			(reg == SDHCI_INT_ENABLE))) {
		/* Erratum: Must enable block gap interrupt detection */
		u8 gap_ctrl = readb(host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
		if (val & SDHCI_INT_CARD_INT)
			gap_ctrl |= 0x8;
		else
			gap_ctrl &= ~0x8;
		writeb(gap_ctrl, host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
	}
#endif
}

static unsigned int tegra_sdhci_get_cd(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	return tegra_host->card_present;
}

#ifndef CONFIG_ARCH_TEGRA_11x_SOC
static unsigned int tegra_sdhci_get_ro(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	if (!gpio_is_valid(plat->wp_gpio))
		return -1;

	return gpio_get_value_cansleep(plat->wp_gpio);
}
#endif

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci)
{
	u16 misc_ctrl;
	u32 vendor_ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	if (tegra_host->sd_stat_head != NULL) {
		tegra_host->sd_stat_head->data_crc_count = 0;
		tegra_host->sd_stat_head->cmd_crc_count = 0;
		tegra_host->sd_stat_head->data_to_count = 0;
		tegra_host->sd_stat_head->cmd_to_count = 0;
	}
	/* Set the base clock frequency */
	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT);

	if (plat->base_clk > 0)
		vendor_ctrl |= (plat->base_clk / 1000000) <<
			SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;
	else
		vendor_ctrl |=
			(tegra3_sdhost_max_clk[tegra_host->instance] / 1000000)
			<< SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;

	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE;

	/* Set tap delay */
	if (plat->tap_delay) {
		vendor_ctrl &= ~(0xFF <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
		vendor_ctrl |= (plat->tap_delay <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	}
	/* Enable frequency tuning for SDR50 mode */
	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDR50_TUNING;
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

	/* Enable SDHOST v3.0 support */
	misc_ctrl = sdhci_readw(sdhci, SDHCI_VENDOR_MISC_CNTRL);
	misc_ctrl |=
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR104_SUPPORT |
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR50_SUPPORT;
	sdhci_writew(sdhci, misc_ctrl, SDHCI_VENDOR_MISC_CNTRL);
}
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void tegra11x_sdhci_post_reset_init(struct sdhci_host *sdhci)
{
	u16 misc_ctrl;
	u32 vendor_ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	struct tegra_sdhci_platform_data *plat;

	if (tegra_host->sd_stat_head != NULL) {
		tegra_host->sd_stat_head->data_crc_count = 0;
		tegra_host->sd_stat_head->cmd_crc_count = 0;
		tegra_host->sd_stat_head->data_to_count = 0;
		tegra_host->sd_stat_head->cmd_to_count = 0;
	}
	plat = pdev->dev.platform_data;
	/* Set the base clock frequency */
	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT);

	if (plat->base_clk > 0)
		vendor_ctrl |= (plat->base_clk / 1000000) <<
			SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;
	else
		vendor_ctrl |=
			(tegra3_sdhost_max_clk[tegra_host->instance] / 1000000)
			<< SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;

	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE;
	vendor_ctrl &=  ~SDHCI_VENDOR_CLOCK_CNTRL_INPUT_IO_CLK;

	/* Set tap delay */
	if (plat->tap_delay) {
		vendor_ctrl &= ~(0xFF <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
		vendor_ctrl |= (plat->tap_delay <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	}

	/* Set trim delay */
	if (plat->trim_delay) {
		vendor_ctrl &= ~(0x1F <<
			SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT);
		vendor_ctrl |= (plat->trim_delay <<
			SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT);
	}
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

	/* Enable SDHOST v3.0 support */
	misc_ctrl = sdhci_readw(sdhci, SDHCI_VENDOR_MISC_CNTRL);
	misc_ctrl |= SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR104_SUPPORT |
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_SDR50_SUPPORT |
		SDHCI_VENDOR_MISC_CNTRL_ENABLE_DDR50_SUPPORT;
	misc_ctrl |= SDHCI_VENDOR_MISC_CNTRL_INFINITE_ERASE_TIMEOUT;
	sdhci_writew(sdhci, misc_ctrl, SDHCI_VENDOR_MISC_CNTRL);
}
#endif

static int tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int uhs)
{
	u16 clk, ctrl_2;
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/* Select Bus Speed Mode for host */
	/* For HS200 we need to set UHS_MODE_SEL to SDR104.
	 * It works as SDR 104 in SD 4-bit mode and HS200 in eMMC 8-bit mode.
	 */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
		break;
	case MMC_TIMING_UHS_DDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
		break;
	}

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (uhs == MMC_TIMING_UHS_DDR50) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	}
	return 0;
}

static void sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *sdhci = (struct sdhci_host *)dev_id;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	struct tegra_sdhci_platform_data *plat;
	unsigned int status, oldstat;

	pr_debug("%s: card_present %d\n", mmc_hostname(sdhci->mmc),
		card_present);

	plat = pdev->dev.platform_data;
	if (!plat->mmc_data.status) {
		mmc_detect_change(sdhci->mmc, 0);
		return;
	}

	status = plat->mmc_data.status(mmc_dev(sdhci->mmc));

	oldstat = plat->mmc_data.card_present;
	plat->mmc_data.card_present = status;
	if (status ^ oldstat) {
		pr_debug("%s: Slot status change detected (%d -> %d)\n",
			mmc_hostname(sdhci->mmc), oldstat, status);
		if (status && !plat->mmc_data.built_in)
			mmc_detect_change(sdhci->mmc, (5 * HZ) / 2);
		else
			mmc_detect_change(sdhci->mmc, 0);
	}
}

static irqreturn_t carddetect_irq(int irq, void *data)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhost);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhost->mmc));
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;

	tegra_host->card_present =
			(gpio_get_value_cansleep(plat->cd_gpio) == 0);

	if (tegra_host->card_present) {
		if (!tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_slot_reg)
				regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg)
				regulator_enable(tegra_host->vdd_io_reg);
			tegra_host->is_rail_enabled = 1;
		}
	} else {
		if (tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_io_reg)
				regulator_disable(tegra_host->vdd_io_reg);
			if (tegra_host->vdd_slot_reg)
				regulator_disable(tegra_host->vdd_slot_reg);
			tegra_host->is_rail_enabled = 0;
                }
	}

	tasklet_schedule(&sdhost->card_tasklet);
	return IRQ_HANDLED;
};

static void tegra_sdhci_reset_exit(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	if (!(mask & SDHCI_RESET_ALL))
		return;

	if (tegra_host->hw_ops->sdhost_init)
		tegra_host->hw_ops->sdhost_init(host);

	/* Erratum: Enable SDHCI spec v3.00 support */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDHCI_SPEC_300) {
		u32 misc_ctrl;

		misc_ctrl = sdhci_readb(host, SDHCI_TEGRA_VENDOR_MISC_CTRL);
		misc_ctrl |= SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
		sdhci_writeb(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);
	}

	/* Mask the support for any UHS modes if specified */
	if (plat->uhs_mask & MMC_UHS_MASK_SDR104)
		host->mmc->caps &= ~MMC_CAP_UHS_SDR104;

	if (plat->uhs_mask & MMC_UHS_MASK_DDR50)
		host->mmc->caps &= ~MMC_CAP_UHS_DDR50;

	if (plat->uhs_mask & MMC_UHS_MASK_SDR50)
		host->mmc->caps &= ~MMC_CAP_UHS_SDR50;

	if (plat->uhs_mask & MMC_UHS_MASK_SDR25)
		host->mmc->caps &= ~MMC_CAP_UHS_SDR25;
}

static int tegra_sdhci_buswidth(struct sdhci_host *sdhci, int bus_width)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	const struct tegra_sdhci_platform_data *plat;
	u32 ctrl;

	plat = pdev->dev.platform_data;

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL);
	if (plat->is_8bit && bus_width == MMC_BUS_WIDTH_8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL);
	return 0;
}

static void tegra_sdhci_set_clk_rate(struct sdhci_host *sdhci,
	unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int clk_rate;
	unsigned int emc_clk;

	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
		/*
		 * In ddr mode, tegra sdmmc controller clock frequency
		 * should be double the card clock frequency.
		 */
		if (tegra_host->ddr_clk_limit) {
			clk_rate = tegra_host->ddr_clk_limit * 2;
			if (tegra_host->emc_clk) {
				emc_clk = clk_get_rate(tegra_host->emc_clk);
				if (emc_clk == tegra_host->emc_max_clk)
					clk_rate = clock * 2;
			}
		} else {
			clk_rate = clock * 2;
		}
	} else 	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_SDR50) {
		/*
		 * In SDR50 mode, run the sdmmc controller at freq greater than
		 * 104MHz to ensure the core voltage is at 1.2V. If the core voltage
		 * is below 1.2V, CRC errors would occur during data transfers.
		 */
		clk_rate = clock * 2;
	} else {
		if (clock <= tegra_sdhost_min_freq)
			clk_rate = tegra_sdhost_min_freq;
		else if (clock <= tegra_sdhost_std_freq)
			clk_rate = tegra_sdhost_std_freq;
		else
			clk_rate = clock;
	}

	if (tegra_host->max_clk_limit &&
		(clk_rate > tegra_host->max_clk_limit))
		clk_rate = tegra_host->max_clk_limit;

	clk_set_rate(pltfm_host->clk, clk_rate);
	sdhci->max_clk = clk_get_rate(pltfm_host->clk);

	/* FPGA supports 26MHz of clock for SDMMC. */
	if (tegra_platform_is_fpga())
		sdhci->max_clk = 26000000;
}
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static void tegra_3x_sdhci_set_card_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;
	u8 ctrl;

	if (clock && clock == sdhci->clock)
		return;

	/*
	 * Disable the card clock before disabling the internal
	 * clock to avoid abnormal clock waveforms.
	 */
	clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
	sdhci_writew(sdhci, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;
	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
		div = 1;
		goto set_clk;
	}

	if (sdhci->version >= SDHCI_SPEC_300) {
		/* Version 3.00 divisors must be a multiple of 2. */
		if (sdhci->max_clk <= clock) {
			div = 1;
		} else {
			for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
				if ((sdhci->max_clk / div) <= clock)
					break;
			}
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < SDHCI_MAX_DIV_SPEC_200; div *= 2) {
			if ((sdhci->max_clk / div) <= clock)
				break;
		}
	}
	div >>= 1;

	/*
	 * Tegra3 sdmmc controller internal clock will not be stabilized when
	 * we use a clock divider value greater than 4. The WAR is as follows.
	 * - Enable internal clock.
	 * - Wait for 5 usec and do a dummy write.
	 * - Poll for clk stable.
	 */
set_clk:
	clk = (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Wait for 5 usec */
	udelay(5);

	/* Do a dummy write */
	ctrl = sdhci_readb(sdhci, SDHCI_CAPABILITIES);
	ctrl |= 1;
	sdhci_writeb(sdhci, ctrl, SDHCI_CAPABILITIES);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Internal clock never stabilised\n");
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
out:
	sdhci->clock = clock;
}
#endif /*	#ifdef CONFIG_ARCH_TEGRA_3x_SOC */

static void tegra_sdhci_set_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	u8 ctrl;

	pr_debug("%s %s %u enabled=%u\n", __func__,
		mmc_hostname(sdhci->mmc), clock, tegra_host->clk_enabled);

	if (clock) {
		/* bring out sd instance from io dpd mode */
		if (tegra_host->dpd) {
			mutex_lock(&tegra_host->dpd->delay_lock);
			cancel_delayed_work_sync(&tegra_host->dpd->delay_dpd);
			tegra_io_dpd_disable(tegra_host->dpd);
			mutex_unlock(&tegra_host->dpd->delay_lock);
		}

		if (!tegra_host->clk_enabled) {
			pm_runtime_get_sync(&pdev->dev);
			clk_prepare_enable(pltfm_host->clk);
			ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
			ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
			tegra_host->clk_enabled = true;
		}
		tegra_sdhci_set_clk_rate(sdhci, clock);
		if (tegra_host->hw_ops->set_card_clock)
			tegra_host->hw_ops->set_card_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		if (tegra_host->hw_ops->set_card_clock)
			tegra_host->hw_ops->set_card_clock(sdhci, clock);
		ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
		ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
		sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
		clk_disable_unprepare(pltfm_host->clk);
		pm_runtime_put_sync(&pdev->dev);
		tegra_host->clk_enabled = false;
		/* io dpd enable call for sd instance */

		if (tegra_host->dpd) {
			mutex_lock(&tegra_host->dpd->delay_lock);
			if (tegra_host->dpd->need_delay_dpd) {
				schedule_delayed_work(
					&tegra_host->dpd->delay_dpd,
					msecs_to_jiffies(100));
			} else {
				tegra_io_dpd_enable(tegra_host->dpd);
			}
			mutex_unlock(&tegra_host->dpd->delay_lock);
		}
	}
}
static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci)
{
	unsigned int val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int timeout = 10;

	/* No Calibration for sdmmc4 */
	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_SDMMC4_CALIB) &&
		(tegra_host->instance == 3))
		return;

	/*
	 * Do not enable auto calibration if the platform doesn't
	 * support it.
	 */
	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CALIBRATION))
		return;

	val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
	val &= ~SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK;
	val |= 0x7;
	sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_CALIBRATION_OFFSETS)) {
		/* Program Auto cal PD offset(bits 8:14) */
		val &= ~(0x7F <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		val |= (SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		/* Program Auto cal PU offset(bits 0:6) */
		val &= ~0x7F;
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET;
	}
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

	/* Wait until the calibration is done */
	do {
		if (!(sdhci_readl(sdhci, SDMMC_AUTO_CAL_STATUS) &
			SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE))
			break;

		mdelay(1);
		timeout--;
	} while (timeout);

	if (!timeout)
		dev_err(mmc_dev(sdhci->mmc), "Auto calibration failed\n");

	/* Disable Auto calibration */
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val &= ~SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_DRIVE_STRENGTH)) {
		unsigned int pulldown_code;
		unsigned int pullup_code;
		int pg;
		int err;

		pg = tegra_drive_get_pingroup(mmc_dev(sdhci->mmc));
		if (pg != -1) {
			/* Get the pull down codes from auto cal status reg */
			pulldown_code = (
				sdhci_readl(sdhci, SDMMC_AUTO_CAL_STATUS) >>
				SDMMC_AUTO_CAL_STATUS_PULLDOWN_OFFSET);
			/* Set the pull down in the pinmux reg */
			err = tegra_drive_pinmux_set_pull_down(pg,
				pulldown_code);
			if (err)
				dev_err(mmc_dev(sdhci->mmc),
				"Failed to set pulldown codes %d err %d\n",
				pulldown_code, err);

			/* Calculate the pull up codes */
			pullup_code = pulldown_code + PULLUP_ADJUSTMENT_OFFSET;
			if (pullup_code >= TEGRA_MAX_PULL)
				pullup_code = TEGRA_MAX_PULL - 1;
			/* Set the pull up code in the pinmux reg */
			err = tegra_drive_pinmux_set_pull_up(pg, pullup_code);
			if (err)
				dev_err(mmc_dev(sdhci->mmc),
				"Failed to set pullup codes %d err %d\n",
				pullup_code, err);
		}
	}
}

static int tegra_sdhci_signal_voltage_switch(struct sdhci_host *sdhci,
	unsigned int signal_voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int min_uV = tegra_host->vddio_min_uv;
	unsigned int max_uV = tegra_host->vddio_max_uv;
	unsigned int rc = 0;
	u16 clk, ctrl;


	ctrl = sdhci_readw(sdhci, SDHCI_HOST_CONTROL2);
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		ctrl |= SDHCI_CTRL_VDD_180;
		min_uV = SDHOST_LOW_VOLT_MIN;
		max_uV = SDHOST_LOW_VOLT_MAX;
	} else if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		if (ctrl & SDHCI_CTRL_VDD_180)
			ctrl &= ~SDHCI_CTRL_VDD_180;
	}

	/* Check if the slot can support the required voltage */
	if (min_uV > tegra_host->vddio_max_uv)
		return 0;

	/* Switch OFF the card clock to prevent glitches on the clock line */
	clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Set/clear the 1.8V signalling */
	sdhci_writew(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	/* Switch the I/O rail voltage */
	if (tegra_host->vdd_io_reg) {
		rc = regulator_set_voltage(tegra_host->vdd_io_reg,
			min_uV, max_uV);
		if (rc) {
			dev_err(mmc_dev(sdhci->mmc), "switching to 1.8V"
			"failed . Switching back to 3.3V\n");
			rc = regulator_set_voltage(tegra_host->vdd_io_reg,
				SDHOST_HIGH_VOLT_MIN,
				SDHOST_HIGH_VOLT_MAX);
			if (rc)
				dev_err(mmc_dev(sdhci->mmc),
				"switching to 3.3V also failed\n");
		}
	}

	/* Wait for 10 msec for the voltage to be switched */
	mdelay(10);

	/* Enable the card clock */
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

	/* Wait for 1 msec after enabling clock */
	mdelay(1);

	return rc;
}

static void tegra_sdhci_reset(struct sdhci_host *sdhci, u8 mask)
{
	unsigned long timeout;

	sdhci_writeb(sdhci, mask, SDHCI_SOFTWARE_RESET);

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(sdhci, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Reset 0x%x never"
				"completed.\n", (int)mask);
			return;
		}
		timeout--;
		mdelay(1);
	}

	tegra_sdhci_reset_exit(sdhci, mask);
}

static void sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	unsigned int tap_delay)
{
	u32 vendor_ctrl;

	/* Max tap delay value is 255 */
	BUG_ON(tap_delay > MAX_TAP_VALUES);

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
}

static int sdhci_tegra_run_frequency_tuning(struct sdhci_host *sdhci, u32 opcode, u32 block_size)
{
	int err = 0;
	u8 ctrl;
	u32 mask;
	unsigned int timeout = 10;
	int flags;
	u32 intstatus;

	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;
	while (sdhci_readl(sdhci, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc), "Controller never"
				"released inhibit bit(s).\n");
			err = -ETIMEDOUT;
			goto out;
		}
		timeout--;
		mdelay(1);
	}

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl &= ~SDHCI_CTRL_TUNED_CLK;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_EXEC_TUNING;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	/*
	 * In response to CMD19, the card sends 64 bytes of tuning
	 * block to the Host Controller. So we set the block size
	 * to 64 here.
	 * In response to CMD21, the card sends 128 bytes of tuning
	 * block for MMC_BUS_WIDTH_8 and 64 bytes for MMC_BUS_WIDTH_4
	 * to the Host Controller. So we set the block size to 64 here.
	 */
	sdhci_writew(sdhci, SDHCI_MAKE_BLKSZ(7, block_size), SDHCI_BLOCK_SIZE);

	sdhci_writeb(sdhci, 0xE, SDHCI_TIMEOUT_CONTROL);

	sdhci_writeb(sdhci, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);

	sdhci_writel(sdhci, 0x0, SDHCI_ARGUMENT);

	/* Set the cmd flags */
	flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_DATA;
	/* Issue the command */
	sdhci_writew(sdhci, SDHCI_MAKE_CMD(
		opcode, flags), SDHCI_COMMAND);

	timeout = 5;
	do {
		timeout--;
		mdelay(1);
		intstatus = sdhci_readl(sdhci, SDHCI_INT_STATUS);
		if (intstatus) {
			sdhci_writel(sdhci, intstatus, SDHCI_INT_STATUS);
			break;
		}
	} while(timeout);

	if ((intstatus & SDHCI_INT_DATA_AVAIL) &&
		!(intstatus & SDHCI_INT_DATA_CRC)) {
		err = 0;
		sdhci->tuning_done = 1;
	} else {
		tegra_sdhci_reset(sdhci, SDHCI_RESET_CMD);
		tegra_sdhci_reset(sdhci, SDHCI_RESET_DATA);
		err = -EIO;
	}

	if (sdhci->tuning_done) {
		sdhci->tuning_done = 0;
		ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_EXEC_TUNING) &&
			(ctrl & SDHCI_CTRL_TUNED_CLK))
			err = 0;
		else
			err = -EIO;
	}
	mdelay(1);
out:
	return err;
}

static int sdhci_tegra_sd_error_stats(struct sdhci_host *host, u32 int_status)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct sdhci_tegra_sd_stats *head = tegra_host->sd_stat_head;

	if (int_status & SDHCI_INT_DATA_CRC)
		head->data_crc_count = head->data_crc_count + 1;
	if (int_status & SDHCI_INT_CRC)
		head->cmd_crc_count = head->cmd_crc_count + 1;
	if (int_status & SDHCI_INT_TIMEOUT)
		head->cmd_to_count = head->cmd_to_count + 1;
	if (int_status & SDHCI_INT_DATA_TIMEOUT)
		head->data_to_count = head->data_to_count + 1;
	return 0;
}

static int sdhci_tegra_execute_tuning(struct sdhci_host *sdhci, u32 opcode)
{
	int err;
	u32 block_size;
	u16 ctrl_2;
	u8 *tap_delay_status;
	unsigned int i = 0;
	unsigned int temp_low_pass_tap = 0;
	unsigned int temp_pass_window = 0;
	unsigned int best_low_pass_tap = 0;
	unsigned int best_pass_window = 0;
	u32 ier;

	/* Tuning is valid only in SDR104 and SDR50 modes */
	ctrl_2 = sdhci_readw(sdhci, SDHCI_HOST_CONTROL2);
	if (!(((ctrl_2 & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR104) ||
		(((ctrl_2 & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR50) &&
		(sdhci->flags & SDHCI_SDR50_NEEDS_TUNING))))
			return 0;

	tap_delay_status = kzalloc(MAX_TAP_VALUES, GFP_KERNEL);
	if (tap_delay_status == NULL) {
		dev_err(mmc_dev(sdhci->mmc), "failed to allocate memory"
			"for storing tap_delay_status\n");
		return -ENOMEM;
	}

	/* Tuning should be done only for MMC_BUS_WIDTH_8 and MMC_BUS_WIDTH_4 */
	if (sdhci->mmc->ios.bus_width == MMC_BUS_WIDTH_8)
		block_size = MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_8;
	else if (sdhci->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		block_size = MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_4;
	else
		return -EINVAL;
	/*
	 * Disable all interrupts signalling.Enable interrupt status
	 * detection for buffer read ready and data crc. We use
	 * polling for tuning as it involves less overhead.
	 */
	ier = sdhci_readl(sdhci, SDHCI_INT_ENABLE);
	sdhci_writel(sdhci, 0, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(sdhci, SDHCI_INT_DATA_AVAIL |
		SDHCI_INT_DATA_CRC, SDHCI_INT_ENABLE);

	/*
	 * Set each tap delay value and run frequency tuning. After each
	 * run, update the tap delay status as working or not working.
	 */
	do {
		/* Set the tap delay */
		sdhci_tegra_set_tap_delay(sdhci, i);

		/* Run frequency tuning */
		err = sdhci_tegra_run_frequency_tuning(sdhci, opcode, block_size);

		/* Update whether the tap delay worked or not */
		tap_delay_status[i] = (err) ? 0: 1;
		i++;
	} while (i < 0xFF);

	/* Find the best possible tap range */
	for (i = 0; i < 0xFF; i++) {
		temp_pass_window = 0;

		/* Find the first passing tap in the current window */
		if (tap_delay_status[i]) {
			temp_low_pass_tap = i;

			/* Find the pass window */
			do {
				temp_pass_window++;
				i++;
				if (i > 0xFF)
					break;
			} while (tap_delay_status[i]);

			if ((temp_pass_window > best_pass_window) &&
					(temp_pass_window > 1)) {
				best_low_pass_tap = temp_low_pass_tap;
				best_pass_window = temp_pass_window;
			}
		}
	}


	pr_debug("%s: best pass tap window: start %d, end %d\n",
		mmc_hostname(sdhci->mmc), best_low_pass_tap,
		(best_low_pass_tap + best_pass_window));

	/* Set the best tap */
	sdhci_tegra_set_tap_delay(sdhci,
		(best_low_pass_tap + ((best_pass_window * 3) / 4)));

	/* Run frequency tuning */
	err = sdhci_tegra_run_frequency_tuning(sdhci, opcode, block_size);

	/* Enable the normal interrupts signalling */
	sdhci_writel(sdhci, ier, SDHCI_INT_ENABLE);
	sdhci_writel(sdhci, ier, SDHCI_SIGNAL_ENABLE);

	if (tap_delay_status)
		kfree(tap_delay_status);

	return err;
}

static int tegra_sdhci_suspend(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	tegra_sdhci_set_clock(sdhci, 0);

	/* Disable the power rails if any */
	if (tegra_host->card_present) {
		if (tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_io_reg)
				regulator_disable(tegra_host->vdd_io_reg);
			if (tegra_host->vdd_slot_reg)
				regulator_disable(tegra_host->vdd_slot_reg);
			tegra_host->is_rail_enabled = 0;
		}
	}

	if (tegra_host->dpd) {
		mutex_lock(&tegra_host->dpd->delay_lock);
		tegra_host->dpd->need_delay_dpd = 1;
		mutex_unlock(&tegra_host->dpd->delay_lock);
	}

	return 0;
}

static int tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct platform_device *pdev;
	struct tegra_sdhci_platform_data *plat;

	pdev = to_platform_device(mmc_dev(sdhci->mmc));
	plat = pdev->dev.platform_data;

	if (gpio_is_valid(plat->cd_gpio)) {
		tegra_host->card_present =
			(gpio_get_value_cansleep(plat->cd_gpio) == 0);
	}

	/* Enable the power rails if any */
	if (tegra_host->card_present) {
		if (!tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_slot_reg)
				regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg) {
				regulator_enable(tegra_host->vdd_io_reg);
				if (plat->mmc_data.ocr_mask &
							SDHOST_1V8_OCR_MASK)
					tegra_sdhci_signal_voltage_switch(sdhci,
							MMC_SIGNAL_VOLTAGE_180);
				else
					tegra_sdhci_signal_voltage_switch(sdhci,
							MMC_SIGNAL_VOLTAGE_330);
			}
			tegra_host->is_rail_enabled = 1;
		}
	}

	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);

	/* Reset the controller and power on if MMC_KEEP_POWER flag is set*/
	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER) {
		tegra_sdhci_reset(sdhci, SDHCI_RESET_ALL);
		sdhci_writeb(sdhci, SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
		sdhci->pwr = 0;
	}

	return 0;
}

static void sdhci_tegra_error_stats_debugfs(struct sdhci_host *host)
{
	struct dentry *root;

	root = debugfs_create_dir(dev_name(mmc_dev(host->mmc)), NULL);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err_root;

	host->debugfs_root = root;

	if (!debugfs_create_file("error_stats", S_IRUSR, root, host,
				&sdhci_host_fops))
		goto err_node;
	return;

err_node:
	debugfs_remove_recursive(root);
	host->debugfs_root = NULL;
err_root:
	pr_err("%s: Failed to initialize debugfs functionality\n", __func__);
	return;
}

static void tegra_sdhci_post_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	/* Turn OFF the clocks if the card is not present */
	if (!(tegra_host->card_present) && tegra_host->clk_enabled)
		tegra_sdhci_set_clock(sdhci, 0);
}

static const struct sdhci_ops tegra_sdhci_ops = {
#ifndef CONFIG_ARCH_TEGRA_11x_SOC
	.get_ro     = tegra_sdhci_get_ro,
#endif
	.get_cd     = tegra_sdhci_get_cd,
	.read_l     = tegra_sdhci_readl,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.platform_bus_width = tegra_sdhci_buswidth,
	.set_clock		= tegra_sdhci_set_clock,
	.suspend		= tegra_sdhci_suspend,
	.resume			= tegra_sdhci_resume,
	.platform_resume	= tegra_sdhci_post_resume,
	.platform_reset_exit	= tegra_sdhci_reset_exit,
	.set_uhs_signaling	= tegra_sdhci_set_uhs_signaling,
	.switch_signal_voltage	= tegra_sdhci_signal_voltage_switch,
	.switch_signal_voltage_exit = tegra_sdhci_do_calibration,
	.execute_freq_tuning	= sdhci_tegra_execute_tuning,
	.sd_error_stats		= sdhci_tegra_sd_error_stats,
};

static struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_NON_STD_VOLTAGE_SWITCHING |
		  SDHCI_QUIRK_NON_STANDARD_TUNING |
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
		  SDHCI_QUIRK_NONSTANDARD_CLOCK |
#endif
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_NO_CALC_MAX_DISCARD_TO,
	.quirks2 = SDHCI_QUIRK2_BROKEN_PRESET_VALUES,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra20 = {
	.pdata = &sdhci_tegra20_pdata,
	.nvquirks = NVQUIRK_FORCE_SDHCI_SPEC_200 |
#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
		    NVQUIRK_SET_DRIVE_STRENGTH |
		    NVQUIRK_DISABLE_SDMMC4_CALIB |
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
		    NVQUIRK_DISABLE_AUTO_CALIBRATION |
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
		    NVQUIRK_SET_CALIBRATION_OFFSETS |
		    NVQUIRK_ENABLE_SDHCI_SPEC_300 |
#endif
		    NVQUIRK_ENABLE_BLOCK_GAP_DET,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	{ .compatible = "nvidia,tegra114-sdhci", .data = &soc_data_tegra20 },
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra20 },
#endif
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	{ .compatible = "nvidia,tegra20-sdhci", .data = &soc_data_tegra20 },
#endif
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dt_ids);

static struct tegra_sdhci_platform_data *sdhci_tegra_dt_parse_pdata(
						struct platform_device *pdev)
{
	int val;
	struct tegra_sdhci_platform_data *plat;
	struct device_node *np = pdev->dev.of_node;
	u32 bus_width;

	if (!np)
		return NULL;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		dev_err(&pdev->dev, "Can't allocate platform data\n");
		return NULL;
	}

	plat->cd_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	plat->wp_gpio = of_get_named_gpio(np, "wp-gpios", 0);
	plat->power_gpio = of_get_named_gpio(np, "power-gpios", 0);

	if (of_property_read_u32(np, "bus-width", &bus_width) == 0 &&
	    bus_width == 8)
		plat->is_8bit = 1;

	of_property_read_u32(np, "tap-delay", &plat->tap_delay);
	of_property_read_u32(np, "trim-delay", &plat->trim_delay);
	of_property_read_u32(np, "ddr-clk-limit", &plat->ddr_clk_limit);

	if (of_property_read_u32(np, "base-clk", &plat->base_clk)) {
		dev_err(&pdev->dev, "base-clk not set\n");
		return NULL;
	}

	if (of_find_property(np, "built-in", NULL))
		plat->mmc_data.built_in = 1;

	if (!of_property_read_u32(np, "mmc-ocr-mask", &val)) {
		if (val == 0)
			plat->mmc_data.ocr_mask = MMC_OCR_1V8_MASK;
		else if (val == 1)
			plat->mmc_data.ocr_mask = MMC_OCR_2V8_MASK;
		else if (val == 2)
			plat->mmc_data.ocr_mask = MMC_OCR_3V2_MASK;
	}
	return plat;
}

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct tegra_sdhci_platform_data *plat;
	struct sdhci_tegra *tegra_host;
	int rc;

	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);

	plat = pdev->dev.platform_data;

	if (plat == NULL)
		plat = sdhci_tegra_dt_parse_pdata(pdev);

	if (plat == NULL) {
		dev_err(mmc_dev(host->mmc), "missing platform data\n");
		rc = -ENXIO;
		goto err_no_plat;
	}

	tegra_host = devm_kzalloc(&pdev->dev, sizeof(*tegra_host), GFP_KERNEL);
	if (!tegra_host) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra_host\n");
		rc = -ENOMEM;
		goto err_no_plat;
	}

	tegra_host->plat = plat;
	pdev->dev.platform_data = plat;
	tegra_host->sd_stat_head =
		devm_kzalloc(&pdev->dev, sizeof(struct sdhci_tegra_sd_stats),
			     GFP_KERNEL);
	if (!tegra_host->sd_stat_head) {
		dev_err(mmc_dev(host->mmc), "failed to allocate sd_stat_head\n");
		rc = -ENOMEM;
		goto err_power_req;
	}
	tegra_host->soc_data = soc_data;

	pltfm_host->priv = tegra_host;

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->mmc_data.embedded_sdio)
		mmc_set_embedded_sdio_data(host->mmc,
			&plat->mmc_data.embedded_sdio->cis,
			&plat->mmc_data.embedded_sdio->cccr,
			plat->mmc_data.embedded_sdio->funcs,
			plat->mmc_data.embedded_sdio->num_funcs);
#endif

	if (gpio_is_valid(plat->power_gpio)) {
		rc = gpio_request(plat->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			goto err_power_req;
		}
		gpio_direction_output(plat->power_gpio, 1);
	}

	if (gpio_is_valid(plat->cd_gpio)) {
		rc = gpio_request(plat->cd_gpio, "sdhci_cd");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate cd gpio\n");
			goto err_cd_req;
		}
		gpio_direction_input(plat->cd_gpio);

		tegra_host->card_present =
			(gpio_get_value_cansleep(plat->cd_gpio) == 0);

		rc = request_threaded_irq(gpio_to_irq(plat->cd_gpio), NULL,
				 carddetect_irq,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				 mmc_hostname(host->mmc), host);

		if (rc)	{
			dev_err(mmc_dev(host->mmc), "request irq error\n");
			goto err_cd_irq_req;
		}
		rc = enable_irq_wake(gpio_to_irq(plat->cd_gpio));
		if (rc < 0)
			dev_err(mmc_dev(host->mmc),
				"SD card wake-up event registration"
					"failed with eroor: %d\n", rc);

	} else if (plat->mmc_data.register_status_notify) {
		plat->mmc_data.register_status_notify(sdhci_status_notify_cb, host);
	}

	if (plat->mmc_data.status) {
		plat->mmc_data.card_present = plat->mmc_data.status(mmc_dev(host->mmc));
	}

	if (gpio_is_valid(plat->wp_gpio)) {
		rc = gpio_request(plat->wp_gpio, "sdhci_wp");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate wp gpio\n");
			goto err_wp_req;
		}
		gpio_direction_input(plat->wp_gpio);
	}

	/*
	 * If there is no card detect gpio, assume that the
	 * card is always present.
	 */
	if (!gpio_is_valid(plat->cd_gpio))
		tegra_host->card_present = 1;

	if (plat->mmc_data.ocr_mask & SDHOST_1V8_OCR_MASK) {
		tegra_host->vddio_min_uv = SDHOST_LOW_VOLT_MIN;
		tegra_host->vddio_max_uv = SDHOST_LOW_VOLT_MAX;
	} else if (plat->mmc_data.ocr_mask & MMC_OCR_2V8_MASK) {
			tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_2V8;
			tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	} else if (plat->mmc_data.ocr_mask & MMC_OCR_3V2_MASK) {
			tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_3V2;
			tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	} else {
		/*
		 * Set the minV and maxV to default
		 * voltage range of 2.7V - 3.6V
		 */
		tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_MIN;
		tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	}

	tegra_host->vdd_io_reg = regulator_get(mmc_dev(host->mmc),
							"vddio_sdmmc");
	if (IS_ERR_OR_NULL(tegra_host->vdd_io_reg)) {
		dev_info(mmc_dev(host->mmc), "%s regulator not found: %ld."
			"Assuming vddio_sdmmc is not required.\n",
			"vddio_sdmmc", PTR_ERR(tegra_host->vdd_io_reg));
		tegra_host->vdd_io_reg = NULL;
	} else {
		rc = regulator_set_voltage(tegra_host->vdd_io_reg,
			tegra_host->vddio_min_uv,
			tegra_host->vddio_max_uv);
		if (rc) {
			dev_err(mmc_dev(host->mmc), "%s regulator_set_voltage failed: %d",
				"vddio_sdmmc", rc);
			regulator_put(tegra_host->vdd_io_reg);
			tegra_host->vdd_io_reg = NULL;
		}
	}

	tegra_host->vdd_slot_reg = regulator_get(mmc_dev(host->mmc),
							"vddio_sd_slot");
	if (IS_ERR_OR_NULL(tegra_host->vdd_slot_reg)) {
		dev_info(mmc_dev(host->mmc), "%s regulator not found: %ld."
			" Assuming vddio_sd_slot is not required.\n",
			"vddio_sd_slot", PTR_ERR(tegra_host->vdd_slot_reg));
		tegra_host->vdd_slot_reg = NULL;
	}

	if (tegra_host->card_present) {
		if (tegra_host->vdd_slot_reg)
			regulator_enable(tegra_host->vdd_slot_reg);
		if (tegra_host->vdd_io_reg)
			regulator_enable(tegra_host->vdd_io_reg);
		tegra_host->is_rail_enabled = 1;
	}

	pm_runtime_enable(&pdev->dev);
	pltfm_host->clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(pltfm_host->clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(pltfm_host->clk);
		goto err_clk_get;
	}
	pm_runtime_get_sync(&pdev->dev);
	rc = clk_prepare_enable(pltfm_host->clk);
	if (rc != 0)
		goto err_clk_put;

	if (!strcmp(dev_name(mmc_dev(host->mmc)), "sdhci-tegra.3")) {
		tegra_host->emc_clk = clk_get(mmc_dev(host->mmc), "emc");
		if (IS_ERR(tegra_host->emc_clk)) {
			dev_err(mmc_dev(host->mmc), "clk err\n");
			rc = PTR_ERR(tegra_host->emc_clk);
			goto err_clk_put;
		}
		tegra_host->emc_max_clk =
			clk_round_rate(tegra_host->emc_clk, ULONG_MAX);
	}

	pltfm_host->priv = tegra_host;
	tegra_host->clk_enabled = true;
	tegra_host->max_clk_limit = plat->max_clk_limit;
	tegra_host->ddr_clk_limit = plat->ddr_clk_limit;
	tegra_host->instance = pdev->id;
	tegra_host->dpd = tegra_io_dpd_get(mmc_dev(host->mmc));

	host->mmc->pm_caps |= plat->pm_caps;
	host->mmc->pm_flags |= plat->pm_flags;

	host->mmc->caps |= MMC_CAP_ERASE;
	/* enable 1/8V DDR capable */
	host->mmc->caps |= MMC_CAP_1_8V_DDR;
	if (plat->is_8bit)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	host->mmc->caps |= MMC_CAP_SDIO_IRQ;
	host->mmc->pm_caps |= MMC_PM_KEEP_POWER | MMC_PM_IGNORE_PM_NOTIFY;
	if (plat->mmc_data.built_in) {
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
	}
	host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;

	tegra_sdhost_min_freq = TEGRA_SDHOST_MIN_FREQ;
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	tegra_host->hw_ops = &tegra_2x_sdhci_ops;
	tegra_sdhost_std_freq = TEGRA2_SDHOST_STD_FREQ;
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
	tegra_host->hw_ops = &tegra_3x_sdhci_ops;
	tegra_sdhost_std_freq = TEGRA3_SDHOST_STD_FREQ;
#else
	tegra_host->hw_ops = &tegra_11x_sdhci_ops;
	tegra_sdhost_std_freq = TEGRA3_SDHOST_STD_FREQ;
#endif
	rc = sdhci_add_host(host);
	sdhci_tegra_error_stats_debugfs(host);
	if (rc)
		goto err_add_host;

	/* Enable async suspend/resume to reduce LP0 latency */
	device_enable_async_suspend(&pdev->dev);

	return 0;

err_add_host:
	clk_put(tegra_host->emc_clk);
	clk_disable_unprepare(pltfm_host->clk);
	pm_runtime_put_sync(&pdev->dev);
err_clk_put:
	clk_put(pltfm_host->clk);
err_clk_get:
	if (gpio_is_valid(plat->wp_gpio))
		gpio_free(plat->wp_gpio);
err_wp_req:
	if (gpio_is_valid(plat->cd_gpio))
		free_irq(gpio_to_irq(plat->cd_gpio), host);
err_cd_irq_req:
	if (gpio_is_valid(plat->cd_gpio))
		gpio_free(plat->cd_gpio);
err_cd_req:
	if (gpio_is_valid(plat->power_gpio))
		gpio_free(plat->power_gpio);
err_power_req:
err_no_plat:
	sdhci_pltfm_free(pdev);
	return rc;
}

static int sdhci_tegra_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	sdhci_remove_host(host, dead);

	disable_irq_wake(gpio_to_irq(plat->cd_gpio));

	if (tegra_host->vdd_slot_reg) {
		regulator_disable(tegra_host->vdd_slot_reg);
		regulator_put(tegra_host->vdd_slot_reg);
	}

	if (tegra_host->vdd_io_reg) {
		regulator_disable(tegra_host->vdd_io_reg);
		regulator_put(tegra_host->vdd_io_reg);
	}

	if (gpio_is_valid(plat->wp_gpio))
		gpio_free(plat->wp_gpio);

	if (gpio_is_valid(plat->cd_gpio)) {
		free_irq(gpio_to_irq(plat->cd_gpio), host);
		gpio_free(plat->cd_gpio);
	}

	if (gpio_is_valid(plat->power_gpio))
		gpio_free(plat->power_gpio);

	if (tegra_host->clk_enabled) {
		clk_disable_unprepare(pltfm_host->clk);
		pm_runtime_put_sync(&pdev->dev);
	}
	clk_put(pltfm_host->clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.owner	= THIS_MODULE,
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
