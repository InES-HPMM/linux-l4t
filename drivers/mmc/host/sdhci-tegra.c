/*
 * Copyright (C) 2010 Google, Inc.
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
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <asm/gpio.h>

#include <linux/platform_data/mmc-sdhci-tegra.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20

#define NVQUIRK_FORCE_SDHCI_SPEC_200	BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET	BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300	BIT(2)

#define SDHCI_VENDOR_CLOCK_CNTRL       0x100
#define SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK	0x1
#define SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT	8

#define SDHCI_VENDOR_MISC_CNTRL		0x120
#define SDHCI_VENDOR_MISC_CNTRL_SDMMC_SPARE0_ENABLE_SD_3_0	0x20

#define SDHOST_1V8_OCR_MASK		0x8

static unsigned int tegra3_sdhost_max_clk[4] = {
	208000000,	104000000,	208000000,	104000000 };

struct tegra_sdhci_hw_ops{
	/* Set the internal clk and card clk.*/
	void	(*set_card_clock)(struct sdhci_host *sdhci, unsigned int clock);
	/* Post reset vendor registers configuration */
	void	(*sdhost_init)(struct sdhci_host *sdhci);
};

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static struct tegra_sdhci_hw_ops tegra_2x_sdhci_ops = {
};
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static void tegra_3x_sdhci_set_card_clock(struct sdhci_host *sdhci, unsigned int clock);
static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci);

static struct tegra_sdhci_hw_ops tegra_3x_sdhci_ops = {
	.set_card_clock = tegra_3x_sdhci_set_card_clock,
	.sdhost_init = tegra3_sdhci_post_reset_init,
};
#endif

struct sdhci_tegra_soc_data {
	struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
	/* Pointer to the chip specific HW ops */
	struct tegra_sdhci_hw_ops *hw_ops;
};

struct sdhci_tegra {
	const struct tegra_sdhci_platform_data *plat;
	const struct sdhci_tegra_soc_data *soc_data;
	bool	clk_enabled;
	struct regulator *vdd_io_reg;
	struct regulator *vdd_slot_reg;
	/* Host controller instance */
	unsigned int instance;
	/* vddio_min */
	unsigned int vddio_min_uv;
	/* vddio_max */
	unsigned int vddio_max_uv;
};

static u32 tegra_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val;

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		/* Use wp_gpio here instead? */
		val = readl(host->ioaddr + reg);
		return val | SDHCI_WRITE_PROTECT;
	}

	return readl(host->ioaddr + reg);
}

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}
	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

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
}

static unsigned int tegra_sdhci_get_ro(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	if (!gpio_is_valid(plat->wp_gpio))
		return -1;

	return gpio_get_value(plat->wp_gpio);
}

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static void tegra3_sdhci_post_reset_init(struct sdhci_host *sdhci)
{
	u16 ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	/* Set the base clock frequency */
	ctrl = sdhci_readw(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	ctrl &= ~(0xFF << SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT);
	ctrl |= (tegra3_sdhost_max_clk[tegra_host->instance] / 1000000) <<
		SDHCI_VENDOR_CLOCK_CNTRL_BASE_CLK_FREQ_SHIFT;
	sdhci_writew(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

	/* Enable SDHOST v3.0 support */
	ctrl = sdhci_readw(sdhci, SDHCI_VENDOR_MISC_CNTRL);
	ctrl |= SDHCI_VENDOR_MISC_CNTRL_SDMMC_SPARE0_ENABLE_SD_3_0;
	sdhci_writew(sdhci, ctrl, SDHCI_VENDOR_MISC_CNTRL);
}
#endif

static int tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int uhs)
{
	u16 clk, ctrl_2;
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	/* Select Bus Speed Mode for host */
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

static void tegra_sdhci_reset_exit(struct sdhci_host *sdhci, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (mask & SDHCI_RESET_ALL) {
		if (soc_data->hw_ops->sdhost_init)
			soc_data->hw_ops->sdhost_init(sdhci);
	}
}

static void sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *host = (struct sdhci_host *)dev_id;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	unsigned int status, oldstat;

	pr_debug("%s: card_present %d\n", mmc_hostname(host->mmc),
		card_present);

	if (!plat->mmc_data.status) {
		mmc_detect_change(host->mmc, 0);
		return;
	}

	status = plat->mmc_data.status(mmc_dev(host->mmc));

	oldstat = plat->mmc_data.card_present;
	plat->mmc_data.card_present = status;
	if (status ^ oldstat) {
		pr_debug("%s: Slot status change detected (%d -> %d)\n",
			mmc_hostname(host->mmc), oldstat, status);
		if (status && !plat->mmc_data.built_in)
			mmc_detect_change(host->mmc, (5 * HZ) / 2);
		else
			mmc_detect_change(host->mmc, 0);
	}
}

static irqreturn_t carddetect_irq(int irq, void *data)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)data;

	tasklet_schedule(&sdhost->card_tasklet);
	return IRQ_HANDLED;
};

static void tegra_sdhci_reset_exit(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (!(mask & SDHCI_RESET_ALL))
		return;

	/* Erratum: Enable SDHCI spec v3.00 support */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDHCI_SPEC_300) {
		u32 misc_ctrl;

		misc_ctrl = sdhci_readb(host, SDHCI_TEGRA_VENDOR_MISC_CTRL);
		misc_ctrl |= SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
		sdhci_writeb(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);
	}
}

static int tegra_sdhci_buswidth(struct sdhci_host *host, int bus_width)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	u32 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
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
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	return 0;
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
	 * - Enable PADPIPE_CLK_OVERRIDE in the vendr clk cntrl register.
	 * - Enable internal clock.
	 * - Wait for 5 usec and do a dummy write.
	 * - Poll for clk stable and disable PADPIPE_CLK_OVERRIDE.
	 */
set_clk:
	/* Enable PADPIPE clk override */
	ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

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

	/* Disable PADPIPE clk override */
	ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);

out:
	sdhci->clock = clock;
}
#endif

static void tegra_sdhci_set_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	u8 ctrl;

	pr_debug("%s %s %u enabled=%u\n", __func__,
		mmc_hostname(sdhci->mmc), clock, tegra_host->clk_enabled);

	if (clock) {
		if (!tegra_host->clk_enabled) {
			clk_prepare_enable(pltfm_host->clk);
			ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
			ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
			tegra_host->clk_enabled = true;
		}
		if (soc_data->hw_ops->set_card_clock)
			soc_data->hw_ops->set_card_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		if (soc_data->hw_ops->set_card_clock)
			soc_data->hw_ops->set_card_clock(sdhci, clock);
		ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
		ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
		sdhci_writeb(sdhci, ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
		clk_disable_unprepare(pltfm_host->clk);
		tegra_host->clk_enabled = false;
	}
}

static void tegra_sdhci_suspend(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	tegra_sdhci_set_clock(sdhci, 0);

	/* Disable the power rails if any */
	if (tegra_host->vdd_slot_reg)
		regulator_disable(tegra_host->vdd_slot_reg);
	if (tegra_host->vdd_io_reg)
		regulator_disable(tegra_host->vdd_io_reg);
}

static void tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned long timeout;

	/* Enable the power rails if any */
	if (tegra_host->vdd_io_reg)
		regulator_enable(tegra_host->vdd_io_reg);
	if (tegra_host->vdd_slot_reg)
		regulator_enable(tegra_host->vdd_slot_reg);

	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);

	/* Reset the controller and power on if MMC_KEEP_POWER flag is set*/
	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER) {
		sdhci_writeb(sdhci, SDHCI_RESET_ALL, SDHCI_SOFTWARE_RESET);

		/* Wait max 100 ms */
		timeout = 100;

		/* hw clears the bit when it's done */
		while (sdhci_readb(sdhci, SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) {
			if (timeout == 0) {
				printk(KERN_ERR "%s: Reset 0x%x never completed.\n",
					mmc_hostname(sdhci->mmc), (int)SDHCI_RESET_ALL);
				return -ETIMEDOUT;
			}
			timeout--;
			mdelay(1);
		}

		sdhci_writeb(sdhci, SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
		sdhci->pwr = 0;
	}
}

static const struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_l     = tegra_sdhci_readl,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.platform_bus_width = tegra_sdhci_buswidth,
	.platform_reset_exit = tegra_sdhci_reset_exit,
	.set_clock  = tegra_sdhci_set_clock,
	.platform_suspend	= tegra_sdhci_suspend,
	.platform_resume	= tegra_sdhci_resume,
	.platform_reset_exit = tegra_sdhci_reset_exit,
	.set_uhs_signaling = tegra_sdhci_set_uhs_signaling,
};

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra20 = {
	.pdata = &sdhci_tegra20_pdata,
	.nvquirks = NVQUIRK_FORCE_SDHCI_SPEC_200 |
		    NVQUIRK_ENABLE_BLOCK_GAP_DET,
	.hw_ops = &tegra_2x_sdhci_ops,
};
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static struct sdhci_pltfm_data sdhci_tegra30_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_NONSTANDARD_CLOCK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra30 = {
	.pdata = &sdhci_tegra30_pdata,
	.nvquirks = NVQUIRK_ENABLE_SDHCI_SPEC_300,
	.hw_ops = &tegra_3x_sdhci_ops,
};
#endif

static const struct of_device_id sdhci_tegra_dt_match[] = {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra30 },
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
	struct clk *clk;
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

		rc = request_irq(gpio_to_irq(plat->cd_gpio), carddetect_irq,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				 mmc_hostname(host->mmc), host);

		if (rc)	{
			dev_err(mmc_dev(host->mmc), "request irq error\n");
			goto err_cd_irq_req;
		}

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


	if (!plat->mmc_data.built_in) {
		if (plat->mmc_data.ocr_mask & SDHOST_1V8_OCR_MASK) {
			tegra_host->vddio_min_uv = 1800000;
			tegra_host->vddio_max_uv = 1800000;
		} else {
			/*
			 * Set the minV and maxV to default
			 * voltage range of 2.7V - 3.6V
			 */
			tegra_host->vddio_min_uv = 2700000;
			tegra_host->vddio_max_uv = 3600000;
		}
		tegra_host->vdd_io_reg = regulator_get(mmc_dev(host->mmc), "vddio_sdmmc");
		if (IS_ERR_OR_NULL(tegra_host->vdd_io_reg)) {
			dev_err(mmc_dev(host->mmc), "%s regulator not found: %ld\n",
				"vddio_sdmmc", PTR_ERR(tegra_host->vdd_io_reg));
			tegra_host->vdd_io_reg = NULL;
		} else {
			rc = regulator_set_voltage(tegra_host->vdd_io_reg,
				tegra_host->vddio_min_uv,
				tegra_host->vddio_max_uv);
			if (rc) {
				dev_err(mmc_dev(host->mmc), "%s regulator_set_voltage failed: %d",
					"vddio_sdmmc", rc);
			} else {
				regulator_enable(tegra_host->vdd_io_reg);
			}
		}

		tegra_host->vdd_slot_reg = regulator_get(mmc_dev(host->mmc), "vddio_sd_slot");
		if (IS_ERR_OR_NULL(tegra_host->vdd_slot_reg)) {
			dev_err(mmc_dev(host->mmc), "%s regulator not found: %ld\n",
				"vddio_sd_slot", PTR_ERR(tegra_host->vdd_slot_reg));
			tegra_host->vdd_slot_reg = NULL;
		} else {
			regulator_enable(tegra_host->vdd_slot_reg);
		}
	}

	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	rc = clk_prepare_enable(clk);
	if (rc != 0)
		goto err_clk_put;
	pltfm_host->clk = clk;
	tegra_host->clk_enabled = true;
	tegra_host->instance = pdev->id;

	host->mmc->pm_caps = plat->pm_flags;

	host->mmc->caps |= MMC_CAP_ERASE;
	/* enable 1/8V DDR capable */
	host->mmc->caps |= MMC_CAP_1_8V_DDR;
	if (plat->is_8bit)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	host->mmc->caps |= MMC_CAP_SDIO_IRQ;

	if (plat->mmc_data.built_in) {
		host->mmc->caps |= MMC_CAP_NONREMOVABLE;
	}
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	/* Do not turn OFF embedded sdio cards as it support Wake on Wireless */
	if (plat->mmc_data.embedded_sdio)
		host->mmc->pm_flags = MMC_PM_KEEP_POWER;
#endif

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	return 0;

err_add_host:
	clk_disable_unprepare(pltfm_host->clk);
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

	if (tegra_host->clk_enabled)
		clk_disable_unprepare(pltfm_host->clk);
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
