/*
 * tegra210_adma.c - Tegra ADMA driver.
 *
 * Author: Dara Ramesh <dramesh@nvidia.com>
 *
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/*
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include  "tegra210_adma.h"

#define DRV_NAME "tegra210-adma"

/* TEGRA210 ADMA driver context */
struct tegra210_adma_ctx {
	struct device		*dev;
	struct clk		*clk;
	struct regmap		*regmap;
	void __iomem		*regs;
	u32			enable_count;
	bool adma_initialized;
};

static struct tegra210_adma_ctx *tegra210_adma;

static DEFINE_MUTEX(tegra210_adma_lock);
static DECLARE_BITMAP(channel_usage, TEGRA210_ADMA_MAX_CHANNELS);
static struct tegra210_adma_channel adma_channels[TEGRA210_ADMA_MAX_CHANNELS];

static void tegra210_adma_init(void);
static void tegra210_adma_update_hw(struct tegra210_adma_channel *ch,
	struct tegra210_adma_req *req);
static void tegra210_adma_oneshot_handle(struct tegra210_adma_channel *ch);

static void tegra210_adma_update_hw(struct tegra210_adma_channel *ch,
	struct tegra210_adma_req *req)
{
	struct tegra210_adma_ctx *adma = tegra210_adma;
	u32 val, valfifo;

	dev_dbg(adma->dev,
		"ch.id = %d, req.size = %d, ahub.ch.req = %d, dir = %d ",
		ch->id, req->size, req->ahub_ch_request,
		req->transfer_direction);

	regmap_write(adma->regmap, (ch->addr + TEGRA210_ADMA_CH_TC),
		    req->size);

	regmap_read(adma->regmap, (ch->addr + TEGRA210_ADMA_CH_CTRL),
		    &val);
	regmap_read(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_AHUB_FIFO_CTRL),
		    &valfifo);
	val  &= ~TEGRA210_ADMA_CH_CTRL_TRANSFER_DIRECTION_MASK;
	val |=  req->transfer_direction <<
		    TEGRA210_ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT;
	val &= ~TEGRA210_ADMA_CH_CTRL_TRANSFER_MODE_MASK;
	val |=  ch->mode << TEGRA210_ADMA_CH_CTRL_TRANSFER_MODE_SHIFT;
	val |=  1 << TEGRA210_ADMA_CH_CTRL_FLOWCTRL_ENABLE_SHIFT;

	if (req->transfer_direction == MEMORY_TO_AHUB) {
		val &= ~TEGRA210_ADMA_CH_CTRL_TX_REQUEST_SELECT_MASK;
		val |=  req->ahub_ch_request <<
		    TEGRA210_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT;
		valfifo &= ~TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_MASK;
		valfifo |= 7 <<
			TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT;
		regmap_write(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_LOWER_SOURCE_ADDR),
		    req->source_addr);
	} else if (req->transfer_direction == AHUB_TO_MEMORY) {
		val &= ~TEGRA210_ADMA_CH_CTRL_RX_REQUEST_SELECT_MASK;
		val |=  req->ahub_ch_request <<
		    TEGRA210_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT;
		valfifo &= ~TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_MASK;
		valfifo |=  3 <<
			TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT;
		regmap_write(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_LOWER_TARGET_ADDR),
		    req->dest_addr);

	} else if (req->transfer_direction == MEMORY_TO_MEMORY) {
		regmap_write(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_LOWER_TARGET_ADDR),
		    req->dest_addr);
		regmap_write(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_LOWER_SOURCE_ADDR),
		    req->source_addr);
	}
	val |=  (1 << TEGRA210_ADMA_CH_CTRL_FLOWCTRL_ENABLE_SHIFT);
	regmap_write(adma->regmap, ch->addr + TEGRA210_ADMA_CH_CTRL,
		    val);

	valfifo |=  (BURST_BASED <<
		    TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_FETCHING_POLICY_SHIFT);
	regmap_write(adma->regmap,
	    (ch->addr + TEGRA210_ADMA_CH_AHUB_FIFO_CTRL),
	    valfifo);

	regmap_read(adma->regmap, ch->addr + TEGRA210_ADMA_CH_CONFIG,
		    &val);
	val &= ~TEGRA210_ADMA_CH_CONFIG_BURST_SIZE_MASK;
	val |=  (req->fixed_burst_size <<
		    TEGRA210_ADMA_CH_CONFIG_BURST_SIZE_SHIFT);
	regmap_write(adma->regmap, ch->addr + TEGRA210_ADMA_CH_CONFIG,
		    val);
}

int tegra210_adma_enqueue_req(struct tegra210_adma_channel *ch,
	struct tegra210_adma_req *req)
{
	if (req->size > TEGRA210_ADMA_MAX_TRANSFER_SIZE ||
		req->size & 0x3 || req->source_addr & 0x3 ||
		req->dest_addr & 0x3) {
		pr_err("Invalid DMA request for channel %d\n", ch->id);
		return -EINVAL;
	}
	tegra210_adma_update_hw(ch, req);
	return 0;
}
EXPORT_SYMBOL(tegra210_adma_enqueue_req);

struct tegra210_adma_channel *tegra210_adma_allocate_channel(int mode,
		const char namefmt[], ...)
{
	struct tegra210_adma_channel *ch = NULL;
	tegra210_adma_isr_handler isr_handler = NULL;
	va_list args;
	int channel;

	if (!tegra210_adma->adma_initialized)
		return NULL;

	mutex_lock(&tegra210_adma_lock);

	channel = find_first_zero_bit(channel_usage,
			ARRAY_SIZE(adma_channels));

	dev_dbg(tegra210_adma->dev, "allocated channel = %d", channel);
	if (channel >= ARRAY_SIZE(adma_channels)) {
		pr_info("all adma channel are used :\n");
		goto out;
	}

	if (mode)
		isr_handler = tegra210_adma_oneshot_handle;
	else
		pr_err("Bad channel mode for DMA\n");

	if (!isr_handler)
		goto out;

	__set_bit(channel, channel_usage);
	ch = &adma_channels[channel];
	ch->mode = mode;
	ch->isr_handler = isr_handler;
	ch->adma_is_paused = false;
	va_start(args, namefmt);
	vsnprintf(ch->client_name, sizeof(ch->client_name),
		namefmt, args);
	va_end(args);
	 pm_runtime_get_sync(tegra210_adma->dev);
out:
	mutex_unlock(&tegra210_adma_lock);
	return ch;
}
EXPORT_SYMBOL(tegra210_adma_allocate_channel);

void tegra210_adma_free_channel(struct tegra210_adma_channel *ch)
{
	struct tegra210_adma_ctx *adma = tegra210_adma;

	dev_vdbg(adma->dev, "freed adma ch. = %d", ch->id);
	regmap_write(adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_INT_CLEAR), 0x1);
	mutex_lock(&tegra210_adma_lock);
	__clear_bit(ch->id, channel_usage);
	memset(ch->client_name, 0, sizeof(ch->client_name));
	ch->isr_handler = NULL;
	ch->callback = NULL;
	ch->cb_req = NULL;
	pm_runtime_put_sync(tegra210_adma->dev);
	mutex_unlock(&tegra210_adma_lock);
}
EXPORT_SYMBOL(tegra210_adma_free_channel);

int tegra210_adma_channel_transfer_status(struct tegra210_adma_channel *ch)
{
	struct tegra210_adma_ctx *adma = tegra210_adma;
	u32  val;
	regmap_read(adma->regmap, (ch->addr + TEGRA210_ADMA_CH_TRANSFER_STATUS),
			&val);
	dev_dbg(adma->dev, "%s ch. = %d transfer status %d", __func__, ch->id,
			val);
	return val;
}
EXPORT_SYMBOL(tegra210_adma_channel_transfer_status);

int tegra210_adma_get_channel_id(struct tegra210_adma_channel *ch)
{
	return ch->id;
}
EXPORT_SYMBOL(tegra210_adma_get_channel_id);

static void tegra210_adma_oneshot_handle(struct tegra210_adma_channel *ch)
{
	regmap_write(tegra210_adma->regmap, ch->addr + TEGRA210_ADMA_CH_CMD,
		    TRANSFER_ENABLE);
	return;
}

static irqreturn_t tegra210_adma_isr(int irq, void *data)
{
	return IRQ_HANDLED;
}

/* Regmap callback functions */
static bool tegra210_adma_rw_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMA_CHANNEL_OFFSET;

	switch (reg) {
	case TEGRA210_ADMA_CH_CMD:
	case TEGRA210_ADMA_CH_CTRL:
	case TEGRA210_ADMA_CH_TC:
	case TEGRA210_ADMA_CH_CONFIG:
	case TEGRA210_ADMA_CH_INT_SET:
	case TEGRA210_ADMA_CH_INT_CLEAR:
	case TEGRA210_ADMA_CH_AHUB_FIFO_CTRL:
	case TEGRA210_ADMA_CH_LOWER_SOURCE_ADDR:
	case TEGRA210_ADMA_CH_LOWER_TARGET_ADDR:
	case TEGRA210_ADMA_CH_LOWER_DESC_ADDR:
	case TEGRA210_ADMA_GLOBAL_CMD:
	case TEGRA210_ADMA_GLOBAL_SOFT_RESET:
	case TEGRA210_ADMA_GLOBAL_CG:
	case TEGRA210_ADMA_GLOBAL_STATUS:
	case TEGRA210_ADMA_GLOBAL_INT_MASK:
	case TEGRA210_ADMA_GLOBAL_INT_SET:
	case TEGRA210_ADMA_GLOBAL_INT_CLEAR:
	case TEGRA210_ADMA_GLOBAL_CTRL:
		return true;
	default:
		return false;
	};
}

static bool tegra210_adma_rd_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMA_CHANNEL_OFFSET;

	switch (reg) {
	case TEGRA210_ADMA_CH_STATUS:
	case TEGRA210_ADMA_CH_INT_STATUS:
	case TEGRA210_ADMA_CH_TC_STATUS:
	case TEGRA210_ADMA_CH_TRANSFER_STATUS:
	case TEGRA210_ADMA_GLOBAL_STATUS:
	case TEGRA210_ADMA_GLOBAL_INT_STATUS:
	case TEGRA210_ADMA_GLOBAL_CH_INT_STATUS:
	case TEGRA210_ADMA_GLOBAL_CH_ENABLE_STATUS:
	case TEGRA210_ADMA_GLOBAL_TX_REQUESTORS:
	case TEGRA210_ADMA_GLOBAL_RX_REQUESTORS:
	case TEGRA210_ADMA_GLOBAL_TRIGGERS:
	case TEGRA210_ADMA_GLOBAL_TRANSFER_ERROR_LOG:
	case TEGRA210_ADMA_CH_CMD:
	case TEGRA210_ADMA_CH_CTRL:
	case TEGRA210_ADMA_CH_TC:
	case TEGRA210_ADMA_CH_CONFIG:
	case TEGRA210_ADMA_CH_INT_SET:
	case TEGRA210_ADMA_CH_INT_CLEAR:
	case TEGRA210_ADMA_CH_AHUB_FIFO_CTRL:
	case TEGRA210_ADMA_CH_LOWER_SOURCE_ADDR:
	case TEGRA210_ADMA_CH_LOWER_TARGET_ADDR:
	case TEGRA210_ADMA_CH_LOWER_DESC_ADDR:
	case TEGRA210_ADMA_GLOBAL_CMD:
	case TEGRA210_ADMA_GLOBAL_SOFT_RESET:
	case TEGRA210_ADMA_GLOBAL_CG:
	case TEGRA210_ADMA_GLOBAL_INT_MASK:
	case TEGRA210_ADMA_GLOBAL_INT_SET:
	case TEGRA210_ADMA_GLOBAL_INT_CLEAR:
	case TEGRA210_ADMA_GLOBAL_CTRL:
	case TEGRA210_ADMA_CH_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static bool tegra210_adma_volatile_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMA_CHANNEL_OFFSET;
	switch (reg) {
	 /* SOFT_RESET bit is auto cleared by HW */
	case TEGRA210_ADMA_CH_SOFT_RESET:
	case TEGRA210_ADMA_CH_INT_STATUS:
	case TEGRA210_ADMA_CH_TC_STATUS:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_adma_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_ADMA_LAST_REG,
	.writeable_reg = tegra210_adma_rw_reg,
	.readable_reg = tegra210_adma_rd_reg,
	.volatile_reg = tegra210_adma_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_adma_runtime_suspend(struct device *dev)
{
	struct tegra210_adma_ctx *adma = dev_get_drvdata(dev);
	dev_dbg(adma->dev, "%s", __func__);
	regcache_cache_only(adma->regmap, true);
#ifndef CONFIG_MACH_GRENADA
	tegra210_adma_disable_clk();
	clk_disable_unprepare(adma->clk);
#endif
	return 0;
}

static int tegra210_adma_runtime_resume(struct device *dev)
{
	struct tegra210_adma_ctx *adma = dev_get_drvdata(dev);
#ifndef CONFIG_MACH_GRENADA
	int ret;

	dev_dbg(adma->dev, "%s", __func__);
	ret = clk_prepare_enable(adma->clk);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}
#endif
	regcache_cache_only(adma->regmap, false);
	return 0;
}

static void tegra210_adma_init(void)
{
	int ret = 0;
	int i = 0;
	unsigned int irq;

	dev_dbg(tegra210_adma->dev, "%s", __func__);
	bitmap_fill(channel_usage, TEGRA210_ADMA_MAX_CHANNELS);
	for (i = 0; i < TEGRA210_ADMA_MAX_CHANNELS; i++) {
		struct tegra210_adma_channel *ch = &adma_channels[i];
		ch->id = i;
		ch->isr_handler = NULL;
		snprintf(ch->name, TEGRA210_ADMA_NAME_SIZE, "adma_channel_%d",
			    i);
		memset(ch->client_name, 0, sizeof(ch->client_name));
		ch->addr = (TEGRA210_ADMA_CHANNEL_OFFSET * i);
		if (0) {
			/* TODO add irq base address */
			irq =  i;
			ret = request_irq(irq, tegra210_adma_isr, 0,
			    adma_channels[i].name, ch);
			if (ret) {
				pr_err("Failed to register IRQ %d for ADMA %d\n",
					irq, i);
				goto fail;
			}
			ch->irq = irq;
		}
	__clear_bit(i, channel_usage);
	}
	tegra210_adma->adma_initialized = true;
	return;
fail:
	for (i = 0; i <= TEGRA210_ADMA_MAX_CHANNELS; i++) {
		struct tegra210_adma_channel *ch = &adma_channels[i];
		if (ch->irq)
			free_irq(ch->irq, ch);
	}
	return;
}

void tegra210_adma_channel_enable(struct tegra210_adma_channel *ch, bool en)
{
	struct tegra210_adma_ctx *adma = tegra210_adma;

	dev_vdbg(adma->dev, "adma channel id = %d, enable = %d ",
		    ch->id, en);
	regmap_write(adma->regmap, ch->addr + TEGRA210_ADMA_CH_CMD,
		    en);
	return;
}
EXPORT_SYMBOL(tegra210_adma_channel_enable);

int tegra210_adma_channel_status_read(struct tegra210_adma_channel *ch)
{
	u32 val;
	regmap_read(tegra210_adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_INT_STATUS), &val);
	return val;
}
EXPORT_SYMBOL(tegra210_adma_channel_status_read);

int tegra210_adma_channel_tc_status(struct tegra210_adma_channel *ch)
{
	u32 val;
	regmap_read(tegra210_adma->regmap,
		    (ch->addr + TEGRA210_ADMA_CH_TC_STATUS),
		    &val);
	return val;
}
EXPORT_SYMBOL(tegra210_adma_channel_tc_status);

void tegra210_adma_set_channel_pause(bool en)
{
	regmap_update_bits(tegra210_adma->regmap, TEGRA210_ADMA_CH_CTRL,
		   TEGRA210_ADMA_CH_CTRL_TRANSFER_PAUSE_MASK,
		   en);
	return;
}
EXPORT_SYMBOL(tegra210_adma_set_channel_pause);

void tegra210_adma_global_enable(bool en)
{
	struct tegra210_adma_ctx *adma = tegra210_adma;

	dev_vdbg(adma->dev, "adma global enable = %d, ref.count = %d",
		en, adma->enable_count);

	if (en) {
		adma->enable_count++;
		if (adma->enable_count > 1)
			return;
	} else {
		adma->enable_count--;
		if (adma->enable_count > 0)
			return;
	}

	regmap_write(adma->regmap, TEGRA210_ADMA_GLOBAL_CMD, en);
	return;
}
EXPORT_SYMBOL(tegra210_adma_global_enable);

void tegra210_adma_set_global_int_mask(bool en)
{
	regmap_write(tegra210_adma->regmap, TEGRA210_ADMA_GLOBAL_INT_MASK, en);
	return;
}

void tegra210_adma_set_global_int()
{
	regmap_write(tegra210_adma->regmap, TEGRA210_ADMA_GLOBAL_INT_SET, true);
	return;
}

void tegra210_adma_clear_global_int()
{
	regmap_write(tegra210_adma->regmap, TEGRA210_ADMA_GLOBAL_INT_CLEAR,
		true);
	return;
}

void tegra210_adma_set_gloabal_pause(bool en)
{
	regmap_update_bits(tegra210_adma->regmap, TEGRA210_ADMA_GLOBAL_CTRL,
		   TEGRA210_ADMA_GLOBAL_CTRL_TRANSFER_PAUSE_MASK,
		   en);
	return;
}
EXPORT_SYMBOL(tegra210_adma_set_gloabal_pause);

void tegra210_adma_set_global_soft_reset(bool en)
{
	regmap_write(tegra210_adma->regmap, TEGRA210_ADMA_GLOBAL_SOFT_RESET,
					en);
	return;
}

static int __init tegra210_adma_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	int ret = 0;

	dev_dbg(&pdev->dev, "adma: starting probe");

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No device tree node for ADMA driver");
		return -ENODEV;
	}

	tegra210_adma = devm_kzalloc(&pdev->dev,
			 sizeof(struct tegra210_adma_ctx),
			 GFP_KERNEL);
	if (!tegra210_adma) {
		dev_err(&pdev->dev, "Can't allocate adma context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_adma->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, tegra210_adma);

/*	tegra210_adma->clk = clk_get(&pdev->dev, "adma");
	if (IS_ERR(tegra210_adma->clk)) {
		dev_err(&pdev->dev, "Can't retrieve adma clock\n");
		ret = PTR_ERR(tegra210_adma->clk);
		goto err_free;
	}
*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_clk_put_adma;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_clk_put_adma;
	}

	tegra210_adma->regs = devm_ioremap(&pdev->dev, res->start,
					    resource_size(res));
	if (!tegra210_adma->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_clk_put_adma;
	}

	tegra210_adma->regmap = devm_regmap_init_mmio(&pdev->dev,
						tegra210_adma->regs,
						&tegra210_adma_regmap_config);
	if (IS_ERR(tegra210_adma->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(tegra210_adma->regmap);
		goto err_clk_put_adma;
	}
	regcache_cache_only(tegra210_adma->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_adma_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}
	tegra210_adma_init();
	return 0;
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk_put_adma:
/*	clk_put(tegra210_adma->clk);
err_free: */
	tegra210_adma = NULL;
exit:
	return 0;
}

static int tegra210_adma_remove(struct platform_device *pdev)
{
	struct tegra210_adma_ctx *adma;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_adma_runtime_suspend(&pdev->dev);

	adma = platform_get_drvdata(pdev);
	/* clk_put(adma->clk); */
	tegra210_adma = NULL;

	dev_dbg(&pdev->dev, "%s adma removed", __func__);

	return 0;
}

static const struct of_device_id tegra210_adma_of_match[] = {
	{ .compatible = "nvidia,tegra210-adma"},
	{ }
};

static const struct dev_pm_ops tegra210_adma_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_adma_runtime_suspend,
			   tegra210_adma_runtime_resume, NULL)
};

static struct platform_driver tegra210_adma_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_adma_of_match,
		.pm = &tegra210_adma_pm_ops,
	},
	.probe = tegra210_adma_probe,
	.remove = tegra210_adma_remove,
};
module_platform_driver(tegra210_adma_driver);

MODULE_AUTHOR("Dara Ramesh <dramesh@nvidia.com>");
MODULE_DESCRIPTION("Tegra APE ADMA driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
