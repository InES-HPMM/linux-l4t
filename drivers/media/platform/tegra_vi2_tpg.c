/*
 * Copyright 2015 Alban Bedel <alban.bedel@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 */

#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/clk.h>

#include <media/v4l2-common.h>
#include <media/v4l2-subdev.h>

#include "tegra_vi2.h"

struct tegra_tpg_color_regs {
	u32 freq;
	u32 freq_rate;
};

struct tegra_tpg_regs {
	u32 ctrl;
	u32 blank;
	u32 phase;
	struct tegra_tpg_color_regs color[3];
};

struct tegra_tpg {
	struct v4l2_subdev sd;
	struct tegra_tpg_regs __iomem *regs;
	struct v4l2_mbus_framefmt fmt;
	struct clk *pll_d_clk;
	unsigned mode;
	bool auto_inc;
};

#if 0
#define TPG_MAX_WIDTH 3840
#define TPG_MAX_HEIGHT 2160
#else
#define TPG_MAX_WIDTH 1280
#define TPG_MAX_HEIGHT 720
#endif

static int tegra_tpg_s_power(struct v4l2_subdev *sd, int enable)
{
	struct tegra_tpg *tpg = v4l2_get_subdevdata(sd);
	int err = 0;

	if (enable) {
		err = clk_prepare_enable(tpg->pll_d_clk);
		if (!err) {
			tegra_clk_cfg_ex(tpg->pll_d_clk,
					TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(tpg->pll_d_clk,
					TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(tpg->pll_d_clk,
					TEGRA_CLK_MIPI_CSI_OUT_ENB, 0);
		}
	} else {
		tegra_clk_cfg_ex(tpg->pll_d_clk,
				TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
		tegra_clk_cfg_ex(tpg->pll_d_clk,
				TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
		tegra_clk_cfg_ex(tpg->pll_d_clk,
				TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
		clk_disable_unprepare(tpg->pll_d_clk);
	}

	return err;
}

static int tegra_tpg_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	switch (index) {
	case 0:
		*code = V4L2_MBUS_FMT_SBGGR10_1X10;
		return 0;
	case 1:
		*code = V4L2_MBUS_FMT_RGB888_1X24;
		return 0;
	default:
		return -EINVAL;
	}
}

static int tegra_tpg_g_mbus_fmt(
	struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct tegra_tpg *tpg = v4l2_get_subdevdata(sd);

	*fmt = tpg->fmt;

	return 0;
}

static int tegra_tpg_try_mbus_fmt(
	struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	switch (fmt->code) {
	case V4L2_MBUS_FMT_RGB888_1X24:
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		break;
	default:
		return -EINVAL;
	}

	v4l_bound_align_image(&fmt->width, 16, TPG_MAX_WIDTH, 4,
				&fmt->height, 16, TPG_MAX_HEIGHT, 1, 16);

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int tegra_tpg_s_mbus_fmt(
	struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct tegra_tpg *tpg = v4l2_get_subdevdata(sd);
	int err;

	err = tegra_tpg_try_mbus_fmt(sd, fmt);
	if (err)
		return err;

	tpg->fmt = *fmt;

	return 0;
}

static int tegra_tpg_g_mbus_config(
	struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

	return 0;
}

static int tegra_tpg_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tegra_tpg *tpg = v4l2_get_subdevdata(sd);
	u32 mode = (tpg->mode & 3) << 2;

	if (!enable) {
		writel(0, &tpg->regs->ctrl);
		return 0;
	}

	if (tpg->auto_inc)
		mode |= BIT(1);

	if (enable)
		mode |= BIT(0);

	writel(0, &tpg->regs->phase);
	writel(0x100010, &tpg->regs->color[0].freq);
	writel(0, &tpg->regs->color[0].freq_rate);
	writel(0x100010, &tpg->regs->color[1].freq);
	writel(0, &tpg->regs->color[1].freq_rate);
	writel(0x100010, &tpg->regs->color[2].freq);
	writel(0, &tpg->regs->color[2].freq_rate);

	/* Slows things down by setting VBLANK as high as possible */
	writel(0xFFFF0008, &tpg->regs->blank);

	writel(mode, &tpg->regs->ctrl);

	return 0;
}

static const struct v4l2_subdev_core_ops tegra_tpg_core_ops = {
	.s_power = tegra_tpg_s_power,
};

static const struct v4l2_subdev_video_ops tegra_tpg_video_ops = {
	.enum_mbus_fmt = tegra_tpg_enum_mbus_fmt,
	.try_mbus_fmt = tegra_tpg_try_mbus_fmt,
	.g_mbus_fmt = tegra_tpg_g_mbus_fmt,
	.s_mbus_fmt = tegra_tpg_s_mbus_fmt,
	.g_mbus_config = tegra_tpg_g_mbus_config,
	.s_stream = tegra_tpg_s_stream,
};

static const struct v4l2_subdev_ops tegra_tpg_ops = {
	.core = &tegra_tpg_core_ops,
	.video = &tegra_tpg_video_ops,
};

struct v4l2_subdev *tegra_tpg_init(
	struct platform_device *pdev, void __iomem *base)
{
	struct tegra_tpg *tpg;

	tpg = devm_kzalloc(&pdev->dev, sizeof(*tpg), GFP_KERNEL);
	if (!tpg)
		return ERR_PTR(-ENOMEM);

	tpg->pll_d_clk = devm_clk_get(&pdev->dev, "pll_d");
	if (IS_ERR(tpg->pll_d_clk)) {
		dev_err(&pdev->dev, "Failed to get PLLD clock\n");
		return ERR_CAST(tpg->pll_d_clk);
	}

	v4l2_subdev_init(&tpg->sd, &tegra_tpg_ops);
	tpg->sd.owner = THIS_MODULE;
	v4l2_set_subdevdata(&tpg->sd, tpg);
	tpg->regs = base;

	tpg->fmt.width = TPG_MAX_WIDTH;
	tpg->fmt.height = TPG_MAX_HEIGHT;
	tpg->fmt.code = V4L2_MBUS_FMT_RGB888_1X24;
	tpg->fmt.field = V4L2_FIELD_NONE;
	tpg->fmt.colorspace = V4L2_COLORSPACE_SRGB;

	/* Set default settings */
	tpg->mode = 1;
	tpg->auto_inc = true;

	return &tpg->sd;
}
