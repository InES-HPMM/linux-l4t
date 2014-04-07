/*
 * mods_debugfs.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mods_internal.h"

#ifdef MODS_HAS_DEBUGFS

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/device.h>

static struct dentry *mods_debugfs_dir;

#ifdef CONFIG_TEGRA_DC
#include <../drivers/video/tegra/dc/dc_config.h>
#include <../drivers/video/tegra/dc/dsi.h>

static int mods_dc_color_formats_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i, j;

	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *fmt_masks;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		fmt_masks = tegra_dc_parse_feature(dc, i, GET_WIN_FORMATS);
		if (!fmt_masks)
			continue;
		seq_printf(s, "window_%u:", i);
		for (j = 0; j < ENTRY_SIZE; j++)
			seq_printf(s, " 0x%08x", fmt_masks[j]);
		seq_puts(s, "\n");
	}
	return 0;
}

static int mods_dc_color_formats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_color_formats_show, inode->i_private);
}

static const struct file_operations mods_dc_color_formats_fops = {
	.open		= mods_dc_color_formats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_blend_gen_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i;

	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *blend_gen;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		blend_gen = tegra_dc_parse_feature(dc, i, HAS_GEN2_BLEND);
		if (!blend_gen)
			continue;
		seq_printf(s, "window_%u: %u\n", i,
			blend_gen[BLEND_GENERATION]);
	}
	return 0;
}

static int mods_dc_blend_gen_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_blend_gen_show, inode->i_private);
}

static const struct file_operations mods_dc_blend_gen_fops = {
	.open		= mods_dc_blend_gen_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_layout_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i;

	seq_puts(s, "          Pitch Tiled Block\n");
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *layouts;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		layouts = tegra_dc_parse_feature(dc, i, HAS_TILED);
		if (!layouts)
			continue;
		seq_printf(s, "window_%u: %5u %5u %5u\n", i,
			layouts[PITCHED_LAYOUT],
			layouts[TILED_LAYOUT],
			layouts[BLOCK_LINEAR]);
	}
	return 0;
}

static int mods_dc_layout_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_layout_show, inode->i_private);
}

static const struct file_operations mods_dc_layout_fops = {
	.open		= mods_dc_layout_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_invert_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i;

	seq_puts(s, "          FlipH FlipV ScanColumn\n");
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *invert_data;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		invert_data = tegra_dc_parse_feature(dc, i, GET_INVERT);
		if (!invert_data)
			continue;
		seq_printf(s, "window_%u: %5u %5u %10u\n", i,
			invert_data[H_INVERT],
			invert_data[V_INVERT],
			invert_data[SCAN_COLUMN]);
	}
	return 0;
}

static int mods_dc_invert_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_invert_show, inode->i_private);
}

static const struct file_operations mods_dc_invert_fops = {
	.open		= mods_dc_invert_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_interlaced_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i;
#ifdef CONFIG_TEGRA_DC_INTERLACE
	const unsigned head_interlaced = 1;
#else
	const unsigned head_interlaced = 0;
#endif

	seq_printf(s, "head: %u\n", head_interlaced);
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *interlaced;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		interlaced = tegra_dc_parse_feature(dc, i, HAS_INTERLACE);
		if (!interlaced)
			continue;
		seq_printf(s, "window_%u: %u\n", i, interlaced[0]);
	}
	return 0;
}

static int mods_dc_interlaced_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_interlaced_show, inode->i_private);
}

static const struct file_operations mods_dc_interlaced_fops = {
	.open		= mods_dc_interlaced_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_window_mask_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	seq_printf(s, "0x%02lx\n", dc->valid_windows);
	return 0;
}

static int mods_dc_window_mask_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_window_mask_show, inode->i_private);
}

static const struct file_operations mods_dc_window_mask_fops = {
	.open		= mods_dc_window_mask_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dc_scaling_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 i;

	seq_puts(s, "          UpH UpV DownH DownV\n");
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win;
		u32 *scaling;
		win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;
		scaling = tegra_dc_parse_feature(dc, i, HAS_SCALE);
		if (!scaling)
			continue;
		seq_printf(s, "window_%u: %3u %3u %5u %5u\n", i,
			scaling[H_SCALE_UP],
			scaling[V_SCALE_UP],
			scaling[H_FILTER_DOWN],
			scaling[V_FILTER_DOWN]);
	}
	return 0;
}

static int mods_dc_scaling_open(struct inode *inode, struct file *file)
{
	return single_open(file, mods_dc_scaling_show, inode->i_private);
}

static const struct file_operations mods_dc_scaling_fops = {
	.open		= mods_dc_scaling_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mods_dsi_ganged_get(void *data, u64 *val)
{
	struct tegra_dc_dsi_data *dsi = data;
	*val = (u64)dsi->info.ganged_type;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mods_dsi_ganged_fops, mods_dsi_ganged_get, NULL,
	"%llu\n");

static int mods_dsi_inst_get(void *data, u64 *val)
{
	struct tegra_dc_dsi_data *dsi = data;
	*val = (u64)dsi->info.dsi_instance;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mods_dsi_inst_fops, mods_dsi_inst_get, NULL, "%llu\n");


static int mods_dc_border_get(void *data, u64 *val)
{
	struct tegra_dc *dc = data;
#if !defined(CONFIG_TEGRA_DC_BLENDER_GEN2)
	u32 blender_reg = DC_DISP_BORDER_COLOR;
#else
	u32 blender_reg = DC_DISP_BLEND_BACKGROUND_COLOR;
#endif
	if (!dc->enabled)
		*val = 0ULL;
	else
		*val = (u64)tegra_dc_readl(dc, blender_reg);
	return 0;
}
static int mods_dc_border_set(void *data, u64 val)
{
	struct tegra_dc *dc = data;
#if !defined(CONFIG_TEGRA_DC_BLENDER_GEN2)
	u32 blender_reg = DC_DISP_BORDER_COLOR;
#else
	u32 blender_reg = DC_DISP_BLEND_BACKGROUND_COLOR;
#endif
	if (!dc->enabled)
		return 0;
	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, val, blender_reg);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mods_dc_border_fops, mods_dc_border_get,
	mods_dc_border_set, "0x%llx\n");
#endif /* CONFIG_TEGRA_DC */

static int mods_debug_get(void *data, u64 *val)
{
	*val = (u64)(mods_get_debug_level() & DEBUG_ALL);
	return 0;
}
static int mods_debug_set(void *data, u64 val)
{
	mods_set_debug_level((int)val & DEBUG_ALL);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mods_debug_fops, mods_debug_get, mods_debug_set,
						"0x%08llx\n");

static int mods_mi_get(void *data, u64 *val)
{
	*val = (u64)mods_get_multi_instance();
	return 0;
}
static int mods_mi_set(void *data, u64 val)
{
	mods_set_multi_instance((int)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mods_mi_fops, mods_mi_get, mods_mi_set, "%llu\n");
#endif /* MODS_HAS_DEBUGFS */

void mods_remove_debugfs(void)
{
#ifdef MODS_HAS_DEBUGFS
	debugfs_remove_recursive(mods_debugfs_dir);
	mods_debugfs_dir = NULL;
#endif
}

int mods_create_debugfs(struct miscdevice *modsdev)
{
#ifdef MODS_HAS_DEBUGFS
	struct dentry *retval;
#ifdef CONFIG_TEGRA_DC
	unsigned dc_idx;
#endif
	int err = 0;

	mods_debugfs_dir = debugfs_create_dir(dev_name(modsdev->this_device),
		NULL);
	if (IS_ERR(mods_debugfs_dir)) {
		err = -EIO;
		goto remove_out;
	}

	retval = debugfs_create_file("debug", S_IRUGO | S_IWUSR,
		mods_debugfs_dir, 0, &mods_debug_fops);
	if (IS_ERR(retval)) {
		err = -EIO;
		goto remove_out;
	}

	retval = debugfs_create_file("multi_instance", S_IRUGO | S_IWUSR,
		mods_debugfs_dir, 0, &mods_mi_fops);
	if (IS_ERR(retval)) {
		err = -EIO;
		goto remove_out;
	}

#ifdef CONFIG_TEGRA_DC
	for (dc_idx = 0; dc_idx < TEGRA_MAX_DC; dc_idx++) {
		struct dentry *dc_debugfs_dir;
		struct tegra_dc *dc = tegra_dc_get_dc(dc_idx);
		if (!dc)
			continue;

		dc_debugfs_dir = debugfs_create_dir(dev_name(&dc->ndev->dev),
			mods_debugfs_dir);
		if (IS_ERR(dc_debugfs_dir)) {
			err = -EIO;
			goto remove_out;
		}

		retval = debugfs_create_file("window_mask", S_IRUGO,
			dc_debugfs_dir, dc, &mods_dc_window_mask_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("color_formats", S_IRUGO,
			dc_debugfs_dir, dc, &mods_dc_color_formats_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("blend_gen", S_IRUGO,
			dc_debugfs_dir, dc, &mods_dc_blend_gen_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("layout", S_IRUGO, dc_debugfs_dir,
			dc, &mods_dc_layout_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("invert", S_IRUGO, dc_debugfs_dir,
			dc, &mods_dc_invert_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("interlaced", S_IRUGO,
			dc_debugfs_dir, dc, &mods_dc_interlaced_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("scaling", S_IRUGO, dc_debugfs_dir,
			dc, &mods_dc_scaling_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}
		retval = debugfs_create_file("border_color", S_IRUGO | S_IWUSR,
			dc_debugfs_dir, dc, &mods_dc_border_fops);
		if (IS_ERR(retval)) {
			err = -EIO;
			goto remove_out;
		}

		if (dc->out->type == TEGRA_DC_OUT_DSI) {
			struct dentry *dsi_debugfs_dir;
			dsi_debugfs_dir = debugfs_create_dir("dsi",
				dc_debugfs_dir);
			if (IS_ERR(dsi_debugfs_dir)) {
				err = -EIO;
				goto remove_out;
			}
			retval = debugfs_create_file("ganged", S_IRUGO,
				dsi_debugfs_dir, tegra_dc_get_outdata(dc),
				&mods_dsi_ganged_fops);
			if (IS_ERR(retval)) {
				err = -EIO;
				goto remove_out;
			}
			retval = debugfs_create_file("instance", S_IRUGO,
				dsi_debugfs_dir, tegra_dc_get_outdata(dc),
				&mods_dsi_inst_fops);
			if (IS_ERR(retval)) {
				err = -EIO;
				goto remove_out;
			}
		}
	}
#endif

	return 0;
remove_out:
	dev_err(modsdev->this_device, "could not create debugfs\n");
	mods_remove_debugfs();
	return err;
#else
	return 0;
#endif
}

