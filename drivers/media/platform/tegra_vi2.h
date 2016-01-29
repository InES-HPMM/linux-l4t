/*
 * Copyright 2015 Alban Bedel <alban.bedel@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 */

#ifndef TEGRA_VI2_H__
#define TEGRA_VI2_H__

#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

struct platform_device;
struct mutex;
struct task_struct;

struct tegra_vi_regs {
	/* 0x00 */
	u32 vi_incr_syncpt;
	u32 vi_incr_syncpt_cntrl;
	u32 vi_incr_syncpt_error;
	/* 0x0C */
	u32 padding[5];
	/* 0x20 */
	u32 ctxsw;
	u32 intstatus;
	/* 0x28 */
	u32 padding2[4];
	/* 0x38 */
	u32 pwm_control;
	u32 pwm_high_pulse;
	/* 0x40 */
	u32 pwm_low_pulse;
	u32 pwm_select_pulse[4];
	/* 0x54 */
	u32 padding3[4];
	/* 0x64 */
	u32 vgp[6];
	/* 0x7C */
	u32 padding4[4];
	/* 0x8c */
	u32 interrupt_mask;
	/* 0x90 */
	u32 interrupt_type_select;
	u32 interrupt_polarity_select;
	u32 interrupt_status;
	/* 0x9C */
	u32 padding5[4];
	/* 0xAC */
	u32 vgp_syncp_config;
	/* 0xB0 */
	u32 padding6;
	u32 vi_sw_reset;
	u32 cg_ctrl;
	/* 0xBC */
	u32 padding7[8];
	/* 0xDC */
	u32 vi_mmcif_fifoctrl;
	u32 timeout_wcoal_vi;
	/* 0xE8 */
	u32 cfg_dvfs[2];
};

struct tegra_reg64 {
	u32 msb;
	u32 lsb;
};

struct tegra_vi_csi_regs {
	/* 0x100 */
	u32 sw_reset;
	u32 single_shot;
	u32 single_shot_state_update;
	u32 image_def;
	/* 0x110 */
	u32 rgb2y_ctrl;
	u32 mem_tiling;
	u32 image_size;
	u32 image_size_wc;
	/* 0x120 */
	u32 image_dt;
	/* 0x124 */
	struct tegra_reg64 surface_offset[3];
	/* 0x13C */
	struct tegra_reg64 surface_bf_offset[3];
	/* 0x154 */
	u32 surface_stride[3];
	/* 0x160 */
	u32 surface_height;
	u32 ispintf_config;
	/* 0x168 */
	u32 padding2[7];
	/* 0x184 */
	u32 error_status;
	u32 error_int_mask;
	u32 wd_ctrl;
	/* 0x190 */
	u32 wd_period;
};

struct tegra_mipi_csi_regs {
	/* 0x838 */
	u32 control;
	u32 control0;
	/* 0x840 */
	u32 control1;
	u32 gap;
	u32 pp_command;
	u32 expected_frame;
	/* 0x850 */
	u32 interrupt_mask;
	u32 status;
	u32 sensor_reset;
};

struct tegra_mipi_phy_regs {
	/* 0x908 */
	u32 cil_command;
	u32 pad_config0;
};

struct tegra_mipi_cil_regs {
	/* 0x92C */
	u32 pad_config0;
	/* 0x930 */
	u32 pad_config1;
	u32 cil_control0;
	u32 interrupt_mask;
	u32 status;
	/* 0x940 */
	u32 cil_status;
	u32 escape_mode_command;
	u32 escape_mode_data;
	u32 sensor_reset;
};

struct tegra_mipi_misc_regs {
	/* 0xA2C */
	u32 dpcm_ctrl_a;
	u32 dpcm_ctrl_b;
	/* 0xA34 */
	u32 padding0[4];
	/* 0xA44 */
	u32 stall_counter;
	u32 readonly_status;
	/* 0xA4C */
	u32 sw_status_reset;
	u32 clken_override;
	u32 debug_control;
	u32 debug_counter[3];
};

struct tegra_mipi_cal_regs {
	/* 0x00 */
	u32 ctrl;
	u32 autocal_ctrl;
	u32 status;
	u32 clk_status;
	/* 0x10 */
	u32 padding0;
	u32 data_config[13];
	/* 0x48 */
	u32 padding1[4];
	/* 0x58 */
	u32 pad_cfg0;
	u32 pad_cfg1;
	/* 0x60 */
	u32 pad_cfg2;
	u32 clk_config[5];
};

struct tegra_vi_buffer {
	struct vb2_buffer vb;
	struct list_head queue;

	unsigned num_planes;
	dma_addr_t addr[3];
	dma_addr_t bf_addr[3];
	int stride[3];
};

enum tegra_vi_input_id {
	INPUT_NONE = -1,
	INPUT_CSI_A,
	INPUT_CSI_B,
	INPUT_CSI_C,
	INPUT_PATTERN_GENERATOR,
	INPUT_COUNT
};

/* Table to convert V4L2 formats to Nvidia formats */
struct tegra_formats {
	u32 v4l2;
	u32 nv;
	u32 mbus[32];
};

/* Input to the VI block, allow accessing the CIL and sensor subdev.
 *
 * Each input can have up to 2 CIL block with 2 lanes each. The pattern
 * generator input is a special case as it has not CIL and is bound to
 * a specific pixel parser.
 */
struct tegra_vi_input {
	enum tegra_vi_input_id id;

	struct tegra_mipi_cil_regs *cil_regs[2];
	struct v4l2_subdev *sensor;
	struct v4l2_async_subdev asd;

	/* Supported MBUS flags on this input */
	unsigned mbus_caps;
	/* Position in the phy cil_command register */
	unsigned phy_shift[2];

	struct clk *cil_clk;

	unsigned use_count;
	unsigned streaming_count;
	unsigned calibrated : 1;
	struct mutex lock;

	/* Decoded CSI settings */
	unsigned csi_lanes;
	unsigned csi_channel;
	bool csi_clk_continuous;

	/* The currently used frame format */
	struct v4l2_mbus_framefmt framefmt;
};

/* Pixel processing channel */
struct tegra_vi_channel {
	unsigned id;
	struct video_device vdev;

	struct tegra_vi_input tpg;
	struct tegra_vi_input *input;
	/* ID of the last input used */
	enum tegra_vi_input_id input_id;

	struct tegra_vi_csi_regs __iomem *vi_regs;
	struct tegra_mipi_csi_regs __iomem *mipi_regs;

	struct clk *sensor_clk;

	/* Generic lock for the register and global state */
	struct mutex lock;
	unsigned use_count;

	/* Format currently set */
	struct v4l2_pix_format pixfmt;

	/* Video queue */
	struct vb2_queue vb;
	void *vb2_alloc_ctx;

	struct task_struct *work_th;
	bool should_stop;

	/* Lock for the buffer list */
	spinlock_t vq_lock;
	struct list_head capture;
	struct tegra_vi_buffer *active_buffer;
	struct tegra_vi_buffer *pending_buffer;
	unsigned long sequence;
	unsigned long missed_buffer;

	int syncpt_id;

	struct tegra_formats formats[16];
	unsigned formats_count;
};

struct tegra_vi2 {
	void __iomem *base;

	struct tegra_vi_regs __iomem *vi_regs;
	struct tegra_mipi_phy_regs *phy_regs[3];
	struct tegra_mipi_misc_regs *misc_regs[3];
	/* TODO: Move that to a separate device */
	struct tegra_mipi_cal_regs *cal_regs;

	struct tegra_vi_channel channel[3];
	struct tegra_vi_input input[3];

	struct v4l2_device v4l2_dev;

	struct v4l2_async_subdev* asd[3];
	struct v4l2_async_notifier sd_notifier;

	struct clk *vi_clk;
	struct clk *csi_clk;
	struct clk *csus_clk;
	struct clk *isp_clk;

	struct regulator *csi_reg;

	struct mutex lock;
};

struct v4l2_subdev *tegra_tpg_init(
	struct platform_device *pdev, void __iomem *base);

int tegra_vi_calibrate_input(struct tegra_vi2 *vi2,
			struct tegra_vi_input *input);

void tegra_vi_input_start(struct tegra_vi2 *vi2, struct tegra_vi_input *input);
void tegra_vi_input_stop(struct tegra_vi2 *vi2, struct tegra_vi_input *input);

int v4l2_pix_format_set_sizeimage(struct v4l2_pix_format *pf);

extern const struct vb2_ops tegra_vi_qops;

#ifdef CONFIG_VIDEO_ADV_DEBUG
#define vi_writel(v, p) do { \
	pr_info("VI2 REG WRITE: 0x%08lx to 0x%03lx\n", (unsigned long)(v), \
		(unsigned long)(p) & 0xFFF);			      \
	writel(v, p); \
} while (0)
#else
#define vi_writel(v, p) writel(v, p)
#endif

#endif /* TEGRA_VI2_H__ */
