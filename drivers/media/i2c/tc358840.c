/*
 * tc358840.c - Toshiba UH2C/D HDMI-CSI bridge driver
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 *
 * This program is based on the tc358743 - Toshiba HDMI to CSI-2 bridge driver
 * from Cisco Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/module.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/interrupt.h>

#include <mach/io_dpd.h>

#include <linux/videodev2.h>
#include <linux/workqueue.h>

#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include <media/soc_camera.h>

#include <media/tc358840.h>
#include "tc358840_regs.h"


static int debug = 2;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

/* TODO: Implement Colorbar TPG */

#define EDID_NUM_BLOCKS_MAX 8
#define EDID_BLOCK_SIZE 128

static const struct v4l2_dv_timings_cap tc358840_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(
		1, 10000, 1, 10000, 0, 297000000,
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
		V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
		V4L2_DV_BT_CAP_PROGRESSIVE |
		V4L2_DV_BT_CAP_REDUCED_BLANKING |
		V4L2_DV_BT_CAP_CUSTOM)
};

struct tc358840_state {
	struct tc358840_platform_data pdata;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	struct i2c_client *i2c_client;

	/* controls */
	struct v4l2_ctrl *detect_ddc_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;

	/* work queues */
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;
	struct work_struct work_isr;

	/* edid  */
	u8 edid_blocks_written;

	/* timing / mbus */
	struct v4l2_dv_timings timings;
	u32 mbus_fmt_code;
};

static inline struct tc358840_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358840_state, sd);
}


static void tc358840_enable_interrupts(
	struct v4l2_subdev *sd, bool cable_connected);

static int tc358840_s_ctrl_detect_ddc_5v(struct v4l2_subdev *sd);

static void tc358840_init_interrupts(struct v4l2_subdev *sd);

static int tc358840_s_dv_timings(
	struct v4l2_subdev *sd, struct v4l2_dv_timings *timings);

/* *** EDID *** */
/*
 * FIXME: The final version shouldn't have a hardcoded EDID. This is only done
 * for the sake of simplicity.
 *
 * Preferred native timing: 1080p60
 */
 /*
static const u32 EDID_DATA[] = {
	0xFFFFFF00, 0x00FFFFFF, 0x02096252, 0x01010101,
	0x030114FF, 0x785AA080, 0xA0C90D0A, 0x27984757,
	0x2F4C4812, 0x808100CF, 0x01010101, 0x01010101,
	0x01010101, 0x3A020101, 0x38711880, 0x2C58402D,
	0x84400045, 0x1E000063, 0xB0502166, 0x301B0051,
	0x00367040, 0x0063843A, 0x00001E00, 0x5400FC00,
	0x4948534F, 0x542D4142, 0x20200A56, 0xFD000000,
	0x0F4C1700, 0x0A000F51, 0x20202020, 0xA9012020,
	0x70250302, 0x04051049, 0x06020703, 0x09262001,
	0x07150707, 0x0C036CC0, 0x38003000, 0x2B2BCF2D,
	0x00E23333, 0x801D017F, 0x161C7118, 0x252C5820,
	0x63844000, 0x8C9E0000, 0x208AD00A, 0x10102DE0,
	0xB000963E, 0x00004384, 0x001F0E18, 0x1E005180,
	0x37804030, 0x5384DC00, 0xF11C0000, 0x51A00027,
	0x50302500, 0xDC003780, 0x00005384, 0x001AA91C,
	0x160050A0, 0x37203030, 0x5384DC00, 0xA21A0000
};
*/
// TESTING: Preferred native timing 2160p30 30Hz @262.92MHz PClk
// Max PClk: 290MHz.
static const u32 EDID_DATA[] = {
/*
	0xffffff00, 0x00ffffff, 0x88886252, 0x88888800,
	0x0301151c, 0x78000080, 0xa0c90d0a, 0x27984757,
	0x004c4812, 0x01010000, 0x01010101, 0x01010101,
	0x01010101, 0x66b40101, 0x70f0a000, 0x2030801f,
	0x88800035, 0x1c000042, 0xa00068ec, 0x803770f0,
	0x003a2030, 0x00f87000, 0x00001c00, 0x5400fc00,
	0x6968736f, 0x552d6162, 0x0a433248, 0xfd000000,
	0x0f3d1700, 0x00001e8c, 0x00000000, 0x13010000,
	0x74170302, 0x03130447, 0x01060702, 0x01070923,
	0x000c0366, 0x8c800030, 0x68ecd00a, 0x70f0a000,
	0x20308037, 0x7000003a, 0x1c0000f8, 0xa00068ec,
	0x803770f0, 0x003a2030, 0x00f87000, 0x68ec1c00,
	0x70f0a000, 0x20308037, 0x7000003a, 0x1c0000f8,
	0xa00068ec, 0x803770f0, 0x003a2030, 0x00f87000,
	0x00001c00, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0xdc000000
*/
	0xffffff00, 0x00ffffff, 0x0029ce1c, 0x00000000,
	0x03010000, 0x78000082, 0xa391ee3e, 0x26994c54,
	0x0054500f, 0x01010000, 0x01010101, 0x01010101,
	0x01010101, 0x74040101, 0x70f23000, 0x58b0805a,
	0x0910008a, 0x1e000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x5200fc00,
	0x4f584f49, 0x2020200a, 0x20202020, 0xfd000000,
	0x441e1e00, 0x0a001e44, 0x20202020, 0xa5012020,
	0x31160302, 0x036f5f41, 0x0000000c, 0x00e03b00,
	0x00000000, 0x00000120, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x5a000000
};

static struct v4l2_subdev_edid EDID_1080p = {
	0,			/* pad */
	0,			/* start_block */
	2,			/* blocks */
	{0, 0, 0, 0, 0},	/* reserved */
	(char *) &EDID_DATA	/* user data */
};

/* --------------- I2C --------------- */

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358840_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358840_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	const uint8_t chunk_size = 8; /* # of bytes to be transmitted at once */
	uint32_t wr_idx = 0;
	int err, i;
	struct i2c_msg msg;
	u8 data[2 + chunk_size];

	msg.addr = client->addr;
	msg.buf = data;
	msg.flags = 0;

	/*
	 * WORKAROUND
	 * On the TX1, the i2c connected to the camera sensor is newly included
	 * in the VI block and cotrolled by Host1x. Thus, a new driver exists,
	 * which is not capable of sending i2c messages larger than ca. 16
	 * bytes. This workaround splits messages into small chunks (8 Bytes).
	 * Otherwise, Host Read errors will occur. 
	 */
	while (wr_idx < n) {
		for (i = 0; i < chunk_size && (i + wr_idx) < n; i++)
			data[2 + i] = values[wr_idx + i];

		msg.len = 2 + i;

		data[0] = (reg + wr_idx ) >> 8;
		data[1] = (reg + wr_idx ) & 0xff;

		wr_idx += i;

		err = i2c_transfer(client->adapter, &msg, 1);
		if (err != 1) {
			v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed\n",
				__func__, reg, client->addr);
			return;
		}
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X\n", reg, data[2]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X%02X\n",
			reg, data[3], data[2]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04X = 0x%02X%02X%02X%02X\n",
			reg, data[5], data[4], data[3], data[2]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04X\n",
			n, reg);
	}
}

static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;

	i2c_rd(sd, reg, &val, 1);

	return val;
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wr(sd, reg, &val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,
		u8 mask, u8 val)
{
	i2c_wr8(sd, reg, (i2c_rd8(sd, reg) & mask) | val);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;

	i2c_rd(sd, reg, (u8 *)&val, 2);

	return val;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wr16(sd, reg, (i2c_rd16(sd, reg) & mask) | val);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	u32 val;

	i2c_rd(sd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 4);
}

static void i2c_wr32_and_or(struct v4l2_subdev *sd, u32 reg, u32 mask, u32 val)
{
	i2c_wr32(sd, reg, (i2c_rd32(sd, reg) & mask) | val);
}

/* --------------- STATUS --------------- */

static inline bool ddc_5v_power_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_DDC5V;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_TMDS);
}

static inline bool no_sync(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_SYNC);
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, AU_STATUS0) & MASK_S_A_SAMPLE;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	static const int code_to_rate[] = {
		44100, 0, 48000, 32000, 22050, 384000, 24000, 352800,
		88200, 768000, 96000, 705600, 176400, 0, 192000, 0
	};

	/* Register FS_SET is not cleared when the cable is disconnected */
	if (no_signal(sd))
		return 0;

	return code_to_rate[i2c_rd8(sd, FS_SET) & MASK_FS];
}

static unsigned tc358840_num_csi_lanes_in_use(struct v4l2_subdev *sd)
{
	/* FIXME: Read # of lanes from both, TX0 and TX1 */
	return i2c_rd32(sd, CSITX0_BASE_ADDR+LANEEN) & MASK_LANES;
}

/* --------------- TIMINGS --------------- */

bool valid_dv_timings(const struct v4l2_dv_timings *t,
		const struct v4l2_dv_timings_cap *dvcap)
{
	const struct v4l2_bt_timings *bt = &t->bt;
	const struct v4l2_bt_timings_cap *cap = &dvcap->bt;
	u32 caps = cap->capabilities;

	if (t->type != V4L2_DV_BT_656_1120)
		return false;
	if (t->type != dvcap->type ||
		bt->height < cap->min_height ||
		bt->height > cap->max_height ||
		bt->width < cap->min_width ||
		bt->width > cap->max_width ||
		bt->pixelclock < cap->min_pixelclock ||
		bt->pixelclock > cap->max_pixelclock ||
		(cap->standards && bt->standards &&
		 !(bt->standards & cap->standards)) ||
		(bt->interlaced && !(caps & V4L2_DV_BT_CAP_INTERLACED)) ||
		(!bt->interlaced && !(caps & V4L2_DV_BT_CAP_PROGRESSIVE)))
		return false;

	return true;
}

static inline unsigned fps(const struct v4l2_bt_timings *bt)
{
	u32 width, height;

	/* TODO: Better use V4L2_DV_BT_FRAME_HEIGHT(bt) and
	 *  V4L2_DV_BT_FRAME_WIDTH(), but they are not available in kernel
	 *  version 3.10. */
	height = (bt->height + bt->vfrontporch + bt->vsync + bt->vbackporch +
		bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch);
	width = (bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch);

	if (!height || !width)
		return 0;

	return DIV_ROUND_CLOSEST((unsigned)bt->pixelclock, height * width);
}

static int tc358840_get_detected_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	unsigned width, height, frame_width, frame_height, frame_interval, fps;
	int sync_timeout_ctr = 100;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}
	while (no_sync(sd) && sync_timeout_ctr) {
		sync_timeout_ctr--;
		mdelay(1);//ndelay(500);
	}
	
	if (sync_timeout_ctr == 0){
		v4l2_dbg(1, debug, sd, "%s: no sync timeout exceeded, EXITING\n", __func__);
		return -ENOLCK;
	} else {
		v4l2_dbg(1, debug, sd, "%s: SYNC SUCCESSFUL, sync_timeout_ctr=%d\n", __func__, sync_timeout_ctr);
	}

	timings->type = V4L2_DV_BT_656_1120;

	bt->interlaced = i2c_rd8(sd, VI_STATUS1) &
		MASK_S_V_INTERLACE ? V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	v4l2_dbg(2, debug, sd, "VI_STATUS3 (input format): 0x%02X\n",
		i2c_rd8(sd, VI_STATUS3));

	width = ((i2c_rd8(sd, DE_HSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_HSIZE_LO);
	height = ((i2c_rd8(sd, DE_VSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_VSIZE_LO);
	frame_width = ((i2c_rd8(sd, IN_HSIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, IN_HSIZE_LO);
	frame_height = (((i2c_rd8(sd, IN_VSIZE_HI) & 0x3f) << 8) +
		i2c_rd8(sd, IN_VSIZE_LO)) / 2;

	printk("%s: DE: width=%d, height=%d  |  IN: frame_width=%d, frame_height=%d\n", __func__, width, height, frame_width, frame_height);
	if (frame_height < height) {
		printk("%s: ERROR: frame_height < height. Exiting..\n", __func__);
		return -EINVAL;
	}

	/* TODO: Check if frame_interval is correct
	 * since the register is not in the datasheet rev. 1.5 */

	/* frame interval in milliseconds * 10
	 * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
	frame_interval = ((i2c_rd8(sd, FV_CNT_HI) & 0x3) << 8) +
		i2c_rd8(sd, FV_CNT_LO);

	fps = (frame_interval > 0) ?
		DIV_ROUND_CLOSEST(10000, frame_interval) : 0;

	bt->width = width;
	bt->height = height;
	bt->vsync = frame_height - height;
	bt->hsync = frame_width - width;
	bt->pixelclock = frame_width * frame_height * fps;
	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}// FIXME: Need to adjust for 4k / duallink?

	return 0;
}

/* --------------- HOTPLUG / HDCP / EDID --------------- */

static void tc358840_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tc358840_state *state = container_of(dwork,
		struct tc358840_state, delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, MASK_HPD_OUT0);
}

static void tc358840_set_hdmi_hdcp(struct v4l2_subdev *sd, bool enable)
{
	v4l2_dbg(2, debug, sd, "%s: DUMMY %s\n", __func__,
		enable ? "enable" : "disable");
}

static void tc358840_disable_edid(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	cancel_delayed_work_sync(&state->delayed_work_enable_hotplug);

	/* DDC access to EDID is also disabled when hotplug is disabled. See
	 * register DDC_CTL */
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, 0x0);
}

static void tc358840_enable_edid(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	if (state->edid_blocks_written == 0) {
		v4l2_dbg(2, debug, sd, "%s: no EDID -> no hotplug\n", __func__);
		return;
	}

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	/* Enable hotplug after 100 ms. DDC access to EDID is also enabled when
	 * hotplug is enabled. See register DDC_CTL */
	queue_delayed_work(state->work_queues,
		&state->delayed_work_enable_hotplug, HZ / 10);

	tc358840_enable_interrupts(sd, true);
	tc358840_s_ctrl_detect_ddc_5v(sd);
}

static void tc358840_erase_bksv(struct v4l2_subdev *sd)
{
	int i;

	for (i = 0; i < 5; i++)
		i2c_wr8(sd, BKSV + i, 0);
}

/* --------------- AVI infoframe --------------- */

/* --------------- CTRLS --------------- */

static int tc358840_s_ctrl_detect_ddc_5v(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	int ret = v4l2_ctrl_s_ctrl(state->detect_ddc_5v_ctrl,
		ddc_5v_power_present(sd));

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (ret < 0)
		return ret;

	return 0;
}

static int tc358840_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	int ret = v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (ret < 0)
		return ret;

	return 0;
}

static int tc358840_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	int ret = v4l2_ctrl_s_ctrl(state->audio_present_ctrl,
			audio_present(sd));

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (ret < 0)
		return ret;

	return 0;
}

static int tc358840_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	ret |= tc358840_s_ctrl_detect_ddc_5v(sd);
	ret |= tc358840_s_ctrl_audio_sampling_rate(sd);
	ret |= tc358840_s_ctrl_audio_present(sd);

	return ret;
}

/* --------------- INIT --------------- */

static void tc358840_reset_phy(struct v4l2_subdev *sd)
{
	v4l2_dbg(1, debug, sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, 0);
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, MASK_RESET_CTRL);
}

static void tc358840_reset(struct v4l2_subdev *sd, uint16_t mask)
{
	u16 sysctl = i2c_rd16(sd, SYSCTL);

	v4l2_dbg(1, debug, sd, "%s():\n", __func__);

	i2c_wr16(sd, SYSCTL, sysctl | mask);
	i2c_wr16(sd, SYSCTL, sysctl & ~mask);
}

static inline void tc358840_sleep_mode(struct v4l2_subdev *sd, bool enable)
{
	v4l2_dbg(1, debug, sd, "%s(): %s\n", __func__,
		enable ? "enable" : "disable");

	i2c_wr16_and_or(sd, SYSCTL, ~MASK_SLEEP, enable ? MASK_SLEEP : 0);
}

static int enable_stream(struct v4l2_subdev *sd, bool enable)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	u32 sync_timeout_ctr;

	v4l2_dbg(2, debug, sd, "%s: %sable\n", __func__, enable ? "en" : "dis");

	if (enable) {
		/* It is critical for CSI receiver to see lane transition
		 * LP11->HS. Set to non-continuous mode to enable clock lane
		 * LP11 state. */
		i2c_wr32_and_or(sd, FUNCMODE, ~(MASK_CONTCLKMODE), 0);
		/* Set to continuous mode to trigger LP11->HS transition */
		i2c_wr32_and_or(sd, FUNCMODE, 0, MASK_CONTCLKMODE);
		/* Unmute video */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE);
		/* Signal end of initialization */
		i2c_wr8(sd, INIT_END, MASK_INIT_END);
	} else {
		/* Enable Registers to be initialized */
		i2c_wr8_and_or(sd, INIT_END, ~(MASK_INIT_END), 0x00);
		printk("%s: enable regs DONE\n", __func__);

		/* Mute video so that all data lanes go to LSP11 state.
		 * No data is output to CSI Tx block. */

		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
		printk("%s: mute DONE\n", __func__);
	}

	/* Wait for HDMI input to become stable */
	if (enable) {
		sync_timeout_ctr = 100;
		while (no_sync(sd) && sync_timeout_ctr)
			sync_timeout_ctr--;

		if (sync_timeout_ctr == 0) {
			/* Disable stream again. Probably no cable inserted.. */
			v4l2_err(sd, "%s: Timeout: HDMI input sync failed.\n",
					__func__);
			enable_stream(sd, false);
			return -EIO;
		}

		v4l2_dbg(2, debug, sd,
			"%s: Stream enabled! Remaining timeout attempts: %d\n"
			, __func__, sync_timeout_ctr);
	}

	i2c_wr16_and_or(sd, CONFCTL0,
		~(MASK_VTX0EN | MASK_VTX1EN | MASK_ABUFEN),
		enable ? ((pdata->csi_port & (MASK_VTX0EN | MASK_VTX1EN)) |
		MASK_ABUFEN | MASK_TX_MSEL | MASK_AUTOINDEX) :
		(MASK_TX_MSEL | MASK_AUTOINDEX));

	printk("%s: confctl write DONE\n", __func__);

	return 0;
}

static void tc358840_set_splitter(struct v4l2_subdev *sd)
{
	/* TODO:
	 * Splitter is currently bypassed, however, it will be needed for 2160p
	 */
	 //FIXME: Set Splitter based on no. of CIS-lanes

	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if(pdata->csi_port == CSI_TX_BOTH)
	{
		printk("%s: set splitter for dual link\n", __func__);
		/*v4l2_dbg(2, debug, sd, "set splitter for dual link");*/
		i2c_wr16_and_or(sd, SPLITTX0_CTRL, 
				~(MASK_IFEN | MASK_LCD_CSEL | MASK_SPBP), MASK_SPEN);
		i2c_wr16_and_or(sd, SPLITTX1_CTRL, 
				~(MASK_IFEN | MASK_LCD_CSEL | MASK_SPBP), MASK_SPEN);

		i2c_wr16_and_or(sd, SPLITTX0_SPLIT, ~(MASK_TX1SEL), MASK_EHW);

/*	Removed in newer Datasheets
		i2c_wr16(sd, SPLITTX0_WC, MASK_WC);
		i2c_wr16(sd, SPLITTX1_WC, MASK_WC);	
*/
	}else
	{
		printk("%s: set splitter for single link\n", __func__);
		/*v4l2_dbg(2, debug, sd, "set splitter for single link");*/
		i2c_wr16_and_or(sd, SPLITTX0_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL),
				MASK_SPBP);
		i2c_wr16_and_or(sd, SPLITTX1_CTRL, ~(MASK_IFEN | MASK_LCD_CSEL),
				MASK_SPBP);

		i2c_wr16_and_or(sd, SPLITTX0_SPLIT, ~(MASK_TX1SEL), MASK_FPXV);
	}
}

static void tc358840_set_pll(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;
#if 0
	struct v4l2_bt_timings *bt = &state->timings.bt;
#endif
	enum tc358840_csi_port port;
	u16 base_addr;
	u32 pllconf;
	u32 pllconf_new;
	u32 hsck;

	v4l2_dbg(2, debug, sd, "%s:\n", __func__);

	BUG_ON((pdata->csi_port <= CSI_TX_NONE) ||
		(pdata->csi_port > CSI_TX_BOTH));

	if (pdata->csi_port == CSI_TX_NONE) {
		v4l2_err(sd, "%s: No CSI port defined!\n", __func__);
		return;
	}

	for (port = CSI_TX_0; port <= CSI_TX_1; port++) {
		if (port == CSI_TX_0)
			base_addr = CSITX0_BASE_ADDR;
		else
			base_addr = CSITX1_BASE_ADDR;

		if ((pdata->csi_port == CSI_TX_BOTH) ||
		    (pdata->csi_port == port)) {

			pllconf = i2c_rd32(sd, base_addr+PLLCONF);
			pllconf_new = SET_PLL_PRD(pdata->pll_prd) |
				SET_PLL_FBD(pdata->pll_fbd);

			hsck = (pdata->refclk_hz / pdata->pll_prd) *
				pdata->pll_fbd;

			/* TODO: Check if this can be useful */
			/*
			 * CISCO HACK: Use all lanes and lower clock speed for
			 * 1080p60 to reduce number of CSI resets
			 */
#if 0
			if (bt->width == 1920 && bt->height == 1080 && fps(bt) == 60 &&
					state->mbus_fmt_code == V4L2_MBUS_FMT_UYVY8_1X16) {
				u16 pll_prd = 4;
				u16 pll_fbd = 83;

				pllconf_new = SET_PLL_PRD(pll_prd) | SET_PLL_FBD(pll_fbd);
				hsck = (pdata->refclk_hz / pll_prd) * pll_fbd;
			}
#endif

			/* Only rewrite when needed, since rewriting triggers
			 * another format change event.
			 */

			if (pllconf != pllconf_new) {
				u16 pll_frs;

				if (hsck > 500000000)
					pll_frs = 0x0;
				else if (hsck > 250000000)
					pll_frs = 0x1;
				else if (hsck > 125000000)
					pll_frs = 0x2;
				else
					pll_frs = 0x3;

				v4l2_dbg(1, debug, sd,
					"%s: Updating PLL clock of CSI TX%d\n",
					__func__, port-1);

				i2c_wr32(sd, base_addr+PLLCONF,
					pllconf_new | SET_PLL_FRS(pll_frs));
			}
		}
	}
}

static void tc358840_set_ref_clk(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	u32 sys_freq;
	u32 lock_ref_freq;
	u32 nco;
	u16 csc;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	BUG_ON((pdata->refclk_hz < 40000000) || (pdata->refclk_hz > 50000000));

	/* System Frequency */
	sys_freq = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SYS_FREQ0, sys_freq & 0x00FF);
	i2c_wr8(sd, SYS_FREQ1, (sys_freq & 0xFF00) >> 8);

	/* Audio System Frequency */
	lock_ref_freq = pdata->refclk_hz / 100;
	i2c_wr8(sd, LOCK_REF_FREQA, lock_ref_freq & 0xFF);
	i2c_wr8(sd, LOCK_REF_FREQB, (lock_ref_freq >> 8) & 0xFF);
	i2c_wr8(sd, LOCK_REF_FREQC, (lock_ref_freq >> 16) & 0x0F);

	/* Audio PLL */
	i2c_wr8(sd, NCO_F0_MOD, MASK_NCO_F0_MOD_REG);
	/* 6.144 * 2^28 = 1649267442 */
	nco = (1649267442 / (pdata->refclk_hz / 1000000));
	i2c_wr8(sd, NCO_48F0A, nco & 0xFF);
	i2c_wr8(sd, NCO_48F0B, (nco >> 8) & 0xFF);
	i2c_wr8(sd, NCO_48F0C, (nco >> 16) & 0xFF);
	i2c_wr8(sd, NCO_48F0D, (nco >> 24) & 0xFF);

	/* Color Space Conversion */
	csc = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SCLK_CSC0, csc & 0xFF);
	i2c_wr8(sd, SCLK_CSC1, (csc >> 8) & 0xFF);
}

static void tc358840_set_csi_mbus_config(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	switch (state->mbus_fmt_code) {
	case V4L2_MBUS_FMT_UYVY8_1X16:
		v4l2_dbg(2, debug, sd, "%s: YCbCr 422 16-bit\n", __func__);

		i2c_wr8(sd, VOUT_FMT, MASK_OUTFMT_422 | MASK_422FMT_NORMAL);
		i2c_wr8(sd, VOUT_FIL, MASK_422FIL_3_TAP_444 |
			MASK_444FIL_2_TAP);
		i2c_wr8(sd, VOUT_SYNC0, MASK_MODE_2);
		i2c_wr8(sd, VOUT_CSC, MASK_CSC_MODE_BUILTIN |
			MASK_COLOR_601_YCBCR_LIMITED);
		i2c_wr16_and_or(sd, CONFCTL0, ~(MASK_YCBCRFMT),
			MASK_YCBCRFMT_YCBCR422_8);
		i2c_wr16(sd, CONFCTL1, 0x0);

		break;

	case V4L2_MBUS_FMT_RGB888_1X24:
		v4l2_dbg(2, debug, sd, "%s: RGB 888 24-bit\n", __func__);

		i2c_wr8(sd, VOUT_FMT, MASK_OUTFMT_444_RGB);
		i2c_wr8(sd, VOUT_FIL, MASK_422FIL_3_TAP_444 |
			MASK_444FIL_2_TAP);
		i2c_wr8(sd, VOUT_SYNC0, MASK_MODE_2);
		i2c_wr8(sd, VOUT_CSC, MASK_CSC_MODE_BUILTIN |
			MASK_COLOR_RGB_LIMITED);
		i2c_wr16_and_or(sd, CONFCTL0, ~(MASK_YCBCRFMT), 0x0);
		i2c_wr16_and_or(sd, CONFCTL1, 0x0, MASK_TX_OUT_FMT_RGB888);

		break;

	default:
		v4l2_dbg(2, debug, sd, "%s: Unsupported format code 0x%x\n",
				__func__, state->mbus_fmt_code);
		break;
	}
}

static unsigned tc358840_num_csi_lanes_needed(struct v4l2_subdev *sd)
{

	/* TODO: Check if this can be useful */
#if 0
	struct tc358840_state *state = to_state(sd);
	struct v4l2_bt_timings *bt = &state->timings.bt;
	struct tc358840_platform_data *pdata = &state->pdata;
	u32 bits_pr_pixel =
		(state->mbus_fmt_code == V4L2_MBUS_FMT_UYVY8_1X16) ?  16 : 24;
	u32 bps = bt->width * bt->height * fps(bt) * bits_pr_pixel;
	u32 bps_pr_lane = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	/* CISCO HACK: Use all lanes and lower clock speed for 1080p60 to
	 * reduce number of CSI resets */
	if (bt->width == 1920 && bt->height == 1080 && fps(bt) == 60 &&
			state->mbus_fmt_code == V4L2_MBUS_FMT_UYVY8_1X16) {
		return 4;
	}

	return DIV_ROUND_UP(bps, bps_pr_lane);
#endif

	/* FIXME : ALWAYS USE 4 LANES FOR TESTING*/
	return 4;
}

static void tc358840_set_csi(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;
#if 0
	struct v4l2_bt_timings *bt = &state->timings.bt;
#endif
	unsigned lanes = tc358840_num_csi_lanes_needed(sd);

	enum tc358840_csi_port port;
	u16 base_addr;

	v4l2_dbg(3, debug, sd, "%s:\n", __func__);

	tc358840_reset(sd, MASK_CTXRST);

	for (port = CSI_TX_0; port <= CSI_TX_1; port++) {
		if (port == CSI_TX_0)
			base_addr = CSITX0_BASE_ADDR;
		else
			base_addr = CSITX1_BASE_ADDR;

		if ((pdata->csi_port == CSI_TX_BOTH) ||
		    (pdata->csi_port == port)) {
			v4l2_dbg(1, debug, sd,
				"%s: Enabling CSI TX%d\n", __func__, port-1);

			/* (0x0108) */
			i2c_wr32(sd, base_addr+CSITX_CLKEN, MASK_CSITX_EN);
			/* (0x010C) */
			i2c_wr32(sd, base_addr+PPICLKEN, MASK_HSTXCLKEN);
			/* (0x02A0) */
			i2c_wr32_and_or(sd, base_addr+MIPICLKEN,
				~(MASK_MP_CKEN), MASK_MP_ENABLE);
			/*
			 * PLL has to be enabled between CSITX_CLKEN and
			 * LANEEN (0x02AC)
			 */
			tc358840_set_pll(sd);
			/* (0x02A0) */
			i2c_wr32(sd, base_addr+MIPICLKEN,
				MASK_MP_CKEN | MASK_MP_ENABLE);
			/*
			 * TODO: Check if writing to MODCONF is necessary
			 * (0x0110)
			 */
			i2c_wr32_and_or(sd, base_addr+MODECONF, 0x0,
				MASK_CSI2MODE | MASK_VSYNC_POL_SW |
				MASK_HSYNC_POL_SW);
			// FIXME: NECESSARY?
			i2c_wr32_and_or(sd, base_addr+MODECONF2, 0x0,
				MASK_CSI2MODE | MASK_VSYNC_POL_SW |
				MASK_HSYNC_POL_SW);

			/* (0x0118) */
			i2c_wr32(sd, base_addr+LANEEN,
				(lanes & MASK_LANES) | MASK_CLANEEN);

			/* (0x0120) */
			i2c_wr32(sd, base_addr+LINEINITCNT, pdata->lineinitcnt);

			/* TODO: Check if necessary (0x0124) */
			i2c_wr32(sd, base_addr+HSTOCNT, 0x00000000);
			/* TODO: Check if INTEN is necessary (0x0128) */
			i2c_wr32(sd, base_addr+INTEN, 0x007F0101);

			/*
			 * TODO: Check if CSI_TATO_COUNT is necessary
			 * (0x0130)
			 */
			i2c_wr32(sd, base_addr+CSI_TATO_COUNT, 0x00010000);

			/*
			 * TODO: Check if CSI_PRESP_BTA_COUNT is necessary
			 * (0x0134)
			 */
			i2c_wr32(sd, base_addr+CSI_PRESP_BTA_COUNT, 0x00005000);

			/*
			 * TODO: Check if CSI_PRESP_LPR_COUNT is necessary
			 * (0x0138)
			 */
			i2c_wr32(sd, base_addr+CSI_PRESP_LPR_COUNT, 0x00010000);

			/*
			 * TODO: Check if CSI_PRESP_LPW_COUNT is necessary
			 * (0x013C)
			 */
			i2c_wr32(sd, base_addr+CSI_PRESP_LPW_COUNT, 0x00010000);

			/* TODO: Check if HSREADCNT is necessary  (0x0140) */
			i2c_wr32(sd, base_addr+HSREADCNT, 0x00010000);
			/* TODO: Check if HSWRITECNT is necessary (0x0144) */
			i2c_wr32(sd, base_addr+HSWRITECNT, 0x00010000);
			/* TODO: Check if PERIRSTCNT is necessary (0x0148) */
			i2c_wr32(sd, base_addr+PERIRSTCNT, 0x00001000);
			/* TODO: Check if LRXHTOCNT is necessary (0x014C) */
			i2c_wr32(sd, base_addr+LRXHTOCNT, 0x00010000);

			/*
			 * TODO: Check if this is the correct register
			 * (0x0150)
			 */
			i2c_wr32(sd, base_addr+FUNCMODE, MASK_CONTCLKMODE);
			/* TODO: Check if RX_VC_EN is necessary (0x0154) */
			i2c_wr32(sd, base_addr+RX_VC_EN, MASK_RX_VC0);
			/* TODO: Check if INPUTTOCNT is necessary (0x0158) */
			i2c_wr32(sd, base_addr+INPUTTOCNT, 0x000000C8);
			/* TODO: Check if HSYNCSTOPCNT is necessary (0x0168) */
			i2c_wr32(sd, base_addr+HSYNCSTOPCNT, 0x0000002A);

			/* NOTE: Probably not necessary */
			/* (0x01A4) */
			i2c_wr32(sd, base_addr+RX_STATE_INT_MASK, 0x0);
			/* (0x01C0) */
			i2c_wr32(sd, base_addr+LPRX_THRESH_COUNT, 0x00000015);

			/* TODO: Check if APPERRMASK is necessary (0x0214) */
			i2c_wr32(sd, base_addr+APPERRMASK, 0x00000000);

			/* NOTE: Probably not necessary */
			/* (0x021C) */
			i2c_wr32(sd, base_addr+RX_ERR_INT_MASK, 0x00000080);
			/* (0x0224) */
			i2c_wr32(sd, base_addr+LPTX_INT_MASK, 0x00000000);

			/* TODO: Check if this can be useful */
#if 0
			/* CISCO HACK: Use all lanes and lower clock speed for 1080p60 to
			 * reduce number of CSI resets */
			if (bt->width == 1920 && bt->height == 1080 && fps(bt) == 60 &&
					state->mbus_fmt_code == V4L2_MBUS_FMT_UYVY8_1X16) {
				i2c_wr32(sd, LINEINITCNT, 0x00000e10);
				i2c_wr32(sd, LPTXTIMECNT, 0x00000003);
				i2c_wr32(sd, TCLK_HEADERCNT, 0x00001303);
				i2c_wr32(sd, TCLK_TRAILCNT, 0x00000000);
				i2c_wr32(sd, THS_HEADERCNT, 0x00000003);
				i2c_wr32(sd, TWAKEUP, 0x00004650);
				i2c_wr32(sd, TCLK_POSTCNT, 0x00000000);
				i2c_wr32(sd, THS_TRAILCNT, 0x00000002);
			} else {
				i2c_wr32(sd, LINEINITCNT, pdata->lineinitcnt);
				i2c_wr32(sd, LPTXTIMECNT, pdata->lptxtimecnt);
				i2c_wr32(sd, TCLK_HEADERCNT, pdata->tclk_headercnt);
				i2c_wr32(sd, TCLK_TRAILCNT, pdata->tclk_trailcnt);
				i2c_wr32(sd, THS_HEADERCNT, pdata->ths_headercnt);
				i2c_wr32(sd, TWAKEUP, pdata->twakeup);
				i2c_wr32(sd, TCLK_POSTCNT, pdata->tclk_postcnt);
				i2c_wr32(sd, THS_TRAILCNT, pdata->ths_trailcnt);
			}
#endif
			/* (0x0254) */
			i2c_wr32(sd, base_addr+LPTXTIMECNT, pdata->lptxtimecnt);
			/* (0x0258) */
			i2c_wr32(sd, base_addr+TCLK_HEADERCNT,
				pdata->tclk_headercnt);
			/* (0x025C) */
			i2c_wr32(sd, base_addr+TCLK_TRAILCNT,
				pdata->tclk_trailcnt);
			/* (0x0260) */
			i2c_wr32(sd, base_addr+THS_HEADERCNT,
				pdata->ths_headercnt);
			/* (0x0264) */
			i2c_wr32(sd, base_addr+TWAKEUP, pdata->twakeup);
			/* (0x0268) */
			i2c_wr32(sd, base_addr+TCLK_POSTCNT,
				pdata->tclk_postcnt);
			/* (0x026C) */
			i2c_wr32(sd, base_addr+THS_TRAILCNT,
				pdata->ths_trailcnt);

			/* (0x0270) */
			i2c_wr32(sd, base_addr+HSTXVREGCNT, pdata->hstxvregcnt);

			/* (0x0274) */
			i2c_wr32(sd, base_addr+HSTXVREGEN,
				((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0x0) |
				((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0x0) |
				((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0x0) |
				((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0x0) |
				((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0x0));

			/* NOTE: Probably not necessary */
			/* (0x0278) */
			i2c_wr32(sd, base_addr+BTA_COUNT, 0x00040003);
			/* (0x027C) */
			i2c_wr32(sd, base_addr+DPHY_TX_ADJUST, 0x00000002);

			/*
			 * Finishing configuration by setting CSITX to start
			 * (0X011C)
			 */
			i2c_wr32(sd, base_addr+CSITX_START, 0x00000001);
		} else {
			v4l2_dbg(1, debug, sd,
				"%s: Disabling CSI TX%d\n", __func__, port-1);

			/* Disable CSI lanes (High Z)*/
			i2c_wr32_and_or(sd, base_addr+LANEEN,
				~(MASK_CLANEEN), 0);
		}
	}
}

static void tc358840_set_hdmi_phy(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* Reset PHY */
	tc358840_reset_phy(sd);

	/* Set PHY to manual */
	i2c_wr8(sd, PHY_CTL, MASK_48_MHZ);

	/* Enable PHY */
	i2c_wr8_and_or(sd, PHY_ENB, ~MASK_ENABLE_PHY, 0x0);
	i2c_wr8_and_or(sd, PHY_ENB, ~MASK_ENABLE_PHY, MASK_ENABLE_PHY);

	/* Enable Audio PLL */
	i2c_wr8(sd, APPL_CTL, MASK_APLL_CPCTL_NORMAL | MASK_APLL_ON);

	/* Enable DDC IO */
	i2c_wr8(sd, DDCIO_CTL, MASK_DDC_PWR_ON);
}

static void tc358840_set_hdmi_audio(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	i2c_wr8(sd, FORCE_MUTE, 0x00);
	i2c_wr8(sd, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(sd, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(sd, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(sd, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(sd, FS_MUTE, 0x00);
	i2c_wr8(sd, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
	i2c_wr8(sd, ACR_MODE, MASK_CTS_MODE);
	i2c_wr8(sd, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
	i2c_wr8(sd, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
	/*
	 * TODO: Set output data bit length (currently 16 bit, 8 bit discarded)
	 */
	i2c_wr8(sd, SDO_MODE1, MASK_SDO_FMT_I2S);
	i2c_wr8(sd, DIV_MODE, SET_DIV_DLY_MS(100));
	i2c_wr16_and_or(sd, CONFCTL0, 0xFFFF, MASK_AUDCHNUM_2 |
			MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
}

static void tc358840_set_hdmi_info_frame_mode(struct v4l2_subdev *sd)
{
	v4l2_dbg(3, debug, sd, "%s(): DUMMY\n", __func__);

	/* TODO: Check which registers are needed/available */
#if 0
	i2c_wr8(sd, PK_INT_MODE, MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE |
			MASK_ACP_INT_MODE | MASK_VS_INT_MODE |
			MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
			MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
	i2c_wr8(sd, NO_PKT_LIMIT, 0x2c);
	i2c_wr8(sd, NO_PKT_CLR, 0x53);
	i2c_wr8(sd, ERR_PK_LIMIT, 0x01);
	i2c_wr8(sd, NO_PKT_LIMIT2, 0x30);
	i2c_wr8(sd, NO_GDB_LIMIT, 0x10);
#endif
}

static void tc358840_initial_setup(struct v4l2_subdev *sd)
{
	//static struct v4l2_dv_timings default_timing = V4L2_DV_BT_CEA_1920X1080P60;
	static struct v4l2_dv_timings default_timing = V4L2_DV_BT_CEA_3840X2160P30;
	struct tc358840_state *state = to_state(sd);
	struct tc358840_platform_data *pdata = &state->pdata;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* *** Reset *** */
	enable_stream(sd, false);

	tc358840_sleep_mode(sd, false);
	tc358840_reset(sd, MASK_RESET_ALL);

	tc358840_init_interrupts(sd);

	/* *** Init CSI *** */
	tc358840_s_dv_timings(sd, &default_timing);

	tc358840_set_ref_clk(sd);

	i2c_wr8_and_or(sd, DDC_CTL, ~MASK_DDC5V_MODE,
		       pdata->ddc5v_delay & MASK_DDC5V_MODE);

	i2c_wr8_and_or(sd, EDID_MODE, ~MASK_EDID_MODE_ALL, MASK_RAM_EDDC);

	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_CTL0, 0);

	tc358840_set_hdmi_phy(sd);

	tc358840_set_hdmi_hdcp(sd, pdata->enable_hdcp);
	tc358840_set_hdmi_audio(sd);
	tc358840_set_hdmi_info_frame_mode(sd);

	/* All CE and IT formats are detected as RGB full range in DVI mode */
	i2c_wr8_and_or(sd, VI_MODE, ~MASK_RGB_DVI, 0);
}

/* --------------- IRQ --------------- */

static void tc358840_format_change(struct v4l2_subdev *sd)
{
	struct tc358840_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS,
		.u.src_change.changes = 0,
	};	//FIXME: Change to H.verkuils version with V4L2_EVENT_SOURCE_CHANGE ?

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);
	printk("%s: entered\n", __func__);

	if (tc358840_get_detected_timings(sd, &timings)) {
		enable_stream(sd, false);

		v4l2_dbg(1, debug, sd, "%s: Format changed. No signal\n",
				__func__);
	} else {
		/* TODO: Replace with v4l2_... for newer Kernels */
		if (!v4l_match_dv_timings(&state->timings, &timings, 0))
		{
/*			const struct v4l2_dv_timings *t1 = &state->timings;
			const struct v4l2_dv_timings *t2 = &timings;
			printk("%s: match_dv_timings t1->bt.width=%d, t2->bt.width=%d\n", __func__, t1->bt.width, t2->bt.width);
			printk("%s: match_dv_timings t1->bt.height=%d, t2->bt.height=%d\n", __func__, t1->bt.height, t2->bt.height);
			printk("%s: match_dv_timings t1->bt.vsync=%d, t2->bt.vsync=%d\n", __func__, t1->bt.vsync, t2->bt.vsync);
			printk("%s: match_dv_timings t1->bt.pixelclock=%llu, t2->bt.pixelclock=%llu\n", __func__, t1->bt.pixelclock, t2->bt.pixelclock);*/

			enable_stream(sd, false);
		}
		else
		{
			ev.type = V4L2_EVENT_SOURCE_CHANGE;
			ev.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;
		}

		/* Printing timings is not supported in Kernel version 3.10 */
		v4l2_dbg(1, debug, sd, "%s: Format changed. New Format\n",
				__func__);
#if 0
		v4l2_print_dv_timings(sd->name, "Format changed. New format: ",
				&timings, false);
#endif
	}

	v4l2_subdev_notify(sd, V4L2_DEVICE_NOTIFY_EVENT, &ev);
}

static void tc358840_init_interrupts(struct v4l2_subdev *sd)
{
	u16 i;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* clear interrupt status registers */
	for (i = SYS_INT; i <= MISC_INT; i++) {
		/* No interrupt register at Address 0x850A */
		if (i != 0x850A)
			i2c_wr8(sd, i, 0xFF);
	}

	/* Clear and disable all interrupts */
	i2c_wr16(sd, INTSTATUS, MASK_INT_STATUS_MASK_ALL);
	i2c_wr16(sd, INTSTATUS, 0x0);

	i2c_wr16(sd, INTMASK, MASK_INT_STATUS_MASK_ALL);
}

static void tc358840_enable_interrupts(struct v4l2_subdev *sd,
		bool cable_connected)
{
	v4l2_dbg(2, debug, sd, "%s: cable connected = %d\n", __func__,
			cable_connected);

	if (cable_connected) {
		i2c_wr8(sd, SYS_INTM, ~(MASK_DDC | MASK_DVI |
					MASK_HDMI) & 0xFF);
		i2c_wr8(sd, CLK_INTM, ~MASK_IN_DE_CHG);
		i2c_wr8(sd, CBIT_INTM, ~(MASK_CBIT_FS | MASK_AF_LOCK |
					MASK_AF_UNLOCK) & 0xFF);
		i2c_wr8(sd, AUDIO_INTM, ~MASK_BUFINIT_END);
		i2c_wr8(sd, MISC_INTM, ~MASK_SYNC_CHG);
	} else {
		i2c_wr8(sd, SYS_INTM, ~MASK_DDC & 0xFF);
		i2c_wr8(sd, CLK_INTM, 0xFF);
		i2c_wr8(sd, CBIT_INTM, 0xFF);
		i2c_wr8(sd, AUDIO_INTM, 0xFF);
		i2c_wr8(sd, MISC_INTM, 0xFF);
	}
}

static void tc358840_hdmi_audio_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 audio_int_mask = i2c_rd8(sd, AUDIO_INTM);
	u8 audio_int = i2c_rd8(sd, AUDIO_INT) & ~audio_int_mask;

	i2c_wr8(sd, AUDIO_INT, audio_int);

	v4l2_dbg(3, debug, sd, "%s: AUDIO_INT = 0x%02x\n", __func__, audio_int);

	tc358840_s_ctrl_audio_sampling_rate(sd);
	tc358840_s_ctrl_audio_present(sd);
}

static void tc358840_hdmi_misc_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 misc_int_mask = i2c_rd8(sd, MISC_INTM);
	u8 misc_int = i2c_rd8(sd, MISC_INT) & ~misc_int_mask;

	i2c_wr8(sd, MISC_INT, misc_int);

	v4l2_dbg(3, debug, sd, "%s: MISC_INT = 0x%02x\n", __func__, misc_int);

	if (misc_int & MASK_SYNC_CHG) {
		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358840_reset_phy(sd);
			tc358840_erase_bksv(sd);
		}

		tc358840_format_change(sd);

		misc_int &= ~MASK_SYNC_CHG;
		if (handled)
			*handled = true;
	}

	if (misc_int) {
		v4l2_err(sd, "%s: Unhandled MISC_INT interrupts: 0x%02x\n",
				__func__, misc_int);
	}
}

static void tc358840_hdmi_cbit_int_handler(struct v4l2_subdev *sd,
		bool *handled)
{
	u8 cbit_int_mask = i2c_rd8(sd, CBIT_INTM);
	u8 cbit_int = i2c_rd8(sd, CBIT_INT) & ~cbit_int_mask;

	i2c_wr8(sd, CBIT_INT, cbit_int);

	v4l2_dbg(3, debug, sd, "%s: CBIT_INT = 0x%02x\n", __func__, cbit_int);

	if (cbit_int & MASK_CBIT_FS) {

		v4l2_dbg(1, debug, sd, "%s: Audio sample rate changed\n",
				__func__);
		tc358840_s_ctrl_audio_sampling_rate(sd);

		cbit_int &= ~MASK_CBIT_FS;
		if (handled)
			*handled = true;
	}

	if (cbit_int & (MASK_AF_LOCK | MASK_AF_UNLOCK)) {

		v4l2_dbg(1, debug, sd, "%s: Audio present changed\n",
				__func__);
		tc358840_s_ctrl_audio_present(sd);

		cbit_int &= ~(MASK_AF_LOCK | MASK_AF_UNLOCK);
		if (handled)
			*handled = true;
	}

	if (cbit_int) {
		v4l2_err(sd, "%s: Unhandled CBIT_INT interrupts: 0x%02x\n",
				__func__, cbit_int);
	}
}

static void tc358840_hdmi_clk_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	u8 clk_int_mask = i2c_rd8(sd, CLK_INTM);
	u8 clk_int = i2c_rd8(sd, CLK_INT) & ~clk_int_mask;

	unsigned width, height, frame_width, frame_height;

	printk("%s: entered\n", __func__);

	/* Bit 7 and bit 6 are set even when they are masked */
	i2c_wr8(sd, CLK_INT, clk_int | 0x80 | MASK_OUT_H_CHG);

	v4l2_dbg(3, debug, sd, "%s: CLK_INT = 0x%02x\n", __func__, clk_int);

	if (clk_int & (MASK_IN_DE_CHG)) {

		v4l2_dbg(1, debug, sd, "%s: DE size or position has changed\n",
				__func__);

		/* TODO: Check if also true for tc358840 */
		/* If the source switch to a new resolution with the same pixel
		 * frequency as the existing (e.g. 1080p25 -> 720p50), the
		 * I_SYNC_CHG interrupt is not always triggered, while the
		 * I_IN_DE_CHG interrupt seems to work fine. FMT_CHANGE
		 * notifications are only sent when the signal is stable to
		 * reduce the number of notifications. */
		if (!no_signal(sd) && !no_sync(sd)) {
			tc358840_format_change(sd);
			
			// DBG!!!
			width = ((i2c_rd8(sd, DE_HSIZE_HI) & 0x1f) << 8) +
				i2c_rd8(sd, DE_HSIZE_LO);
			height = ((i2c_rd8(sd, DE_VSIZE_HI) & 0x1f) << 8) +
				i2c_rd8(sd, DE_VSIZE_LO);
			frame_width = ((i2c_rd8(sd, IN_HSIZE_HI) & 0x1f) << 8) +
				i2c_rd8(sd, IN_HSIZE_LO);
			frame_height = (((i2c_rd8(sd, IN_VSIZE_HI) & 0x3f) << 8) +
				i2c_rd8(sd, IN_VSIZE_LO)) / 2;

			printk("%s: DE: width=%d, height=%d  |  IN: frame_width=%d, frame_height=%d\n", __func__, width, height, frame_width, frame_height);
			if (frame_height < height) {
				printk("%s: DBG: ERROR: frame_height < height. \n", __func__);
			}
		} else {
			mdelay(1);
		}

		clk_int &= ~MASK_IN_DE_CHG;
		if (handled)
			*handled = true;
	}

	if (clk_int) {
		v4l2_err(sd, "%s: Unhandled CLK_INT interrupts: 0x%02x\n",
				__func__, clk_int);
	}
}

static void tc358840_hdmi_sys_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	struct tc358840_state *state = to_state(sd);
	u8 sys_int_mask = i2c_rd8(sd, SYS_INTM);
	u8 sys_int = i2c_rd8(sd, SYS_INT) & ~sys_int_mask;

	i2c_wr8(sd, SYS_INT, sys_int);

	v4l2_dbg(3, debug, sd, "%s: SYS_INT = 0x%02x\n", __func__, sys_int);

	if (sys_int & MASK_DDC) {
		bool ddc_5v = ddc_5v_power_present(sd);

		v4l2_dbg(1, debug, sd, "%s: Tx 5V power present: %s\n",
				__func__, ddc_5v ?  "yes" : "no");

		if (ddc_5v) {
			tc358840_enable_edid(sd);
		} else {
			tc358840_enable_interrupts(sd, false);
			tc358840_disable_edid(sd);
			memset(&state->timings, 0, sizeof(state->timings));
			tc358840_erase_bksv(sd);
			tc358840_update_controls(sd);
		}

		sys_int &= ~MASK_DDC;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_DVI) {
		v4l2_dbg(1, debug, sd, "%s: HDMI->DVI change detected\n",
				__func__);

		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358840_reset_phy(sd);
			tc358840_erase_bksv(sd);
		}

		/* CISCO HACK: User space must be notified when the source
		 * switch from HDMI to DVI mode. If not, the video will freeze.
		 * The video is received by the OMAP (omapconf read 0×52001070),
		 * but the picture in the monitor app is frozen.
		 * TODO: Investigate what goes wrong in user space.
		 */
// FIXME: NECESSARY?
		tc358840_format_change(sd);

		sys_int &= ~MASK_DVI;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_HDMI) {
		v4l2_dbg(1, debug, sd, "%s: DVI->HDMI change detected\n",
				__func__);

		/* TODO: Check if reg is reqired. Reg not found in Rev. 1.5 */
#if 0
		i2c_wr8(sd, ANA_CTL, MASK_APPL_PCSX_NORMAL | MASK_ANALOG_ON);
#endif
		sys_int &= ~MASK_HDMI;
		if (handled)
			*handled = true;
	}

	if (sys_int) {
		v4l2_err(sd, "%s: Unhandled SYS_INT interrupts: 0x%02x\n",
				__func__, sys_int);
	}
}

/* --------------- CORE OPS --------------- */

static int tc358840_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	u16 intstatus = i2c_rd16(sd, INTSTATUS);
	unsigned retry = 10;

	v4l2_dbg(1, debug, sd, "%s: IntStatus = 0x%04X\n", __func__, intstatus);

	mdelay(1);
	if (intstatus & MASK_HDMI_INT) {
		u8 hdmi_int0;
		u8 hdmi_int1;
retry:
		retry--;
		hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		mdelay(1);
		hdmi_int1 = i2c_rd8(sd, HDMI_INT1);
		mdelay(1);

		if (hdmi_int0 & MASK_MISC)
			tc358840_hdmi_misc_int_handler(sd, handled);
		if (hdmi_int1 & MASK_ACBIT)
			tc358840_hdmi_cbit_int_handler(sd, handled);
		if (hdmi_int1 & MASK_CLK)
			tc358840_hdmi_clk_int_handler(sd, handled);
		if (hdmi_int1 & MASK_SYS)
			tc358840_hdmi_sys_int_handler(sd, handled);
		if (hdmi_int1 & MASK_AUD)
			tc358840_hdmi_audio_int_handler(sd, handled);

		mdelay(1);
		i2c_wr16(sd, INTSTATUS, MASK_HDMI_INT);
		intstatus &= ~MASK_HDMI_INT;
		mdelay(1);

		/* Display unhandled HDMI interrupts */
		hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		if (hdmi_int0) {
			v4l2_dbg(1, debug, sd,
				"%s: Unhandled HDMI_INT0 interrupts: 0x%02X\n",
				__func__, hdmi_int0);
			if (retry)
				goto retry;
		}
		mdelay(1);
		hdmi_int1 = i2c_rd8(sd, HDMI_INT1);
		if (hdmi_int1) {
			v4l2_dbg(1, debug, sd,
				"%s: Unhandled HDMI_INT1 interrupts: 0x%02X\n",
				__func__, hdmi_int1);
			if (retry)
				goto retry;
		}
	}

	if (intstatus & MASK_CSITX0_INT) {
		v4l2_dbg(3, debug, sd, "%s: MASK_CSITX0_INT\n", __func__);

		i2c_wr16(sd, INTSTATUS, MASK_CSITX0_INT);
		intstatus &= ~MASK_CSITX0_INT;
	}

	if (intstatus & MASK_CSITX1_INT) {
		v4l2_dbg(3, debug, sd, "%s: MASK_CSITX1_INT\n", __func__);

		i2c_wr16(sd, INTSTATUS, MASK_CSITX1_INT);
		intstatus &= ~MASK_CSITX1_INT;
	}

	intstatus = i2c_rd16(sd, INTSTATUS);
	if (intstatus) {
		v4l2_dbg(1, debug, sd,
			"%s: Unhandled IntStatus interrupts: 0x%04X\n",
			__func__, intstatus);
	}

	return 0;
}

static void tc358840_work_isr(struct work_struct *work)
{
	struct tc358840_state *state = container_of(work,
			struct tc358840_state, work_isr);
	struct v4l2_subdev *sd = &state->sd;
	bool handled = false;

	tc358840_isr(sd, 0, &handled);

	/* Enable interrupts again */
	enable_irq(state->pdata.interrupt);
}

static irq_handler_t tc358840_irq_handler(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)(dev_id);
	struct tc358840_state *state = to_state(sd);
	bool ret = true;

	v4l2_dbg(1, debug, sd, "%s()\n", __func__);

	/* Launch ISR as workqueue because I2C access is very slow */
	ret = queue_work(state->work_queues, &state->work_isr);

	if (ret)
		/* Disable interrupts while ISR is active */
		disable_irq_nosync(state->pdata.interrupt);
	else
		v4l2_err(sd, "%s(): ISR already in work queue!\n", __func__);

	return (irq_handler_t) (IRQ_HANDLED);
}


/* --------------- PAD OPS --------------- */

static int tc358840_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *format)
{
	struct tc358840_state *state = to_state(sd);
	// u8 vout_csc = i2c_rd8(sd, VOUT_CSC);
	struct v4l2_mbus_framefmt *fmt;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	format->format.code = state->mbus_fmt_code;
	format->format.width = state->timings.bt.width;
	format->format.height = state->timings.bt.height;
	format->format.field = V4L2_FIELD_NONE;

	switch (state->mbus_fmt_code) {
	case V4L2_MBUS_FMT_UYVY8_1X16:
		format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;	/* 601 YCbCr Limited */
		break;
	case V4L2_MBUS_FMT_RGB888_1X24:
		format->format.colorspace = V4L2_COLORSPACE_SRGB;	/* RGB Full */
		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unsupported format code 0x%x\n",
				__func__, state->mbus_fmt_code);
		break;
	}
	// switch (vout_csc & MASK_COLOR) {
	// case MASK_COLOR_RGB_FULL:
	// case MASK_COLOR_RGB_LIMITED:
	// 	format->format.colorspace = V4L2_COLORSPACE_SRGB;
	// 	break;
	// case MASK_COLOR_601_YCBCR_FULL:
	// case MASK_COLOR_601_YCBCR_LIMITED:
	// 	format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
	// 	break;
	// case MASK_COLOR_709_YCBCR_FULL:
	// case MASK_COLOR_709_YCBCR_LIMITED:
	// 	format->format.colorspace = V4L2_COLORSPACE_REC709;
	// 	break;
	// default:
	// 	format->format.colorspace = 0;
	// 	break;
	// }

	fmt = &format->format;
	/*v4l2_dbg(3, debug, sd,*/printk(
		"%s(): width=%d, height=%d, code=0x%04X, field=%d, colorspace=%d\n",
		__func__, fmt->width, fmt->height, fmt->code, fmt->field, fmt->colorspace);

	return 0;
}

static int tc358840_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *format)
{
	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret = tc358840_get_fmt(sd, fh, format);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	format->format.code = code;

	printk("%s: got code=%u,  ret=%d\n", __func__, code, ret);

	if (ret)
		return ret;

	switch (code) {
	case V4L2_MBUS_FMT_RGB888_1X24:
	case V4L2_MBUS_FMT_UYVY8_1X16:
		break;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	v4l2_dbg(3, debug, sd, "%s(): format->which=%d\n",
		__func__, format->which);

	enable_stream(sd, false);
	tc358840_set_csi(sd);
	tc358840_set_csi_mbus_config(sd);

	return 0;
}

static int tc358840_g_edid(struct v4l2_subdev *sd,
		struct v4l2_subdev_edid *edid)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = state->edid_blocks_written;
		return 0;
	}

	if (state->edid_blocks_written == 0)
		return -ENODATA;

	if (edid->start_block >= state->edid_blocks_written ||
			edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block + edid->blocks > state->edid_blocks_written)
		edid->blocks = state->edid_blocks_written - edid->start_block;

	i2c_rd(sd, EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE), edid->edid,
		   edid->blocks * EDID_BLOCK_SIZE);

	return 0;
}

static int tc358840_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid)
{
	struct tc358840_state *state = to_state(sd);
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;

	v4l2_dbg(2, debug, sd, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	tc358840_disable_edid(sd);

	i2c_wr8(sd, EDID_LEN1, edid_len & 0xFF);
	i2c_wr8(sd, EDID_LEN2, edid_len >> 8);

	if (edid->blocks == 0) {
		state->edid_blocks_written = 0;
		return 0;
	}

	i2c_wr(sd, EDID_RAM, edid->edid, edid_len);

	state->edid_blocks_written = edid->blocks;

	if (ddc_5v_power_present(sd))
		tc358840_enable_edid(sd);

	return 0;
}


/* --------------- VIDEO OPS --------------- */

static int tc358840_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	*status |= no_sync(sd) ? V4L2_IN_ST_NO_SYNC : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int tc358840_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358840_state *state = to_state(sd);
#if 0
	struct v4l2_bt_timings *bt;
#endif
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (!timings)
		return -EINVAL;

	/* Printing the timings is not supported in Kernel version 3.10 */
#if 0
	if (debug)
		v4l_print_dv_timings(sd->name, "tc358840_s_dv_timings: ",
				timings, false);
#endif
	/* TODO: Replace with v4l2_... for newer Kernels */
	if (v4l_match_dv_timings(&state->timings, timings, 0)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

#if 0
	bt = &timings->bt;
#endif

	/* WORKAROUND: v4l2_valid_dv_timings() is not in Kernel v.3.10 */
	/* TODO: Use original function for newer Kernels */
	if (!(valid_dv_timings)(timings,
			&tc358840_timings_cap)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	state->timings = *timings;

	enable_stream(sd, false);
	tc358840_set_csi(sd);
	tc358840_set_splitter(sd);

	return 0;
}

static int tc358840_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*timings = state->timings;

	return 0;
}

static int tc358840_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	v4l2_dbg(3, debug, sd, "%s(): DUMMY\n", __func__);

	/*
	 * Too old kernel for pad in struct v4l2_enum_dv_timings
	 * use first reserved which is same position
	 * if (timings->pad != 0)
	 */
	if (timings->reserved[0] != 0)
		return -EINVAL;

	/* FIXME: Return correct enum */
	/* v4l2_enum_dv_timings_cap is not in Kernel v3.10 */
#if 0
	return v4l2_enum_dv_timings_cap(timings,
			&tc358840_timings_cap, NULL, NULL);
#endif
	return 0;
}

static int tc358840_query_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	int ret;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	ret = tc358840_get_detected_timings(sd, timings);
	if (ret)
		return ret;

	/* Printing the timings is not supported in Kernel version 3.10 */
#if 0
	if (debug)
		v4l_print_dv_timings(sd->name, "tc358840_query_dv_timings: ",
			timings, false);
#endif
	if (!valid_dv_timings(timings,
			&tc358840_timings_cap)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int tc358840_dv_timings_cap(struct v4l2_subdev *sd,
				   struct v4l2_dv_timings_cap *cap)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/*
	 * Too old kernel for pad in struct v4l2_dv_timings_cap
	 * use first reserved which is same position
	 * if (cap->pad != 0)
	 */
	if (cap->reserved[0] != 0)
		return -EINVAL;

	*cap = tc358840_timings_cap;

	return 0;
}

static int tc358840_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	cfg->type = V4L2_MBUS_CSI2;

	/* Support for non-continuous CSI-2 clock is missing in the driver */
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK | V4L2_MBUS_CSI2_CHANNEL_0;

	switch (tc358840_num_csi_lanes_in_use(sd)) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	v4l2_dbg(2, debug, sd, "%s: Lanes: 0x%02X\n",
		__func__, cfg->flags & 0x0F);

	return 0;
}

static int tc358840_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	ret = enable_stream(sd, enable);
	if (ret)
		return ret;

	return 0;
}

static int tc358840_s_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	memcpy(&format.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.pad = 0;

	return tc358840_set_fmt(sd, NULL, &format);
}

static int tc358840_g_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;
	int ret;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	format.pad = 0;
	ret = tc358840_get_fmt(sd, NULL, &format);
	if (ret)
		return ret;

	memcpy(fmt, &format.format, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tc358840_try_mbus_fmt(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;

	v4l2_dbg(2, debug, sd,
		"%s(): width=%d, height=%d, code=0x%04X, field=%d\n",
		__func__, fmt->width, fmt->height, fmt->code, fmt->field);

	memcpy(&format.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	format.which = V4L2_SUBDEV_FORMAT_TRY;
	format.pad = 0;

	return tc358840_set_fmt(sd, NULL, &format);
}

// In newer V4L2 patches, enum_mbus_fmt was replaced by enum_mbus_code
static int tc358840_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct tc358840_state *state = to_state(sd);

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	if (index >= 1)
		return -EINVAL;

	*code = state->mbus_fmt_code;

	return 0;
}

static int tc358840_g_chip_ident(struct v4l2_subdev *sd,
				 struct v4l2_dbg_chip_ident *id)
{
	uint16_t chip_id;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	chip_id = i2c_rd16(sd, CHIPID_ADDR);

	id->ident = (chip_id >> 8) & 0xff;
	id->revision = chip_id & 0xff;

	return 0;
}

static int tc358840_s_power(struct v4l2_subdev *sd, int on)
{
	v4l2_dbg(2, debug, sd, "%s: DUMMY\n", __func__);

	/* FIXME: Fill dummy function */

	return 0;
}

/*	Get a multi link configuration request and return a config suitable for this
	subdev. Currently supported use cases:
	 - 3840x2160 via dual CSI link
	 - 4096x2160 via dual CSI link

	Parameter:	multi_format with width, height and pixelformat
	Return:		multi_format holding suitable configuration
*/
static int tc358840_g_multi_config(struct v4l2_subdev *sd, 
					struct tegra_vi_multi_format *multi_format)
{
	struct v4l2_mbus_framefmt try_fmt;
	int ret, i;

	/* Test if input pf is suitable */
	if (multi_format->composite_pf.width < 3840 || 
		multi_format->composite_pf.height < 2160 /*||
		multi_format->composite_pf.pixelformat != V4L2_PIX_FMT_UYVY*/) {
		v4l2_err(sd, "%s: Input format not supported for multi link config\n", 
			__func__);
		v4l2_err(sd, "%s: width=%d, height=%d, pixelformat=0x%x\n", 
		__func__, multi_format->composite_pf.width, multi_format->composite_pf.height, multi_format->composite_pf.pixelformat);
		printk("%s: pixelformat=0x%x\n", __func__, multi_format->composite_pf.pixelformat);
		printk("%s: required=0x%x\n", __func__, V4L2_PIX_FMT_UYVY);
		return -EINVAL;
	}
	//FIXME: need to check composite_pf.field ?

	/* Test if we support the corresponding single link config */
	try_fmt.width = multi_format->composite_pf.width / 2;
	try_fmt.height = multi_format->composite_pf.height;		//FIXME: correct?
	try_fmt.code = V4L2_MBUS_FMT_UYVY8_1X16;				//FIXME: correct?
	try_fmt.field = V4L2_FIELD_NONE; //multi_format->composite_pf.field;
	try_fmt.colorspace = multi_format->composite_pf.colorspace;
	ret = tc358840_try_mbus_fmt(sd, &try_fmt);
	if(ret) {
		v4l2_err(sd, "%s: Corresponding single link config not supported", 
			__func__);
		return -EINVAL;
	}

	/* Find the closest possible configuration that is supported */
	// FIXME: Replace fixed config by dynamic config
	for (i=0; i<2; i++) {
		multi_format->framefmt[i].width = 3840/2; //multi_format->composite_pf.width / 2;
		multi_format->framefmt[i].height = 2160; //multi_format->composite_pf.height;
		multi_format->framefmt[i].code = V4L2_MBUS_FMT_UYVY8_1X16;	//FIXME: correct?
		multi_format->framefmt[i].field = V4L2_FIELD_NONE; //multi_format->composite_pf.field;
		multi_format->framefmt[i].colorspace = multi_format->composite_pf.colorspace;	//FIXME: Need to fix colorspace?

		multi_format->orientation[i] = i; 	//FIXME: Define orientations
	}
	multi_format->framefmt_count = 2;
	multi_format->composite_pf.sizeimage = 
		multi_format->composite_pf.width * 2 * multi_format->composite_pf.height; // Set fixed?

	return 0;
}

static struct v4l2_subdev_video_ops tc358840_subdev_video_ops = {
	.g_input_status = tc358840_g_input_status,
	.s_dv_timings = tc358840_s_dv_timings,
	.g_dv_timings = tc358840_g_dv_timings,
	.enum_dv_timings = tc358840_enum_dv_timings,
	.query_dv_timings = tc358840_query_dv_timings,
	.dv_timings_cap = tc358840_dv_timings_cap,
	.g_mbus_config = tc358840_g_mbus_config,
	.s_stream = tc358840_s_stream,

	.s_mbus_fmt = tc358840_s_mbus_fmt,
	.g_mbus_fmt = tc358840_g_mbus_fmt,
	.try_mbus_fmt = tc358840_try_mbus_fmt,
	.enum_mbus_fmt = tc358840_enum_mbus_fmt,

	.g_multi_config = tc358840_g_multi_config,
};

static struct v4l2_subdev_core_ops tc358840_subdev_core_ops = {
	.g_chip_ident = tc358840_g_chip_ident,
	.s_power = tc358840_s_power,
	.subscribe_event = v4l2_src_change_event_subdev_subscribe,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	/* TODO: Make the IRQ externally available */
#if 0
	.interrupt_service_routine = tc358840_isr,
#endif
};

static const struct v4l2_subdev_pad_ops tc358840_pad_ops = {
	.set_fmt = tc358840_set_fmt,
	.get_fmt = tc358840_get_fmt,
	.get_edid = tc358840_g_edid,
	.set_edid = tc358840_s_edid,
};

static struct v4l2_subdev_ops tc358840_ops = {
	.core = &tc358840_subdev_core_ops,
	.video = &tc358840_subdev_video_ops,
	.pad = &tc358840_pad_ops,
};

/* TODO: Add EDID to pad ops */


/* --------------- CUSTOM CTRLS --------------- */
static const struct v4l2_ctrl_config tc358840_ctrl_rx_power_present = {
	.id = V4L2_CID_TC358840_DDC_5V_POWER_PRESENT,
	.name = "DDC 5V power present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358840_ctrl_audio_sampling_rate = {
	.id = TC358840_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358840_ctrl_audio_present = {
	.id = TC358840_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* --------------- PROBE / REMOVE --------------- */

#ifdef CONFIG_OF
static struct tc358840_platform_data *of_tc358840(struct i2c_client *client,
						  struct device_node *node)
{
	struct device *dev = &client->dev;
	struct tc358840_platform_data *pdata;
	const u32 *property;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	v4l_dbg(1, debug, client, "Device Tree Parameters:\n");

	pdata->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (pdata->reset_gpio == 0)
		return NULL;
	v4l_dbg(1, debug, client, "reset_gpio = %d\n", pdata->reset_gpio);

	pdata->interrupt = irq_of_parse_and_map(node, 0);
	if (pdata->interrupt <= 0)
		return NULL;
	v4l_dbg(1, debug, client, "interrupt = %d\n", pdata->interrupt);

	property = of_get_property(node, "refclk_hz", NULL);
	if (property == NULL)
		return NULL;
	pdata->refclk_hz = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "refclk_hz = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "ddc5v_delay", NULL);
	if (property == NULL)
		return NULL;
	pdata->ddc5v_delay = be32_to_cpup(property);
	if (pdata->ddc5v_delay > DDC5V_DELAY_MAX)
		pdata->ddc5v_delay = DDC5V_DELAY_MAX;
	v4l_dbg(1, debug, client, "ddc5v_delay = %d ms\n",
		50 * pdata->ddc5v_delay);

	property = of_get_property(node, "enable_hdcp", NULL);
	if (property == NULL)
		return NULL;
	pdata->enable_hdcp = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "enable_hdcp = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "csi_port", NULL);
	if (property == NULL)
		return NULL;
	pdata->csi_port = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "csi_port = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lineinitcnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->lineinitcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "lineinitcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lptxtimecnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->lptxtimecnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "lptxtimecnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_headercnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->tclk_headercnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_headercnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "tclk_trailcnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->tclk_trailcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_trailcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "ths_headercnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->ths_headercnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "ths_headercnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "twakeup", NULL);
	if (property == NULL)
		return NULL;
	pdata->twakeup = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "twakeup = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_postcnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->tclk_postcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "tclk_postcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "ths_trailcnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->ths_trailcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "ths_trailcnt = %d\n",
		be32_to_cpup(property));

	property = of_get_property(node, "hstxvregcnt", NULL);
	if (property == NULL)
		return NULL;
	pdata->hstxvregcnt = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "hstxvregcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_prd", NULL);
	if (property == NULL)
		return NULL;
	pdata->pll_prd = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "pll_prd = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_fbd", NULL);
	if (property == NULL)
		return NULL;
	pdata->pll_fbd = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "pll_fbd = %d\n", be32_to_cpup(property));

	return pdata;
}
#endif

static int tc358840_verify_chipid(struct v4l2_subdev *sd)
{
	uint16_t cid = 0;

	cid = i2c_rd16(sd, CHIPID_ADDR);
	if (cid != TC358840_CHIPID) {
		v4l2_err(sd, "Invalid chip ID 0x%04X\n", cid);
		return -ENODEV;
	}

	v4l2_dbg(1, debug, sd, "TC358840 ChipID 0x%02x, Revision 0x%02x\n",
		(cid & MASK_CHIPID) >> 8, cid & MASK_REVID);

	return 0;
}

static int tc358840_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tc358840_state *state;
	struct tc358840_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	int irq;
	int err;

#ifdef CONFIG_OF
	struct device_node *node = client->dev.of_node;
#endif

	state = devm_kzalloc(&client->dev, sizeof(struct tc358840_state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

/* platform data */
#ifdef CONFIG_OF
	if (!pdata && node)
		pdata = of_tc358840(client, node);
#endif

	if (!pdata) {
		v4l_err(client, "No platform data!\n");
		return -ENODEV;
	}
	state->pdata = *pdata;
	state->i2c_client = client;
	sd = &state->sd;

	i2c_set_clientdata(client, state);

	v4l2_i2c_subdev_init(sd, client, &tc358840_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	/* Release System Reset (pin K8) */
	v4l2_info(sd, "Releasing System Reset (gpio 0x%04X)\n",
		state->pdata.reset_gpio);
	if (!gpio_is_valid(state->pdata.reset_gpio)) {
		v4l_err(client, "Reset GPIO is invalid!\n");
		return state->pdata.reset_gpio;
	}
	err = devm_gpio_request_one(&client->dev, state->pdata.reset_gpio,
					GPIOF_OUT_INIT_HIGH, "tc358840-reset");
	if (err) {
		dev_err(&client->dev,
			"Failed to request Reset GPIO 0x%04X: %d\n",
			state->pdata.reset_gpio, err);
		return err;
	}

	/*  */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_dbg(1, debug, client, "Chip found @ 7h%02X (%s)\n",
		client->addr, client->adapter->name);


	/* Verify chip ID */
	err = tc358840_verify_chipid(sd);
	if (err)
		return err;

	/* Control Handlers */
	v4l2_ctrl_handler_init(&state->hdl, 3);

	/* Custom controls */
	state->detect_ddc_5v_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_rx_power_present, NULL);

	state->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_audio_sampling_rate, NULL);

	state->audio_present_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358840_ctrl_audio_present, NULL);


	if (tc358840_update_controls(sd)) {
		err = -ENODEV;
		goto err_hdl;
	}

	/* Work Queues */
	state->work_queues = create_singlethread_workqueue(client->name);
	if (!state->work_queues) {
		v4l2_err(sd, "Could not create work queue!\n");
		err = -ENOMEM;
		goto err_hdl;
	}
	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug,
			tc358840_delayed_work_enable_hotplug);
	INIT_WORK(&state->work_isr, tc358840_work_isr);

	/* Initial Setup */
	state->mbus_fmt_code = V4L2_MBUS_FMT_UYVY8_1X16;
	tc358840_initial_setup(sd);

	tc358840_set_csi_mbus_config(sd);

	/* Get interrupt */
	irq = state->pdata.interrupt;
	err = devm_request_irq(&state->i2c_client->dev,
			irq, (irq_handler_t)tc358840_irq_handler, 0,
			"tc358840", (void *)sd);
	if (err) {
		v4l2_err(sd, "Could not request interrupt %d!\n", irq);
		goto err_hdl;
	}

	tc358840_enable_interrupts(sd, ddc_5v_power_present(sd));

	/*
	 * FIXME: Don't know what MASK_CSITX0_INT and MASK_CSITX1_INT do.
	 * Thus, disable them for now...
	 */
#if 0
	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_INT | MASK_CSITX0_INT |
		MASK_CSITX1_INT) & 0xFFFF);
#endif
	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_INT) & 0xFFFF);

	v4l2_ctrl_handler_setup(sd->ctrl_handler);

	/* FIXME: Static EDID configuration */
	tc358840_s_edid(sd, &EDID_1080p);

	v4l2_info(sd, "%s found @ 7h%02X (%s)\n", client->name,
		  client->addr, client->adapter->name);

	return v4l2_async_register_subdev(sd);

err_hdl:
	v4l2_ctrl_handler_free(&state->hdl);
	return err;
}

static int tc358840_remove(struct i2c_client *client)
{
	v4l_dbg(1, debug, client, "%s()\n", __func__);

	return 0;
}

static const struct i2c_device_id tc358840_id[] = {
	{ "tc358840", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc358840_id);

#ifdef CONFIG_OF
static const struct of_device_id tc358840_of_table[] = {
	{ .compatible = "toshiba,tc358840xbg" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358840_of_table);
#endif

static struct i2c_driver tc358840_driver = {
	.driver = {
		.of_match_table = of_match_ptr(tc358840_of_table),
		.name = "tc358840",
		.owner = THIS_MODULE,
	},
	.probe = tc358840_probe,
	.remove = tc358840_remove,
	.id_table = tc358840_id,
};
module_i2c_driver(tc358840_driver);

MODULE_DESCRIPTION("Driver for Toshiba TC358840 HDMI to CSI-2 Bridge");
MODULE_AUTHOR("Armin Weiss (weii@zhaw.ch)");
MODULE_LICENSE("GPL v2");
