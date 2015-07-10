/*
 * es-d300.c  --  Audience es D300 component ALSA Audio driver
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Greg Clemson <gclemson@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include "escore.h"
#include "es755.h"
#include "escore-list.h"
#include "escore-slim.h"
#include "es-d300.h"
#include "es-d300-route.h"

struct cachedcmd_t {
	u8 reg;
	u8 refcnt;
};

struct cachedcmd_t prev_cmdlist[ES_API_ADDR_MAX];

struct channel_id {
	u8 tx_chan_id;
	u8 rx_chan_id;
} channel_ids[ALGO_MAX][PORT_MAX];

static struct cachedcmd_t **cachedcmd_list;
static int es_vp_tx;
static int es_vp_rx;
static int es_az_tx;
static int es_az_rx;

#define LEFT_CAPTURE		0x1
#define RIGHT_CAPTURE		0x2
#define MONO_CAPTURE		(LEFT_CAPTURE)
#define STEREO_CAPTURE		(LEFT_CAPTURE | RIGHT_CAPTURE)

#define OUTPUT_NONE	0x0
#define OUTPUT_AO1	0x1
#define OUTPUT_MO2	0x2
#define OUTPUT_PO1	0x4
#define OUTPUT_PO2	0x8

/* Used to provide mixed output on a single channel output */
#define OUTPUT_MIXED_MONO	(OUTPUT_AO1)

/* Used to copy the stereo output to extra two channels output */
#define OUTPUT_PT_COPY		(OUTPUT_AO1 | OUTPUT_MO2)

/* Mask to keep track of chmgrs set by UCM */
static u16 chn_mgr_mask[ALGO_MAX];

static const u8 pcm_port[] = { 0x0, 0xA, 0xB, 0xC };
static int loopback_mode;

static const u32 pt_vp_aec_msgblk[] = {
	0xB0640528,
	0xB0640134,
};

static const char * const proc_block_input_texts[] = {
	"None",
	"PCM0.0", "PCM0.1", "PCM0.2", "PCM0.3",
	"PCM1.0", "PCM1.1", "PCM1.2", "PCM1.3",
	"PCM2.0", "PCM2.1", "PCM2.2", "PCM2.3",
	"PDMI0", "PDMI1", "PDMI2", "PDMI3",
	"SBUS.RX0", "SBUS.RX1", "SBUS.RX2", "SBUS.RX3",
	"SBUS.RX4", "SBUS.RX5", "SBUS.RX6", "SBUS.RX7",
	"SBUS.RX8", "SBUS.RX9",
	"ADC0", "ADC1", "ADC2", "ADC3",
};

static const u16 es300_input_mux_text_to_api[] = {
	0xffff, /* Default value for all input MUXes */
	ES300_DATA_PATH(PCM0, 0, 0), ES300_DATA_PATH(PCM0, 1, 0),
	ES300_DATA_PATH(PCM0, 2, 0), ES300_DATA_PATH(PCM0, 3, 0),
	ES300_DATA_PATH(PCM1, 0, 0), ES300_DATA_PATH(PCM1, 1, 0),
	ES300_DATA_PATH(PCM1, 2, 0), ES300_DATA_PATH(PCM1, 3, 0),
	ES300_DATA_PATH(PCM2, 0, 0), ES300_DATA_PATH(PCM2, 1, 0),
	ES300_DATA_PATH(PCM2, 2, 0), ES300_DATA_PATH(PCM2, 3, 0),
	ES300_DATA_PATH(PDMI0, 0, 0),
	ES300_DATA_PATH(PDMI1, 0, 0),
	ES300_DATA_PATH(PDMI2, 0, 0),
	ES300_DATA_PATH(PDMI3, 0, 0),
	ES300_DATA_PATH(SBUS, 0, 0), ES300_DATA_PATH(SBUS, 1, 0),
	ES300_DATA_PATH(SBUS, 2, 0), ES300_DATA_PATH(SBUS, 3, 0),
	ES300_DATA_PATH(SBUS, 4, 0), ES300_DATA_PATH(SBUS, 5, 0),
	ES300_DATA_PATH(SBUS, 6, 0), ES300_DATA_PATH(SBUS, 7, 0),
	ES300_DATA_PATH(SBUS, 8, 0), ES300_DATA_PATH(SBUS, 9, 0),
	ES300_DATA_PATH(ADC0, 0, 0),
	ES300_DATA_PATH(ADC1, 0, 0),
	ES300_DATA_PATH(ADC2, 0, 0),
	ES300_DATA_PATH(ADC3, 0, 0),
};

static const char * const proc_block_output_texts[] = {
	"None",
	"VP CSOUT1", "VP CSOUT2", "VP FEOUT1", "VP FEOUT2",
	"AudioZoom CSOUT",
	"AudioZoom AOUT1",
	"MM AUDOUT1", "MM AUDOUT2", "MM PASSOUT1", "MM PASSOUT2",
	"MM MONOUT1", "MM MONOUT2",
	"Pass AUDOUT1", "Pass AUDOUT2", "Pass AUDOUT3", "Pass AUDOUT4",
	"Pass AO1", "Pass MO2",
	"MONOUT1", "MONOUT2", "MONOUT3", "MONOUT4",
};

static const u16 es300_output_mux_text_to_api[] = {
	0xffff, /* Default value for all output MUXes */

	/* VP outputs */
	ES300_PATH_ID(TXCHMGR0, ES300_CSOUT1),
	ES300_PATH_ID(TXCHMGR1, ES300_CSOUT2),
	ES300_PATH_ID(TXCHMGR2, ES300_FEOUT1),
	ES300_PATH_ID(TXCHMGR3, ES300_FEOUT2),

	/* AudioZoom */
	ES300_PATH_ID(TXCHMGR0, ES300_CSOUT1),
	ES300_PATH_ID(TXCHMGR1, ES300_AUDOUT1),

	/* MM AUDOUT */
	ES300_PATH_ID(TXCHMGR0, ES300_AUDOUT1),
	ES300_PATH_ID(TXCHMGR1, ES300_AUDOUT2),

	/* MM PASSOUT */
	ES300_PATH_ID(TXCHMGR4, ES300_PASSOUT1),
	ES300_PATH_ID(TXCHMGR5, ES300_PASSOUT2),

	/* MM MONOUT */
	ES300_PATH_ID(TXCHMGR2, ES300_MM_MONOUT1),
	ES300_PATH_ID(TXCHMGR3, ES300_MM_MONOUT2),

	/* Passthru AUDOUT */
	ES300_PATH_ID(TXCHMGR0, ES300_PASSOUT1),
	ES300_PATH_ID(TXCHMGR1, ES300_PASSOUT2),
	ES300_PATH_ID(TXCHMGR2, ES300_PASSOUT3),
	ES300_PATH_ID(TXCHMGR3, ES300_PASSOUT4),
	ES300_PATH_ID(TXCHMGR4, ES300_AUDOUT1),
	ES300_PATH_ID(TXCHMGR5, ES300_MONOUT2),

	/* UI Tone MONOUT */
	ES300_PATH_ID(TXCHMGR1, ES300_MONOUT1),
	ES300_PATH_ID(TXCHMGR2, ES300_MONOUT2),
	ES300_PATH_ID(0, ES300_MONOUT3),
	ES300_PATH_ID(0, ES300_MONOUT4),
};

struct out_mux_map es_out_mux_map[] = {
	{ .port_desc = ES300_DATA_PATH(DAC0, 0, 0), .mux_id = ES_DAC0_0_MUX },
	{ .port_desc = ES300_DATA_PATH(DAC0, 0, 0), .mux_id = ES_DAC0_1_MUX },
	{ .port_desc = ES300_DATA_PATH(DAC1, 0, 0), .mux_id = ES_DAC1_0_MUX },
	{ .port_desc = ES300_DATA_PATH(DAC1, 0, 0), .mux_id = ES_DAC1_1_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM0, 0, 0), .mux_id = ES_PCM0_0_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM0, 0, 0), .mux_id = ES_PCM0_1_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM0, 0, 0), .mux_id = ES_PCM0_2_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM0, 0, 0), .mux_id = ES_PCM0_3_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM1, 0, 0), .mux_id = ES_PCM1_0_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM1, 0, 0), .mux_id = ES_PCM1_1_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM1, 0, 0), .mux_id = ES_PCM1_2_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM1, 0, 0), .mux_id = ES_PCM1_3_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM2, 0, 0), .mux_id = ES_PCM2_0_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM2, 0, 0), .mux_id = ES_PCM2_1_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM2, 0, 0), .mux_id = ES_PCM2_2_MUX },
	{ .port_desc = ES300_DATA_PATH(PCM2, 0, 0), .mux_id = ES_PCM2_3_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX0_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX1_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX2_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX3_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX4_MUX },
	{ .port_desc = ES300_DATA_PATH(SBUS, 0, 0), .mux_id = ES_SBUSTX5_MUX },
};

static const u32 es_base_route_preset[ALGO_MAX] = {
	[VP] = 0x90311771,
	[VP_MM] = 0x90311777,
	[PASSTHRU_VP] = 0x90311785,
	[AUDIOZOOM] = 0x90311774,
#if defined(CONFIG_SND_SOC_ES_SLIM)
	[MM] = 0x90311773,
	[PASSTHRU] = 0x90311776,
#else
	[MM] = 0x90311772,
	[PASSTHRU] = 0x9031177D,
#endif
	[DHWPT] = 0x9031178F,
};

static const struct es_ch_mgr_max es_chn_mgr_max[ALGO_MAX] = {
	[VP] = {
		.rx = VP_RXCHMGR_MAX,
		.tx = VP_TXCHMGR_MAX,
	},
	[MM] = {
		.rx = MM_RXCHMGR_MAX,
		.tx = MM_TXCHMGR_MAX,
	},
	[VP_MM] = {
		.rx = VP_MM_RXCHMGR_MAX,
		.tx = VP_MM_TXCHMGR_MAX,
	},
	[PASSTHRU_VP] = {
		.tx = PT_VP_TXCHMGR_MAX,
	},
	[DHWPT] = {
		.rx = DHWPT_RXCHMGR_MAX,
		.tx = DHWPT_TXCHMGR_MAX,
	},
};

static u32 switch_arr[] = {
	[SWIN1_I0] = 0x90660009,
	[SWIN1_I1] = 0x90660109,
	[SWIN2_I0] = 0x9066000A,
	[SWIN2_I1] = 0x9066010A,
	[SWOUT0_O1] = 0x90660100,
	[SWOUT1_O1] = 0x90660101,
	[SWOUT2_O1] = 0x90660102,
	[SWOUT3_O1] = 0x90660103,
};

static int escore_set_switch(int id)
{
	struct escore_priv *escore = &escore_priv;
	u32 cmd, resp;
	int rc = 0;

	/*
	 * The delay is required to make sure the route is active.
	 * Without delay, the switch settings are not coming into
	 * effect.
	 */
	usleep_range(1000, 1005);
	cmd = switch_arr[id];

	mutex_lock(&escore->api_mutex);
	rc = escore->bus.ops.cmd(escore, cmd, &resp);
	if (rc) {
		pr_err("%s(): Error %d setting switch preset %x\n",
				__func__, rc, cmd);
		goto err;
	}

	/*
	cmd = ES_SYNC_CMD << 16;
	rc = escore->bus.ops.cmd(escore, cmd, &resp);
	if (rc) {
		pr_err("%s(): Error %d in sending sync cmd\n",
				__func__, rc);
		goto err;
	}
	*/
err:
	mutex_unlock(&escore->api_mutex);
	return rc;
}

static int es300_codec_stop_algo(struct escore_priv *escore)
{
	u32 stop_route_cmd = ES_STOP_ROUTE<<16;
	u32 resp;
	int ret = 0;

	/* Disable DHWPT if enabled */
	if (escore->algo_type == DHWPT) {
		u32 cmd = escore->dhwpt_cmd & 0xFFFF0000;
		ret = escore_cmd(escore, cmd, &resp);
		if (ret) {
			pr_err("%s: Disabling DHWPT failed = %d\n",
					__func__, ret);
			return ret;
		}
	}

	/* Stop the route */
	ret = escore_cmd(escore, stop_route_cmd, &resp);
	if (ret) {
		pr_err("%s: Route stop failed\n", __func__);
		return ret;
	}

	escore->current_preset = 0;
	return ret;
}

static void update_chan_id(u32 *msg, int chan_id)
{
	*msg &= ES_API_WORD(ES_SET_PATH_CMD,
			ES300_DATA_PATH(0x7F, 0x0, 0xF));
	*msg |= ES_API_WORD(ES_SET_PATH_CMD,
			ES300_DATA_PATH(0x0, chan_id, 0x0));
}
static int convert_input_mux_to_cmd(struct escore_priv *escore, int reg)
{
	unsigned int value;
	int msg_len = escore->api_access[reg].write_msg_len;
	u32 msg[ES_CMD_ACCESS_WR_MAX] = {0};
	u16 port, chnum;
	u16 ch_mgr;
	u8 path_id = 0;
	u8 update_cmds = 0;
	u8 update_chmgr_mask = 1;
	int mux = cachedcmd_list[escore->algo_type][reg].reg;

	memcpy((char *)msg, (char *)escore->api_access[reg].write_msg,
			msg_len);
	value = es300_input_mux_text_to_api[mux];

	pr_debug("%s(): reg = %d mux = %d\n", __func__, reg, mux);
	msg[0] |= value;
	path_id = msg[1] & 0xFF;
	ch_mgr = ES_CHMGR_DATAPATH(msg[0]);

	switch (escore->algo_type) {
	case AUDIOZOOM:
		if (reg == ES_TERTIARY_MUX) {
			/* Connect RxChMgr2.o0 */
			msg[2] = ES_API_WORD(ES_CONNECT_CMD,
					ES300_ENDPOINT(FILTER_RXCHANMGR2, OUT,
						RxChMgr_o0));
			msg_len += sizeof(*msg);

			/* Connect AZ.i2 */
			msg[3] = ES_API_WORD(ES_CONNECT_CMD,
					ES300_ENDPOINT(FILTER_AZ, IN, az_i2));
			msg_len += sizeof(*msg);

			/* Set Rate 48k */
			msg[4] = ES_API_WORD(ES_SET_RATE_CMD,
					ES300_RATE(FILTER_RXCHANMGR2, 3));
			msg_len += sizeof(*msg);

			/* Set Rate 48k */
			msg[5] = ES_API_WORD(ES_SET_GROUP_CMD,
					ES300_GROUP(FILTER_RXCHANMGR2, 0));
			msg_len += sizeof(*msg);

		} else if (reg == ES_AZ_AI1_MUX) {
			port = (value >> 9) & 0x1f;
			chnum = (value >> 4) & 0x1f;

			/* Update the data path */
			msg[0] = ES_API_WORD(ES_SET_MUX_CMD,
					ES300_DATA_PATH(port, chnum, RXCHMGR3));

			/* Set corresponding path id */
			msg[1] = ES_API_WORD(ES_SET_PATH_ID_CMD,
				ES300_PATH_ID(RXCHMGR3, path_id));

			msg_len = 8;
			chn_mgr_mask[escore->algo_type] |= 1 << RXCHMGR3;
			update_chmgr_mask = 0;

		}
		break;

	case PASSTHRU:
		break;

	case MM:
		update_cmds = 1;
		port = (value >> 9) & 0x1f;
		chnum = (value >> 4) & 0x1f;

		switch (reg) {
		case ES_MM_PASSIN1_MUX:
			ch_mgr = RXCHMGR4;
			break;
		case ES_MM_PASSIN2_MUX:
			ch_mgr = RXCHMGR5;
			break;
		default:
			update_cmds = 0;
			break;
		}

		if (update_cmds) {
			/* Update the data path */
			msg[0] = ES_API_WORD(ES_SET_MUX_CMD,
					ES300_DATA_PATH(port, chnum, ch_mgr));

			/* Set corresponding path id */
			msg[1] = ES_API_WORD(ES_SET_PATH_ID_CMD,
					ES300_PATH_ID(ch_mgr, path_id));

			msg_len = 8;
			chn_mgr_mask[escore->algo_type] |= 1 << ch_mgr;
			update_chmgr_mask = 0;
		}
		break;

	case VP_MM:
		update_cmds = 1;
		port = (value >> 9) & 0x1f;
		chnum = (value >> 4) & 0x1f;

		switch (reg) {
		case ES_AUDIN1_MUX:
			ch_mgr = RXCHMGR8;
			break;
		case ES_AUDIN2_MUX:
			ch_mgr = RXCHMGR9;
			break;
		default:
			update_cmds = 0;
			break;
		}

		if (update_cmds) {
			/* Update the data path */
			msg[0] = ES_API_WORD(ES_SET_MUX_CMD,
					ES300_DATA_PATH(port, chnum, ch_mgr));

			/* Set corresponding path id */
			msg[1] = ES_API_WORD(ES_SET_PATH_ID_CMD,
					ES300_PATH_ID(ch_mgr, path_id));

			msg_len = 8;
			chn_mgr_mask[escore->algo_type] |= 1 << ch_mgr;
			update_chmgr_mask = 0;
		}
		break;
	case VP:
		prepare_mux_cmd(reg, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_vp_mux_info, CMD_INPUT);
		msg[0] |= value;
		update_chmgr_mask = 0;
		break;

	case DHWPT:
		/* Unused channel managers in base route 6031 */
		chn_mgr_mask[escore->algo_type] |= BIT(RXCHMGR5);
		prepare_mux_cmd(reg, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_dhwpt_mux_info, CMD_INPUT);
		if (reg != ES_PASSIN1_MUX && reg != ES_PASSIN2_MUX) {
			msg[0] |= value;
			update_chmgr_mask = 0;
		} else {
			goto done;
		}
		break;
	case PASSTHRU_VP:
		prepare_mux_cmd(reg, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_pt_vp_mux_info, CMD_INPUT);
		msg[0] |= value;
		update_chmgr_mask = 0;
		break;
	}

	port = (msg[0] >> 9) & 0x1f;
	update_chan_id(&msg[0],
			channel_ids[escore->algo_type][port].rx_chan_id);
	channel_ids[escore->algo_type][port].rx_chan_id++;

	if (update_chmgr_mask)
		chn_mgr_mask[escore->algo_type] |= 1 << ch_mgr;
done:
	return escore_queue_msg_to_list(escore, (char *)msg, msg_len);
}

static int convert_output_mux_to_cmd(struct escore_priv *escore, int reg)
{
	unsigned int value;
	int msg_len = escore->api_access[reg].write_msg_len;
	u32 msg[ES_CMD_ACCESS_WR_MAX] = {0};
	u16 port, ch_mgr;
	u8 path_id = 0;
	u8 update_chmgr_mask = 1, update_msgs = 1;
	u32 i;
	int mux = cachedcmd_list[escore->algo_type][reg].reg;

	memcpy((char *)msg, (char *)escore->api_access[reg].write_msg,
				msg_len);
	value = es300_output_mux_text_to_api[mux];

	pr_debug("%s(): reg = %d mux = %d\n", __func__, reg, mux);

	path_id = value & 0x3f;
	ch_mgr = (value >> 8) & 0xf;

	switch (escore->algo_type) {
	case VP:
		prepare_mux_cmd(mux, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_vp_mux_info, CMD_OUTPUT);
		for (i = 0; i < ARRAY_SIZE(es_out_mux_map); i++) {
			if (es_out_mux_map[i].mux_id == reg) {
				msg[0] |= es_out_mux_map[i].port_desc;
				break;
			}
		}
		update_chmgr_mask = 0;
		update_msgs = 0;
		break;
	case AUDIOZOOM:
		break;
	case MM:
		break;
	case PASSTHRU:
		break;
	case PASSTHRU_VP:
		prepare_mux_cmd(mux, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_pt_vp_mux_info, CMD_OUTPUT);
		for (i = 0; i < ARRAY_SIZE(es_out_mux_map); i++) {
			if (es_out_mux_map[i].mux_id == reg) {
				msg[0] |= es_out_mux_map[i].port_desc;
				break;
			}
		}
		update_chmgr_mask = 0;
		update_msgs = 0;
		break;
	case DHWPT:
		prepare_mux_cmd(mux, msg, &msg_len,
				&chn_mgr_mask[escore->algo_type],
				&es_dhwpt_mux_info, CMD_OUTPUT);
		for (i = 0; i < ARRAY_SIZE(es_out_mux_map); i++) {
			if (es_out_mux_map[i].mux_id == reg) {
				if (mux != PASS_AUDOUT1 && mux != PASS_AUDOUT2)
					msg[0] |= es_out_mux_map[i].port_desc;
				else
					goto done;
			}
		}
		update_chmgr_mask = 0;
		update_msgs = 0;
		break;
	}

	port = (msg[0] >> 9) & 0x1f;
	update_chan_id(&msg[0],
			channel_ids[escore->algo_type][port].tx_chan_id);
	channel_ids[escore->algo_type][port].tx_chan_id++;

	if (update_msgs) {
		msg[0] |= ES300_DATA_PATH(0, 0, ch_mgr);
		msg[1] |= ES300_PATH_ID(ch_mgr, path_id);
	}

	if (update_chmgr_mask)
		chn_mgr_mask[escore->algo_type] |= 1 << ch_mgr;
done:
	return escore_queue_msg_to_list(escore, (char *)msg, msg_len);
}

static void set_chmgr_null(struct escore_priv *escore)
{
	u32 msg = ES_SET_MUX_NULL;
	int i;
	u16 mask = chn_mgr_mask[escore->algo_type];
	pr_debug("%s: mask %04x\n", __func__, mask);

	/* Set RXCHMGR NULL */
	for (i = 0; i < es_chn_mgr_max[escore->algo_type].rx; i++) {
		if (mask & (1 << i))
			continue;
		msg |= i;
		escore_queue_msg_to_list(escore, (char *)&msg, sizeof(msg));
		msg &= ES_SET_MUX_NULL_MASK;
	}

	/* Set TXCHMGR NULL */
	for (i = 0; i < es_chn_mgr_max[escore->algo_type].tx; i++) {
		if (mask & (1 << (i + TXCHMGR0)))
			continue;
		msg |= i + TXCHMGR0;
		escore_queue_msg_to_list(escore, (char *)&msg, sizeof(msg));
		msg &= ES_SET_MUX_NULL_MASK;
	}
}

static int add_algo_base_route(struct escore_priv *escore)
{
	u32 msg;
	u32 cmd = ES_SYNC_CMD << 16;
	int rc;
	int algo = escore->algo_type;

	/* Set unused CHMGRs to NULL */
	set_chmgr_null(escore);

	msg = es_base_route_preset[algo];

	escore->current_preset =  msg & 0xFFFF;
	rc = escore_queue_msg_to_list(escore, (char *)&msg, sizeof(msg));

	/* Configure command completion mode */
	if (escore->cmd_compl_mode == ES_CMD_COMP_INTR) {
		cmd |= escore->pdata->gpio_a_irq_type;
		escore_queue_msg_to_list(escore, (char *)&cmd,
				sizeof(cmd));
	}

	return rc;
}

static void es_clear_route(struct escore_priv *escore)
{

	pr_debug("%s\n", __func__);

	if (escore->algo_type == AUDIOZOOM) {
		es_az_tx = ES_AZ_NONE;
		es_az_rx = ES_AZ_NONE;
	}
}

static int es_set_algo_rate(struct escore_priv *escore, int algo)
{
	int rc = 0;
	u32 filter;
	u32 rate_msg, resp;

	switch (algo) {
	case VP:
	case PASSTHRU_VP:
		filter = FILTER_VP;
		break;
	case MM:
		filter = FILTER_MM;
		break;
	case PASSTHRU:
		filter = FILTER_PASSTHRU;
		break;
	case AUDIOZOOM:
		filter = FILTER_AZ;
		break;
	default:
		pr_err("%s(): Algorithm rate not supported for algo type:%d\n",
				__func__, algo);
		break;
	}
	/* Set algo rate if specified */
	if (cachedcmd_list[escore->algo_type][ES_ALGO_SAMPLE_RATE].reg && filter) {

		usleep_range(5000, 5005);
		/* Added None as first param in algorithm_rate_enum
		 * so array index is shifted by 1
		 */
		pr_debug("%s(): Algorithm rate :%d\n",
				__func__, (cachedcmd_list[escore->algo_type][ES_ALGO_SAMPLE_RATE].reg - 1));
		rate_msg = (ES_SET_RATE_CMD << 16) |
			(((cachedcmd_list[escore->algo_type][ES_ALGO_SAMPLE_RATE].reg - 1) << 8) | filter);
		/* SetRate command must be a COMMIT command to come into effect.
		 * By clearing BIT 29 will clear the STAGE bit in a command
		 * and make it a COMMIT command.
		 */
		/* clear_bit(ES_SC_BIT, (unsigned long *) &rate_msg); */

		/* On Loki, crash is observed with clear_bit API.
		Replaced clear_bit API with bit wise operation */
		rate_msg &= ~(1 << ES_SC_BIT);

		rc = escore_cmd(escore, rate_msg, &resp);
		if (rc)
			pr_err("%s(): Fail to set algorithm rate :%d\n",
					__func__, rc);
	}
	return rc;
}

static int es_set_final_route(struct escore_priv *escore)
{
	int rc = 0, i;
	u16 preset;

	pr_debug("%s\n", __func__);

	if ((escore->algo_type == VP) && !escore->vp_asr) {
		if (es_vp_tx != ES_VP_TX_INIT ||
				es_vp_rx != ES_VP_RX_INIT) {
			pr_debug("%s VP not ready rx=%d tx=%d\n", __func__,
				es_vp_rx, es_vp_tx);
			return 0;
		}
	} else if (escore->algo_type == AUDIOZOOM) {
		if (es_az_tx != ES_AZ_TX_INIT ||
				es_az_rx != ES_AZ_RX_INIT) {
			pr_debug("%s AZ not ready rx=%d tx=%d\n", __func__,
				es_az_rx, es_az_tx);
			return 0;
		}
	}

	preset = es_base_route_preset[escore->algo_type] & 0xFFFF;

	/* In case of simultaneous playback and capture usecase, both playback
	 * and capture muxes will set. If playback and capture are started
	 * independent of each other, the route needs not to be set again. This
	 * check will make sure that if a preset is already set and route
	 * is running, there is no need to set the route again
	 */
	if (escore->current_preset == preset) {
		if (!memcmp(prev_cmdlist, cachedcmd_list[escore->algo_type],
					sizeof(prev_cmdlist))) {
			pr_debug("%s(): Skip same preset %x\n", __func__, preset);
			return 0;
		}
		es300_codec_stop_algo(escore);
	}

	/* If the new use-case is started and still there are pending streams
	 * from previous use-case, just go ahead and write the new route to
	 * firmware after stopping the current route. The stream count will
	 * anyway will be decremented when the HAL layer stops the PCM stream
	 * of previous use-case. */
	else if (unlikely(atomic_read(&escore->active_streams)))
		es300_codec_stop_algo(escore);

	escore_flush_msg_list(escore);

	/* Reset the channel manager mask */
	chn_mgr_mask[escore->algo_type] = 0;

	/* Reset channel ids */
	memset(&channel_ids[escore->algo_type], 0x0,
			sizeof(channel_ids[escore->algo_type]));
	escore_flush_msg_list(escore);

	for (i = ES_PRIMARY_MUX; i <= ES_PASSIN4_MUX; i++) {
		if (cachedcmd_list[escore->algo_type][i].reg != 0) {
			rc = convert_input_mux_to_cmd(escore, i);
			if (rc)
				return rc;
		}
	}

	escore->capture_mode = 0;
	escore->output_mode = 0;
	for (i = ES_DAC0_0_MUX; i <= ES_SBUSTX5_MUX; i++) {
		if (escore->algo_type == PASSTHRU_VP
				|| escore->algo_type == DHWPT) {
			int mux = cachedcmd_list[escore->algo_type][i].reg;
			switch (mux) {
			case PASS_AO1:
				escore->output_mode |= OUTPUT_AO1;
				break;
			case PASS_MO2:
				escore->output_mode |= OUTPUT_MO2;
				break;
			case PASS_AUDOUT1:
				escore->output_mode |= OUTPUT_PO1;
				break;
			case PASS_AUDOUT2:
				escore->output_mode |= OUTPUT_PO2;
				break;
			case VP_CSOUT1:
				escore->capture_mode |= LEFT_CAPTURE;
				break;
			case VP_FEOUT1:
				escore->capture_mode |= RIGHT_CAPTURE;
				break;
			}
		}
		if (cachedcmd_list[escore->algo_type][i].reg != 0) {
			rc = convert_output_mux_to_cmd(escore, i);
			if (rc)
				return rc;
		}
	}

	/* Setup AEC if enabled */
	if (escore->vp_aec) {
		escore_queue_msg_to_list(escore, (char *)pt_vp_aec_msgblk,
				sizeof(pt_vp_aec_msgblk));
	}
	/* add umbrella base route */
	rc = add_algo_base_route(escore);
	if (!rc)
		escore_write_msg_list(escore);

	memcpy(prev_cmdlist, cachedcmd_list[escore->algo_type],
			sizeof(prev_cmdlist));

	/* Do necessary switch settings */
	if (escore->algo_type == PASSTHRU_VP || escore->algo_type == DHWPT) {
		int pri, fein;

		pri = cachedcmd_list[escore->algo_type][ES_PRIMARY_MUX].reg;
		fein = cachedcmd_list[escore->algo_type][ES_FEIN_MUX].reg;

		if (escore->output_mode & OUTPUT_PO1)
			escore_set_switch(SWOUT0_O1);
		if (escore->output_mode & OUTPUT_PO2)
			escore_set_switch(SWOUT1_O1);

		if ((escore->output_mode & OUTPUT_PT_COPY) == OUTPUT_PT_COPY) {
			/* PT Copy use case which uses both A01 and MO2 */
			escore_set_switch(SWIN2_I1);
			escore_set_switch(SWOUT3_O1);
			escore_set_switch(SWOUT2_O1);
		} else if ((escore->output_mode & OUTPUT_MIXED_MONO) ==
				OUTPUT_MIXED_MONO) {
			/* Mixed mono use case - which uses only AO1 */
			escore_set_switch(SWIN2_I0);
			escore_set_switch(SWOUT3_O1);
		}

		/* Decide the capture use-case out of three:
		 * 1. mono capture
		 * 2. stereo capture
		 * 3. mono to stereo capture
		 */
		switch (escore->capture_mode) {
		case MONO_CAPTURE:
			/* No switch settings required  - dead code */
			break;
		case STEREO_CAPTURE:
			if (pri && fein) {
				/* Stereo capture */
				escore_set_switch(SWIN1_I0);
			} else if (pri) {
				/* mono to stereo capture */
				escore_set_switch(SWIN1_I1);
			}
			break;
		}
	}

	if (escore->algo_preset_one != 0) {
		usleep_range(5000, 5005);
		pr_debug("%s:add algo preset one: %d", __func__,
				escore->algo_preset_one);
		rc = escore_write(NULL, ES_PRESET, escore->algo_preset_one);
		if (rc)
			dev_err(escore_priv.dev, "%s(): Set Algo Preset one failed:%d\n",
				__func__, rc);
		escore->algo_preset_one = 0;
	}

	if (escore->algo_preset_two != 0) {
		usleep_range(5000, 5005);
		pr_debug("%s:add algo preset two: %d", __func__,
				escore->algo_preset_two);
		rc = escore_write(NULL, ES_PRESET, escore->algo_preset_two);
		if (rc)
			dev_err(escore_priv.dev, "%s(): Set Algo Preset two failed:%d\n",
				__func__, rc);
		escore->algo_preset_two = 0;
	}

	if (cachedcmd_list[escore->algo_type][ES_ALGO_SAMPLE_RATE].reg)
		rc = es_set_algo_rate(escore, escore->algo_type);

	return rc;
}

#if defined(CONFIG_SND_SOC_ES_SLIM)
static int es300_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	u32  j = 0;
	int  ret = 0;
	int dai_id = -1;

	pr_debug("%s: %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:

		for (j = 0; j < escore->dai_nr; j++) {
			if ((escore->dai[j].id == ES_SLIM_1_PB) ||
					(escore->dai[j].id == ES_SLIM_2_PB) ||
					(escore->dai[j].id == ES_SLIM_3_PB))
				continue;

			if (!strncmp(w->sname,
				escore->dai[j].capture.stream_name, 13)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				++escore->slim_dai_data[dai_id].ch_act;
				break;
			}
		}

		if (escore->slim_dai_data[dai_id].ch_act ==
				escore->slim_dai_data[dai_id].ch_tot) {

			ret = es_set_final_route(escore);
			if (!ret) {
				ret = escore_cfg_slim_tx(escore->gen0_client,
					escore->slim_dai_data[dai_id].ch_num,
					escore->slim_dai_data[dai_id].ch_tot,
					escore->slim_dai_data[dai_id].rate);
				atomic_inc(&escore->active_streams);
			}

		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < escore->dai_nr; j++) {
			if ((escore->dai[j].id == ES_SLIM_1_PB) ||
					(escore->dai[j].id == ES_SLIM_2_PB) ||
					(escore->dai[j].id == ES_SLIM_3_PB))
				continue;
			if (!strncmp(w->sname,
				escore->dai[j].capture.stream_name, 13)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				--escore->slim_dai_data[dai_id].ch_act;
				break;
			}
		}

		if (dai_id >= 0 && !escore->slim_dai_data[dai_id].ch_act) {
			ret = escore_close_slim_tx(escore->gen0_client,
					escore->slim_dai_data[dai_id].ch_num,
					escore->slim_dai_data[dai_id].ch_tot);

			if (atomic_read(&escore->active_streams))
				atomic_dec(&escore->active_streams);

			memset(escore->slim_dai_data[dai_id].ch_num, 0,
			(sizeof(u32)*escore->slim_dai_data[dai_id].ch_tot));

			escore->slim_dai_data[dai_id].ch_tot = 0;

			if (codec && codec->dev && codec->dev->parent) {
				pm_runtime_mark_last_busy(codec->dev->parent);
				pm_runtime_put(codec->dev->parent);
			}
			if (!ret)
				es_clear_route(escore);

		}
		if (atomic_read(&escore->active_streams) == 0)
			ret = es300_codec_stop_algo(escore);
	}
	return ret;
}

static int es300_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	u32  j = 0;
	int  ret = 0;
	int dai_id = -1;

	pr_debug(" ==== AIF RX ==== %s: %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (j = 0; j < escore->dai_nr; j++) {
			if ((escore->dai[j].id == ES_SLIM_1_CAP) ||
					(escore->dai[j].id == ES_SLIM_2_CAP) ||
					(escore->dai[j].id == ES_SLIM_3_CAP))
				continue;

			if (!strncmp(w->sname,
				escore->dai[j].playback.stream_name, 13)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				++escore->slim_dai_data[dai_id].ch_act;
				break;
			}
		}

		if (escore->slim_dai_data[dai_id].ch_act ==
				escore->slim_dai_data[dai_id].ch_tot) {

			ret = es_set_final_route(escore);
			if (!ret) {
				ret = escore_cfg_slim_rx(escore->gen0_client,
					escore->slim_dai_data[dai_id].ch_num,
					escore->slim_dai_data[dai_id].ch_tot,
					escore->slim_dai_data[dai_id].rate);
				atomic_inc(&escore->active_streams);
			}
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < escore->dai_nr; j++) {
			if ((escore->dai[j].id == ES_SLIM_1_CAP) ||
					(escore->dai[j].id == ES_SLIM_2_CAP) ||
					(escore->dai[j].id == ES_SLIM_3_CAP))
				continue;
			if (!strncmp(w->sname,
				escore->dai[j].playback.stream_name, 13)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				--escore->slim_dai_data[dai_id].ch_act;
				break;
			}
		}
		if (dai_id >= 0 && !escore->slim_dai_data[dai_id].ch_act) {
			ret = escore_close_slim_rx(escore->gen0_client,
					escore->slim_dai_data[dai_id].ch_num,
					escore->slim_dai_data[dai_id].ch_tot);

			if (atomic_read(&escore->active_streams))
				atomic_dec(&escore->active_streams);

			memset(escore->slim_dai_data[dai_id].ch_num, 0,
			(sizeof(u32)*escore->slim_dai_data[dai_id].ch_tot));

			escore->slim_dai_data[dai_id].ch_tot = 0;

			if (codec && codec->dev && codec->dev->parent) {
				pm_runtime_mark_last_busy(codec->dev->parent);
				pm_runtime_put(codec->dev->parent);
			}
			if (!ret)
				es_clear_route(escore);
		}
		if (atomic_read(&escore->active_streams) == 0)
			ret = es300_codec_stop_algo(escore);
	}
	return ret;
}
static int es300_codec_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}
static int es300_codec_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}
#else
static int es300_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}
static int es300_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}
static int es300_codec_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int j = 0;
	int dai_id = 0;

	pr_debug("%s(): event:%d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (j = 0; j < escore->dai_nr; j++) {
			if (!strncmp(w->sname,
				escore->dai[j].playback.stream_name, 14)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				++escore->i2s_dai_data[dai_id].rx_ch_act;
				break;
			}
		}

		if (escore->i2s_dai_data[dai_id].rx_ch_act ==
				escore->i2s_dai_data[dai_id].rx_ch_tot) {
			ret = es_set_final_route(escore);
			msleep(20);
			atomic_inc(&escore->active_streams);
		}
		pr_debug("%s: RX PMU Active channels %d Active streams %d\n", __func__,
			escore->i2s_dai_data[dai_id].rx_ch_act, escore->active_streams);
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < escore->dai_nr; j++) {
			if (!strncmp(w->sname,
				escore->dai[j].playback.stream_name, 14)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				--escore->i2s_dai_data[dai_id].rx_ch_act;
				break;
			}
		}

		if (!escore->i2s_dai_data[dai_id].rx_ch_act) {
			es_clear_route(escore);

			if (atomic_read(&escore->active_streams) &&
				atomic_dec_and_test(&escore->active_streams))
				ret = es300_codec_stop_algo(escore);
		}
		pr_debug("%s: RX PMD Active channels %d Active streams %d\n", __func__,
			escore->i2s_dai_data[dai_id].rx_ch_act, escore->active_streams);
		break;
	}
	return ret;
}
static int es300_codec_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int j = 0;
	int dai_id = 0;

	pr_debug("%s(): event:%d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (j = 0; j < escore->dai_nr; j++) {
			if (!strncmp(w->sname,
				escore->dai[j].capture.stream_name, 14)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				++escore->i2s_dai_data[dai_id].tx_ch_act;
				break;
			}
		}

		if (escore->i2s_dai_data[dai_id].tx_ch_act ==
				escore->i2s_dai_data[dai_id].tx_ch_tot) {

			ret = es_set_final_route(escore);
			atomic_inc(&escore->active_streams);
		}
		pr_debug("%s: TX PMU Active channels %d Active streams %d\n", __func__,
			escore->i2s_dai_data[dai_id].tx_ch_act, escore->active_streams);
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < escore->dai_nr; j++) {
			if (!strncmp(w->sname,
				escore->dai[j].capture.stream_name, 14)) {
				dai_id = DAI_INDEX(escore->dai[j].id);
				--escore->i2s_dai_data[dai_id].tx_ch_act;
				break;
			}
		}

		if (!escore->i2s_dai_data[dai_id].tx_ch_act) {
			es_clear_route(escore);

			if (atomic_read(&escore->active_streams) &&
				atomic_dec_and_test(&escore->active_streams))
				ret = es300_codec_stop_algo(escore);
		}
		pr_debug("%s: TX PMD Active channels %d Active streams %d\n", __func__,
			escore->i2s_dai_data[dai_id].tx_ch_act, escore->active_streams);
		break;
	}
	return ret;
}
#endif

static int get_noise_suppression(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	u16 value;

	value = escore_read(codec, reg);

	ucontrol->value.enumerated.item[0] = value;

	pr_debug("%s: Noise Suppression value %u\n", __func__, value);

	return 0;
}
static int put_noise_suppression(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;

	value = ucontrol->value.enumerated.item[0];

	if (value > 1) {
		pr_err("%s(): Invalid value %d for the control\n", __func__,
				value);
		return -EINVAL;
	}
	pr_debug("%s: Noise Suppression value %u\n", __func__, value);
	value = value << 8;

	return escore_write(codec, reg, value);
}

static int put_aec_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value > ES_MAX_AEC_MODE) {
		pr_err("%s(): Invalid value %d for the control\n", __func__,
				value);
		return -EINVAL;
	}
	pr_debug("%s: AEC mode value %u\n", __func__, value);

	return escore_write(codec, reg, value);
}

static int get_aec_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	u16 value;

	value = escore_read(codec, reg);

	if (value > ES_MAX_AEC_MODE)
		pr_err("%s(): Invalid value %d\n", __func__, value);

	ucontrol->value.enumerated.item[0] = value;

	pr_debug("%s: AEC mode value %u\n", __func__, value);

	return 0;
}

static int put_az_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value > ES_MAX_AZ_MODE) {
		pr_err("%s(): Invalid value %d for the control\n", __func__,
				value);
		return -EINVAL;
	}

	pr_debug("%s: Audio Zoom mode value %u\n", __func__, value);

	return escore_write(codec, reg, value);
}

static int get_az_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	u16 value;

	value = escore_read(codec, reg);

	if (value > ES_MAX_AZ_MODE)
		pr_err("%s(): Invalid value %d\n", __func__, value);

	ucontrol->value.enumerated.item[0] = value;

	pr_debug("%s: Audio Zoom mode value %u\n", __func__, value);

	return 0;
}

static int put_pcm_port_sel(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];

	escore->pcm_port = value;
	cachedcmd_list[escore->algo_type][reg].reg = value;

	return 0;
}

static int get_cmd_compl_mode_sel(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = escore->cmd_compl_mode;

	return 0;
}

static int put_cmd_compl_mode_sel(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];
	u32 cmd = ES_SYNC_CMD << 16;
	u32 resp;
	int rc;

	if (value > ES_CMD_COMP_INTR) {
		pr_err("%s(): Invalid command completion mode :%d\n",
				__func__, value);
		return -EINVAL;
	}

	if (value == ES_CMD_COMP_INTR) {
		if (escore->pdata->gpioa_gpio == -1) {
			pr_err("%s(): Interrupt GPIO not configured.\n",
					__func__);
			return -EINVAL;
		}
		cmd |= escore->pdata->gpio_a_irq_type;
	}

	/* Set the suppress response bit */
	cmd |= BIT(ES_SR_BIT);
	rc = escore_cmd(escore, cmd, &resp);
	if (rc < 0) {
		pr_err("%s(): Error %d to set command mode", __func__, rc);
		return rc;
	}

	escore->cmd_compl_mode = value;
	cachedcmd_list[escore->algo_type][reg].reg = value;

	return 0;
}

static int es300_get_algo_state(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;

	ucontrol->value.enumerated.item[0] = cachedcmd_list[0][reg].reg;

	return 0;
}

static int es300_put_algo_state(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0, algo_type = NONE;
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];

	pr_debug("%s(): %s algo :%s\n", __func__, (value) ? "Enabling" :
			"Disabling", kcontrol->id.name);

	/* Use 0th array to store the algo status */
	cachedcmd_list[0][reg].reg = value;
	switch (reg) {
	case ES_ALGORITHM_VP:
		algo_type = VP;
		break;
	case ES_ALGORITHM_MM:
		algo_type = MM;
		break;
	case ES_ALGORITHM_PT:
		algo_type = PASSTHRU;
		break;
	case ES_ALGORITHM_PT_VP:
		algo_type = PASSTHRU_VP;
		break;
	case ES_ALGORITHM_AZ:
		algo_type = AUDIOZOOM;
		break;
	case ES_ALGORITHM_DHWPT:
		algo_type = DHWPT;
		break;
	default:
		pr_err("%s(): Algo type not implemented: %s\n", __func__, kcontrol->id.name);
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		goto out;

	if (value)
		escore->algo_type = algo_type;
	else {
		/* During the disable sequence the algo_type will not be
		 * updated. Use the algo_type_off as the array index to reset
		 * the kcontrols and update cachedcmd_list. Consider the
		 * following scenario:
		 *
		 * 1. Simultaneous playback and capture is running (PT_VP)
		 * 2. Playback is stopped
		 * 3. Disable sequence of Playback clears cachedcmd_list for PB
		 * 4. VoIP incoming call executes enable sequence for RX. Algo
		 * type is VP in this case.
		 * 5. Capture is stopped and the disable sequence of capture
		 * resets the kcontrols. But as the algo_type is changed to VP,
		 * wrong index of cachedcmd_list will be updated.
		 *
		 * To avoid this, while resetting the MUXes, use the
		 * algo_type_off variable as index to cachedcmd_list for correct
		 * algo type.
		 */
		escore->algo_type_off = algo_type;
	}
out:
	return ret;
}
static int es300_put_algo_rate(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	int ret = 0;
	unsigned int value = ucontrol->value.enumerated.item[0];
	cachedcmd_list[escore->algo_type][reg].reg = value;
	return ret;
}
static int get_control_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;

	value = cachedcmd_list[escore->algo_type][reg].reg;
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int put_input_route_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	int mux = ucontrol->value.enumerated.item[0];
	int rc = 0;
	u8 algo_type;

	if (mux >= ARRAY_SIZE(proc_block_input_texts) || mux < 0) {
		pr_err("%s(): Invalid input mux:%d Max valid value:%lu\n",
			__func__, mux, ARRAY_SIZE(proc_block_input_texts));
		return -EINVAL;
	}

	algo_type = (mux) ? escore->algo_type : escore->algo_type_off;

	cachedcmd_list[algo_type][reg].reg = mux;

	if (!mux && atomic_read(&escore->active_streams))
		goto exit;

#if (defined(CONFIG_ARCH_OMAP) || defined(CONFIG_ARCH_EXYNOS) || \
	defined(CONFIG_X86_32) || defined(CONFIG_ARCH_TEGRA))
	rc = snd_soc_dapm_mux_update_power(widget, kcontrol, mux, e);
#else
	rc = snd_soc_dapm_mux_update_power(widget, kcontrol, 1, mux, e);
#endif

exit:
	pr_debug("put input control %s (%d) val %s (%d)\n", kcontrol->id.name, reg, proc_block_input_texts[mux], mux);

	return rc;
}

static int get_input_route_value(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;

	value = cachedcmd_list[escore->algo_type][reg].reg;

	/* TBD: translation */
	/* value = escore_read(NULL, reg); */
	ucontrol->value.enumerated.item[0] = value;
	pr_debug("get input control %s (%d) val %s (%d)\n", kcontrol->id.name, reg, proc_block_input_texts[value], value);

	return 0;
}

static int put_output_route_value(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	int rc = 0;
	int mux = ucontrol->value.enumerated.item[0];
	int prev_mux;
	u8 algo_type;

	if (mux >= ARRAY_SIZE(proc_block_output_texts) || mux < 0) {
		pr_err("%s(): Invalid output mux:%d Max valid value:%lu\n",
			__func__, mux, ARRAY_SIZE(proc_block_output_texts));
		return -EINVAL;
	}
	/* VP CSOUT signals Tx init and VP FEOUT signals Rx init */
	if (escore->algo_type == VP &&
		strncmp(proc_block_output_texts[mux], "VP CSOUT", 8) == 0) {
		es_vp_tx = ES_VP_TX_INIT;
	} else if (escore->algo_type == VP &&
		strncmp(proc_block_output_texts[mux], "VP FEOUT1", 9) == 0) {
		es_vp_rx = ES_VP_RX_INIT;
	} else if (escore->algo_type == AUDIOZOOM &&
			strncmp(proc_block_output_texts[mux],
				"AudioZoom CSOUT", 15) == 0) {
		es_az_tx = ES_AZ_TX_INIT;
	} else if (escore->algo_type == AUDIOZOOM &&
			strncmp(proc_block_output_texts[mux],
				"AudioZoom AOUT1", 15) == 0) {
		es_az_rx = ES_AZ_RX_INIT;
	}

	algo_type = (mux) ? escore->algo_type : escore->algo_type_off;

	prev_mux = cachedcmd_list[algo_type][reg].reg;
	cachedcmd_list[algo_type][reg].reg = mux;

	/* Request of clearing the VP output */
	if (!mux && prev_mux && algo_type == VP) {
		if (!strncmp(proc_block_output_texts[prev_mux],
					"VP CSOUT", sizeof("VP CSOUT") - 1))
			es_vp_tx = ES_VP_NONE;
		else if (!strncmp(proc_block_output_texts[prev_mux],
					"VP FEOUT1", sizeof("VP FEOUT1") - 1))
			es_vp_rx = ES_VP_NONE;
	}

	if (!mux && atomic_read(&escore->active_streams))
		goto exit;

#if (defined(CONFIG_ARCH_OMAP) || defined(CONFIG_ARCH_EXYNOS) || \
	defined(CONFIG_X86_32) || defined(CONFIG_ARCH_TEGRA))
	rc = snd_soc_dapm_mux_update_power(widget, kcontrol, mux, e);
#else
	rc = snd_soc_dapm_mux_update_power(widget, kcontrol, 1, mux, e);
#endif

exit:
	pr_debug("put output control %s (%d) val %s (%d)\n", kcontrol->id.name, reg, proc_block_output_texts[mux], mux);
	return rc;
}

static int get_output_route_value(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;

	value = cachedcmd_list[escore->algo_type][reg].reg;

	/* TBD: translation */
	/* value = escore_read(NULL, reg); */
	ucontrol->value.enumerated.item[0] = value;
	pr_debug("get output control %s (%d)  val %s (%d)\n", kcontrol->id.name, reg, proc_block_output_texts[value], value);

	return 0;
}

static int get_write_route_cmds(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int put_write_route_cmds(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.enumerated.item[0])
		escore_write_msg_list(NULL);

	return 0;
}

static int put_asr_sel(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	escore->vp_asr = ucontrol->value.enumerated.item[0];

	return 0;
}

static int get_asr_sel(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = escore->vp_asr;
	return 0;
}

static int put_vp_aec(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	escore->vp_aec = ucontrol->value.enumerated.item[0];
	return 0;
}

static int get_vp_aec(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = escore->vp_aec;
	return 0;
}


static int get_reset(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int put_reset(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	es_d300_reset_cmdcache();
	es_vp_tx = ES_VP_NONE;
	es_vp_rx = ES_VP_NONE;
	es_az_tx = ES_AZ_NONE;
	es_az_rx = ES_AZ_NONE;
	return 0;
}

static int get_flush_route_cmds(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int put_flush_route_cmds(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.enumerated.item[0])
		escore_flush_msg_list(NULL);

	return 0;
}

static int get_pcm_port_param(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];
	struct escore_api_access *api_access;
	int msg_len;
	u32 msg[2] = { 0 };
	u32 resp;
	u8 pcm_port_id;
	int rc;

	if (!escore->pcm_port) {
		pr_err("%s(): PCM Port not selected\n", __func__);
		return -EINVAL;
	}

	if (reg > escore->api_addr_max) {
		pr_err("%s(): invalid address = 0x%04x\n", __func__, reg);
		return -EINVAL;
	}

	api_access = &escore->api_access[reg];
	pcm_port_id = pcm_port[escore->pcm_port];

	/* Update the Port info in read command */
	api_access->read_msg[0] |= ES_API_WORD(ES_GET_DEV_PARAM,
			pcm_port_id << 8);

	rc = escore_prepare_msg(escore, reg, value, (char *)msg, &msg_len,
			ES_MSG_READ);
	if (rc) {
		pr_err("%s(): Preparing write message failed\n", __func__);
		return rc;
	}
	/* Clear the Port info in read command */
	api_access->read_msg[0] &= ES_API_WORD(ES_GET_DEV_PARAM, 0x00ff);

	rc = escore_cmd(escore, msg[0], &resp);
	if (rc < 0) {
		pr_err("%s(): escore_cmd()", __func__);
		return rc;
	}
	msg[0] = escore->bus.last_response;

	value = msg[0] & 0xffff;
	ucontrol->value.enumerated.item[0] = value;
	pr_debug("get input reg %d value 0x%08x", reg, value);

	return 0;
}

static int put_pcm_port_param(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];
	struct escore_api_access *api_access;
	u8 pcm_port_id;
	int rc;

	if (!escore->pcm_port) {
		pr_err("%s(): PCM Port not selected\n", __func__);
		return -EINVAL;
	}

	if (reg > escore->api_addr_max) {
		pr_err("%s(): invalid address = 0x%04x\n", __func__, reg);
		return -EINVAL;
	}

	api_access = &escore->api_access[reg];
	pcm_port_id = pcm_port[escore->pcm_port];

	/* Update the Port info in write command */
	api_access->write_msg[0] |=  ES_API_WORD(ES_SET_DEV_PARAM_ID,
			(pcm_port_id << 8));

	value = ucontrol->value.enumerated.item[0];
	rc = escore_write(codec, reg, value);
	if (rc) {
		pr_err("%s(): Preparing write message failed\n", __func__);
		return rc;
	}

	/* Clear the Port info in write command */
	api_access->write_msg[0] &= ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x00ff);

	return 0;
}

static int get_mic_config(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	u16 value;

	value = escore_read(codec, reg);

	ucontrol->value.enumerated.item[0] = value;

	pr_debug("%s: mic configuration value %u\n", __func__,
			value);

	return 0;
}
static int put_mic_config(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value = ucontrol->value.enumerated.item[0];

	pr_debug("%s: mic configuration value %u\n", __func__,
		value);

	return escore_write(codec, reg, value);
}

static int get_lp_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = loopback_mode;
	return 0;
}

static int put_lp_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	loopback_mode = ucontrol->value.enumerated.item[0];
	return 0;
}

static int get_lp_route(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int put_lp_route(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);

	unsigned int value;
	int ret = 0;
	value = ucontrol->value.enumerated.item[0];
	if (!loopback_mode) {
		pr_debug("%s(): loopback mode is not enabled\n", __func__);
		return 0;
	}
	if (!value) {
		pr_debug("%s(): discard cached list\n", __func__);
		ret = es300_codec_stop_algo(escore);
	} else {
		pr_debug("%s(): commit cached list\n", __func__);
		ret = es_set_final_route(escore);
	}
	return ret;
}

static int put_preset_value_one(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int rc = 0;
	unsigned int value;

	value = ucontrol->value.integer.value[0];

	escore->algo_preset_one = value;

	/* Ignore Preset ID 0 and don't send command to device,5
	 * but reset algo_preset value so that next preset value
	 * can be set properly. */
	if (!escore->algo_preset_one) {
		dev_dbg(escore_priv.dev,
			"%s(): Algo Preset %d is not supported, Skipped\n",
			__func__, value);
		return rc;
	}

	if (atomic_read(&escore->active_streams)) {
		/* Set the preset immediately */
		usleep_range(5000, 5005);
		pr_debug("%s:add algo preset one: %d", __func__,
				escore->algo_preset_one);
		rc = escore_write(NULL, ES_PRESET, escore->algo_preset_one);
		if (rc)
			pr_err("%s(): Set Algo Preset one failed:%d\n",
				__func__, rc);
	}

	return rc;
}

static int get_preset_value_one(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.integer.value[0] = escore->algo_preset_one;

	return 0;
}

static int put_preset_value_two(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int rc = 0;
	unsigned int value;

	value = ucontrol->value.integer.value[0];

	escore->algo_preset_two = value;

	/* Ignore Preset ID 0 and don't send command to device,5
	 * but reset algo_preset value so that next preset value
	 * can be set properly. */
	if (!escore->algo_preset_two) {
		dev_dbg(escore_priv.dev,
			"%s(): Algo Preset %d is not supported, Skipped\n",
			__func__, value);
		return rc;
	}

	if (atomic_read(&escore->active_streams)) {
		/* Set the preset immediately */
		usleep_range(5000, 5005);
		pr_debug("%s:add algo preset two: %d", __func__,
				escore->algo_preset_two);
		rc = escore_write(NULL, ES_PRESET, escore->algo_preset_two);
		if (rc)
			pr_err("%s(): Set Algo Preset two failed:%d\n",
				__func__, rc);
	}

	return rc;
}

static int get_preset_value_two(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = escore_priv.codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.integer.value[0] = escore->algo_preset_two;

	return 0;
}

static const char * const algorithm_texts[] = {
	"Off", "On"
};

static const char * const cmd_compl_mode_sel_texts[] = {
	"Polling", "Interrupt",
};

static const struct soc_enum cmd_compl_mode_sel_enum =
	SOC_ENUM_SINGLE(ES_CMD_COMPL_MODE, 0,
			ARRAY_SIZE(cmd_compl_mode_sel_texts),
			cmd_compl_mode_sel_texts);

static const char * const pcm_port_sel_texts[] = {
	"None", "Port A", "Port B", "Port C",
};

static const struct soc_enum pcm_port_sel_enum =
	SOC_ENUM_SINGLE(ES_PCM_PORT, 0,
			ARRAY_SIZE(pcm_port_sel_texts),
			pcm_port_sel_texts);

static const struct soc_enum vp_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_VP, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);
static const struct soc_enum mm_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_MM, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);
static const struct soc_enum pt_vp_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_PT_VP, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);
static const struct soc_enum pass_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_PT, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);
static const struct soc_enum az_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_AZ, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);
static const struct soc_enum dhwpt_algorithm_enum =
	SOC_ENUM_SINGLE(ES_ALGORITHM_DHWPT, 0,
			ARRAY_SIZE(algorithm_texts),
			algorithm_texts);

static const char * const es755_algo_rates_text[] = {
	"None", "SR_8k", "SR_16k", "SR_24k", "SR_48k", "SR_96k", "SR_192k",
};
/*replaced for ES_ALGORITHM_RATE*/
static const struct soc_enum algorithm_rate_enum =
	SOC_ENUM_SINGLE(ES_ALGO_SAMPLE_RATE, 0,
			ARRAY_SIZE(es755_algo_rates_text),
			es755_algo_rates_text);

static const char * const noise_suppression_texts[] = {
	"Off", "On"
};

static const struct soc_enum noise_suppression_enum =
	SOC_ENUM_SINGLE(ES_SET_NS, 0,
			ARRAY_SIZE(noise_suppression_texts),
			noise_suppression_texts);

static const char * const mic_config_texts[] = {
	"CT Multi-mic", "FT Multi-mic", "DV 1-mic", "EXT 1-mic", "BT 1-mic",
	"CT ASR Multi-mic", "FT ASR Multi-mic", "EXT ASR 1-mic",
	"FT ASR 1-mic",	"LN Multi-mic", "LN ASR Multi-mic", "FO Multi-mic",
	"FO ASR Multi-mic"
};

static const struct soc_enum mic_config_enum =
	SOC_ENUM_SINGLE(ES_MIC_CONFIG, 0,
		ARRAY_SIZE(mic_config_texts),
		mic_config_texts);

static const char * const aec_mode_texts[] = {
	"off", "on", "NpNs+Envelope AEC", "Envelope AEC", "Volterra",
	"On (Half-duplex)", "On (Multi-mic enhanced)"
};
static const struct soc_enum aec_mode_enum =
	SOC_ENUM_SINGLE(ES_AEC_MODE, 0,
		ARRAY_SIZE(aec_mode_texts),
		aec_mode_texts);


static const char * const es755_audio_zoom_texts[] = {
	"Narrator", "Scene", "Narration"
};

static const struct soc_enum es755_az_mode_enum =
	SOC_ENUM_SINGLE(ES_AZ_MODE, 0, ARRAY_SIZE(es755_audio_zoom_texts),
			es755_audio_zoom_texts);

static const char * const lp_mode_texts[] = {
	"Off", "On"
};
static const struct soc_enum lp_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(lp_mode_texts),
			lp_mode_texts);

static const char * const lp_commit_texts[] = {
	"Discard", "Commit"
};
static const struct soc_enum lp_commit_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(lp_commit_texts),
			lp_commit_texts);

static const char * const vp_aec_texts[] = {
	"Off", "On"
};
static const struct soc_enum vp_aec_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(vp_aec_texts),
			vp_aec_texts);


static const struct snd_kcontrol_new es_d300_snd_controls[] = {
	SOC_ENUM_EXT("VP Algorithm", vp_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),
	SOC_ENUM_EXT("MM Algorithm", mm_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),
	SOC_ENUM_EXT("PT_VP Algorithm", pt_vp_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),
	SOC_ENUM_EXT("PT Algorithm", pass_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),
	SOC_ENUM_EXT("AZ Algorithm", az_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),
	SOC_ENUM_EXT("DHWPT Algorithm", dhwpt_algorithm_enum,
			 es300_get_algo_state, es300_put_algo_state),

	SOC_ENUM_EXT("Algorithm Rate", algorithm_rate_enum,
			 get_control_enum, es300_put_algo_rate),
	SOC_SINGLE_EXT("ASR", SND_SOC_NOPM, 0, 1, 0,
			 get_asr_sel, put_asr_sel),
	SOC_SINGLE_EXT("Write Route Cmds", SND_SOC_NOPM, 0, 1, 0,
		       get_write_route_cmds, put_write_route_cmds),
	SOC_SINGLE_EXT("Flush Route Cmds", SND_SOC_NOPM, 0, 1, 0,
		       get_flush_route_cmds, put_flush_route_cmds),
	SOC_ENUM_EXT("Noise Suppression", noise_suppression_enum,
			get_noise_suppression, put_noise_suppression),
	SOC_ENUM_EXT("Mic Config", mic_config_enum,
			get_mic_config, put_mic_config),
	SOC_ENUM_EXT("Audio Zoom", es755_az_mode_enum,
			get_az_mode, put_az_mode),
	SOC_ENUM_EXT("PCM Port Selection", pcm_port_sel_enum,
			get_control_enum, put_pcm_port_sel),
	SOC_ENUM_EXT("Command Mode Selection", cmd_compl_mode_sel_enum,
			get_cmd_compl_mode_sel, put_cmd_compl_mode_sel),
	SOC_ENUM_EXT("AEC mode", aec_mode_enum,
			get_aec_mode, put_aec_mode),
	SOC_SINGLE_EXT("PCM Port Word Length", ES_PORT_WORD_LEN, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port TDM Slots Per Frame",
			ES_PORT_TDM_SLOTS_PER_FRAME, 0,	65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port TX Delay From FS",
			ES_PORT_TX_DELAY_FROM_FS, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Rx Delay From FS",
			ES_PORT_RX_DELAY_FROM_FS, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Latch Edge",
			ES_PORT_LATCH_EDGE, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Endian", ES_PORT_ENDIAN, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Tristate", ES_PORT_TRISTATE, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Audio Mode", ES_PORT_AUDIO_MODE, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port TDM Enabled", ES_PORT_TDM_ENABLED, 0,
			65535, 0, get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Clock Control", ES_PORT_CLOCK_CONTROL, 0,
			65535, 0, get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port Data Justification",
			ES_PORT_DATA_JUSTIFICATION, 0, 65535, 0,
			get_pcm_port_param, put_pcm_port_param),
	SOC_SINGLE_EXT("PCM Port FS Duration", ES_PORT_FS_DURATION, 0,
			65535, 0, get_pcm_port_param, put_pcm_port_param),
	SOC_ENUM_EXT("Loopback mode", lp_mode_enum,
			 get_lp_mode, put_lp_mode),
	SOC_ENUM_EXT("Loopback routes commit", lp_commit_enum,
			 get_lp_route, put_lp_route),
	SOC_SINGLE_EXT("Reset", SND_SOC_NOPM, 0, 1, 0,
			 get_reset, put_reset),
	SOC_SINGLE_EXT("Algo Preset 1",
			SND_SOC_NOPM, 0, 65535, 0, get_preset_value_one,
			put_preset_value_one),
	SOC_SINGLE_EXT("Algo Preset 2",
			SND_SOC_NOPM, 0, 65535, 0, get_preset_value_two,
			put_preset_value_two),
	SOC_ENUM_EXT("VP AEC", vp_aec_enum,
			 get_vp_aec, put_vp_aec),
};

static const struct soc_enum vp_pri_enum =
	SOC_ENUM_SINGLE(ES_PRIMARY_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_pri_control =
	SOC_DAPM_ENUM_EXT("VP Primary MUX Mux", vp_pri_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_sec_enum =
	SOC_ENUM_SINGLE(ES_SECONDARY_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_sec_control =
	SOC_DAPM_ENUM_EXT("VP Secondary MUX Mux", vp_sec_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_ter_enum =
	SOC_ENUM_SINGLE(ES_TERTIARY_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_ter_control =
	SOC_DAPM_ENUM_EXT("VP Teritary MUX Mux", vp_ter_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_aecref_enum =
	SOC_ENUM_SINGLE(ES_AECREF1_MUX, 0,
		ARRAY_SIZE(proc_block_input_texts),
		proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_aecref_control =
	SOC_DAPM_ENUM_EXT("VP AECREF MUX Mux", vp_aecref_enum,
		get_input_route_value, put_input_route_value);

static const struct soc_enum vp_fein_enum =
	SOC_ENUM_SINGLE(ES_FEIN_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_fein_control =
	SOC_DAPM_ENUM_EXT("VP FEIN MUX Mux", vp_fein_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_fein2_enum =
	SOC_ENUM_SINGLE(ES_FEIN2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_fein2_control =
	SOC_DAPM_ENUM_EXT("VP FEIN2 MUX Mux", vp_fein2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_uitone1_enum =
	SOC_ENUM_SINGLE(ES_UITONE1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_uitone1_control =
	SOC_DAPM_ENUM_EXT("VP UITONE1 MUX Mux", vp_uitone1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_uitone2_enum =
	SOC_ENUM_SINGLE(ES_UITONE2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_uitone2_control =
	SOC_DAPM_ENUM_EXT("VP UITONE2 MUX Mux", vp_uitone2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_uitone3_enum =
	SOC_ENUM_SINGLE(ES_UITONE3_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_uitone3_control =
	SOC_DAPM_ENUM_EXT("VP UITONE3 MUX Mux", vp_uitone3_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum vp_uitone4_enum =
	SOC_ENUM_SINGLE(ES_UITONE4_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_vp_uitone4_control =
	SOC_DAPM_ENUM_EXT("VP UITONE4 MUX Mux", vp_uitone4_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_passin1_enum =
	SOC_ENUM_SINGLE(ES_MM_PASSIN1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_passin1_control =
	SOC_DAPM_ENUM_EXT("MM PASSIN1 MUX Mux", mm_passin1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_passin2_enum =
	SOC_ENUM_SINGLE(ES_MM_PASSIN2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_passin2_control =
	SOC_DAPM_ENUM_EXT("MM PASSIN2 MUX Mux", mm_passin2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_uitone1_enum =
	SOC_ENUM_SINGLE(ES_MMUITONE1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_uitone1_control =
	SOC_DAPM_ENUM_EXT("MM UITONE1 MUX Mux", mm_uitone1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_uitone2_enum =
	SOC_ENUM_SINGLE(ES_MMUITONE2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_uitone2_control =
	SOC_DAPM_ENUM_EXT("MM UITONE2 MUX Mux", mm_uitone2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_audin1_enum =
	SOC_ENUM_SINGLE(ES_AUDIN1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_audin1_control =
	SOC_DAPM_ENUM_EXT("MM AUDIN1 MUX Mux", mm_audin1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum mm_audin2_enum =
	SOC_ENUM_SINGLE(ES_AUDIN2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_mm_audin2_control =
	SOC_DAPM_ENUM_EXT("MM AUDIN2 MUX Mux", mm_audin2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum pass_audin1_enum =
	SOC_ENUM_SINGLE(ES_PASSIN1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_pass_audin1_control =
	SOC_DAPM_ENUM_EXT("Pass AUDIN1 MUX Mux", pass_audin1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum pass_audin2_enum =
	SOC_ENUM_SINGLE(ES_PASSIN2_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_pass_audin2_control =
	SOC_DAPM_ENUM_EXT("Pass AUDIN2 MUX Mux", pass_audin2_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum pass_audin3_enum =
	SOC_ENUM_SINGLE(ES_PASSIN3_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_pass_audin3_control =
	SOC_DAPM_ENUM_EXT("Pass AUDIN3 MUX Mux", pass_audin3_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum pass_audin4_enum =
	SOC_ENUM_SINGLE(ES_PASSIN4_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_pass_audin4_control =
	SOC_DAPM_ENUM_EXT("Pass AUDIN4 MUX Mux", pass_audin4_enum,
			  get_input_route_value, put_input_route_value);
static const struct soc_enum az_pri_enum =
	SOC_ENUM_SINGLE(ES_AZ_PRI_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_az_pri_control =
	SOC_DAPM_ENUM_EXT("AudioZoom Primary MUX Mux", az_pri_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum az_sec_enum =
	SOC_ENUM_SINGLE(ES_AZ_SEC_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_az_sec_control =
	SOC_DAPM_ENUM_EXT("AudioZoom Secondary MUX Mux", az_sec_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum az_ter_enum =
	SOC_ENUM_SINGLE(ES_AZ_TER_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_az_ter_control =
	SOC_DAPM_ENUM_EXT("AudioZoom Tertiary MUX Mux", az_ter_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum az_ai1_enum =
	SOC_ENUM_SINGLE(ES_AZ_AI1_MUX, 0,
		     ARRAY_SIZE(proc_block_input_texts),
		     proc_block_input_texts);
static const struct snd_kcontrol_new dapm_az_ai1_control =
	SOC_DAPM_ENUM_EXT("AudioZoom AI1 MUX Mux", az_ai1_enum,
			  get_input_route_value, put_input_route_value);

static const struct soc_enum pcm0_0_enum =
	SOC_ENUM_SINGLE(ES_PCM0_0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm0_0_control =
	SOC_DAPM_ENUM_EXT("PCM0.0 MUX Mux", pcm0_0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm0_1_enum =
	SOC_ENUM_SINGLE(ES_PCM0_1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm0_1_control =
	SOC_DAPM_ENUM_EXT("PCM0.1 MUX Mux", pcm0_1_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm0_2_enum =
	SOC_ENUM_SINGLE(ES_PCM0_2_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm0_2_control =
	SOC_DAPM_ENUM_EXT("PCM0.2 MUX Mux", pcm0_2_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm0_3_enum =
	SOC_ENUM_SINGLE(ES_PCM0_3_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm0_3_control =
	SOC_DAPM_ENUM_EXT("PCM0.3 MUX Mux", pcm0_3_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm1_0_enum =
	SOC_ENUM_SINGLE(ES_PCM1_0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm1_0_control =
	SOC_DAPM_ENUM_EXT("PCM1.0 MUX Mux", pcm1_0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm1_1_enum =
	SOC_ENUM_SINGLE(ES_PCM1_1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm1_1_control =
	SOC_DAPM_ENUM_EXT("PCM1.1 MUX Mux", pcm1_1_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm1_2_enum =
	SOC_ENUM_SINGLE(ES_PCM1_2_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm1_2_control =
	SOC_DAPM_ENUM_EXT("PCM1.2 MUX Mux", pcm1_2_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm1_3_enum =
	SOC_ENUM_SINGLE(ES_PCM1_3_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm1_3_control =
	SOC_DAPM_ENUM_EXT("PCM1.3 MUX Mux", pcm1_3_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm2_0_enum =
	SOC_ENUM_SINGLE(ES_PCM2_0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm2_0_control =
	SOC_DAPM_ENUM_EXT("PCM2.0 MUX Mux", pcm2_0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm2_1_enum =
	SOC_ENUM_SINGLE(ES_PCM2_1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm2_1_control =
	SOC_DAPM_ENUM_EXT("PCM2.1 MUX Mux", pcm2_1_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm2_2_enum =
	SOC_ENUM_SINGLE(ES_PCM2_2_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm2_2_control =
	SOC_DAPM_ENUM_EXT("PCM2.2 MUX Mux", pcm2_2_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum pcm2_3_enum =
	SOC_ENUM_SINGLE(ES_PCM2_3_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_pcm2_3_control =
	SOC_DAPM_ENUM_EXT("PCM2.3 MUX Mux", pcm2_3_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx0_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx0_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX0 MUX Mux", sbustx0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx1_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx1_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX1 MUX Mux", sbustx1_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx2_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX2_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx2_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX2 MUX Mux", sbustx2_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx3_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX3_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx3_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX3 MUX Mux", sbustx3_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx4_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX4_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx4_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX4 MUX Mux", sbustx4_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum sbustx5_enum =
	SOC_ENUM_SINGLE(ES_SBUSTX5_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_sbustx5_control =
	SOC_DAPM_ENUM_EXT("SBUS.TX5 MUX Mux", sbustx5_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum dac0_0_enum =
	SOC_ENUM_SINGLE(ES_DAC0_0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_dac0_0_control =
	SOC_DAPM_ENUM_EXT("DAC0.0 MUX Mux", dac0_0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum dac0_1_enum =
	SOC_ENUM_SINGLE(ES_DAC0_1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);

static const struct snd_kcontrol_new dapm_dac0_1_control =
	SOC_DAPM_ENUM_EXT("DAC0.1 MUX Mux", dac0_1_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum dac1_0_enum =
	SOC_ENUM_SINGLE(ES_DAC1_0_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_dac1_0_control =
	SOC_DAPM_ENUM_EXT("DAC1.0 MUX Mux", dac1_0_enum,
			  get_output_route_value, put_output_route_value);

static const struct soc_enum dac1_1_enum =
	SOC_ENUM_SINGLE(ES_DAC1_1_MUX, 0,
		     ARRAY_SIZE(proc_block_output_texts),
		     proc_block_output_texts);
static const struct snd_kcontrol_new dapm_dac1_1_control =
	SOC_DAPM_ENUM_EXT("DAC1.1 MUX Mux", dac1_1_enum,
			  get_output_route_value, put_output_route_value);

static const struct snd_soc_dapm_widget es_d300_dapm_widgets[] = {

	/* AIF */
	SND_SOC_DAPM_AIF_IN_E("PCM0.0 RX", "PORTA Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.1 RX", "PORTA Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.2 RX", "PORTA Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.3 RX", "PORTA Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),


	SND_SOC_DAPM_AIF_IN_E("PCM1.0 RX", "PORTB Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.1 RX", "PORTB Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.2 RX", "PORTB Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.3 RX", "PORTB Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM2.0 RX", "PORTC Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.1 RX", "PORTC Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.2 RX", "PORTC Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.3 RX", "PORTC Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),


	SND_SOC_DAPM_AIF_OUT_E("PCM0.0 TX", "PORTA Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.1 TX", "PORTA Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.2 TX", "PORTA Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.3 TX", "PORTA Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM1.0 TX", "PORTB Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.1 TX", "PORTB Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.2 TX", "PORTB Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.3 TX", "PORTB Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM2.0 TX", "PORTC Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.1 TX", "PORTC Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.2 TX", "PORTC Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.3 TX", "PORTC Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),


	SND_SOC_DAPM_AIF_IN_E("SBUS.RX0", "SLIM_PORT-1 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SBUS.RX1", "SLIM_PORT-1 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SBUS.RX2", "SLIM_PORT-2 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SBUS.RX3", "SLIM_PORT-2 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SBUS.RX4", "SLIM_PORT-3 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SBUS.RX5", "SLIM_PORT-3 Playback", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimrx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN("SBUS.RX6", "", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SBUS.RX7", "", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SBUS.RX8", "", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SBUS.RX9", "", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX0", "SLIM_PORT-1 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX1", "SLIM_PORT-1 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX2", "SLIM_PORT-2 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX3", "SLIM_PORT-2 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX4", "SLIM_PORT-3 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SBUS.TX5", "SLIM_PORT-3 Capture", 0,
			SND_SOC_NOPM, 1, 0, es300_codec_enable_slimtx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* voice processing */
	SND_SOC_DAPM_MUX("VP Primary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_pri_control),
	SND_SOC_DAPM_MUX("VP Secondary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_sec_control),
	SND_SOC_DAPM_MUX("VP Teritary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_ter_control),
	SND_SOC_DAPM_MUX("VP AECREF MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_aecref_control),
	SND_SOC_DAPM_MUX("VP FEIN MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_fein_control),
	SND_SOC_DAPM_MUX("VP FEIN2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_fein2_control),

	SND_SOC_DAPM_MUX("VP UITONE1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_uitone1_control),
	SND_SOC_DAPM_MUX("VP UITONE2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_vp_uitone2_control),

	/* Passthrough */
	SND_SOC_DAPM_MUX("Pass AUDIN1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_pass_audin1_control),
	SND_SOC_DAPM_MUX("Pass AUDIN2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_pass_audin2_control),
	SND_SOC_DAPM_MUX("Pass AUDIN3 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_pass_audin3_control),
	SND_SOC_DAPM_MUX("Pass AUDIN4 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_pass_audin4_control),

	/* Multimedia */
	SND_SOC_DAPM_MUX("MM AUDIN1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_audin1_control),
	SND_SOC_DAPM_MUX("MM AUDIN2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_audin2_control),
	SND_SOC_DAPM_MUX("MM UITONE1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_uitone1_control),
	SND_SOC_DAPM_MUX("MM UITONE2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_uitone2_control),
	SND_SOC_DAPM_MUX("MM PASSIN1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_passin1_control),
	SND_SOC_DAPM_MUX("MM PASSIN2 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_mm_passin2_control),


	/* AudioZoom */
	SND_SOC_DAPM_MUX("AudioZoom Primary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_az_pri_control),
	SND_SOC_DAPM_MUX("AudioZoom Secondary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_az_sec_control),
	SND_SOC_DAPM_MUX("AudioZoom Teritary MUX", SND_SOC_NOPM, 0, 0,
			&dapm_az_ter_control),
	SND_SOC_DAPM_MUX("AudioZoom AI1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_az_ai1_control),

	SND_SOC_DAPM_MUX("PCM0.0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm0_0_control),
	SND_SOC_DAPM_MUX("PCM0.1 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm0_1_control),
	SND_SOC_DAPM_MUX("PCM0.2 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm0_2_control),
	SND_SOC_DAPM_MUX("PCM0.3 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm0_3_control),

	SND_SOC_DAPM_MUX("PCM1.0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm1_0_control),
	SND_SOC_DAPM_MUX("PCM1.1 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm1_1_control),
	SND_SOC_DAPM_MUX("PCM1.2 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm1_2_control),
	SND_SOC_DAPM_MUX("PCM1.3 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm1_3_control),

	SND_SOC_DAPM_MUX("PCM2.0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm2_0_control),
	SND_SOC_DAPM_MUX("PCM2.1 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm2_1_control),
	SND_SOC_DAPM_MUX("PCM2.2 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm2_2_control),
	SND_SOC_DAPM_MUX("PCM2.3 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_pcm2_3_control),

	SND_SOC_DAPM_MUX("SBUS.TX0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx0_control),
	SND_SOC_DAPM_MUX("SBUS.TX1 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx1_control),
	SND_SOC_DAPM_MUX("SBUS.TX2 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx2_control),
	SND_SOC_DAPM_MUX("SBUS.TX3 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx3_control),
	SND_SOC_DAPM_MUX("SBUS.TX4 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx4_control),
	SND_SOC_DAPM_MUX("SBUS.TX5 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_sbustx5_control),

	SND_SOC_DAPM_MUX("DAC0.0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_dac0_0_control),
	SND_SOC_DAPM_MUX("DAC0.1 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_dac0_1_control),
	SND_SOC_DAPM_MUX("DAC1.0 MUX", SND_SOC_NOPM, 0, 0,
			 &dapm_dac1_0_control),
	SND_SOC_DAPM_MUX("DAC1.1 MUX", SND_SOC_NOPM, 0, 0,
			&dapm_dac1_1_control),

	SND_SOC_DAPM_MIXER("VP CSOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("VP CSOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("VP FEOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("VP FEOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("VP FEOUT_CSOUT Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM AUDOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM AUDOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM PASSOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM PASSOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM MONOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("MM MONOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass AUDOUT1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass AUDOUT2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass AUDOUT3 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass AUDOUT4 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass AO1 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Pass MO2 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("AudioZoom CSOUT Mixer", SND_SOC_NOPM,
			0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("AudioZoom AOUT1 Mixer", SND_SOC_NOPM,
			0, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("PCM PORT A.0"),
	SND_SOC_DAPM_OUTPUT("PCM PORT A.1"),
	SND_SOC_DAPM_OUTPUT("PCM PORT A.2"),
	SND_SOC_DAPM_OUTPUT("PCM PORT A.3"),

	SND_SOC_DAPM_OUTPUT("PCM PORT B.0"),
	SND_SOC_DAPM_OUTPUT("PCM PORT B.1"),
	SND_SOC_DAPM_OUTPUT("PCM PORT B.2"),
	SND_SOC_DAPM_OUTPUT("PCM PORT B.3"),

	SND_SOC_DAPM_OUTPUT("PCM PORT C.0"),
	SND_SOC_DAPM_OUTPUT("PCM PORT C.1"),
	SND_SOC_DAPM_OUTPUT("PCM PORT C.2"),
	SND_SOC_DAPM_OUTPUT("PCM PORT C.3"),

	SND_SOC_DAPM_MIC("PDMI0", NULL),
	SND_SOC_DAPM_MIC("PDMI1", NULL),
	SND_SOC_DAPM_MIC("PDMI2", NULL),
	SND_SOC_DAPM_MIC("PDMI3", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {

	{"VP Primary MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP Primary MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP Primary MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP Primary MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP Primary MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP Primary MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP Primary MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP Primary MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP Primary MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP Primary MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP Primary MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP Primary MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP Primary MUX", "PDMI0", "PDMI0"},
	{"VP Primary MUX", "PDMI1", "PDMI1"},
	{"VP Primary MUX", "PDMI2", "PDMI2"},
	{"VP Primary MUX", "PDMI3", "PDMI3"},
	{"VP Primary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP Primary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP Primary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP Primary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP Primary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP Primary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP Primary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP Primary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP Primary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP Primary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP Primary MUX", "ADC0", "ADC0"},
	{"VP Primary MUX", "ADC1", "ADC1"},
	{"VP Primary MUX", "ADC2", "ADC2"},
	{"VP Primary MUX", "ADC3", "ADC3"},

	{"VP Secondary MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP Secondary MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP Secondary MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP Secondary MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP Secondary MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP Secondary MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP Secondary MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP Secondary MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP Secondary MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP Secondary MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP Secondary MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP Secondary MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP Secondary MUX", "PDMI0", "PDMI0"},
	{"VP Secondary MUX", "PDMI1", "PDMI1"},
	{"VP Secondary MUX", "PDMI2", "PDMI2"},
	{"VP Secondary MUX", "PDMI3", "PDMI3"},
	{"VP Secondary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP Secondary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP Secondary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP Secondary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP Secondary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP Secondary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP Secondary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP Secondary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP Secondary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP Secondary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP Secondary MUX", "ADC0", "ADC0"},
	{"VP Secondary MUX", "ADC1", "ADC1"},
	{"VP Secondary MUX", "ADC2", "ADC2"},
	{"VP Secondary MUX", "ADC3", "ADC3"},

	{"VP Teritary MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP Teritary MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP Teritary MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP Teritary MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP Teritary MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP Teritary MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP Teritary MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP Teritary MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP Teritary MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP Teritary MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP Teritary MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP Teritary MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP Teritary MUX", "PDMI0", "PDMI0"},
	{"VP Teritary MUX", "PDMI1", "PDMI1"},
	{"VP Teritary MUX", "PDMI2", "PDMI2"},
	{"VP Teritary MUX", "PDMI3", "PDMI3"},
	{"VP Teritary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP Teritary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP Teritary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP Teritary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP Teritary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP Teritary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP Teritary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP Teritary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP Teritary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP Teritary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP Teritary MUX", "ADC0", "ADC0"},
	{"VP Teritary MUX", "ADC1", "ADC1"},
	{"VP Teritary MUX", "ADC2", "ADC2"},
	{"VP Teritary MUX", "ADC3", "ADC3"},

	{"VP AECREF MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP AECREF MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP AECREF MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP AECREF MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP AECREF MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP AECREF MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP AECREF MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP AECREF MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP AECREF MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP AECREF MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP AECREF MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP AECREF MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP AECREF MUX", "PDMI0", "PDMI0"},
	{"VP AECREF MUX", "PDMI1", "PDMI1"},
	{"VP AECREF MUX", "PDMI2", "PDMI2"},
	{"VP AECREF MUX", "PDMI3", "PDMI3"},
	{"VP AECREF MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP AECREF MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP AECREF MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP AECREF MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP AECREF MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP AECREF MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP AECREF MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP AECREF MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP AECREF MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP AECREF MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP AECREF MUX", "ADC0", "ADC0"},
	{"VP AECREF MUX", "ADC1", "ADC1"},
	{"VP AECREF MUX", "ADC2", "ADC2"},
	{"VP AECREF MUX", "ADC3", "ADC3"},

	{"VP FEIN MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP FEIN MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP FEIN MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP FEIN MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP FEIN MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP FEIN MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP FEIN MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP FEIN MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP FEIN MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP FEIN MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP FEIN MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP FEIN MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP FEIN MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP FEIN MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP FEIN MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP FEIN MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP FEIN MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP FEIN MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP FEIN MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP FEIN MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP FEIN MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP FEIN MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP FEIN MUX", "ADC0", "ADC0"},
	{"VP FEIN MUX", "ADC1", "ADC1"},
	{"VP FEIN MUX", "ADC2", "ADC2"},
	{"VP FEIN MUX", "ADC3", "ADC3"},
	{"VP FEIN MUX", "PDMI0", "PDMI0"},
	{"VP FEIN MUX", "PDMI1", "PDMI1"},
	{"VP FEIN MUX", "PDMI2", "PDMI2"},
	{"VP FEIN MUX", "PDMI3", "PDMI3"},

	{"VP FEIN2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP FEIN2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP FEIN2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP FEIN2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP FEIN2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP FEIN2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP FEIN2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP FEIN2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP FEIN2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP FEIN2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP FEIN2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP FEIN2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP FEIN2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP FEIN2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP FEIN2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP FEIN2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP FEIN2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP FEIN2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP FEIN2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP FEIN2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP FEIN2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP FEIN2 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP FEIN2 MUX", "ADC0", "ADC0"},
	{"VP FEIN2 MUX", "ADC1", "ADC1"},
	{"VP FEIN2 MUX", "ADC2", "ADC2"},
	{"VP FEIN2 MUX", "ADC3", "ADC3"},
	{"VP FEIN2 MUX", "PDMI0", "PDMI0"},
	{"VP FEIN2 MUX", "PDMI1", "PDMI1"},
	{"VP FEIN2 MUX", "PDMI2", "PDMI2"},
	{"VP FEIN2 MUX", "PDMI3", "PDMI3"},


	{"VP UITONE1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP UITONE1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP UITONE1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP UITONE1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP UITONE1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP UITONE1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP UITONE1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP UITONE1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP UITONE1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP UITONE1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP UITONE1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP UITONE1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP UITONE1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP UITONE1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP UITONE1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP UITONE1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP UITONE1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP UITONE1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP UITONE1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP UITONE1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP UITONE1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP UITONE1 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP UITONE1 MUX", "ADC0", "ADC0"},
	{"VP UITONE1 MUX", "ADC1", "ADC1"},
	{"VP UITONE1 MUX", "ADC2", "ADC2"},
	{"VP UITONE1 MUX", "ADC3", "ADC3"},

	{"VP UITONE2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"VP UITONE2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"VP UITONE2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"VP UITONE2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"VP UITONE2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"VP UITONE2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"VP UITONE2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"VP UITONE2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"VP UITONE2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"VP UITONE2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"VP UITONE2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"VP UITONE2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"VP UITONE2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"VP UITONE2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"VP UITONE2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"VP UITONE2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"VP UITONE2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"VP UITONE2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"VP UITONE2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"VP UITONE2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"VP UITONE2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"VP UITONE2 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"VP UITONE2 MUX", "ADC0", "ADC0"},
	{"VP UITONE2 MUX", "ADC1", "ADC1"},
	{"VP UITONE2 MUX", "ADC2", "ADC2"},
	{"VP UITONE2 MUX", "ADC3", "ADC3"},

	{"VP CSOUT1 Mixer", NULL, "VP Primary MUX"},
	{"VP CSOUT1 Mixer", NULL, "VP Secondary MUX"},
	{"VP CSOUT1 Mixer", NULL, "VP Teritary MUX"},
	{"VP CSOUT1 Mixer", NULL, "VP AECREF MUX"},
	{"VP CSOUT2 Mixer", NULL, "VP Primary MUX"},
	{"VP CSOUT2 Mixer", NULL, "VP Secondary MUX"},
	{"VP CSOUT2 Mixer", NULL, "VP Teritary MUX"},
	{"VP CSOUT2 Mixer", NULL, "VP AECREF MUX"},
	{"VP FEOUT1 Mixer", NULL, "VP FEIN MUX"},
	{"VP FEOUT2 Mixer", NULL, "VP FEIN2 MUX"},
	{"VP FEOUT1 Mixer", NULL, "VP UITONE1 MUX"},
	{"VP FEOUT1 Mixer", NULL, "VP UITONE2 MUX"},
	{"VP FEOUT_CSOUT Mixer", NULL, "VP Primary MUX"},

	{"MM AUDIN1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM AUDIN1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM AUDIN1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM AUDIN1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM AUDIN1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM AUDIN1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM AUDIN1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM AUDIN1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM AUDIN1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM AUDIN1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM AUDIN1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM AUDIN1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM AUDIN1 MUX", "PDMI0", "PDMI0"},
	{"MM AUDIN1 MUX", "PDMI1", "PDMI1"},
	{"MM AUDIN1 MUX", "PDMI2", "PDMI2"},
	{"MM AUDIN1 MUX", "PDMI3", "PDMI3"},
	{"MM AUDIN1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM AUDIN1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM AUDIN1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM AUDIN1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM AUDIN1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM AUDIN1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM AUDIN1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM AUDIN1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM AUDIN1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM AUDIN1 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"MM AUDIN1 MUX", "ADC0", "ADC0"},
	{"MM AUDIN1 MUX", "ADC1", "ADC1"},
	{"MM AUDIN1 MUX", "ADC2", "ADC2"},
	{"MM AUDIN1 MUX", "ADC3", "ADC3"},

	{"MM AUDIN2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM AUDIN2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM AUDIN2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM AUDIN2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM AUDIN2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM AUDIN2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM AUDIN2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM AUDIN2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM AUDIN2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM AUDIN2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM AUDIN2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM AUDIN2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM AUDIN2 MUX", "PDMI0", "PDMI0"},
	{"MM AUDIN2 MUX", "PDMI1", "PDMI1"},
	{"MM AUDIN2 MUX", "PDMI2", "PDMI2"},
	{"MM AUDIN2 MUX", "PDMI3", "PDMI3"},
	{"MM AUDIN2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM AUDIN2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM AUDIN2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM AUDIN2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM AUDIN2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM AUDIN2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM AUDIN2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM AUDIN2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM AUDIN2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM AUDIN2 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"MM AUDIN2 MUX", "ADC0", "ADC0"},
	{"MM AUDIN2 MUX", "ADC1", "ADC1"},
	{"MM AUDIN2 MUX", "ADC2", "ADC2"},
	{"MM AUDIN2 MUX", "ADC3", "ADC3"},

	{"MM PASSIN1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM PASSIN1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM PASSIN1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM PASSIN1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM PASSIN1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM PASSIN1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM PASSIN1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM PASSIN1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM PASSIN1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM PASSIN1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM PASSIN1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM PASSIN1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM PASSIN1 MUX", "PDMI0", "PDMI0"},
	{"MM PASSIN1 MUX", "PDMI1", "PDMI1"},
	{"MM PASSIN1 MUX", "PDMI2", "PDMI2"},
	{"MM PASSIN1 MUX", "PDMI3", "PDMI3"},
	{"MM PASSIN1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM PASSIN1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM PASSIN1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM PASSIN1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM PASSIN1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM PASSIN1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM PASSIN1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM PASSIN1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM PASSIN1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM PASSIN1 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"MM PASSIN1 MUX", "ADC0", "ADC0"},
	{"MM PASSIN1 MUX", "ADC1", "ADC1"},
	{"MM PASSIN1 MUX", "ADC2", "ADC2"},
	{"MM PASSIN1 MUX", "ADC3", "ADC3"},

	{"MM PASSIN2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM PASSIN2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM PASSIN2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM PASSIN2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM PASSIN2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM PASSIN2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM PASSIN2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM PASSIN2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM PASSIN2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM PASSIN2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM PASSIN2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM PASSIN2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM PASSIN2 MUX", "PDMI0", "PDMI0"},
	{"MM PASSIN2 MUX", "PDMI1", "PDMI1"},
	{"MM PASSIN2 MUX", "PDMI2", "PDMI2"},
	{"MM PASSIN2 MUX", "PDMI3", "PDMI3"},
	{"MM PASSIN2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM PASSIN2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM PASSIN2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM PASSIN2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM PASSIN2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM PASSIN2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM PASSIN2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM PASSIN2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM PASSIN2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM PASSIN2 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"MM PASSIN2 MUX", "ADC0", "ADC0"},
	{"MM PASSIN2 MUX", "ADC1", "ADC1"},
	{"MM PASSIN2 MUX", "ADC2", "ADC2"},
	{"MM PASSIN2 MUX", "ADC3", "ADC3"},

	{"MM UITONE1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM UITONE1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM UITONE1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM UITONE1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM UITONE1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM UITONE1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM UITONE1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM UITONE1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM UITONE1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM UITONE1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM UITONE1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM UITONE1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM UITONE1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM UITONE1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM UITONE1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM UITONE1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM UITONE1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM UITONE1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM UITONE1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM UITONE1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM UITONE1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM UITONE1 MUX", "SBUS.RX9", "SBUS.RX9"},

	{"MM UITONE2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"MM UITONE2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"MM UITONE2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"MM UITONE2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"MM UITONE2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"MM UITONE2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"MM UITONE2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"MM UITONE2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"MM UITONE2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"MM UITONE2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"MM UITONE2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"MM UITONE2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"MM UITONE2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"MM UITONE2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"MM UITONE2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"MM UITONE2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"MM UITONE2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"MM UITONE2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"MM UITONE2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"MM UITONE2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"MM UITONE2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"MM UITONE2 MUX", "SBUS.RX9", "SBUS.RX9"},

	{"MM AUDOUT1 Mixer", NULL, "MM AUDIN1 MUX"},
	{"MM AUDOUT2 Mixer", NULL, "MM AUDIN2 MUX"},
	{"MM PASSOUT1 Mixer", NULL, "MM PASSIN1 MUX"},
	{"MM PASSOUT2 Mixer", NULL, "MM PASSIN2 MUX"},
	{"MM MONOUT1 Mixer", NULL, "MM UITONE1 MUX"},
	{"MM MONOUT2 Mixer", NULL, "MM UITONE2 MUX"},

	{"Pass AUDIN1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"Pass AUDIN1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"Pass AUDIN1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"Pass AUDIN1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"Pass AUDIN1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"Pass AUDIN1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"Pass AUDIN1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"Pass AUDIN1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"Pass AUDIN1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"Pass AUDIN1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"Pass AUDIN1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"Pass AUDIN1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"Pass AUDIN1 MUX", "PDMI0", "PDMI0"},
	{"Pass AUDIN1 MUX", "PDMI1", "PDMI1"},
	{"Pass AUDIN1 MUX", "PDMI2", "PDMI2"},
	{"Pass AUDIN1 MUX", "PDMI3", "PDMI3"},
	{"Pass AUDIN1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"Pass AUDIN1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"Pass AUDIN1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"Pass AUDIN1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"Pass AUDIN1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"Pass AUDIN1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"Pass AUDIN1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"Pass AUDIN1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"Pass AUDIN1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"Pass AUDIN1 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"Pass AUDIN1 MUX", "ADC0", "ADC0"},
	{"Pass AUDIN1 MUX", "ADC1", "ADC1"},
	{"Pass AUDIN1 MUX", "ADC2", "ADC2"},
	{"Pass AUDIN1 MUX", "ADC3", "ADC3"},

	{"Pass AUDIN2 MUX", "PCM0.0", "PCM0.0 RX"},
	{"Pass AUDIN2 MUX", "PCM0.1", "PCM0.1 RX"},
	{"Pass AUDIN2 MUX", "PCM0.2", "PCM0.2 RX"},
	{"Pass AUDIN2 MUX", "PCM0.3", "PCM0.3 RX"},
	{"Pass AUDIN2 MUX", "PCM1.0", "PCM1.0 RX"},
	{"Pass AUDIN2 MUX", "PCM1.1", "PCM1.1 RX"},
	{"Pass AUDIN2 MUX", "PCM1.2", "PCM1.2 RX"},
	{"Pass AUDIN2 MUX", "PCM1.3", "PCM1.3 RX"},
	{"Pass AUDIN2 MUX", "PCM2.0", "PCM2.0 RX"},
	{"Pass AUDIN2 MUX", "PCM2.1", "PCM2.1 RX"},
	{"Pass AUDIN2 MUX", "PCM2.2", "PCM2.2 RX"},
	{"Pass AUDIN2 MUX", "PCM2.3", "PCM2.3 RX"},
	{"Pass AUDIN2 MUX", "PDMI0", "PDMI0"},
	{"Pass AUDIN2 MUX", "PDMI1", "PDMI1"},
	{"Pass AUDIN2 MUX", "PDMI2", "PDMI2"},
	{"Pass AUDIN2 MUX", "PDMI3", "PDMI3"},
	{"Pass AUDIN2 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"Pass AUDIN2 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"Pass AUDIN2 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"Pass AUDIN2 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"Pass AUDIN2 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"Pass AUDIN2 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"Pass AUDIN2 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"Pass AUDIN2 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"Pass AUDIN2 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"Pass AUDIN2 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"Pass AUDIN2 MUX", "ADC0", "ADC0"},
	{"Pass AUDIN2 MUX", "ADC1", "ADC1"},
	{"Pass AUDIN2 MUX", "ADC2", "ADC2"},
	{"Pass AUDIN2 MUX", "ADC3", "ADC3"},

	{"Pass AUDIN3 MUX", "PCM0.0", "PCM0.0 RX"},
	{"Pass AUDIN3 MUX", "PCM0.1", "PCM0.1 RX"},
	{"Pass AUDIN3 MUX", "PCM0.2", "PCM0.2 RX"},
	{"Pass AUDIN3 MUX", "PCM0.3", "PCM0.3 RX"},
	{"Pass AUDIN3 MUX", "PCM1.0", "PCM1.0 RX"},
	{"Pass AUDIN3 MUX", "PCM1.1", "PCM1.1 RX"},
	{"Pass AUDIN3 MUX", "PCM1.2", "PCM1.2 RX"},
	{"Pass AUDIN3 MUX", "PCM1.3", "PCM1.3 RX"},
	{"Pass AUDIN3 MUX", "PCM2.0", "PCM2.0 RX"},
	{"Pass AUDIN3 MUX", "PCM2.1", "PCM2.1 RX"},
	{"Pass AUDIN3 MUX", "PCM2.2", "PCM2.2 RX"},
	{"Pass AUDIN3 MUX", "PCM2.3", "PCM2.3 RX"},
	{"Pass AUDIN3 MUX", "PDMI0", "PDMI0"},
	{"Pass AUDIN3 MUX", "PDMI1", "PDMI1"},
	{"Pass AUDIN3 MUX", "PDMI2", "PDMI2"},
	{"Pass AUDIN3 MUX", "PDMI3", "PDMI3"},
	{"Pass AUDIN3 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"Pass AUDIN3 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"Pass AUDIN3 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"Pass AUDIN3 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"Pass AUDIN3 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"Pass AUDIN3 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"Pass AUDIN3 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"Pass AUDIN3 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"Pass AUDIN3 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"Pass AUDIN3 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"Pass AUDIN3 MUX", "ADC0", "ADC0"},
	{"Pass AUDIN3 MUX", "ADC1", "ADC1"},
	{"Pass AUDIN3 MUX", "ADC2", "ADC2"},
	{"Pass AUDIN3 MUX", "ADC3", "ADC3"},

	{"Pass AUDIN4 MUX", "PCM0.0", "PCM0.0 RX"},
	{"Pass AUDIN4 MUX", "PCM0.1", "PCM0.1 RX"},
	{"Pass AUDIN4 MUX", "PCM0.2", "PCM0.2 RX"},
	{"Pass AUDIN4 MUX", "PCM0.3", "PCM0.3 RX"},
	{"Pass AUDIN4 MUX", "PCM1.0", "PCM1.0 RX"},
	{"Pass AUDIN4 MUX", "PCM1.1", "PCM1.1 RX"},
	{"Pass AUDIN4 MUX", "PCM1.2", "PCM1.2 RX"},
	{"Pass AUDIN4 MUX", "PCM1.3", "PCM1.3 RX"},
	{"Pass AUDIN4 MUX", "PCM2.0", "PCM2.0 RX"},
	{"Pass AUDIN4 MUX", "PCM2.1", "PCM2.1 RX"},
	{"Pass AUDIN4 MUX", "PCM2.2", "PCM2.2 RX"},
	{"Pass AUDIN4 MUX", "PCM2.3", "PCM2.3 RX"},
	{"Pass AUDIN4 MUX", "PDMI0", "PDMI0"},
	{"Pass AUDIN4 MUX", "PDMI1", "PDMI1"},
	{"Pass AUDIN4 MUX", "PDMI2", "PDMI2"},
	{"Pass AUDIN4 MUX", "PDMI3", "PDMI3"},
	{"Pass AUDIN4 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"Pass AUDIN4 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"Pass AUDIN4 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"Pass AUDIN4 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"Pass AUDIN4 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"Pass AUDIN4 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"Pass AUDIN4 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"Pass AUDIN4 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"Pass AUDIN4 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"Pass AUDIN4 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"Pass AUDIN4 MUX", "ADC0", "ADC0"},
	{"Pass AUDIN4 MUX", "ADC1", "ADC1"},
	{"Pass AUDIN4 MUX", "ADC2", "ADC2"},
	{"Pass AUDIN4 MUX", "ADC3", "ADC3"},

	{"Pass AUDOUT1 Mixer", NULL, "Pass AUDIN1 MUX"},
	{"Pass AUDOUT2 Mixer", NULL, "Pass AUDIN2 MUX"},
	{"Pass AUDOUT3 Mixer", NULL, "Pass AUDIN3 MUX"},
	{"Pass AUDOUT4 Mixer", NULL, "Pass AUDIN4 MUX"},
	{"Pass AO1 Mixer", NULL, "Pass AUDIN1 MUX"},
	{"Pass AO1 Mixer", NULL, "Pass AUDIN2 MUX"},
	{"Pass MO2 Mixer", NULL, "Pass AUDIN2 MUX"},

	{"AudioZoom Primary MUX", "PCM0.0", "PCM0.0 RX"},
	{"AudioZoom Primary MUX", "PCM0.1", "PCM0.1 RX"},
	{"AudioZoom Primary MUX", "PCM0.2", "PCM0.2 RX"},
	{"AudioZoom Primary MUX", "PCM0.3", "PCM0.3 RX"},
	{"AudioZoom Primary MUX", "PCM1.0", "PCM1.0 RX"},
	{"AudioZoom Primary MUX", "PCM1.1", "PCM1.1 RX"},
	{"AudioZoom Primary MUX", "PCM1.2", "PCM1.2 RX"},
	{"AudioZoom Primary MUX", "PCM1.3", "PCM1.3 RX"},
	{"AudioZoom Primary MUX", "PCM2.0", "PCM2.0 RX"},
	{"AudioZoom Primary MUX", "PCM2.1", "PCM2.1 RX"},
	{"AudioZoom Primary MUX", "PCM2.2", "PCM2.2 RX"},
	{"AudioZoom Primary MUX", "PCM2.3", "PCM2.3 RX"},
	{"AudioZoom Primary MUX", "PDMI0", "PDMI0"},
	{"AudioZoom Primary MUX", "PDMI1", "PDMI1"},
	{"AudioZoom Primary MUX", "PDMI2", "PDMI2"},
	{"AudioZoom Primary MUX", "PDMI3", "PDMI3"},
	{"AudioZoom Primary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"AudioZoom Primary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"AudioZoom Primary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"AudioZoom Primary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"AudioZoom Primary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"AudioZoom Primary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"AudioZoom Primary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"AudioZoom Primary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"AudioZoom Primary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"AudioZoom Primary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"AudioZoom Primary MUX", "ADC0", "ADC0"},
	{"AudioZoom Primary MUX", "ADC1", "ADC1"},
	{"AudioZoom Primary MUX", "ADC2", "ADC2"},
	{"AudioZoom Primary MUX", "ADC3", "ADC3"},

	{"AudioZoom Secondary MUX", "PCM0.0", "PCM0.0 RX"},
	{"AudioZoom Secondary MUX", "PCM0.1", "PCM0.1 RX"},
	{"AudioZoom Secondary MUX", "PCM0.2", "PCM0.2 RX"},
	{"AudioZoom Secondary MUX", "PCM0.3", "PCM0.3 RX"},
	{"AudioZoom Secondary MUX", "PCM1.0", "PCM1.0 RX"},
	{"AudioZoom Secondary MUX", "PCM1.1", "PCM1.1 RX"},
	{"AudioZoom Secondary MUX", "PCM1.2", "PCM1.2 RX"},
	{"AudioZoom Secondary MUX", "PCM1.3", "PCM1.3 RX"},
	{"AudioZoom Secondary MUX", "PCM2.0", "PCM2.0 RX"},
	{"AudioZoom Secondary MUX", "PCM2.1", "PCM2.1 RX"},
	{"AudioZoom Secondary MUX", "PCM2.2", "PCM2.2 RX"},
	{"AudioZoom Secondary MUX", "PCM2.3", "PCM2.3 RX"},
	{"AudioZoom Secondary MUX", "PDMI0", "PDMI0"},
	{"AudioZoom Secondary MUX", "PDMI1", "PDMI1"},
	{"AudioZoom Secondary MUX", "PDMI2", "PDMI2"},
	{"AudioZoom Secondary MUX", "PDMI3", "PDMI3"},
	{"AudioZoom Secondary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"AudioZoom Secondary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"AudioZoom Secondary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"AudioZoom Secondary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"AudioZoom Secondary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"AudioZoom Secondary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"AudioZoom Secondary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"AudioZoom Secondary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"AudioZoom Secondary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"AudioZoom Secondary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"AudioZoom Secondary MUX", "ADC0", "ADC0"},
	{"AudioZoom Secondary MUX", "ADC1", "ADC1"},
	{"AudioZoom Secondary MUX", "ADC2", "ADC2"},
	{"AudioZoom Secondary MUX", "ADC3", "ADC3"},

	{"AudioZoom Teritary MUX", "PCM0.0", "PCM0.0 RX"},
	{"AudioZoom Teritary MUX", "PCM0.1", "PCM0.1 RX"},
	{"AudioZoom Teritary MUX", "PCM0.2", "PCM0.2 RX"},
	{"AudioZoom Teritary MUX", "PCM0.3", "PCM0.3 RX"},
	{"AudioZoom Teritary MUX", "PCM1.0", "PCM1.0 RX"},
	{"AudioZoom Teritary MUX", "PCM1.1", "PCM1.1 RX"},
	{"AudioZoom Teritary MUX", "PCM1.2", "PCM1.2 RX"},
	{"AudioZoom Teritary MUX", "PCM1.3", "PCM1.3 RX"},
	{"AudioZoom Teritary MUX", "PCM2.0", "PCM2.0 RX"},
	{"AudioZoom Teritary MUX", "PCM2.1", "PCM2.1 RX"},
	{"AudioZoom Teritary MUX", "PCM2.2", "PCM2.2 RX"},
	{"AudioZoom Teritary MUX", "PCM2.3", "PCM2.3 RX"},
	{"AudioZoom Teritary MUX", "PDMI0", "PDMI0"},
	{"AudioZoom Teritary MUX", "PDMI1", "PDMI1"},
	{"AudioZoom Teritary MUX", "PDMI2", "PDMI2"},
	{"AudioZoom Teritary MUX", "PDMI3", "PDMI3"},
	{"AudioZoom Teritary MUX", "SBUS.RX0", "SBUS.RX0"},
	{"AudioZoom Teritary MUX", "SBUS.RX1", "SBUS.RX1"},
	{"AudioZoom Teritary MUX", "SBUS.RX2", "SBUS.RX2"},
	{"AudioZoom Teritary MUX", "SBUS.RX3", "SBUS.RX3"},
	{"AudioZoom Teritary MUX", "SBUS.RX4", "SBUS.RX4"},
	{"AudioZoom Teritary MUX", "SBUS.RX5", "SBUS.RX5"},
	{"AudioZoom Teritary MUX", "SBUS.RX6", "SBUS.RX6"},
	{"AudioZoom Teritary MUX", "SBUS.RX7", "SBUS.RX7"},
	{"AudioZoom Teritary MUX", "SBUS.RX8", "SBUS.RX8"},
	{"AudioZoom Teritary MUX", "SBUS.RX9", "SBUS.RX9"},
	{"AudioZoom Teritary MUX", "ADC0", "ADC0"},
	{"AudioZoom Teritary MUX", "ADC1", "ADC1"},
	{"AudioZoom Teritary MUX", "ADC2", "ADC2"},
	{"AudioZoom Teritary MUX", "ADC3", "ADC3"},

	{"AudioZoom AI1 MUX", "PCM0.0", "PCM0.0 RX"},
	{"AudioZoom AI1 MUX", "PCM0.1", "PCM0.1 RX"},
	{"AudioZoom AI1 MUX", "PCM0.2", "PCM0.2 RX"},
	{"AudioZoom AI1 MUX", "PCM0.3", "PCM0.3 RX"},
	{"AudioZoom AI1 MUX", "PCM1.0", "PCM1.0 RX"},
	{"AudioZoom AI1 MUX", "PCM1.1", "PCM1.1 RX"},
	{"AudioZoom AI1 MUX", "PCM1.2", "PCM1.2 RX"},
	{"AudioZoom AI1 MUX", "PCM1.3", "PCM1.3 RX"},
	{"AudioZoom AI1 MUX", "PCM2.0", "PCM2.0 RX"},
	{"AudioZoom AI1 MUX", "PCM2.1", "PCM2.1 RX"},
	{"AudioZoom AI1 MUX", "PCM2.2", "PCM2.2 RX"},
	{"AudioZoom AI1 MUX", "PCM2.3", "PCM2.3 RX"},
	{"AudioZoom AI1 MUX", "PDMI0", "PDMI0"},
	{"AudioZoom AI1 MUX", "PDMI1", "PDMI1"},
	{"AudioZoom AI1 MUX", "PDMI2", "PDMI2"},
	{"AudioZoom AI1 MUX", "PDMI3", "PDMI3"},
	{"AudioZoom AI1 MUX", "SBUS.RX0", "SBUS.RX0"},
	{"AudioZoom AI1 MUX", "SBUS.RX1", "SBUS.RX1"},
	{"AudioZoom AI1 MUX", "SBUS.RX2", "SBUS.RX2"},
	{"AudioZoom AI1 MUX", "SBUS.RX3", "SBUS.RX3"},
	{"AudioZoom AI1 MUX", "SBUS.RX4", "SBUS.RX4"},
	{"AudioZoom AI1 MUX", "SBUS.RX5", "SBUS.RX5"},
	{"AudioZoom AI1 MUX", "SBUS.RX6", "SBUS.RX6"},
	{"AudioZoom AI1 MUX", "SBUS.RX7", "SBUS.RX7"},
	{"AudioZoom AI1 MUX", "SBUS.RX8", "SBUS.RX8"},
	{"AudioZoom AI1 MUX", "SBUS.RX9", "SBUS.RX9"},
	{"AudioZoom AI1 MUX", "ADC0", "ADC0"},
	{"AudioZoom AI1 MUX", "ADC1", "ADC1"},
	{"AudioZoom AI1 MUX", "ADC2", "ADC2"},
	{"AudioZoom AI1 MUX", "ADC3", "ADC3"},

	{"AudioZoom CSOUT Mixer", NULL, "AudioZoom Primary MUX"},
	{"AudioZoom CSOUT Mixer", NULL, "AudioZoom Secondary MUX"},
	{"AudioZoom CSOUT Mixer", NULL, "AudioZoom Teritary MUX"},
	{"AudioZoom AOUT1 Mixer", NULL, "AudioZoom AI1 MUX"},

	{"PCM0.0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM0.0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM0.0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM0.0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM0.0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM0.0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM0.0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM0.0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM0.0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM0.0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM0.0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM0.0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM0.0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM0.0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM0.0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM0.0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM0.0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM0.0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM0.0 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM0.1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM0.1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM0.1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM0.1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM0.1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM0.1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM0.1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM0.1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM0.1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM0.1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM0.1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM0.1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM0.1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM0.1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM0.1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM0.1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM0.1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM0.1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM0.1 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM0.2 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM0.2 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM0.2 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM0.2 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM0.2 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM0.2 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM0.2 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM0.2 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM0.2 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM0.2 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM0.2 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM0.2 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM0.2 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM0.2 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM0.2 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM0.2 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM0.2 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM0.2 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM0.2 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM0.3 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM0.3 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM0.3 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM0.3 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM0.3 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM0.3 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM0.3 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM0.3 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM0.3 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM0.3 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM0.3 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM0.3 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM0.3 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM0.3 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM0.3 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM0.3 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM0.3 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM0.3 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM0.3 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"PCM1.0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM1.0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM1.0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM1.0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM1.0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM1.0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM1.0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM1.0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM1.0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM1.0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM1.0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM1.0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM1.0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM1.0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM1.0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM1.0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM1.0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM1.0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM1.0 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM1.1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM1.1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM1.1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM1.1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM1.1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM1.1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM1.1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM1.1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM1.1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM1.1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM1.1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM1.1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM1.1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM1.1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM1.1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM1.1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM1.1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM1.1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM1.1 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM1.2 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM1.2 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM1.2 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM1.2 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM1.2 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM1.2 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM1.2 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM1.2 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM1.2 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM1.2 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM1.2 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM1.2 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM1.2 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM1.2 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM1.2 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM1.2 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM1.2 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM1.2 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM1.2 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM1.3 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM1.3 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM1.3 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM1.3 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM1.3 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM1.3 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM1.3 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM1.3 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM1.3 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM1.3 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM1.3 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM1.3 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM1.3 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM1.3 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM1.3 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM1.3 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM1.3 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM1.3 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM1.3 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"PCM2.0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM2.0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM2.0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM2.0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM2.0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM2.0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM2.0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM2.0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM2.0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM2.0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM2.0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM2.0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM2.0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM2.0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM2.0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM2.0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM2.0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM2.0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM2.0 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM2.1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM2.1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM2.1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM2.1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM2.1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM2.1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM2.1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM2.1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM2.1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM2.1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM2.1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM2.1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM2.1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM2.1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM2.1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM2.1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM2.1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM2.1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM2.1 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM2.2 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM2.2 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM2.2 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM2.2 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM2.2 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM2.2 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM2.2 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM2.2 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM2.2 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM2.2 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM2.2 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM2.2 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM2.2 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM2.2 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM2.2 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM2.2 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM2.2 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM2.2 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM2.2 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"PCM2.3 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"PCM2.3 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"PCM2.3 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"PCM2.3 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"PCM2.3 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"PCM2.3 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"PCM2.3 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"PCM2.3 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"PCM2.3 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"PCM2.3 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"PCM2.3 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"PCM2.3 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"PCM2.3 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"PCM2.3 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"PCM2.3 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"PCM2.3 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"PCM2.3 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"PCM2.3 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"PCM2.3 MUX", "Pass MO2", "Pass MO2 Mixer"},


	{"SBUS.TX0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX0 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"SBUS.TX1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX1 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"SBUS.TX2 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX2 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX2 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX2 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX2 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX2 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX2 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX2 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX2 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX2 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX2 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX2 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX2 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX2 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX2 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX2 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX2 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX2 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX2 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"SBUS.TX3 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX3 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX3 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX3 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX3 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX3 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX3 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX3 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX3 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX3 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX3 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX3 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX3 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX3 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX3 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX3 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX3 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX3 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX3 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"SBUS.TX4 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX4 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX4 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX4 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX4 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX4 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX4 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX4 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX4 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX4 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX4 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX4 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX4 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX4 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX4 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX4 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX4 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX4 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX4 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"SBUS.TX5 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"SBUS.TX5 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"SBUS.TX5 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"SBUS.TX5 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"SBUS.TX5 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"SBUS.TX5 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"SBUS.TX5 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"SBUS.TX5 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"SBUS.TX5 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"SBUS.TX5 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"SBUS.TX5 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"SBUS.TX5 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"SBUS.TX5 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"SBUS.TX5 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"SBUS.TX5 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"SBUS.TX5 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"SBUS.TX5 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"SBUS.TX5 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"SBUS.TX5 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"DAC0.0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"DAC0.0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"DAC0.0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"DAC0.0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"DAC0.0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"DAC0.0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"DAC0.0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"DAC0.0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"DAC0.0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"DAC0.0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"DAC0.0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"DAC0.0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"DAC0.0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"DAC0.0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"DAC0.0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"DAC0.0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"DAC0.0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"DAC0.0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"DAC0.0 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"DAC0.1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"DAC0.1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"DAC0.1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"DAC0.1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"DAC0.1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"DAC0.1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"DAC0.1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"DAC0.1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"DAC0.1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"DAC0.1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"DAC0.1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"DAC0.1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"DAC0.1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"DAC0.1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"DAC0.1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"DAC0.1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"DAC0.1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"DAC0.1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"DAC0.1 MUX", "Pass MO2", "Pass MO2 Mixer"},

	{"DAC1.0 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"DAC1.0 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"DAC1.0 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"DAC1.0 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"DAC1.0 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"DAC1.0 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"DAC1.0 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"DAC1.0 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"DAC1.0 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"DAC1.0 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"DAC1.0 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"DAC1.0 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"DAC1.0 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"DAC1.0 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"DAC1.0 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"DAC1.0 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"DAC1.0 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"DAC1.0 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"DAC1.0 MUX", "Pass MO2", "Pass MO2 Mixer"},
	{"DAC1.1 MUX", "VP CSOUT1", "VP CSOUT1 Mixer"},
	{"DAC1.1 MUX", "VP CSOUT2", "VP CSOUT2 Mixer"},
	{"DAC1.1 MUX", "VP FEOUT1", "VP FEOUT1 Mixer"},
	{"DAC1.1 MUX", "VP FEOUT1", "VP FEOUT_CSOUT Mixer"},
	{"DAC1.1 MUX", "VP FEOUT2", "VP FEOUT2 Mixer"},
	{"DAC1.1 MUX", "AudioZoom CSOUT", "AudioZoom CSOUT Mixer"},
	{"DAC1.1 MUX", "AudioZoom AOUT1", "AudioZoom AOUT1 Mixer"},
	{"DAC1.1 MUX", "MM PASSOUT1", "MM PASSOUT1 Mixer"},
	{"DAC1.1 MUX", "MM PASSOUT2", "MM PASSOUT2 Mixer"},
	{"DAC1.1 MUX", "MM AUDOUT1", "MM AUDOUT1 Mixer"},
	{"DAC1.1 MUX", "MM AUDOUT2", "MM AUDOUT2 Mixer"},
	{"DAC1.1 MUX", "MM MONOUT1", "MM MONOUT1 Mixer"},
	{"DAC1.1 MUX", "MM MONOUT2", "MM MONOUT2 Mixer"},
	{"DAC1.1 MUX", "Pass AUDOUT1", "Pass AUDOUT1 Mixer"},
	{"DAC1.1 MUX", "Pass AUDOUT2", "Pass AUDOUT2 Mixer"},
	{"DAC1.1 MUX", "Pass AUDOUT3", "Pass AUDOUT3 Mixer"},
	{"DAC1.1 MUX", "Pass AUDOUT4", "Pass AUDOUT4 Mixer"},
	{"DAC1.1 MUX", "Pass AO1", "Pass AO1 Mixer"},
	{"DAC1.1 MUX", "Pass MO2", "Pass MO2 Mixer"},

	/* AIF TX <--> PCM PORTA  */

	{"PCM0.0 TX", NULL, "PCM0.0 MUX"},
	{"PCM0.1 TX", NULL, "PCM0.1 MUX"},
	{"PCM0.2 TX", NULL, "PCM0.2 MUX"},
	{"PCM0.3 TX", NULL, "PCM0.3 MUX"},

	/* Digital PCM Port A <--> PCM PORTA MUX */

	{"PCM PORT A.0", NULL, "PCM0.0 MUX"},
	{"PCM PORT A.1", NULL, "PCM0.1 MUX"},
	{"PCM PORT A.2", NULL, "PCM0.2 MUX"},
	{"PCM PORT A.3", NULL, "PCM0.3 MUX"},


	/* AIF TX <--> PCM PORTB  */

	{"PCM1.0 TX", NULL, "PCM1.0 MUX"},
	{"PCM1.1 TX", NULL, "PCM1.1 MUX"},
	{"PCM1.2 TX", NULL, "PCM1.2 MUX"},
	{"PCM1.3 TX", NULL, "PCM1.3 MUX"},

	/* Digital PCM Port B <--> PCM PORTB MUX */

	{"PCM PORT B.0", NULL, "PCM1.0 MUX"},
	{"PCM PORT B.1", NULL, "PCM1.1 MUX"},
	{"PCM PORT B.2", NULL, "PCM1.2 MUX"},
	{"PCM PORT B.3", NULL, "PCM1.3 MUX"},

	/* AIF TX <--> PCM PORTC  */

	{"PCM2.0 TX", NULL, "PCM2.0 MUX"},
	{"PCM2.1 TX", NULL, "PCM2.1 MUX"},
	{"PCM2.2 TX", NULL, "PCM2.2 MUX"},
	{"PCM2.3 TX", NULL, "PCM2.3 MUX"},

	/* Digital PCM Port C <--> PCM PORTC MUX */

	{"PCM PORT C.0", NULL, "PCM2.0 MUX"},
	{"PCM PORT C.1", NULL, "PCM2.1 MUX"},
	{"PCM PORT C.2", NULL, "PCM2.2 MUX"},
	{"PCM PORT C.3", NULL, "PCM2.3 MUX"},

	/* AIF TX <--> SBUS.TX */

	{"SBUS.TX0", NULL, "SBUS.TX0 MUX"},
	{"SBUS.TX1", NULL, "SBUS.TX1 MUX"},
	{"SBUS.TX2", NULL, "SBUS.TX2 MUX"},
	{"SBUS.TX3", NULL, "SBUS.TX3 MUX"},
	{"SBUS.TX4", NULL, "SBUS.TX4 MUX"},
	{"SBUS.TX5", NULL, "SBUS.TX5 MUX"},

	/* A300 DAC <--> D300 RX  */

	{"DAC0L", NULL, "DAC0.0 MUX"},
	{"DAC0R", NULL, "DAC0.1 MUX"},
	{"DAC1L", NULL, "DAC1.0 MUX"},
	{"DAC1R", NULL, "DAC1.1 MUX"},
};

void es_d300_reset_cmdcache(void)
{
	int i;
	for (i = 0; i < ALGO_MAX; i++) {
		memset(cachedcmd_list[i], 0x0,
			ES_API_ADDR_MAX * sizeof(struct cachedcmd_t));
	}
}

int es_d300_fill_cmdcache(struct snd_soc_codec *codec)
{
	u8 i;
	const char *algo_text[ALGO_MAX] = {
		[VP] = "VP",
		[MM] = "MM",
		[AUDIOZOOM] = "AudioZoom",
		[PASSTHRU] = "Passthru",
	};

	cachedcmd_list = kzalloc(ALGO_MAX*sizeof(struct cachedcmd_t *),
			GFP_KERNEL);
	if (!cachedcmd_list) {
		pr_err("%s(): Error while allocating regcache\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ALGO_MAX; i++) {
		cachedcmd_list[i] = kzalloc(ES_API_ADDR_MAX *
				sizeof(struct cachedcmd_t),
				GFP_KERNEL);
		if (!cachedcmd_list[i]) {
			pr_err("%s(): Error while allocating regcache for %s\n",
					__func__, algo_text[i]);
			return -ENOMEM;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(es_d300_fill_cmdcache);

int es_d300_add_snd_soc_controls(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_add_codec_controls(codec, es_d300_snd_controls,
			ARRAY_SIZE(es_d300_snd_controls));

	return rc;
}

int es_d300_add_snd_soc_dapm_controls(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_dapm_new_controls(&codec->dapm, es_d300_dapm_widgets,
			ARRAY_SIZE(es_d300_dapm_widgets));

	return rc;
}

int es_d300_add_snd_soc_route_map(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_dapm_add_routes(&codec->dapm, intercon,
			ARRAY_SIZE(intercon));

	return rc;
}
