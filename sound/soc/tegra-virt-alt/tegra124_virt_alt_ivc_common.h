/*
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA124_VIRT_IVC_TYPES_H__
#define __TEGRA124_VIRT_IVC_TYPES_H__

enum ivc_audio_err_t {
	NVAUDIO_ERR_OK,
	NVAUDIO_ERR_SERVER_STATE,
	NVAUDIO_ERR_ARGS,
	NVAUDIO_ERR_REQ,
	NVAUDIO_ERR_UNSUPPORTED_REQ
};


enum rx_state_t {
	RX_INIT,
	RX_PENDING,
	RX_AVAIL,
	RX_DONE,
};

enum nvaudio_ivc_cmd_t {
	NVAUDIO_APBIF_SET_RXCIF,
	NVAUDIO_APBIF_SET_TXCIF,
	NVAUDIO_APBIF_START_PLAYBACK,
	NVAUDIO_APBIF_STOP_PLAYBACK,
	NVAUDIO_APBIF_START_CAPTURE,
	NVAUDIO_APBIF_STOP_CAPTURE,
	NVAUDIO_DAM_SET_IN_SRATE,
	NVAUDIO_DAM_GET_IN_SRATE,
	NVAUDIO_DAM_SET_OUT_SRATE,
	NVAUDIO_DAM_GET_OUT_SRATE,
	NVAUDIO_DAM_CHANNEL_SET_GAIN,
	NVAUDIO_DAM_CHANNEL_GET_GAIN,
	NVAUDIO_XBAR_SET_ROUTE,
	NVAUDIO_XBAR_GET_ROUTE,
	NVAUDIO_CMD_MAX,
};

struct nvaudio_ivc_dam_info {
	int id;
	unsigned int in_srate;
	unsigned int out_srate;
	unsigned int channel_reg;
	unsigned int gain;
};

struct nvaudio_ivc_xbar_link {
	unsigned int	rx_reg;
	unsigned int	tx_value;
	unsigned int	bit_pos;
};

struct nvaudio_ivc_apbif_info {
	int id;
	int value;
};

struct nvaudio_ivc_msg {
	int			channel_id;
	enum nvaudio_ivc_cmd_t	cmd;
	union {
		struct nvaudio_ivc_apbif_info	apbif_info;
		struct nvaudio_ivc_dam_info	dam_info;
		struct nvaudio_ivc_xbar_link	xbar_info;
	} params;
	bool			ack_required;
	int			err;
};

#endif
