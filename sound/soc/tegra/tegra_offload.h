/*
 * tegra_offload.h - Definitions for Tegra offload platform driver interface
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __TEGRA_OFFLOAD_H__
#define __TEGRA_OFFLOAD_H__

struct tegra_offload_dma_params {
	unsigned long	addr;
	unsigned long	width;
	unsigned long	req_sel;
	unsigned long	max_burst;
};

struct tegra_offload_mem {
	struct device	*dev;
	unsigned char	*virt_addr;
	dma_addr_t	phys_addr;
	size_t		bytes;
};

struct tegra_offload_pcm_params {
	int				rate;
	int				channels;
	int				bits_per_sample;
	int				buffer_size;	/* bytes */
	int				period_size;	/* bytes */
	struct tegra_offload_mem	source_buf;
	struct tegra_offload_dma_params dma_params;
	void	(*period_elapsed_cb)(void *args, unsigned int is_eos);
	void	*period_elapsed_args;
};

struct tegra_offload_compr_params {
	unsigned int			codec_type; /* SND_AUDIOCODEC_* type */
	int				rate;
	int				channels;
	int				bits_per_sample;
	int				fragment_size;	/* bytes */
	int				fragments;
	struct tegra_offload_dma_params	dma_params;
	void	(*fragments_elapsed_cb)(void *args, unsigned int is_eos);
	void	*fragments_elapsed_args;
};

struct tegra_offload_pcm_ops {
	int	(*stream_open)(int *id, char *stream);
	void	(*stream_close)(int id);
	int	(*set_stream_params)(int id,
			struct tegra_offload_pcm_params *params);
	int	(*set_stream_state)(int id, int state);
	size_t	(*get_stream_position)(int id);
	void	(*data_ready)(int id, int bytes);
};

struct tegra_offload_compr_ops {
	int	(*stream_open)(int *id);
	void	(*stream_close)(int id);
	int	(*set_stream_params)(int id,
			struct tegra_offload_compr_params *params);
	int	(*set_stream_state)(int id, int state);
	int	(*get_stream_position)(int id, struct snd_compr_tstamp *tstamp);
	void	(*data_ready)(int id, int bytes);
	int	(*write)(int id, char __user *buf, int bytes);
	int	(*get_caps)(struct snd_compr_caps *caps);
	int	(*get_codec_caps)(struct snd_compr_codec_caps *codec);
	int	(*set_stream_volume)(int id, int left, int right);
};

struct tegra_offload_device_ops {
	int	(*set_hw_rate)(int rate);
	int	(*alloc_shared_mem)(struct tegra_offload_mem *mem, int bytes);
	void	(*free_shared_mem)(struct tegra_offload_mem *mem);
};

struct tegra_offload_ops {
	struct tegra_offload_device_ops device_ops;
	struct tegra_offload_pcm_ops	pcm_ops;
	struct tegra_offload_pcm_ops	loopback_ops;
	struct tegra_offload_compr_ops	compr_ops;
};

int tegra_register_offload_ops(struct tegra_offload_ops *ops);
#endif
