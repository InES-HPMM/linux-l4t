/*
 * tegra_avp_audio.h - Shared interface between AVP and kernel audio avp driver
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_AVP_AUDIO_H
#define _TEGRA_AVP_AUDIO_H

#define USE_IRAM_DEVICE_BUFFER  1

/* Runtime profiling options */
#define PROFILE_RENDER_PROCESSING           0x00000001
#define PROFILE_RENDER_OVERALL_PROCESSING  \
	(0x00000002 | PROFILE_RENDER_PROCESSING)
#define PROFILE_RENDER_MEMORY               0x00000004

#define DEVICE_BUFFER_SIZE      (0x1000)

#define RENDERSW_MAX_LOOPBACK_STREAMS        1
#define RENDERSW_MAX_RENDER_STREAMS          4
#define RENDERSW_MAX_STREAMS                 \
	(RENDERSW_MAX_LOOPBACK_STREAMS + RENDERSW_MAX_RENDER_STREAMS)
#define RENDERSW_MAX_CHANNELS                2

#define AAC_PAYLOAD_RAW     0
#define AAC_PAYLOAD_ADTS    1
#define AAC_PAYLOAD_ADIF    2
#define AAC_PAYLOAD_LOAS    3

enum KSSTATE {
	KSSTATE_STOP,
	KSSTATE_ACQUIRE,
	KSSTATE_PAUSE,
	KSSTATE_RUN,

	KSSTATE_Force32 = 0x7fffffff
};

enum audio_format {
	FORMAT_PCM,
	FORMAT_MP3,
	FORMAT_AAC,
	FORMAT_WMA,
	FORMAT_NUM,
};

enum {
	CODEC_PCM = 0,
	CODEC_MP3 = 1,
	CODEC_AAC = 2,
	CODEC_WMA = 3,
	NUM_CODECS,
};

enum avp_audio_stream_id {
	loopback_stream_id = 0,
	pcm_stream_id,
	pcm2_stream_id,
	decode_stream_id,
	decode2_stream_id,
	max_stream_id
};

enum tegra_arch {
	NV_TEGRA_T20 = 0x20,
	NV_TEGRA_T30 = 0x30,
	NV_TEGRA_T114 = 0x35,
	NV_TEGRA_T148 = 0x14,
	NV_TEGRA_T124 = 0x40,
	NV_TEGRA_Force32 = 0x7FFFFFFF
};

enum aac_profile {
	AAC_PROFILE_MAIN,
	AAC_PROFILE_LC,
	AAC_PROFILE_SSR,
};

enum ucode_desc_params {
	UCODE_DESC_OFFSET,
	UCODE_DESC_SIZE,
	UCODE_DESC_NUM,
};

struct audio_params {
	unsigned int                rate;
	unsigned int                channels;
	unsigned int                bits_per_sample;
};

struct stream_data {
	/* Writeable by mixer */
	enum KSSTATE                stream_state_current;
	unsigned int                halted;
	unsigned int                source_buffer_read_position;
	unsigned int                source_buffer_read_position_fraction;
	unsigned int                source_buffer_presentation_position;
	unsigned int                source_buffer_linear_position;
	unsigned int                source_buffer_frames_decoded;
	unsigned int                stream_notification_offset;
	unsigned int                stream_notification_request;
	unsigned int                stream_buffer_samples_consumed;
	unsigned long long          stream_buffer_read_time_us;

	/* Read-only for mixer */
	unsigned int                stream_allocated;
	enum KSSTATE                stream_state_target;

	unsigned long long          source_buffer_system;
	unsigned int                source_buffer_avp;
	unsigned int                source_buffer_size;
	unsigned int                source_buffer_write_position;
	unsigned int                source_buffer_write_count;

	unsigned int                stream_notification_interval;
	int                         stream_notification_enable;

	unsigned int                fragment_size;
	unsigned int                fragment_byte_offset;
	unsigned int                fragments_elapsed;

	struct audio_params         stream_params;
	enum audio_format           stream_format;
	int                         stream_volume[RENDERSW_MAX_CHANNELS];

	union {
		/* AAC Specific Data */
		struct {
			unsigned int            payload_type;
			unsigned int            audio_profile;
			unsigned int            sampling_freq_index;
			unsigned int            sampling_freq;
			unsigned int            channel_configuration;
			unsigned int            extension_sampling_freq_index;
			unsigned int            extension_sampling_freq;
		} aac;
	} u;
};

struct audio_engine_data {
	unsigned int            apb_channel_handle;

	unsigned long long      device_buffer_system;
	unsigned int            device_buffer_avp;

	unsigned int            track_audio_latency;
	unsigned int            hw_write_time;

	unsigned int            codec_ucode_base_address;
	unsigned int            codec_ucode_desc[NUM_CODECS][UCODE_DESC_NUM];
					/* [codec][offset,size] */

	enum KSSTATE            device_state_target;
	unsigned int            profile_state;

	struct audio_params     device_format;

	enum KSSTATE            device_state_current;
	unsigned int            device_buffer_write_position;
	unsigned int            device_buffer_read_position;
	unsigned long long      device_buffer_read_time_us;
	unsigned int            chip_id;
	unsigned int            only_loopback_flag;
	unsigned int            starvation_count;
	unsigned int            starvation_frames;

	struct stream_data      stream[RENDERSW_MAX_STREAMS];
};

#endif
