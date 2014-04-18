/* include/linux/tegra_audio.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Author:
 *     Iliyan Malchev <malchev@google.com>
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

#ifndef _TEGRA_AUDIO_H
#define _TEGRA_AUDIO_H

#include <linux/ioctl.h>

#define TEGRA_AUDIO_MAGIC 't'

#define TEGRA_AUDIO_IN_START _IO(TEGRA_AUDIO_MAGIC, 0)
#define TEGRA_AUDIO_IN_STOP  _IO(TEGRA_AUDIO_MAGIC, 1)

enum tegra_audio_test_id {
	TEGRA_AUDIO_TEST_NONE = 0,
	TEGRA_AUDIO_TEST_AHUB_BBC1_DL,
	TEGRA_AUDIO_TEST_AHUB_BBC1_UL,
	TEGRA_AUDIO_TEST_AHUB_BBC1_UL_DL_LB,
	TEGRA_AUDIO_TEST_APBIF_APBIF_LB,

	TEGRA_AUDIO_TEST_TOTALS
};

struct tegra_audio_in_config {
	int rate;
	int stereo;
};

enum dam_mix_ch_wait_mode {
	TEGRA_AUDIO_MIX_CH0_CH1_WAIT,
	TEGRA_AUDIO_MIX_CH0_WAIT,
	TEGRA_AUDIO_MIX_CH1_WAIT,
	TEGRA_AUDIO_MIX_NO_WAIT
};

struct dam_srate {
	unsigned int in_sample_rate;
	unsigned int out_sample_rate;
	unsigned int audio_bits;
	unsigned int client_bits;
	unsigned int audio_channels;
	unsigned int client_channels;
	unsigned int apbif_chan;
};

struct tegra_audio_test_params {
	unsigned int test_id;
	unsigned int sample_rate;
	unsigned int audio_bits;
	unsigned int client_bits;
	unsigned int audio_channels;
	unsigned int client_channels;
	unsigned int write_size;
	unsigned int read_size;
};

struct dmic_params_t {
	unsigned int over_sampling_ratio;
	unsigned int sample_rate;
	unsigned int save_data_bits;
	unsigned int capture_data_bits;
	unsigned int save_data_channels;
	unsigned int capture_data_channels;
	unsigned int bypass_mode;
	unsigned int apbif_channels;
	unsigned int captured_data_size;
};

enum AHUB_MODULE_ID {
	MODULE_ADMAIF,
	MODULE_SFC,
	MODULE_OPE,
	MODULE_PEQ,
	MODULE_MBDRC,
	MODULE_SPKPROT,
	MODULE_I2S,
	MODULE_AMIXER,
	MODULE_DMIC,
	MODULE_ADSP,
	MODULE_MVC,
	MODULE_NUM,
};

enum tegra210_audio_test_id {
	TEST_ID_REG_ACCESS = 0x1,
	TEST_ID_STATUS_REG_CHECK,
	TEST_ID_RUNTIME_SOFT_RESET,
	TEST_ID_GET_POSITION,
	TEST_ID_COEFF_RAM_SEQ,
	TEST_ID_COEFF_RAM_NON_SEQ,
	TEST_ID_MVC_MUTE_CTRL,
	TEST_ID_MVC_UNMUTE_CTRL,
	TEST_ID_AMIXER_PEAK_VALUE_SAMPLE_COUNT,
	TEST_ID_MVC_MUTE_UNMUTE_CTRL,
	TEST_ID_MVC_START_STOP,
	TEST_ID_RUNTIME_PEAKMETER,
	TEST_ID_RUNTIME_PARAM_SWITCH,
	TEST_ID_MVC_UNMUTE_FIRST_CH,
	TEST_ID_MVC_CHANGE_LIN_CURVE_PARAMS,
	TEST_ID_MVC_CHANGE_POLY_CURVE_PARAMS,
	TEST_ID_MVC_DISABLE_ENABLE,
	TEST_ID_MVC_DISABLE_UNMUTE,
};

struct tegra210_audio_test_param {
	enum AHUB_MODULE_ID module;
	u32 cif;
	enum tegra210_audio_test_id test_id;
};

struct tegra210_audio_axbar_test_param {
	u32 rx_cif;
	u32 tx_cif;
};

struct tegra210_audio_admaif_test_param {
	int admaif_id;
	int test_id;
	unsigned int in_channels;
	unsigned int out_channels;
	unsigned int in_bps;
	unsigned int out_bps;
	unsigned int buffer_size;
	unsigned int periods;
};


struct tegra210_audio_ope_test_param {
	int ope_id;
	int test_id;
	int peq_to_drc;
};

#define TEGRA210_PEQ_MAX_BIQ_STAGES 12
#define PEQ_MAX_CHAN 8
struct tegra210_audio_peq_test_param {
	int peq_id;
	int test_id;
	int is_bypass;
	int chan_mask;
	unsigned int biquad_stages;
	unsigned int pre_gain[PEQ_MAX_CHAN];
	unsigned int post_gain[PEQ_MAX_CHAN];
	unsigned int pre_shift[PEQ_MAX_CHAN];
	unsigned int post_shift[PEQ_MAX_CHAN];
	unsigned int biquad_shifts[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
	unsigned int biquad_b0[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
	unsigned int biquad_b1[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
	unsigned int biquad_b2[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
	unsigned int biquad_a1[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
	unsigned int biquad_a2[PEQ_MAX_CHAN][TEGRA210_PEQ_MAX_BIQ_STAGES];
};

enum mbdrc_mode {
	MBDRC_FULL_BAND,
	MBDRC_DUAL_BAND,
	MBDRC_MULTI_BAND,
};

enum mbdrc_bands {
	MBDRC_BAND_LOW,
	MBDRC_BAND_MID,
	MBDRC_BAND_HIGH,
	MBDRC_BAND_NUM,
};

#define TEGRA210_MBDRC_MAX_BIQ_STAGES 8
struct tegra210_audio_mbdrc_test_param {
	int mbdrc_id;
	int test_id;
	int is_bypass;
	unsigned int master_volume;
	int mode;
	unsigned int rms_mode_peak;
	unsigned int rms_offset;
	int is_all_pass_tree;
	unsigned int shift_ctrl;
	unsigned int frame_size;
	unsigned int channel_mask;
	unsigned int fast_attack_factor;
	unsigned int fast_release_factor;
	unsigned int num_iir_stages[MBDRC_BAND_NUM];
	unsigned int in_attack_time_const[MBDRC_BAND_NUM];
	unsigned int in_release_time_const[MBDRC_BAND_NUM];
	unsigned int fast_attack_time_const[MBDRC_BAND_NUM];
	unsigned int in_threshold[MBDRC_BAND_NUM][4];
	unsigned int out_threshold[MBDRC_BAND_NUM][4];
	unsigned int ratio[MBDRC_BAND_NUM][5];
	unsigned int makeup_gain[MBDRC_BAND_NUM];
	unsigned int gain_init[MBDRC_BAND_NUM];
	unsigned int gain_attack_time_const[MBDRC_BAND_NUM];
	unsigned int gain_release_time_const[MBDRC_BAND_NUM];
	unsigned int fast_release_time_const[MBDRC_BAND_NUM];
	unsigned int biquad_b0[MBDRC_BAND_NUM][TEGRA210_MBDRC_MAX_BIQ_STAGES];
	unsigned int biquad_b1[MBDRC_BAND_NUM][TEGRA210_MBDRC_MAX_BIQ_STAGES];
	unsigned int biquad_b2[MBDRC_BAND_NUM][TEGRA210_MBDRC_MAX_BIQ_STAGES];
	unsigned int biquad_a1[MBDRC_BAND_NUM][TEGRA210_MBDRC_MAX_BIQ_STAGES];
	unsigned int biquad_a2[MBDRC_BAND_NUM][TEGRA210_MBDRC_MAX_BIQ_STAGES];
};

struct tegra210_audio_sfc_test_param {
	int sfc_id;
	int test_id;
	unsigned int in_rate;
	unsigned int out_rate;
};

#define TEGRA210_SPKPROT_ARF_MAX_BIQ 3
#define SPKPROT_MAX_CH 8
struct tegra210_audio_spkprot_test_param {
	int spkprot_id;
	int test_id;
	unsigned int mode;
	unsigned int bias;
	unsigned int channels;
	unsigned int bandfilter_type;
	unsigned int bandfilter_biquad_stages;
	unsigned int arfilter_biquad_stages;
	unsigned int spfilter_biquad_stages;
	unsigned int gain_l;
	unsigned int gain_h;
	int threshold;
	unsigned int reconfig;
	unsigned int pre_gain[SPKPROT_MAX_CH];
	unsigned int post_gain[SPKPROT_MAX_CH];
	unsigned int biquad_b0[SPKPROT_MAX_CH][TEGRA210_SPKPROT_ARF_MAX_BIQ];
	unsigned int biquad_b1[SPKPROT_MAX_CH][TEGRA210_SPKPROT_ARF_MAX_BIQ];
	unsigned int biquad_b2[SPKPROT_MAX_CH][TEGRA210_SPKPROT_ARF_MAX_BIQ];
	unsigned int biquad_a1[SPKPROT_MAX_CH][TEGRA210_SPKPROT_ARF_MAX_BIQ];
	unsigned int biquad_a2[SPKPROT_MAX_CH][TEGRA210_SPKPROT_ARF_MAX_BIQ];
};

struct tegra210_audio_i2s_test_param {
	int i2s_id;
	int test_id;
	unsigned int i2s_mode;
	unsigned int is_i2s_loopback;
	unsigned int is_i2s_master;
	unsigned int in_channels;
	unsigned int out_channels;
	unsigned int sample_rate;
	unsigned int in_bps;
	unsigned int out_bps;
};

struct tegra210_audio_amixer_test_param {
	int test_id;
	int rxpath_id;
	unsigned int act_th;
	unsigned int deact_th;
	unsigned int gain;
	unsigned int peakmeter_mode;
	unsigned int peakmeter_window_size;
};

enum T210_MVC_CURVE_TYPE {
	MVC_CURVE_POLYNOMIAL = 0,
	MVC_CURVE_LINEAR_RAMP = 1,
};

struct tegra210_audio_mvc_test_param {
	int mvc_id;
	int test_id;
	unsigned int bypass_en;
	unsigned int channels;
	unsigned int per_ch_ctrl_en;
	unsigned int mute_unmute_ctrl;
	unsigned int curve_type;
	unsigned int rounding_type;
	unsigned int init_vol[8];
	unsigned int target_vol[8];
	unsigned int duration_in_samples;
	unsigned int inv_duration_in_samples;
	unsigned int curve_split_points[2];
	unsigned int curve_coef[9];
	unsigned int peakmeter_mode;
	unsigned int peakmeter_window_size;
};

struct tegra210_audio_dmic_test_param {
	int test_id;
	int dmic_id;
	int osr;
	unsigned int bps;
	unsigned int channels;
	unsigned int sample_rate;
};

#define TEGRA_AUDIO_IN_SET_CONFIG	_IOW(TEGRA_AUDIO_MAGIC, 2, \
			const struct tegra_audio_in_config *)
#define TEGRA_AUDIO_IN_GET_CONFIG	_IOR(TEGRA_AUDIO_MAGIC, 3, \
			struct tegra_audio_in_config *)

#define TEGRA_AUDIO_IN_SET_NUM_BUFS	_IOW(TEGRA_AUDIO_MAGIC, 4, \
			const unsigned int *)
#define TEGRA_AUDIO_IN_GET_NUM_BUFS	_IOW(TEGRA_AUDIO_MAGIC, 5, \
			unsigned int *)
#define TEGRA_AUDIO_OUT_SET_NUM_BUFS	_IOW(TEGRA_AUDIO_MAGIC, 6, \
			const unsigned int *)
#define TEGRA_AUDIO_OUT_GET_NUM_BUFS	_IOW(TEGRA_AUDIO_MAGIC, 7, \
			unsigned int *)

#define TEGRA_AUDIO_OUT_FLUSH		_IO(TEGRA_AUDIO_MAGIC, 10)

#define TEGRA_AUDIO_BIT_FORMAT_DEFAULT 0
#define TEGRA_AUDIO_BIT_FORMAT_DSP 1
#define TEGRA_AUDIO_SET_BIT_FORMAT	_IOW(TEGRA_AUDIO_MAGIC, 11, \
			const unsigned int *)
#define TEGRA_AUDIO_GET_BIT_FORMAT	_IOR(TEGRA_AUDIO_MAGIC, 12, \
			unsigned int *)

#define DAM_SRC_START	_IOW(TEGRA_AUDIO_MAGIC, 13, struct dam_srate *)
#define DAM_SRC_STOP	_IO(TEGRA_AUDIO_MAGIC, 14)
#define DAM_MIXING_START	_IOW(TEGRA_AUDIO_MAGIC, 15, struct dam_srate *)
#define DAM_MIXING_STOP	_IO(TEGRA_AUDIO_MAGIC, 16)
#define DAM_SET_MIXING_FLAG	_IO(TEGRA_AUDIO_MAGIC, 17)
#define DAM_SET_MIXING_OUTPUT_I2S _IO(TEGRA_AUDIO_MAGIC, 18)
#define DAM_SET_MIXING_CH_WAIT_MODE _IO(TEGRA_AUDIO_MAGIC, 19)

#define I2S_START	_IOW(TEGRA_AUDIO_MAGIC, 21, struct i2s_pcm_format *)
#define I2S_STOP	_IOW(TEGRA_AUDIO_MAGIC, 22, struct i2s_pcm_format *)
#define I2S_DUAL_CODEC	_IOW(TEGRA_AUDIO_MAGIC, 23, unsigned int *)

#define SPDIF_START	_IO(TEGRA_AUDIO_MAGIC, 27)
#define SPDIF_STOP	_IO(TEGRA_AUDIO_MAGIC, 28)

#define AMX_APBIF_LOOPBACK_START	_IOW(TEGRA_AUDIO_MAGIC, 29, void *)
#define AMX_APBIF_LOOPBACK_STOP		_IOW(TEGRA_AUDIO_MAGIC, 30, void *)
#define AMX_CLOSE			_IOW(TEGRA_AUDIO_MAGIC, 31, void *)
#define AMX_I2S_SETUP			_IOW(TEGRA_AUDIO_MAGIC, 32, void *)
#define AMX_I2S_CLOSE			_IOW(TEGRA_AUDIO_MAGIC, 33, void *)

#define ADX_APBIF_LOOPBACK_START	_IOW(TEGRA_AUDIO_MAGIC, 34, void *)
#define ADX_APBIF_LOOPBACK_STOP		_IOW(TEGRA_AUDIO_MAGIC, 35, void *)
#define ADX_CLOSE			_IOW(TEGRA_AUDIO_MAGIC, 36, void *)
#define ADX_I2S_SETUP			_IOW(TEGRA_AUDIO_MAGIC, 37, void *)
#define ADX_I2S_CLOSE			_IOW(TEGRA_AUDIO_MAGIC, 38, void *)

#define AMX_ADX_APBIF_LOOPBACK_START	_IOW(TEGRA_AUDIO_MAGIC, 39, void *)
#define AMX_ADX_APBIF_LOOPBACK_STOP	_IOW(TEGRA_AUDIO_MAGIC, 40, void *)
#define AMX_ADX_CLOSE			_IOW(TEGRA_AUDIO_MAGIC, 41, void *)
#define AMX_ADX_I2S_SETUP		_IOW(TEGRA_AUDIO_MAGIC, 42, void *)
#define AMX_ADX_I2S_CLOSE		_IOW(TEGRA_AUDIO_MAGIC, 43, void *)
#define AMX_ADX_AUDIO_START		_IOW(TEGRA_AUDIO_MAGIC, 44, void *)
#define AMX_ADX_AUDIO_STOP		_IOW(TEGRA_AUDIO_MAGIC, 45, void *)

#define DMIC_CAPTURE_START		_IOW(TEGRA_AUDIO_MAGIC, 51, \
			struct dmic_params_t *)
#define DMIC_CAPTURE_STOP		_IO(TEGRA_AUDIO_MAGIC, 52)

#define TEGRA_AUDIO_TEST_START			_IO(TEGRA_AUDIO_MAGIC, 61)
#define TEGRA_AUDIO_TEST_SET_PARAMS		_IO(TEGRA_AUDIO_MAGIC, 62)
#define TEGRA_AUDIO_TEST_GET_PARAMS		_IO(TEGRA_AUDIO_MAGIC, 63)
#define TEGRA_AUDIO_TEST_STOP			_IO(TEGRA_AUDIO_MAGIC, 64)

/* Tegra210 AHUB test specific macros */
#define TEGRA210_AUDIO_TEST_EXEC	_IOW(TEGRA_AUDIO_MAGIC, 71, \
				    struct tegra210_audio_test_param *)
#define TEGRA210_AUDIO_AXBAR_CONNECT	_IOW(TEGRA_AUDIO_MAGIC, 72, \
				    struct tegra210_audio_axbar_test_param *)
#define TEGRA210_AUDIO_ADMAIF_TEST_PARAM _IOW(TEGRA_AUDIO_MAGIC, 73, \
				    struct tegra210_audio_admaif_test_param *)
#define TEGRA210_AUDIO_SFC_TEST_PARAM	_IOW(TEGRA_AUDIO_MAGIC, 74, \
				    struct tegra210_audio_sfc_test_param *)
#define TEGRA210_AUDIO_OPE_TEST_PARAM    _IOW(TEGRA_AUDIO_MAGIC, 75, \
				    struct tegra210_audio_ope_test_param *)
#define TEGRA210_AUDIO_SPKPROT_TEST_PARAM    _IOW(TEGRA_AUDIO_MAGIC, 76, \
				struct tegra210_audio_spkprot_test_param *)
#define TEGRA210_AUDIO_PEQ_TEST_PARAM	_IOW(TEGRA_AUDIO_MAGIC, 77, \
				    struct tegra210_audio_peq_test_param *)
#define TEGRA210_AUDIO_MBDRC_TEST_PARAM	_IOW(TEGRA_AUDIO_MAGIC, 78, \
				    struct tegra210_audio_mbdrc_test_param *)
#define TEGRA210_AUDIO_I2S_TEST_PARAM    _IOW(TEGRA_AUDIO_MAGIC, 79, \
				struct tegra210_audio_i2s_test_param *)
#define TEGRA210_AUDIO_AMIXER_TEST_PARAM	_IOW(TEGRA_AUDIO_MAGIC, 80, \
				    struct tegra210_audio_amixer_test_param *)
#define TEGRA210_AUDIO_DMIC_TEST_PARAM		_IOW(TEGRA_AUDIO_MAGIC, 81, \
				    struct tegra210_audio_dmic_test_param *)
#define TEGRA210_AUDIO_MVC_TEST_PARAM	_IOW(TEGRA_AUDIO_MAGIC, 82, \
				    struct tegra210_audio_mvc_test_param *)

#ifdef CONFIG_SND_SOC_TEGRA
extern bool tegra_is_voice_call_active(void);
#else
static inline bool tegra_is_voice_call_active(void)
{
	return false;
}
#endif

#endif/*_CPCAP_AUDIO_H*/
