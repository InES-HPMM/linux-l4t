/*
 * tegra124_virt_alt_xbar.c
 *
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 , __func__* version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra_pcm_alt.h"
#include "tegra124_virt_alt_xbar.h"
#include "tegra124_virt_alt_ivc.h"

#define DRV_NAME	"tegra124-virt-xbar-control"

static struct nvaudio_ivc_ctxt *hivc_client;

static const char * const tegra124_virt_xbar_mux_texts[] = {
	"None",
	"APBIF0",
	"APBIF1",
	"APBIF2",
	"APBIF3",
	"I2S0",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"DAM0",
	"DAM1",
	"DAM2",
	"SPDIF",
	/* index 0..13 above are used on Tegra30 */
	"APBIF4",
	"APBIF5",
	"APBIF6",
	"APBIF7",
	"APBIF8",
	"APBIF9",
	"AMX0",
	"ADX0-0",
	"ADX0-1",
	"ADX0-2",
	"ADX0-3",
	/* index 0..24 above are used on Tegra114 */
	"AMX1",
	"ADX1-0",
	"ADX1-1",
	"ADX1-2",
	"ADX1-3",
	"AFC0",
	"AFC1",
	"AFC2",
	"AFC3",
	"AFC4",
	"AFC5",
	/* index 0..35 above are used on Tegra124 */
};

#define MUX_VALUE(npart, nbit) (1 + nbit + npart * 32)
static const int tegra124_virt_xbar_mux_values[] = {
	0,
	MUX_VALUE(0, 0),
	MUX_VALUE(0, 1),
	MUX_VALUE(0, 2),
	MUX_VALUE(0, 3),
	MUX_VALUE(0, 4),
	MUX_VALUE(0, 5),
	MUX_VALUE(0, 6),
	MUX_VALUE(0, 7),
	MUX_VALUE(0, 8),
	MUX_VALUE(0, 9),
	MUX_VALUE(0, 10),
	MUX_VALUE(0, 11),
	MUX_VALUE(0, 12),
	/* index 0..13 above are used on Tegra30 */
	MUX_VALUE(0, 14),
	MUX_VALUE(0, 15),
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
	MUX_VALUE(0, 20),
	MUX_VALUE(0, 21),
	MUX_VALUE(0, 22),
	MUX_VALUE(0, 23),
	MUX_VALUE(0, 24),
	/* index 0..24 above are used on Tegra114 */
	MUX_VALUE(1, 0),
	MUX_VALUE(1, 1),
	MUX_VALUE(1, 2),
	MUX_VALUE(1, 3),
	MUX_VALUE(1, 4),
	MUX_VALUE(1, 5),
	MUX_VALUE(1, 6),
	MUX_VALUE(1, 7),
	MUX_VALUE(1, 8),
	MUX_VALUE(1, 9),
	MUX_VALUE(1, 10),
	/* index 0..35 above are used on Tegra124 */
};

int tegra124_virt_xbar_get_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	int err;
	struct nvaudio_ivc_msg msg;
	unsigned int bit_pos = 0, i;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd				= NVAUDIO_XBAR_GET_ROUTE;
	msg.params.xbar_info.rx_reg	= e->reg;

	err = nvaudio_ivc_send(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("error on ivc_send\n");
		goto fail;
	}
	/* TODO: wait for the signal.  */
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	err = nvaudio_ivc_receive_cmd(hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg),
				NVAUDIO_XBAR_GET_ROUTE);
	if (err < 0) {
		pr_err("error on ivc_receive\n");
		goto fail;
	}
	if (msg.cmd == NVAUDIO_XBAR_GET_ROUTE &&
			msg.params.xbar_info.rx_reg == e->reg) {
		bit_pos =  msg.params.xbar_info.bit_pos;
		for (i = 0; i < e->max; i++) {
			if (bit_pos == e->values[i]) {
				ucontrol->value.enumerated.item[0] = i;
				break;
			}
		}
	} else
		pr_err("invalid reply from server\n");
fail:
	return 0;
}

int tegra124_virt_xbar_put_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd				= NVAUDIO_XBAR_SET_ROUTE;
	msg.params.xbar_info.rx_reg	= e->reg;
	msg.params.xbar_info.tx_value	= e->values[item[0]];

	err = nvaudio_ivc_send(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");

	return 0;
}

#define MUX_REG(id) (TEGRA_AHUB_AUDIO_RX_STRIDE * (id))

#define SOC_VALUE_ENUM_WIDE(xreg, shift, xmax, xtexts, xvalues) \
{	.reg = xreg, .shift_l = shift, .shift_r = shift, \
	.max = xmax, .texts = xtexts, .values = xvalues, \
	.mask = xmax ? roundup_pow_of_two(xmax) - 1 : 0}

#define SOC_VALUE_ENUM_WIDE_DECL(name, xreg, shift, \
		xtexts, xvalues) \
	struct soc_enum name = SOC_VALUE_ENUM_WIDE(xreg, shift, \
					ARRAY_SIZE(xtexts), xtexts, xvalues)

#define MUX_ENUM_CTRL_DECL(ename, id) \
	SOC_VALUE_ENUM_WIDE_DECL(ename##_enum, MUX_REG(id), 0,	\
			tegra124_virt_xbar_mux_texts, \
			tegra124_virt_xbar_mux_values);	\
	static const struct snd_kcontrol_new ename##_control = \
		SOC_DAPM_ENUM_EXT("Route", ename##_enum,\
				tegra124_virt_xbar_get_value_enum,\
				tegra124_virt_xbar_put_value_enum)

MUX_ENUM_CTRL_DECL(vapbif0_tx, 0x00);
MUX_ENUM_CTRL_DECL(vapbif1_tx, 0x01);
MUX_ENUM_CTRL_DECL(vapbif2_tx, 0x02);
MUX_ENUM_CTRL_DECL(vapbif3_tx, 0x03);
MUX_ENUM_CTRL_DECL(vi2s0_tx, 0x04);
MUX_ENUM_CTRL_DECL(vi2s1_tx, 0x05);
MUX_ENUM_CTRL_DECL(vi2s2_tx, 0x06);
MUX_ENUM_CTRL_DECL(vi2s3_tx, 0x07);
MUX_ENUM_CTRL_DECL(vi2s4_tx, 0x08);
MUX_ENUM_CTRL_DECL(vdam00_tx, 0x09);
MUX_ENUM_CTRL_DECL(vdam01_tx, 0x0a);
MUX_ENUM_CTRL_DECL(vdam10_tx, 0x0b);
MUX_ENUM_CTRL_DECL(vdam11_tx, 0x0c);
MUX_ENUM_CTRL_DECL(vdam20_tx, 0x0d);
MUX_ENUM_CTRL_DECL(vdam21_tx, 0x0e);
MUX_ENUM_CTRL_DECL(vspdif_tx, 0x0f);
/* above controls are used on Tegra30 */
MUX_ENUM_CTRL_DECL(vapbif4_tx, 0x11);
MUX_ENUM_CTRL_DECL(vapbif5_tx, 0x12);
MUX_ENUM_CTRL_DECL(vapbif6_tx, 0x13);
MUX_ENUM_CTRL_DECL(vapbif7_tx, 0x14);
MUX_ENUM_CTRL_DECL(vapbif8_tx, 0x15);
MUX_ENUM_CTRL_DECL(vapbif9_tx, 0x16);
MUX_ENUM_CTRL_DECL(vamx00_tx, 0x17);
MUX_ENUM_CTRL_DECL(vamx01_tx, 0x18);
MUX_ENUM_CTRL_DECL(vamx02_tx, 0x19);
MUX_ENUM_CTRL_DECL(vamx03_tx, 0x1a);
MUX_ENUM_CTRL_DECL(vadx0_tx, 0x1b);
/* above controls are used on Tegra114 */
MUX_ENUM_CTRL_DECL(vamx10_tx, 0x1e);
MUX_ENUM_CTRL_DECL(vamx11_tx, 0x1f);
MUX_ENUM_CTRL_DECL(vamx12_tx, 0x20);
MUX_ENUM_CTRL_DECL(vamx13_tx, 0x21);
MUX_ENUM_CTRL_DECL(vadx1_tx, 0x22);
MUX_ENUM_CTRL_DECL(vafc0_tx, 0x23);
MUX_ENUM_CTRL_DECL(vafc1_tx, 0x24);
MUX_ENUM_CTRL_DECL(vafc2_tx, 0x25);
MUX_ENUM_CTRL_DECL(vafc3_tx, 0x26);
MUX_ENUM_CTRL_DECL(vafc4_tx, 0x27);
MUX_ENUM_CTRL_DECL(vafc5_tx, 0x28);
/* above controls are used on Tegra124 */

#define WIDGETS(sname, ename) \
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_VIRT_MUX(sname " Mux", SND_SOC_NOPM, 0, 0, \
	&ename##_control)

#define TX_WIDGETS(sname) \
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0)

/*
 * The number of entries in, and order of, this array is closely tied to the
 * calculation of tegra124_xbar_codec.num_dapm_widgets near the end of
 * tegra30_xbar_probe()
 */
static const struct snd_soc_dapm_widget tegra124_virt_xbar_widgets[] = {
	WIDGETS("APBIF0", vapbif0_tx),
	WIDGETS("APBIF1", vapbif1_tx),
	WIDGETS("APBIF2", vapbif2_tx),
	WIDGETS("APBIF3", vapbif3_tx),
	WIDGETS("I2S0", vi2s0_tx),
	WIDGETS("I2S1", vi2s1_tx),
	WIDGETS("I2S2", vi2s2_tx),
	WIDGETS("I2S3", vi2s3_tx),
	WIDGETS("I2S4", vi2s4_tx),
	WIDGETS("DAM0-0", vdam00_tx),
	WIDGETS("DAM0-1", vdam01_tx),
	WIDGETS("DAM1-0", vdam10_tx),
	WIDGETS("DAM1-1", vdam11_tx),
	WIDGETS("DAM2-0", vdam20_tx),
	WIDGETS("DAM2-1", vdam21_tx),
	WIDGETS("SPDIF", vspdif_tx),
	TX_WIDGETS("DAM0"),
	TX_WIDGETS("DAM1"),
	TX_WIDGETS("DAM2"),
	/* index 0..18 above are used on Tegra30 */
	WIDGETS("APBIF4", vapbif4_tx),
	WIDGETS("APBIF5", vapbif5_tx),
	WIDGETS("APBIF6", vapbif6_tx),
	WIDGETS("APBIF7", vapbif7_tx),
	WIDGETS("APBIF8", vapbif8_tx),
	WIDGETS("APBIF9", vapbif9_tx),
	WIDGETS("AMX0-0", vamx00_tx),
	WIDGETS("AMX0-1", vamx01_tx),
	WIDGETS("AMX0-2", vamx02_tx),
	WIDGETS("AMX0-3", vamx03_tx),
	WIDGETS("ADX0", vadx0_tx),
	TX_WIDGETS("AMX0"),
	TX_WIDGETS("ADX0-0"),
	TX_WIDGETS("ADX0-1"),
	TX_WIDGETS("ADX0-2"),
	TX_WIDGETS("ADX0-3"),
	/* index 0..34 above are used on Tegra114 */
	WIDGETS("AMX1-0", vamx10_tx),
	WIDGETS("AMX1-1", vamx11_tx),
	WIDGETS("AMX1-2", vamx12_tx),
	WIDGETS("AMX1-3", vamx13_tx),
	WIDGETS("ADX1", vadx1_tx),
	WIDGETS("AFC0", vafc0_tx),
	WIDGETS("AFC1", vafc1_tx),
	WIDGETS("AFC2", vafc2_tx),
	WIDGETS("AFC3", vafc3_tx),
	WIDGETS("AFC4", vafc4_tx),
	WIDGETS("AFC5", vafc5_tx),
	TX_WIDGETS("AMX1"),
	TX_WIDGETS("ADX1-0"),
	TX_WIDGETS("ADX1-1"),
	TX_WIDGETS("ADX1-2"),
	TX_WIDGETS("ADX1-3"),
	/* index 0..50 above are used on Tegra124 */
};

/* These routes used on Tegra30, Tegra114, Tegra124 */
#define TEGRA30_ROUTES(name)					\
	{ name " RX",	   NULL,	name " Receive"},	\
	{ name " Transmit", NULL,	name " TX"},		\
	{ name " TX",	   NULL,	name " Mux" },		\
	{ name " Mux",	  "APBIF0",	"APBIF0 RX" },		\
	{ name " Mux",	  "APBIF1",	"APBIF1 RX" },		\
	{ name " Mux",	  "APBIF2",	"APBIF2 RX" },		\
	{ name " Mux",	  "APBIF3",	"APBIF3 RX" },		\
	{ name " Mux",	  "I2S0",	"I2S0 RX" },		\
	{ name " Mux",	  "I2S1",	"I2S1 RX" },		\
	{ name " Mux",	  "I2S2",	"I2S2 RX" },		\
	{ name " Mux",	  "I2S3",	"I2S3 RX" },		\
	{ name " Mux",	  "I2S4",	"I2S4 RX" },		\
	{ name " Mux",	  "DAM0",	"DAM0 RX" },		\
	{ name " Mux",	  "DAM1",	"DAM1 RX" },		\
	{ name " Mux",	  "DAM2",	"DAM2 RX" },		\
	{ name " Mux",	  "SPDIF",	"SPDIF RX" },

/* These routes used on Tegra114 and Tegra124 */
#define TEGRA114_ROUTES(name)					\
	{ name " Mux",	  "APBIF4",	"APBIF4 RX" },		\
	{ name " Mux",	  "APBIF5",	"APBIF5 RX" },		\
	{ name " Mux",	  "APBIF6",	"APBIF6 RX" },		\
	{ name " Mux",	  "APBIF7",	"APBIF7 RX" },		\
	{ name " Mux",	  "APBIF8",	"APBIF8 RX" },		\
	{ name " Mux",	  "APBIF9",	"APBIF9 RX" },		\
	{ name " Mux",	  "AMX0",	"AMX0 RX" },		\
	{ name " Mux",	  "ADX0-0",	"ADX0-0 RX" },		\
	{ name " Mux",	  "ADX0-1",	"ADX0-1 RX" },		\
	{ name " Mux",	  "ADX0-2",	"ADX0-2 RX" },		\
	{ name " Mux",	  "ADX0-3",	"ADX0-3 RX" },

#define AMX_OUT_ADX_IN_ROUTES(name)				\
	{ name " RX",	   NULL,	name " Receive"},	\
	{ name " Transmit", NULL,	name " TX"},

/* These routes used on Tegra124 only */
#define TEGRA124_ROUTES(name)					\
	{ name " Mux",	  "AMX1",	"AMX1 RX" },		\
	{ name " Mux",	  "ADX1-0",	"ADX1-0 RX" },	\
	{ name " Mux",	  "ADX1-1",	"ADX1-1 RX" },	\
	{ name " Mux",	  "ADX1-2",	"ADX1-2 RX" },	\
	{ name " Mux",	  "ADX1-3",	"ADX1-3 RX" },	\
	{ name " Mux",	  "AFC0",		"AFC0 RX" },		\
	{ name " Mux",	  "AFC1",		"AFC1 RX" },		\
	{ name " Mux",	  "AFC2",		"AFC2 RX" },		\
	{ name " Mux",	  "AFC3",		"AFC3 RX" },		\
	{ name " Mux",	  "AFC4",		"AFC4 RX" },		\
	{ name " Mux",	  "AFC5",		"AFC5 RX" },		\

/* These routes used on Tegra30, Tegra114, Tegra124 */
#define TEGRA_VIRT_ROUTES(name)					\
	{ name " Mux",	  "APBIF0",	"APBIF0 RX" },		\
	{ name " Mux",	  "APBIF1",	"APBIF1 RX" },		\
	{ name " Mux",	  "APBIF2",	"APBIF2 RX" },		\
	{ name " Mux",	  "APBIF3",	"APBIF3 RX" },		\
	{ name " Mux",	  "APBIF4",	"APBIF4 RX" },		\
	{ name " Mux",	  "APBIF5",	"APBIF5 RX" },		\
	{ name " Mux",	  "APBIF6",	"APBIF6 RX" },		\
	{ name " Mux",	  "APBIF7",	"APBIF7 RX" },		\
	{ name " Mux",	  "APBIF8",	"APBIF8 RX" },		\
	{ name " Mux",	  "APBIF9",	"APBIF9 RX" },
/*
 * The number of entries in, and order of, this array is closely tied to the
 * calculation of tegra124_xbar_codec.num_dapm_routes near the end of
 * tegra124_virt_alt_xbar_probe()
 */
static const struct snd_soc_dapm_route tegra124_virt_xbar_routes[] = {
	TEGRA30_ROUTES("APBIF0")
	TEGRA30_ROUTES("APBIF1")
	TEGRA30_ROUTES("APBIF2")
	TEGRA30_ROUTES("APBIF3")
	TEGRA30_ROUTES("I2S0")
	TEGRA30_ROUTES("I2S1")
	TEGRA30_ROUTES("I2S2")
	TEGRA30_ROUTES("I2S3")
	TEGRA30_ROUTES("I2S4")
	TEGRA30_ROUTES("DAM0-0")
	TEGRA30_ROUTES("DAM0-1")
	TEGRA30_ROUTES("DAM1-0")
	TEGRA30_ROUTES("DAM1-1")
	TEGRA30_ROUTES("DAM2-0")
	TEGRA30_ROUTES("DAM2-1")
	AMX_OUT_ADX_IN_ROUTES("DAM0")
	AMX_OUT_ADX_IN_ROUTES("DAM1")
	AMX_OUT_ADX_IN_ROUTES("DAM2")
	TEGRA30_ROUTES("SPDIF")
	/* above routes are used on Tegra30 */
	TEGRA30_ROUTES("APBIF4")
	TEGRA30_ROUTES("APBIF5")
	TEGRA30_ROUTES("APBIF6")
	TEGRA30_ROUTES("APBIF7")
	TEGRA30_ROUTES("APBIF8")
	TEGRA30_ROUTES("APBIF9")
	TEGRA30_ROUTES("AMX0-0")
	TEGRA30_ROUTES("AMX0-1")
	TEGRA30_ROUTES("AMX0-2")
	TEGRA30_ROUTES("AMX0-3")
	TEGRA30_ROUTES("ADX0")
	TEGRA114_ROUTES("APBIF0")
	TEGRA114_ROUTES("APBIF1")
	TEGRA114_ROUTES("APBIF2")
	TEGRA114_ROUTES("APBIF3")
	TEGRA114_ROUTES("I2S0")
	TEGRA114_ROUTES("I2S1")
	TEGRA114_ROUTES("I2S2")
	TEGRA114_ROUTES("I2S3")
	TEGRA114_ROUTES("I2S4")
	TEGRA114_ROUTES("DAM0-0")
	TEGRA114_ROUTES("DAM0-1")
	TEGRA114_ROUTES("DAM1-0")
	TEGRA114_ROUTES("DAM1-1")
	TEGRA114_ROUTES("DAM2-0")
	TEGRA114_ROUTES("DAM2-1")
	TEGRA114_ROUTES("SPDIF")
	TEGRA114_ROUTES("APBIF4")
	TEGRA114_ROUTES("APBIF5")
	TEGRA114_ROUTES("APBIF6")
	TEGRA114_ROUTES("APBIF7")
	TEGRA114_ROUTES("APBIF8")
	TEGRA114_ROUTES("APBIF9")
	TEGRA114_ROUTES("AMX0-0")
	TEGRA114_ROUTES("AMX0-1")
	TEGRA114_ROUTES("AMX0-2")
	TEGRA114_ROUTES("AMX0-3")
	TEGRA114_ROUTES("ADX0")
	AMX_OUT_ADX_IN_ROUTES("AMX0")
	AMX_OUT_ADX_IN_ROUTES("ADX0-0")
	AMX_OUT_ADX_IN_ROUTES("ADX0-1")
	AMX_OUT_ADX_IN_ROUTES("ADX0-2")
	AMX_OUT_ADX_IN_ROUTES("ADX0-3")
	/* above routes are used on Tegra114 */
	TEGRA30_ROUTES("AMX1-0")
	TEGRA30_ROUTES("AMX1-1")
	TEGRA30_ROUTES("AMX1-2")
	TEGRA30_ROUTES("AMX1-3")
	TEGRA30_ROUTES("ADX1")
	TEGRA30_ROUTES("AFC0")
	TEGRA30_ROUTES("AFC1")
	TEGRA30_ROUTES("AFC2")
	TEGRA30_ROUTES("AFC3")
	TEGRA30_ROUTES("AFC4")
	TEGRA30_ROUTES("AFC5")
	TEGRA114_ROUTES("AMX1-0")
	TEGRA114_ROUTES("AMX1-1")
	TEGRA114_ROUTES("AMX1-2")
	TEGRA114_ROUTES("AMX1-3")
	TEGRA114_ROUTES("ADX1")
	TEGRA114_ROUTES("AFC0")
	TEGRA114_ROUTES("AFC1")
	TEGRA114_ROUTES("AFC2")
	TEGRA114_ROUTES("AFC3")
	TEGRA114_ROUTES("AFC4")
	TEGRA114_ROUTES("AFC5")
	TEGRA124_ROUTES("APBIF0")
	TEGRA124_ROUTES("APBIF1")
	TEGRA124_ROUTES("APBIF2")
	TEGRA124_ROUTES("APBIF3")
	TEGRA124_ROUTES("I2S0")
	TEGRA124_ROUTES("I2S1")
	TEGRA124_ROUTES("I2S2")
	TEGRA124_ROUTES("I2S3")
	TEGRA124_ROUTES("I2S4")
	TEGRA124_ROUTES("DAM0-0")
	TEGRA124_ROUTES("DAM0-1")
	TEGRA124_ROUTES("DAM1-0")
	TEGRA124_ROUTES("DAM1-1")
	TEGRA124_ROUTES("DAM2-0")
	TEGRA124_ROUTES("DAM2-1")
	TEGRA124_ROUTES("SPDIF")
	TEGRA124_ROUTES("APBIF4")
	TEGRA124_ROUTES("APBIF5")
	TEGRA124_ROUTES("APBIF6")
	TEGRA124_ROUTES("APBIF7")
	TEGRA124_ROUTES("APBIF8")
	TEGRA124_ROUTES("APBIF9")
	TEGRA124_ROUTES("AMX0-0")
	TEGRA124_ROUTES("AMX0-1")
	TEGRA124_ROUTES("AMX0-2")
	TEGRA124_ROUTES("AMX0-3")
	TEGRA124_ROUTES("ADX0")
	TEGRA124_ROUTES("AMX1-0")
	TEGRA124_ROUTES("AMX1-1")
	TEGRA124_ROUTES("AMX1-2")
	TEGRA124_ROUTES("AMX1-3")
	TEGRA124_ROUTES("ADX1")
	TEGRA124_ROUTES("AFC0")
	TEGRA124_ROUTES("AFC1")
	TEGRA124_ROUTES("AFC2")
	TEGRA124_ROUTES("AFC3")
	TEGRA124_ROUTES("AFC4")
	TEGRA124_ROUTES("AFC5")
	AMX_OUT_ADX_IN_ROUTES("AMX1")
	AMX_OUT_ADX_IN_ROUTES("ADX1-0")
	AMX_OUT_ADX_IN_ROUTES("ADX1-1")
	AMX_OUT_ADX_IN_ROUTES("ADX1-2")
	AMX_OUT_ADX_IN_ROUTES("ADX1-3")
	/* above routes are used on Tegra124 */
};

static int tegra124_virt_alt_xbar_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver tegra124_virt_alt_xbar_codec = {
	.probe = tegra124_virt_alt_xbar_codec_probe,
	.dapm_widgets = tegra124_virt_xbar_widgets,
	.dapm_routes = tegra124_virt_xbar_routes,
	.num_dapm_widgets = ARRAY_SIZE(tegra124_virt_xbar_widgets),
	.num_dapm_routes = ARRAY_SIZE(tegra124_virt_xbar_routes),
	.idle_bias_off = 1,
};

#define DAI(sname)						\
	{							\
		.name = #sname,					\
		.playback = {					\
			.stream_name = #sname " Receive",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = #sname " Transmit",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
	}

static struct snd_soc_dai_driver tegra124_virt_alt_xbar_dais[] = {
	DAI(APBIF0),
	DAI(APBIF1),
	DAI(APBIF2),
	DAI(APBIF3),
	DAI(I2S0),
	DAI(I2S1),
	DAI(I2S2),
	DAI(I2S3),
	DAI(I2S4),
	DAI(DAM0),
	DAI(DAM1),
	DAI(DAM2),
	DAI(DAM0-0),
	DAI(DAM0-1),
	DAI(DAM1-0),
	DAI(DAM1-1),
	DAI(DAM2-0),
	DAI(DAM2-1),
	DAI(SPDIF),
	DAI(APBIF4),
	DAI(APBIF5),
	DAI(APBIF6),
	DAI(APBIF7),
	DAI(APBIF8),
	DAI(APBIF9),
	DAI(AMX0),
	DAI(AMX0-0),
	DAI(AMX0-1),
	DAI(AMX0-2),
	DAI(AMX0-3),
	DAI(ADX0-0),
	DAI(ADX0-1),
	DAI(ADX0-2),
	DAI(ADX0-3),
	DAI(ADX0),
	DAI(AMX1),
	DAI(AMX1-0),
	DAI(AMX1-1),
	DAI(AMX1-2),
	DAI(AMX1-3),
	DAI(ADX1-0),
	DAI(ADX1-1),
	DAI(ADX1-2),
	DAI(ADX1-3),
	DAI(ADX1),
	DAI(AFC0),
	DAI(AFC1),
	DAI(AFC2),
	DAI(AFC3),
	DAI(AFC4),
	DAI(AFC5),
};

struct of_dev_auxdata tegra124_virt_alt_xbar_auxdata[] = {
	OF_DEV_AUXDATA("nvidia,tegra124-virt-control", 0x0,
				"tegra124-virt-control", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-alt-dam", 0x70302000,
				"tegra124-dam.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-alt-dam", 0x70302200,
				"tegra124-dam.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-alt-dam", 0x70302400,
				"tegra124-dam.2", NULL),
	{}
};

static const struct snd_soc_component_driver tegra124_virt_xbar_dai_driver = {
	.name		= DRV_NAME,
};

static const struct of_device_id tegra124_virt_alt_xbar_of_match[] = {
	{ .compatible = "nvidia,tegra124-virt-xbar-control",},
	{},
};

static int tegra124_virt_xbar_probe(struct platform_device *pdev)
{
	int ret;
	struct  tegra124_virt_xbar *xbar_data;

	xbar_data = devm_kzalloc(&pdev->dev,
					sizeof(*xbar_data),	GFP_KERNEL);
	if (!xbar_data) {
		dev_err(&pdev->dev, "Can't allocate tegra124_virt_xbar\n");
		ret = -ENOMEM;
		goto err;
	}

	xbar_data->hivc_client = nvaudio_ivc_alloc_ctxt(&pdev->dev);
	hivc_client = xbar_data->hivc_client;

	dev_set_drvdata(&pdev->dev, xbar_data);

	ret = snd_soc_register_codec(&pdev->dev,
				&tegra124_virt_alt_xbar_codec,
				tegra124_virt_alt_xbar_dais,
				ARRAY_SIZE(tegra124_virt_alt_xbar_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err;
	}

	of_platform_populate(pdev->dev.of_node, NULL,
					tegra124_virt_alt_xbar_auxdata,
					&pdev->dev);

	return 0;
err:
	return ret;
}

static int tegra124_virt_xbar_remove(struct platform_device *pdev)
{

	tegra_alt_pcm_platform_unregister(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static struct platform_driver tegra124_virt_alt_xbar_driver = {
	.probe = tegra124_virt_xbar_probe,
	.remove = tegra124_virt_xbar_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table =
			of_match_ptr(tegra124_virt_alt_xbar_of_match),
	},
};
module_platform_driver(tegra124_virt_alt_xbar_driver);

MODULE_AUTHOR("Ranjith Kannikara <rkannikara@nvidia.com>");
MODULE_DESCRIPTION("Tegra124 virt xbar control driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, tegra124_virt_alt_xbar_of_match);
MODULE_ALIAS("platform:" DRV_NAME);
