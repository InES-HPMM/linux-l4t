/*
 * drivers/platform/tegra/tegra13_edp.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#ifdef CONFIG_ARCH_TEGRA_13x_SOC

#include <mach/edp.h>

#ifdef CONFIG_SYSEDP_FRAMEWORK
static struct tegra_sysedp_corecap t132_sysedp_corecap[] = {
	/*mW	 CPU intensive load	   GPU intensive load	 */
	/*mW	 budget	 gpu(khz) mem(khz)  budget  gpu(khz) mem(khz) pthrot(mW)*/
	{5000,  {3000,  180000, 933000}, {3000,  180000, 933000}, 2604 },
	{6000,  {4000,  180000, 933000}, {3500,  252000, 933000}, 3374 },
	{7000,  {5000,  180000, 933000}, {3500,  324000, 933000}, 3840 },
	{8000,  {5000,  252000, 933000}, {3500,  396000, 933000}, 4265 },
	{9000,  {5000,  396000, 933000}, {3500,  468000, 933000}, 4849 },
	{10000, {6000,  324000, 933000}, {3500,  540000, 933000}, 5710 },
	{11000, {7000,  252000, 933000}, {4500,  540000, 933000}, 6159 },
	{12000, {7000,  396000, 933000}, {5500,  540000, 933000}, 7009 },
	{13000, {8000,  252000, 933000}, {6000,  540000, 933000}, 6765 },
	{14000, {10000, 252000, 933000}, {6000,  612000, 933000}, 8491 },
	{15000, {10000, 324000, 933000}, {6000,  684000, 933000}, 9185 },
	{16000, {11000, 252000, 933000}, {6000,  708000, 933000}, 9640 },
	{17000, {11000, 396000, 933000}, {6000,  756000, 933000}, 10508 },
	{18000, {12000, 324000, 933000}, {7000,  756000, 933000}, 11149 },
	{19000, {13000, 252000, 933000}, {7000,  804000, 933000}, 11745 },
	{20000, {13000, 324000, 933000}, {7500,  804000, 933000}, 12170 },
	{21000, {14000, 252000, 933000}, {6500,  853000, 933000}, 12570 },
	{22000, {14000, 396000, 933000}, {9000,  853000, 933000}, 13421 },
	{23000, {15000, 252000, 933000}, {9000,  853000, 933000}, 13453 },
	{24000, {15000, 396000, 933000}, {9500,  853000, 933000}, 14304 },
	{25000, {16000, 324000, 933000}, {10000, 853000, 933000}, 15147 },
	{26000, {16800, 324000, 933000}, {11000, 853000, 933000}, 16530 },
	{27000, {16800, 468000, 933000}, {11000, 853000, 933000}, 17282 },
	{28000, {16800, 540000, 933000}, {11500, 853000, 933000}, 17282 },
	{29000, {16800, 540000, 933000}, {13000, 853000, 933000}, 18400 },
	{30000, {16800, 648000, 933000}, {13000, 853000, 933000}, 19238 },
	{31000, {16800, 708000, 933000}, {13500, 853000, 933000}, 20064 },
	{32000, {16800, 708000, 933000}, {14000, 853000, 933000}, 20064 },
	{33000, {16800, 756000, 933000}, {14500, 853000, 933000}, 20947 },
};

struct tegra_sysedp_corecap *tegra_get_sysedp_corecap(unsigned int *sz)
{
	BUG_ON(sz == NULL);
	*sz = ARRAY_SIZE(t132_sysedp_corecap);
	return t132_sysedp_corecap;
}
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS

#ifdef CONFIG_TEGRA_GPU_EDP
struct tegra_edp_gpu_powermodel_params *tegra13x_get_gpu_powermodel_params(void)
{
	/* T13x has same GPU power model params as T12x */
	return tegra12x_get_gpu_powermodel_params();
}
#endif

#define LEAKAGE_CONSTS_IJK_COMMON					\
{									\
	/* i = 0 */							\
	{ {     37941815,   -34693402,    7820012,   -441775, },	\
	  {   -120688312,   110469778,  -24921798,   1406135, },	\
	  {    125400868,  -114967356,   25961411,  -1462457, },	\
	  {    -42517654,    39125172,   -8844968,    497219, },	\
	},								\
	/* i = 1 */							\
	{ {    -50381367,    44032432,   -9833725,    555725, },	\
	  {    160275950,  -139793882,   31257629,  -1764975, },	\
	  {   -166543984,   144936902,  -32453803,   1830681, },	\
	  {     56516540,   -49076991,   11008971,   -619802, },	\
	},								\
	/* i = 2 */							\
	{ {     17384788,   -15152169,    3354364,   -189265, },	\
	  {    -55185216,    48061420,  -10652770,    601257, },	\
	  {     57179883,   -49766548,   11046206,   -623597, },	\
	  {    -19334275,    16829382,   -3741212,    211201, },	\
	},								\
	/* i = 3 */							\
	{ {     -1678751,     1460723,    -322310,     18193, },	\
	  {      5323806,    -4629524,    1022876,    -57779, },	\
	  {     -5509910,     4788629,   -1059681,     59896, },	\
	  {      1861103,    -1617204,     358518,    -20274, },	\
	},								\
}

/* On T132, offlining core1 within Linux is not sufficient to guarantee that
 * it is powered-off all the time. Until we have a sufficient guarantee, we
 * use EDP tables to account for partial times core-1 could be ONLINE.
 *
 * The one-core value in dynamic_constants[] is adjusted to account for this.
 * The one-core value in leakage_constants[] is set to the two-core value.
 *
 * The original 'pure single-core' values for these two arrays are saved in
 * comments for reference.
 */
#define EDP_PARAMS_COMMON_PART						\
{									\
	.temp_scaled      = 10,						\
	.dyn_scaled       = 1000,					\
	.dyn_consts_n     = { 3900, 5900 }, /* { save: 2700, 5900 } */	\
	.consts_scaled    = 100,					\
	.leakage_consts_n = { 100, 100 }, /* { save: 60, 100 } */	\
	.ijk_scaled       = 1000,					\
	.leakage_min      = 30,						\
	.leakage_consts_ijk = LEAKAGE_CONSTS_IJK_COMMON			\
}

static struct tegra_edp_cpu_powermodel_params t13x_cpu_powermodel_params[] = {
	{
		.cpu_speedo_id = 1, /* array-index[0] is DSC */
		.common = EDP_PARAMS_COMMON_PART,
		.safety_cap = { 2500000, 2300000, },
	},
	{
		.cpu_speedo_id = 1, /* array-index[1] is MID */
		.common = EDP_PARAMS_COMMON_PART,
		/* .safety_cap = { 2500000, 2500000, }, */
	},
};

struct tegra_edp_cpu_powermodel_params *tegra13x_get_cpu_powermodel_params(
							int index,
							unsigned int *sz)
{
	BUG_ON(index >= ARRAY_SIZE(t13x_cpu_powermodel_params));
	if (sz)
		*sz = ARRAY_SIZE(t13x_cpu_powermodel_params);
	return &t13x_cpu_powermodel_params[index];
}
#endif

#endif /* CONFIG_ARCH_TEGRA_13x_SOC */
