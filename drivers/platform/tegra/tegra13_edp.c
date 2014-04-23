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

#include <mach/edp.h>

#ifdef CONFIG_SYSEDP_FRAMEWORK
static struct tegra_sysedp_corecap t132_sysedp_corecap[] = {
/*
   Initial table for T132
*/
	/*mW	 CPU intensive load        GPU intensive load    */
	/*mW     budget  gpu(khz) mem(khz)  budget  gpu(khz) mem(khz) pthrot(mW)*/
	{5000,  {4000,  108000, 933000}, {4000,  108000, 933000}, 918 },
	{6000,  {6000,  108000, 933000}, {4500,  180000, 933000}, 2109 },
	{7000,  {7000,  180000, 933000}, {4500,  396000, 933000}, 2589 },
	{8000,  {7000,  180000, 933000}, {4500,  468000, 933000}, 3068 },
	{9000,  {8000,  252000, 933000}, {4500,  612000, 933000}, 3630 },
	{10000, {10000, 252000, 933000}, {7500,  396000, 933000}, 4425 },
	{11000, {10000, 396000, 933000}, {7000,  468000, 933000}, 5301 },
	{12000, {12000, 324000, 933000}, {7000,  612000, 933000}, 5253 },
	{13000, {13000, 324000, 933000}, {7000,  684000, 933000}, 6874 },
	{14000, {14000, 252000, 933000}, {7000,  708000, 933000}, 6771 },
	{15000, {14000, 396000, 933000}, {7500,  708000, 933000}, 7819 },
	{16000, {14000, 468000, 933000}, {7000,  804000, 933000}, 8053 },
	{17000, {14000, 468000, 933000}, {7000,  853000, 933000}, 8975 },
	{18000, {14000, 540000, 933000}, {7500,  853000, 933000}, 9204 },
	{19000, {14000, 612000, 933000}, {9000,  853000, 933000}, 9998 },
	{20000, {14000, 612000, 933000}, {10000, 853000, 933000}, 10825 },
	{21000, {14000, 708000, 933000}, {10500, 853000, 933000}, 10908 },
	{22000, {14000, 708000, 933000}, {12000, 853000, 933000}, 11305 },
	{23000, {14000, 708000, 933000}, {12500, 853000, 933000}, 12696 },
	{24000, {14000, 756000, 933000}, {13000, 853000, 933000}, 13524 },
	{25000, {14000, 853000, 933000}, {14000, 853000, 933000}, 13524 },
	{26000, {14000, 853000, 933000}, {14000, 853000, 933000}, 13999 },
	{27000, {14000, 853000, 933000}, {14000, 853000, 933000}, 15002 },
	{28000, {14000, 853000, 933000}, {14000, 853000, 933000}, 15022 },
	{29000, {14000, 853000, 933000}, {14000, 853000, 933000}, 15621 },
	{30000, {14000, 853000, 933000}, {14000, 853000, 933000}, 15621 },
	{31000, {14000, 853000, 933000}, {14000, 853000, 933000}, 15621 },
	{32000, {14000, 853000, 933000}, {14000, 853000, 933000}, 16330 },
	{33000, {14000, 853000, 933000}, {14000, 853000, 933000}, 17721 },
};

struct tegra_sysedp_corecap *tegra_get_sysedp_corecap(unsigned int *sz)
{
	BUG_ON(sz == NULL);
	*sz = ARRAY_SIZE(t132_sysedp_corecap);
	return t132_sysedp_corecap;
}
#endif
