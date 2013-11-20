/*
 * linux/arch/arm/mach-tegra/include/mach/pinmux.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __MACH_TEGRA_PINMUX_H
#define __MACH_TEGRA_PINMUX_H

#include <mach/pinmux-defines.h>

struct tegra_drive_pingroup_desc {
	const char *name;
	s16 reg_bank;
	s16 reg;
	u8 drvup_offset;
	u16 drvup_mask;
	u8 drvdown_offset;
	u16 drvdown_mask;
	u8 slewrise_offset;
	u16 slewrise_mask;
	u8 slewfall_offset;
	u16 slewfall_mask;
	u8 drvtype_valid;
	u8 drvtype_offset;
	u8 drvtype_mask;
	const char *dev_id;
};

struct tegra_pingroup_desc {
	const char *name;
	int funcs[4];
	int func_safe;
	int vddio;
	enum tegra_pin_io io_default;
	s16 tri_bank;	/* Register bank the tri_reg exists within */
	s16 mux_bank;	/* Register bank the mux_reg exists within */
	s16 pupd_bank;	/* Register bank the pupd_reg exists within */
	s16 tri_reg;	/* offset into the TRISTATE_REG_* register bank */
	s16 mux_reg;	/* offset into the PIN_MUX_CTL_* register bank */
	s16 pupd_reg;	/* offset into the PULL_UPDOWN_REG_* register bank */
	s8 tri_bit;	/* offset into the TRISTATE_REG_* register bit */
	s8 mux_bit;	/* offset into the PIN_MUX_CTL_* register bit */
	s8 pupd_bit;	/* offset into the PULL_UPDOWN_REG_* register bit */
	s8 lock_bit;	/* offset of the LOCK bit into mux register bit */
	s8 od_bit;	/* offset of the OD bit into mux register bit */
	s8 ioreset_bit;	/* offset of the IO_RESET bit into mux register bit */
	s8 rcv_sel_bit;	/* offset of the RCV_SEL bit into mux register bit */
	int gpionr;
};

u32 pg_readl(u32 bank, u32 reg);
void pg_writel(u32 val, u32 bank, u32 reg);

typedef void (*pinmux_init) (const struct tegra_pingroup_desc **pg,
	int *pg_max, const struct tegra_drive_pingroup_desc **pgdrive,
	int *pgdrive_max, const int **gpiomap, int *gpiomap_max);

void tegra20_pinmux_init(const struct tegra_pingroup_desc **pg, int *pg_max,
	const struct tegra_drive_pingroup_desc **pgdrive, int *pgdrive_max,
	const int **gpiomap, int *gpiomap_max);

void tegra30_pinmux_init(const struct tegra_pingroup_desc **pg, int *pg_max,
	const struct tegra_drive_pingroup_desc **pgdrive, int *pgdrive_max,
	const int **gpiomap, int *gpiomap_max);

void tegra11x_pinmux_init(const struct tegra_pingroup_desc **pg, int *pg_max,
	const struct tegra_drive_pingroup_desc **pgdrive, int *pgdrive_max,
	const int **gpiomap, int *gpiomap_max);

void tegra12x_pinmux_init(const struct tegra_pingroup_desc **pg, int *pg_max,
	const struct tegra_drive_pingroup_desc **pgdrive, int *pgdrive_max,
	const int **gpiomap, int *gpiomap_max);

void tegra14x_pinmux_init(const struct tegra_pingroup_desc **pg, int *pg_max,
	const struct tegra_drive_pingroup_desc **pgdrive, int *pgdrive_max,
	const int **gpiomap, int *gpiomap_max);
void tegra11x_default_pinmux(void);
void tegra12x_default_pinmux(void);

int tegra_pinmux_get_func(int pg);
int tegra_pinmux_set_tristate(int pg, enum tegra_tristate tristate);
int tegra_pinmux_set_io(int pg, enum tegra_pin_io input);
int tegra_pinmux_get_pingroup(int gpio_nr);
int tegra_pinmux_set_pullupdown(int pg, enum tegra_pullupdown pupd);

void tegra_pinmux_config_table(const struct tegra_pingroup_config *config,
	int len);

void tegra_drive_pinmux_config_table(struct tegra_drive_pingroup_config *config,
	int len);

int tegra_drive_pinmux_set_pull_down(int pg,
	enum tegra_pull_strength pull_down);

int tegra_drive_pinmux_set_pull_up(int pg, enum tegra_pull_strength pull_up);

struct device;
int tegra_drive_get_pingroup(struct device *dev);

void tegra_pinmux_set_safe_pinmux_table(const struct tegra_pingroup_config *config,
	int len);
void tegra_pinmux_config_pinmux_table(const struct tegra_pingroup_config *config,
	int len);
void tegra_pinmux_config_tristate_table(const struct tegra_pingroup_config *config,
	int len, enum tegra_tristate tristate);
void tegra_pinmux_config_pullupdown_table(const struct tegra_pingroup_config *config,
	int len, enum tegra_pullupdown pupd);
#endif
