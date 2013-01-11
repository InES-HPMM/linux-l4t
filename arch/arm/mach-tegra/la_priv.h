/*
 * arch/arm/mach-tegra/la_priv.h
 *
 * Copyright (C) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _MACH_TEGRA_LA_PRIV_H_
#define _MACH_TEGRA_LA_PRIV_H_

#define ENABLE_LA_DEBUG		0

#define la_debug(fmt, ...) \
do { \
	if (ENABLE_LA_DEBUG) { \
		printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__); \
	} \
} while (0)

#define MASK(x) \
	((0xFFFFFFFFUL >> (31 - (1 ? x) + (0 ? x))) << (0 ? x))
#define SHIFT(x) \
	(0 ? x)
#define ID(id) \
	TEGRA_LA_##id

#define VALIDATE_ID(id, p) \
do { \
	if (id >= TEGRA_LA_MAX_ID || (p)->id_to_index[(id)] == 0xFFFF) { \
		WARN_ONCE(1, "%s: invalid Id=%d", __func__, (id)); \
		return -EINVAL; \
	} \
	BUG_ON((p)->la_info_array[(p)->id_to_index[(id)]].id != (id)); \
} while (0)

#define VALIDATE_BW(bw_in_mbps) \
do { \
	if (bw_in_mbps >= 4096) \
		return -EINVAL; \
} while (0)

#define VALIDATE_THRESHOLDS(tl, tm, th) \
do { \
	if ((tl) > 100 || (tm) > 100 || (th) > 100) \
		return -EINVAL; \
} while (0)

struct la_client_info {
	unsigned int fifo_size_in_atoms;
	unsigned int expiration_in_ns;	/* worst case expiration value */
	unsigned long reg_addr;
	unsigned long mask;
	unsigned long shift;
	enum tegra_la_id id;
	char *name;
	bool scaling_supported;
	unsigned int init_la;		/* initial la to set for client */
};

struct la_scaling_info {
	unsigned int threshold_low;
	unsigned int threshold_mid;
	unsigned int threshold_high;
	int scaling_ref_count;
	int actual_la_to_set;
	int la_set;
};

struct la_scaling_reg_info {
	enum tegra_la_id id;
	unsigned int tl_reg_addr;
	unsigned int tl_mask;
	unsigned int tl_shift;
	unsigned int tm_reg_addr;
	unsigned int tm_mask;
	unsigned int tm_shift;
	unsigned int th_reg_addr;
	unsigned int th_mask;
	unsigned int th_shift;
};

struct la_chip_specific {
	int ns_per_tick;
	int atom_size;
	int la_max_value;
	spinlock_t lock;
	int la_info_array_size;
	struct la_client_info *la_info_array;
	unsigned short id_to_index[ID(MAX_ID) + 1];
	unsigned int disp_bw_array[ID(DISPLAY_HCB) - ID(DISPLAY_0A) + 1];
	struct la_scaling_info scaling_info[ID(MAX_ID)];
	int la_scaling_enable_count;
	struct dentry *latency_debug_dir;

	void (*init_ptsa)(void);
	void (*update_display_ptsa_rate)(unsigned int *disp_bw_array);
	int (*set_la)(enum tegra_la_id id, unsigned int bw_mbps);
	int (*enable_la_scaling)(enum tegra_la_id id,
				unsigned int threshold_low,
				unsigned int threshold_mid,
				unsigned int threshold_high);
	void (*disable_la_scaling)(enum tegra_la_id id);
};

void tegra_la_get_t3_specific(struct la_chip_specific *cs);
void tegra_la_get_t14x_specific(struct la_chip_specific *cs);
void tegra_la_get_t11x_specific(struct la_chip_specific *cs);

#endif /* _MACH_TEGRA_LA_PRIV_H_ */
