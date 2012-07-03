/*
 * Copyright (c) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef _LINUX_EDP_H
#define _LINUX_EDP_H

#include <linux/kernel.h>
#include <linux/errno.h>

#define EDP_NAME_LEN	16

struct edp_manager {
	const char name[EDP_NAME_LEN];
	const unsigned int imax;

	/* internal */
	struct list_head link;
	struct list_head clients;
	bool registered;
};

/*
 * @states: EDP state array holding the IMAX for each state.
 *	This must be sorted in descending order.
 * @num_states: length of the above array
 * @e0_index: index of the E0 state in the above array
 * Note that each EDP client is tied to a single EDP manager
 */
struct edp_client {
	const char name[EDP_NAME_LEN];
	const unsigned int *const states;
	const unsigned int num_states;
	const unsigned int e0_index;

	/* internal */
	struct list_head link;
	struct edp_manager *manager;
};

#ifdef CONFIG_EDP_FRAMEWORK
extern int edp_register_manager(struct edp_manager *mgr);
extern int edp_unregister_manager(struct edp_manager *mgr);
extern struct edp_manager *edp_get_manager(const char *name);

extern int edp_register_client(struct edp_manager *mgr,
		struct edp_client *client);
extern int edp_unregister_client(struct edp_client *client);
#else
static inline int edp_register_manager(struct edp_manager *mgr)
{ return -ENODEV; }
static inline int edp_unregister_manager(struct edp_manager *mgr)
{ return -ENODEV; }
static inline struct edp_manager *edp_get_manager(const char *name)
{ return NULL; }
static inline int edp_register_client(struct edp_manager *mgr,
		struct edp_client *client)
{ return -ENODEV; }
static inline int edp_unregister_client(struct edp_client *client)
{ return -ENODEV; }
#endif

#endif
