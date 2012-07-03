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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/edp.h>

DEFINE_MUTEX(edp_lock);
static LIST_HEAD(edp_managers);

static struct edp_manager *find_manager(const char *name)
{
	struct edp_manager *mgr;

	if (!name)
		return NULL;

	list_for_each_entry(mgr, &edp_managers, link)
		if (!strcmp(mgr->name, name))
			return mgr;

	return NULL;
}

int edp_register_manager(struct edp_manager *mgr)
{
	int r = -EEXIST;

	if (!mgr)
		return -EINVAL;

	mutex_lock(&edp_lock);
	if (!find_manager(mgr->name)) {
		list_add_tail(&mgr->link, &edp_managers);
		mgr->registered = true;
		r = 0;
	}
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_register_manager);

int edp_unregister_manager(struct edp_manager *mgr)
{
	int r = -ENODEV;

	if (!mgr)
		return -EINVAL;

	mutex_lock(&edp_lock);
	if (mgr->registered) {
		list_del(&mgr->link);
		mgr->registered = false;
		r = 0;
	}
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_unregister_manager);

struct edp_manager *edp_get_manager(const char *name)
{
	struct edp_manager *mgr;

	mutex_lock(&edp_lock);
	mgr = find_manager(name);
	mutex_unlock(&edp_lock);

	return mgr;
}
EXPORT_SYMBOL(edp_get_manager);
