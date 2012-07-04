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
		mgr->remaining = mgr->imax;
		INIT_LIST_HEAD(&mgr->clients);
		r = 0;
	}
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_register_manager);

int edp_unregister_manager(struct edp_manager *mgr)
{
	int r = 0;

	if (!mgr)
		return -EINVAL;

	mutex_lock(&edp_lock);
	if (!mgr->registered) {
		r = -ENODEV;
	} else if (!list_empty(&mgr->clients)) {
		r = -EBUSY;
	} else {
		list_del(&mgr->link);
		mgr->registered = false;
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

static struct edp_client *find_client(struct edp_manager *mgr,
		const char *name)
{
	struct edp_client *p;

	if (!name)
		return NULL;

	list_for_each_entry(p, &mgr->clients, link)
		if (!strcmp(p->name, name))
			return p;

	return NULL;
}

static unsigned int e0_current_sum(struct edp_manager *mgr)
{
	struct edp_client *p;
	unsigned int sum = 0;

	list_for_each_entry(p, &mgr->clients, link)
		sum += p->states[p->e0_index];

	return sum;
}

static bool states_ok(struct edp_client *client)
{
	int i;

	if (!client->states || !client->num_states ||
			client->e0_index >= client->num_states)
		return false;

	/* state array should be sorted in descending order */
	for (i = 1; i < client->num_states; i++)
		if (client->states[i] >= client->states[i - 1])
			return false;

	return true;
}

static int register_client(struct edp_manager *mgr, struct edp_client *client)
{
	if (!mgr || !client)
		return -EINVAL;

	if (client->manager || find_client(mgr, client->name))
		return -EEXIST;

	if (!mgr->registered)
		return -ENODEV;

	if (!states_ok(client))
		return -EINVAL;

	/* make sure that we can satisfy E0 for all registered clients */
	if (e0_current_sum(mgr) + client->states[client->e0_index] > mgr->imax)
		return -E2BIG;

	list_add_tail(&client->link, &mgr->clients);
	client->manager = mgr;
	client->req = NULL;
	client->cur = NULL;

	return 0;
}

int edp_register_client(struct edp_manager *mgr, struct edp_client *client)
{
	int r;

	mutex_lock(&edp_lock);
	r = register_client(mgr, client);
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_register_client);

static int mod_request(struct edp_client *client, const unsigned int *req)
{
	unsigned int old = client->cur ? *client->cur : 0;
	unsigned int new = req ? *req : 0;
	unsigned int need;

	if (new < old) {
		client->cur = req;
		client->manager->remaining += old - new;
	} else {
		need = new - old;
		if (need > client->manager->remaining)
			return -ENODEV;
		client->manager->remaining -= need;
		client->cur = req;
	}

	return 0;
}

static int unregister_client(struct edp_client *client)
{
	if (!client)
		return -EINVAL;

	if (!client->manager)
		return -ENODEV;

	mod_request(client, NULL);
	list_del(&client->link);
	client->manager = NULL;

	return 0;
}

int edp_unregister_client(struct edp_client *client)
{
	int r;

	mutex_lock(&edp_lock);
	r = unregister_client(client);
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_unregister_client);

static int update_client_request(struct edp_client *client, unsigned int req,
		int *approved)
{
	int r;

	if (!client)
		return -EINVAL;

	if (!client->manager)
		return -ENODEV;

	if (req >= client->num_states)
		return -EINVAL;

	r = mod_request(client, client->states + req);
	if (!r && approved)
		*approved = client->cur - client->states;

	return r;
}

int edp_update_client_request(struct edp_client *client, unsigned int req,
		unsigned int *approved)
{
	int r;

	mutex_lock(&edp_lock);
	r = update_client_request(client, req, approved);
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_update_client_request);
