/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/edp.h>

struct loan_client {
	struct list_head link;
	struct edp_client *client;
	unsigned int size;
};

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
	INIT_LIST_HEAD(&client->borrowers);
	client->num_borrowers = 0;
	client->num_loans = 0;
	client->ithreshold = client->states[0];

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

static void update_loans(struct edp_client *client)
{
	struct loan_client *p;
	unsigned int size;

	if (!client->cur || list_empty(&client->borrowers))
		return;

	size = *client->cur < client->ithreshold ? 0 :
		*client->cur - client->ithreshold;

	/* TODO: multi-party loans */
	if (!list_is_singular(&client->borrowers)) {
		WARN_ON(1);
		return;
	}

	p = list_first_entry(&client->borrowers, struct loan_client, link);

	/* avoid spurious change notifications */
	if (size != p->size) {
		p->size = size;
		p->client->notify_loan_update(size, client);
	}
}

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

	update_loans(client);

	return 0;
}

static void del_borrower(struct edp_client *lender, struct loan_client *pcl)
{
	pcl->client->notify_loan_close(lender);
	lender->num_borrowers--;
	pcl->client->num_loans--;
	list_del(&pcl->link);
	kfree(pcl);
}

static void close_all_loans(struct edp_client *client)
{
	struct loan_client *p;

	while (!list_empty(&client->borrowers)) {
		p = list_first_entry(&client->borrowers, struct loan_client,
				link);
		del_borrower(client, p);
	}
}

static inline bool registered_client(struct edp_client *client)
{
	return client ? client->manager : false;
}

static int unregister_client(struct edp_client *client)
{
	if (!registered_client(client))
		return -EINVAL;

	if (client->num_loans)
		return -EBUSY;

	close_all_loans(client);
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

	if (!registered_client(client))
		return -EINVAL;

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

static struct edp_client *get_client(const char *name)
{
	struct edp_client *client;
	struct edp_manager *mgr;

	if (!name)
		return NULL;

	list_for_each_entry(mgr, &edp_managers, link) {
		client = find_client(mgr, name);
		if (client)
			return client;
	}

	return NULL;
}

struct edp_client *edp_get_client(const char *name)
{
	struct edp_client *client;

	mutex_lock(&edp_lock);
	client = get_client(name);
	mutex_unlock(&edp_lock);

	return client;
}
EXPORT_SYMBOL(edp_get_client);

static struct loan_client *find_borrower(struct edp_client *lender,
		struct edp_client *borrower)
{
	struct loan_client *p;

	list_for_each_entry(p, &lender->borrowers, link)
		if (p->client == borrower)
			return p;
	return NULL;
}

static int register_loan(struct edp_client *lender, struct edp_client *borrower)
{
	struct loan_client *p;

	if (!registered_client(lender) || !registered_client(borrower))
		return -EINVAL;

	if (lender->manager != borrower->manager ||
			!borrower->notify_loan_update ||
			!borrower->notify_loan_close)
		return -EINVAL;

	if (find_borrower(lender, borrower))
		return -EEXIST;

	if (lender->num_borrowers >= lender->max_borrowers)
		return -EBUSY;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->client = borrower;
	lender->num_borrowers++;
	borrower->num_loans++;
	list_add_tail(&p->link, &lender->borrowers);

	update_loans(lender);
	return 0;
}

int edp_register_loan(struct edp_client *lender, struct edp_client *borrower)
{
	int r;

	mutex_lock(&edp_lock);
	r = register_loan(lender, borrower);
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_register_loan);

static int unregister_loan(struct edp_client *lender,
		struct edp_client *borrower)
{
	struct loan_client *p;

	if (!registered_client(lender) || !registered_client(borrower))
		return -EINVAL;

	p = find_borrower(lender, borrower);
	if (!p)
		return -EINVAL;

	del_borrower(lender, p);
	update_loans(lender);
	return 0;
}

int edp_unregister_loan(struct edp_client *lender, struct edp_client *borrower)
{
	int r;

	mutex_lock(&edp_lock);
	r = unregister_loan(lender, borrower);
	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_unregister_loan);

int edp_update_loan_threshold(struct edp_client *client, unsigned int threshold)
{
	int r = -EINVAL;

	mutex_lock(&edp_lock);

	if (registered_client(client)) {
		client->ithreshold = threshold;
		update_loans(client);
		r = 0;
	}

	mutex_unlock(&edp_lock);

	return r;
}
EXPORT_SYMBOL(edp_update_loan_threshold);
