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

#ifndef _LINUX_EDP_H
#define _LINUX_EDP_H

#include <linux/kernel.h>
#include <linux/errno.h>

#define EDP_NAME_LEN	16

struct edp_manager {
	char name[EDP_NAME_LEN];
	unsigned int imax;

	/* internal */
	struct list_head link;
	struct list_head clients;
	bool registered;
	unsigned int remaining;
};

/*
 * @states: EDP state array holding the IMAX for each state.
 *	This must be sorted in descending order.
 * @num_states: length of the above array
 * @e0_index: index of the E0 state in the above array
 * @max_borrowers: maximum number of clients allowed to borrow from this
 * @notify_loan_update: for receiving loan size change notifications
 * @notify_loan_close: for receiving loan closure notification
 * Note that each EDP client is tied to a single EDP manager
 */
struct edp_client {
	char name[EDP_NAME_LEN];
	unsigned int *states;
	unsigned int num_states;
	unsigned int e0_index;
	unsigned int max_borrowers;

	void (*notify_loan_update)(unsigned int new_size,
			struct edp_client *lender);
	void (*notify_loan_close)(struct edp_client *lender);

	/* internal */
	struct list_head link;
	struct list_head borrowers;
	struct edp_manager *manager;
	const unsigned int *req;
	const unsigned int *cur;
	unsigned int ithreshold;
	unsigned int num_borrowers;
	unsigned int num_loans;
};

#ifdef CONFIG_EDP_FRAMEWORK
extern int edp_register_manager(struct edp_manager *mgr);
extern int edp_unregister_manager(struct edp_manager *mgr);
extern struct edp_manager *edp_get_manager(const char *name);

extern int edp_register_client(struct edp_manager *mgr,
		struct edp_client *client);
extern int edp_unregister_client(struct edp_client *client);
extern int edp_update_client_request(struct edp_client *client,
		unsigned int req, unsigned int *approved);
extern struct edp_client *edp_get_client(const char *name);

/*
 * EDP lender: An EDP client whose device (a) typically draws current less
 * than some (dynamically varying) threshold (b) occasionally draws more
 * than its threshold but not more than what is allowed by its current E-state
 * and (c) asserts a side-band signal prior to exceeding the threshold
 *
 * EDP borrower: An EDP client which (a) gets its base current consumption
 * budget by setting an E-state with the EDP manager (b) borrows from a
 * lender's additional current budget according to the difference between
 * its E-state and threshold when the side-band is deasserted and
 * (c) stops borrowing whenever the side-band is asserted
 *
 * EDP loan: a contract allowing an EDP borrower to borrow from a lender.
 *
 * The loan is registered via edp_register_loan. In order to register
 * a loan, both the lender and borrower needs to be registered with the same
 * manager. The contract is terminated with edp_unregister_loan.
 * The lender updates its threshold values using edp_update_loan_threshold.
 * Whenever there is a change in the loan size (due to a change in the
 * lender's E-state or threshold), the borrowr is notified through
 * notify_loan_update.
 */
extern int edp_register_loan(struct edp_client *lender,
		struct edp_client *borrower);
extern int edp_unregister_loan(struct edp_client *lender,
		struct edp_client *borrower);
extern int edp_update_loan_threshold(struct edp_client *lender,
		unsigned int threshold);
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
static inline int edp_update_client_request(struct edp_client *client,
		unsigned int req, unsigned int *approved)
{ return -ENODEV; }
static inline struct edp_client *edp_get_client(const char *name)
{ return NULL; }
static inline int edp_register_loan(struct edp_client *lender,
		struct edp_client *borrower)
{ return -ENODEV; }
static inline int edp_unregister_loan(struct edp_client *lender,
		struct edp_client *borrower)
{ return -ENODEV; }
static inline int edp_update_loan_threshold(struct edp_client *lender,
		unsigned int threshold)
{ return -ENODEV; }
#endif

#endif
