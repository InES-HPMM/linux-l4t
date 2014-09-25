/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/tegra_prod.h>

#define TEGRA_PROD_SETTING "prod-settings"

/**
 * tegra_prod_get - Read the prod setting form Device tree.
 * @np:		device node from which the property value is to be read.
 * @tegra_prod_list:	the list of tegra prods.
 *
 * Read the prod setting form DT according the prod name in tegra prod list.
 * prod tuple will be allocated dynamically according to the tuple number of
 * each prod in DT.
 *
 * Returns 0 on success.
 */
static int tegra_prod_get(const struct device_node *np,
		struct tegra_prod_list *tegra_prod_list)
{
	int i;
	int cnt;
	int ret;
	struct device_node *child;
	struct tegra_prod *t_prod;
	struct prod_tuple *p_tuple;

	if (!tegra_prod_list && !tegra_prod_list->tegra_prod) {
		pr_err("Node %s: Invalid tegra prods list.\n", np->name);
		return -EINVAL;
	};

	child = of_get_child_by_name(np, TEGRA_PROD_SETTING);
	if (!child)
		return -ENODEV;

	for (i = 0; i < tegra_prod_list->num; i++) {
		t_prod = (struct tegra_prod *)&tegra_prod_list->tegra_prod[i];
		t_prod->count = of_property_count_u32(child, t_prod->name);
		if ((t_prod->count < PROD_TUPLE_NUM) ||
			(t_prod->count % PROD_TUPLE_NUM != 0)) {
			pr_err("Node %s: Not found proper setting in %s\n",
				np->name, t_prod->name);
			ret = -EINVAL;
			goto err_parsing;
		}
		t_prod->count /= PROD_TUPLE_NUM;
		t_prod->prod_tuple = kzalloc(
			sizeof(struct prod_tuple) * t_prod->count, GFP_KERNEL);
		if (!t_prod->prod_tuple) {
			ret = -ENOMEM;
			goto err_parsing;
		}

		for (cnt = 0; cnt < t_prod->count; cnt++) {
			p_tuple = (struct prod_tuple *)&t_prod->prod_tuple[cnt];
			ret = of_property_read_u32_index(child, t_prod->name,
				cnt * PROD_TUPLE_NUM, &p_tuple->addr);
			if (ret)
				goto err_parsing;

			ret = of_property_read_u32_index(child, t_prod->name,
				cnt * PROD_TUPLE_NUM + 1, &p_tuple->mask);
			if (ret)
				goto err_parsing;

			ret = of_property_read_u32_index(child, t_prod->name,
				cnt * PROD_TUPLE_NUM + 2, &p_tuple->val);
			if (ret)
				goto err_parsing;
		}
	}

	return 0;

err_parsing:
	of_node_put(child);
	return ret;
}

/**
 * tegra_prod_set_tuple - Only set a tuple.
 * @base:		base address of the register.
 * @prod_tuple:		the tuple to set.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_tuple(void __iomem *base, struct prod_tuple *prod_tuple)
{
	u32 reg;

	if (!prod_tuple)
		return -EINVAL;

	reg = readl(base + prod_tuple->addr);
	reg = ((reg & prod_tuple->mask) |
		(prod_tuple->val & ~prod_tuple->mask));
	writel(reg, base + prod_tuple->addr);

	return 0;
};

/**
 * tegra_prod_set - Set one prod setting.
 * @base:		base address of the register.
 * @tegra_prod:		the prod setting to set.
 *
 * Set all the tuples in one tegra_prod.
 * Returns 0 on success.
 */
int tegra_prod_set(void __iomem *base, struct tegra_prod *tegra_prod)
{
	int i;
	int ret;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->count; i++) {
		ret = tegra_prod_set_tuple(base, &tegra_prod->prod_tuple[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * tegra_prod_set_list - Set all the prod settings of the list in sequence.
 * @base:		base address of the register.
 * @tegra_prod_list:	the list of tegra prods.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_list(void __iomem *base,
		struct tegra_prod_list *tegra_prod_list)
{
	int i;
	int ret;

	if (!tegra_prod_list)
		return -EINVAL;

	for (i = 0; i < tegra_prod_list->num; i++) {
		ret = tegra_prod_set(base, &tegra_prod_list->tegra_prod[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * tegra_prod_set_by_name - Set the prod setting according the name.
 * @base:		base address of the register.
 * @name:		the name of tegra prod need to set.
 * @tegra_prod_list:	the list of tegra prods.
 *
 * Find the tegra prod in the list according to the name. Then set
 * that tegra prod.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_by_name(void __iomem *base, const char *name,
		struct tegra_prod_list *tegra_prod_list)
{
	int i;
	int ret;
	struct tegra_prod *t_prod;

	if (!tegra_prod_list)
		return -EINVAL;

	for (i = 0; i < tegra_prod_list->num; i++) {
		t_prod = &tegra_prod_list->tegra_prod[i];
		if (!t_prod)
			return -EINVAL;
		if (!strcmp(t_prod->name, name)) {
			ret = tegra_prod_set(base, t_prod);
			if (ret)
				return ret;
			else
				return 0;
		}
	}

	return -ENODEV;
}

/**
 * tegra_prod_init - Init tegra prod list.
 * @np		device node from which the property value is to be read.
 *
 * Query all the prod settings under DT node & Init the tegra prod list
 * automatically.
 *
 * Returns 0 on success, -EINVAL for wrong prod number, -ENOMEM if faild
 * to allocate memory for tegra prod list.
 */
struct tegra_prod_list *tegra_prod_init(const struct device_node *np)
{
	int prod_num = 0;
	int ncount;
	int ret;
	struct device_node *child;
	struct property *pp;
	struct tegra_prod_list *tegra_prod_list;

	child = of_get_child_by_name(np, TEGRA_PROD_SETTING);
	if (!child)
		return ERR_PTR(-ENODEV);

	for_each_property_of_node(child, pp) {
		prod_num++;
	}
	prod_num--; /* exclusive of the name property of dt node */
	if (prod_num <= 0) {
		ret = -ENODEV;
		goto err_init;
	}

	tegra_prod_list = kzalloc(sizeof(struct tegra_prod_list), GFP_KERNEL);
	if (!tegra_prod_list) {
		ret = -ENOMEM;
		goto err_init;
	}

	tegra_prod_list->tegra_prod = kzalloc(
		sizeof(struct tegra_prod) * prod_num, GFP_KERNEL);
	if (!tegra_prod_list->tegra_prod) {
		ret = -ENOMEM;
		goto err_init;
	}

	ncount = 0;
	for_each_property_of_node(child, pp) {
		tegra_prod_list->tegra_prod[ncount].name = pp->name;
		ncount++;
	}

	tegra_prod_list->num = prod_num;
	of_node_put(child);

	ret = tegra_prod_get(np, tegra_prod_list);
	if (ret) {
		pr_err("Node %s: Faild to read the Prod Setting.\n", np->name);
		goto err_get;
	}

	return tegra_prod_list;

err_init:
	of_node_put(child);
err_get:
	return ERR_PTR(ret);
}

/**
 * tegra_prod_release - Release all the resources.
 * @tegra_prod_list:	the list of tegra prods.
 *
 * Release all the resources of tegra_prod_list.
 * Returns 0 on success.
 */
int tegra_prod_release(struct tegra_prod_list *tegra_prod_list)
{
	int i;
	struct tegra_prod *t_prod;

	if (tegra_prod_list) {
		if (tegra_prod_list->tegra_prod) {
			for (i = 0; i < tegra_prod_list->num; i++) {
				t_prod = (struct tegra_prod *)
					&tegra_prod_list->tegra_prod[i];
				if (t_prod)
					kfree(t_prod->prod_tuple);
			}
			kfree(tegra_prod_list->tegra_prod);
		}
		kfree(tegra_prod_list);
	}

	return 0;
}

