/*
 * arch/arm/mach-tegra/tegra_bbc_proxy.c
 *
 * Copyright (C) 2013 NVIDIA Corporation.
 *
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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/edp.h>

#include <mach/isomgr.h>
#include <mach/latency_allowance.h>
#include <mach/tegra_bbc_proxy.h>

#define MAX_MODEM_EDP_STATES 10

#define MAX_ISO_BW_REQ  900000

struct tegra_bbc_proxy {
	struct edp_client *modem_boot_edp_client;
	struct edp_client modem_edp_client;
	unsigned int modem_edp_states[MAX_MODEM_EDP_STATES];
	int edp_client_registered;
	int edp_boot_client_registered;
	int edp_initialized;
	char *edp_manager_name;
	unsigned int i_breach_ppm; /* percent time current exceeds i_thresh */
	unsigned int i_thresh_3g_adjperiod; /* 3g i_thresh adj period */
	unsigned int i_thresh_lte_adjperiod; /* lte i_thresh adj period */
	struct work_struct edp_work;
	struct mutex edp_lock;
	tegra_isomgr_handle isomgr_handle;
};

static void edp_work(struct work_struct *ws)
{
	return;
}

#define EDP_INT_ATTR(field)						\
static ssize_t								\
field ## _show(struct device *pdev, struct device_attribute *attr,      \
	       char *buf)						\
{									\
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(pdev);		\
									\
	if (!bbc)							\
		return -EAGAIN;						\
									\
	return sprintf(buf, "%d\n", bbc->field);			\
}									\
static DEVICE_ATTR(field, S_IRUSR, field ## _show, NULL);

EDP_INT_ATTR(i_breach_ppm);
EDP_INT_ATTR(i_thresh_3g_adjperiod);
EDP_INT_ATTR(i_thresh_lte_adjperiod);

static ssize_t i_max_show(struct device *pdev, struct device_attribute *attr,
			  char *buf)
{
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(pdev);
	int i = 0;
	int count = 0;

	mutex_lock(&bbc->edp_lock);
	for (i = 0; i < MAX_MODEM_EDP_STATES; i++) {
		if (bbc->modem_edp_states[i] <= 0)
			break;

		count += sprintf(&buf[count], "%d ",
				 bbc->modem_edp_states[i]);
	}
	mutex_unlock(&bbc->edp_lock);

	count += sprintf(&buf[count], "\n");

	return count;
}

static ssize_t i_max_store(struct device *pdev, struct device_attribute *attr,
			   const char *buff, size_t size)
{
	struct tegra_bbc_proxy *bbc;
	char *s, *state_i_max, buf[50];
	unsigned int num_states = 0;
	struct edp_manager *mgr;
	int ret;

	bbc = (struct tegra_bbc_proxy *)dev_get_drvdata(pdev);

	mutex_lock(&bbc->edp_lock);

	/* client should only be registered once per modem boot */
	if (bbc->edp_client_registered) {
		pr_err("bbc edp client already registered\n");
		ret = -EBUSY;
		goto done;
	}

	memset(bbc->modem_edp_states, 0, sizeof(bbc->modem_edp_states));
	memset(&bbc->modem_edp_client, 0, sizeof(bbc->modem_edp_client));

	/* retrieve max current for supported states */
	strlcpy(buf, buff, sizeof(buf));
	s = strim(buf);
	while (s && (num_states < MAX_MODEM_EDP_STATES)) {
		state_i_max = strsep(&s, ",");
		ret = kstrtoul(state_i_max, 10,
			(unsigned long *)&bbc->modem_edp_states[num_states]);
		if (ret) {
			pr_err("invalid bbc state-current setting\n");
			goto done;
		}
		num_states++;
	}

	if (s && (num_states == MAX_MODEM_EDP_STATES)) {
		pr_err("number of bbc EDP states exceeded max\n");
		ret = -EINVAL;
		goto done;
	}

	strncpy(bbc->modem_edp_client.name, "bbc", EDP_NAME_LEN);
	bbc->modem_edp_client.name[EDP_NAME_LEN - 1] = '\0';
	bbc->modem_edp_client.states = bbc->modem_edp_states;
	bbc->modem_edp_client.num_states = num_states;
	bbc->modem_edp_client.e0_index = 0;
	bbc->modem_edp_client.priority = EDP_MAX_PRIO;

	mgr = edp_get_manager(bbc->edp_manager_name);
	if (!mgr) {
		dev_err(pdev, "can't get edp manager\n");
		ret = -EINVAL;
		goto done;
	}

	/* register modem client */
	ret = edp_register_client(mgr, &bbc->modem_edp_client);
	if (ret) {
		dev_err(pdev, "unable to register bbc edp client\n");
		goto done;
	}
	bbc->edp_client_registered = 1;

	/* unregister modem_boot_client */
	ret = edp_unregister_client(bbc->modem_boot_edp_client);
	if (ret) {
		dev_err(pdev, "unable to register bbc boot edp client\n");
		goto done;
	}

	bbc->edp_boot_client_registered = 0;
	ret = size;

done:
	mutex_unlock(&bbc->edp_lock);
	return ret;
}
static DEVICE_ATTR(i_max, S_IRUSR | S_IWUSR, i_max_show, i_max_store);

static ssize_t request_store(struct device *pdev, struct device_attribute *attr,
			     const char *buff, size_t size)
{
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(pdev);
	struct edp_client *c;
	unsigned int id;
	int ret;

	if (sscanf(buff, "%u", &id) != 1)
		return -EINVAL;

	if (!bbc->edp_client_registered)
		return -EINVAL;

	c = &bbc->modem_edp_client;
	if (id >= c->num_states)
		return -EINVAL;

	ret = edp_update_client_request(c, id, NULL);
	if (ret)
		dev_err(pdev, "state update to %u failed\n", id);
	else
		ret = size;

	return ret;
}
static DEVICE_ATTR(request, S_IWUSR, NULL, request_store);

static ssize_t threshold_store(struct device *pdev,
			       struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(pdev);
	unsigned int tv;
	int ret;

	if (sscanf(buff, "%u", &tv) != 1)
		return -EINVAL;

	if (!bbc->edp_client_registered)
		return -EINVAL;

	ret = edp_update_loan_threshold(&bbc->modem_edp_client, tv);
	if (ret)
		dev_err(pdev, "threshold update to %u failed\n", tv);
	else
		ret = size;

	return ret;
}
DEVICE_ATTR(threshold, S_IWUSR, NULL, threshold_store);

static struct device_attribute *edp_attributes[] = {
	&dev_attr_i_breach_ppm,
	&dev_attr_i_thresh_3g_adjperiod,
	&dev_attr_i_thresh_lte_adjperiod,
	&dev_attr_i_max,
	&dev_attr_request,
	&dev_attr_threshold,
	NULL
};

static ssize_t la_bbcr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int la_val;

	sscanf(buf, "%u", &la_val);
	if (la_val > 4096)
		return -EINVAL;

	tegra_set_latency_allowance(TEGRA_LA_BBCR, la_val);

	return size;
}

static ssize_t la_bbcw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int la_val;

	sscanf(buf, "%u", &la_val);
	if (la_val > 4096)
		return -EINVAL;

	tegra_set_latency_allowance(TEGRA_LA_BBCW, la_val);

	return size;
}

static ssize_t la_bbcllr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int la_val;

	sscanf(buf, "%u", &la_val);
	if (la_val > 4096)
		return -EINVAL;

	tegra_set_latency_allowance(TEGRA_LA_BBCLLR, la_val);

	return size;
}

static ssize_t iso_reserve_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int bw;
	unsigned int ult = 4;
	char buff[50];
	char *val, *s;
	int ret;
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(dev);

	strlcpy(buff, buf, sizeof(buff));
	s = strim(buff);

	/* first param is bw */
	val = strsep(&s, ",");
	ret = kstrtouint(val, 10, &bw);
	if (ret) {
		pr_err("invalid bw setting\n");
		return -EINVAL;
	}

	/* second param is latency */
	if (s) {
		ret = kstrtouint(s, 10, &ult);
		if (ret) {
			pr_err("invalid latency setting\n");
			return -EINVAL;
		}
	}

	if (bw > MAX_ISO_BW_REQ)
		return -EINVAL;

	ret = tegra_isomgr_reserve(bbc->isomgr_handle, bw, ult);
	if (!ret)
		dev_err(dev, "can't reserve iso bw\n");

	return size;
}

static ssize_t iso_realize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(dev);

	ret = tegra_isomgr_realize(bbc->isomgr_handle);
	if (!ret)
		dev_err(dev, "can't realize iso bw\n");

	return size;
}


static ssize_t iso_res_realize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	unsigned int bw;
	unsigned int ult = 4;
	char buff[50];
	char *val, *s;
	int ret;
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(dev);

	strlcpy(buff, buf, sizeof(buff));
	s = strim(buff);

	/* first param is bw */
	val = strsep(&s, ",");
	ret = kstrtouint(val, 10, &bw);
	if (ret) {
		pr_err("invalid bw setting\n");
		return -EINVAL;
	}

	/* second param is latency */
	if (s) {
		ret = kstrtouint(s, 10, &ult);
		if (ret) {
			pr_err("invalid latency setting\n");
			return -EINVAL;
		}
	}

	if (bw > MAX_ISO_BW_REQ)
		return -EINVAL;

	ret = tegra_isomgr_reserve(bbc->isomgr_handle, bw, ult);
	if (!ret) {
		dev_err(dev, "can't reserve iso bw\n");
		return size;
	}

	ret = tegra_isomgr_realize(bbc->isomgr_handle);

	if (!ret) {
		dev_err(dev, "can't realize iso bw\n");
		return size;
	}

/* TODO: set LA*/
	return size;
}

static ssize_t iso_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	unsigned int bw;
	struct tegra_bbc_proxy *bbc = dev_get_drvdata(dev);

	sscanf(buf, "%u", &bw);

	if (bbc->isomgr_handle)
		tegra_isomgr_unregister(bbc->isomgr_handle);

	bbc->isomgr_handle = tegra_isomgr_register(TEGRA_ISO_CLIENT_BBC_0,
		bw, NULL, NULL);

	return size;
}



static DEVICE_ATTR(la_bbcr, S_IWUSR, NULL, la_bbcr_store);
static DEVICE_ATTR(la_bbcw, S_IWUSR, NULL, la_bbcw_store);
static DEVICE_ATTR(la_bbcllr, S_IWUSR, NULL, la_bbcllr_store);
static DEVICE_ATTR(iso_reserve, S_IWUSR, NULL, iso_reserve_store);
static DEVICE_ATTR(iso_realize, S_IWUSR, NULL, iso_realize_store);
static DEVICE_ATTR(iso_reserve_realize, S_IWUSR, NULL, iso_res_realize_store);
static DEVICE_ATTR(iso_register, S_IWUSR, NULL, iso_register_store);


static struct device_attribute *mc_attributes[] = {
	&dev_attr_la_bbcr,
	&dev_attr_la_bbcw,
	&dev_attr_la_bbcllr,
	&dev_attr_iso_reserve,
	&dev_attr_iso_realize,
	&dev_attr_iso_reserve_realize,
	&dev_attr_iso_register,
	NULL
};


static int tegra_bbc_proxy_probe(struct platform_device *pdev)
{
	struct tegra_bbc_proxy_platform_data *pdata = pdev->dev.platform_data;
	struct tegra_bbc_proxy *bbc;
	struct edp_manager *mgr;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int ret = 0;

	/* check for platform data */
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not available\n");
		return -ENODEV;
	}

	bbc = kzalloc(sizeof(struct tegra_bbc_proxy), GFP_KERNEL);
	if (!bbc) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	if (pdata->modem_boot_edp_client && pdata->edp_manager_name) {
		mutex_init(&bbc->edp_lock);

		/* register bbc boot client */
		bbc->edp_manager_name = pdata->edp_manager_name;
		mgr = edp_get_manager(pdata->edp_manager_name);
		if (!mgr) {
			dev_err(&pdev->dev, "can't get edp manager\n");
			goto error;
		}

		bbc->modem_boot_edp_client = pdata->modem_boot_edp_client;
		ret = edp_register_client(mgr, bbc->modem_boot_edp_client);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to register bbc boot edp client\n");
			goto error;
		}

		/* request E0 */
		ret = edp_update_client_request(bbc->modem_boot_edp_client,
						0, NULL);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to set e0 state\n");
			goto edp_req_error;
		}

		bbc->edp_boot_client_registered = 1;

		bbc->i_breach_ppm = pdata->i_breach_ppm;
		bbc->i_thresh_3g_adjperiod = pdata->i_thresh_3g_adjperiod;
		bbc->i_thresh_lte_adjperiod = pdata->i_thresh_lte_adjperiod;

		attrs = edp_attributes;
		while ((attr = *attrs++)) {
			ret = device_create_file(&pdev->dev, attr);
			if (ret) {
				dev_err(&pdev->dev,
					"can't create sysfs file\n");
				goto edp_req_error;
			}
		}

		INIT_WORK(&bbc->edp_work, edp_work);

		bbc->edp_initialized = 1;
	}

	/* for bringup we will reserve/realize through sysfs */
	bbc->isomgr_handle = tegra_isomgr_register(TEGRA_ISO_CLIENT_BBC_0,
		MAX_ISO_BW_REQ, NULL, NULL);
	if (!bbc->isomgr_handle)
		goto iso_error;

	attrs = mc_attributes;
	while ((attr = *attrs++)) {
		ret = device_create_file(&pdev->dev, attr);
		if (ret) {
			dev_err(&pdev->dev, "can't create sysfs file\n");
			goto mc_error;
		}
	}

	dev_set_drvdata(&pdev->dev, bbc);

	return 0;

mc_error:
	tegra_isomgr_unregister(bbc->isomgr_handle);

iso_error:
	if (bbc->edp_initialized) {
		attrs = edp_attributes;
		while ((attr = *attrs++))
			device_remove_file(&pdev->dev, attr);
	}

edp_req_error:
	if (bbc->edp_boot_client_registered)
		edp_unregister_client(bbc->modem_boot_edp_client);

error:
	kfree(bbc);

	return ret;
}

static int __exit tegra_bbc_proxy_remove(struct platform_device *pdev)
{
	struct tegra_bbc_proxy *bbc = platform_get_drvdata(pdev);
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = mc_attributes;
	while ((attr = *attrs++))
		device_remove_file(&pdev->dev, attr);

	tegra_isomgr_unregister(bbc->isomgr_handle);

	if (bbc->edp_initialized) {
		cancel_work_sync(&bbc->edp_work);

		attrs = edp_attributes;
		while ((attr = *attrs++))
			device_remove_file(&pdev->dev, attr);
	}

	if (bbc->edp_boot_client_registered)
		edp_unregister_client(bbc->modem_boot_edp_client);

	if (bbc->edp_client_registered)
		edp_unregister_client(&bbc->modem_edp_client);

	kfree(bbc);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_bbc_proxy_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int tegra_bbc_proxy_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver tegra_bbc_proxy_driver = {
	.driver = {
		.name = "tegra_bbc_proxy",
		.owner = THIS_MODULE,
	},
	.probe = tegra_bbc_proxy_probe,
	.remove = tegra_bbc_proxy_remove,
#ifdef CONFIG_PM
	.suspend = tegra_bbc_proxy_suspend,
	.resume = tegra_bbc_proxy_resume,
#endif
};

static int __init tegra_bbc_proxy_init(void)
{
	return platform_driver_register(&tegra_bbc_proxy_driver);
}

static void __exit tegra_bbc_proxy_exit(void)
{
	platform_driver_unregister(&tegra_bbc_proxy_driver);
}

module_init(tegra_bbc_proxy_init);
module_exit(tegra_bbc_proxy_exit);

MODULE_DESCRIPTION("Tegra T148 BBC Proxy Module");
MODULE_LICENSE("GPL");
