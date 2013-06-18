/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <linux/edp.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_data/tegra_edp.h>

struct freqcap {
	unsigned int cpu;
	unsigned int gpu;
	unsigned int emc;
};

static unsigned int gpu_high_threshold = 500;
static unsigned int gpu_window = 80;
static unsigned int gpu_high_hist;
static unsigned int gpu_high_count = 2;
static unsigned int online_cpu_count;
static bool gpu_busy;
static unsigned int core_state;
static unsigned int core_loan;
static struct tegra_sysedp_corecap *cur_corecap;
static struct clk *emc_cap_clk;
static struct clk *gpu_cap_clk;
static struct pm_qos_request cpufreq_qos;
static unsigned int cpu_power_offset = 499;
static unsigned int cpu_power_balance;
static unsigned int force_gpu_pri;
static struct delayed_work core_work;
static unsigned int *core_edp_states;
static struct tegra_sysedp_platform_data *core_platdata;
static struct freqcap core_policy;
static struct freqcap forced_caps;
static struct freqcap cur_caps;
static DEFINE_MUTEX(core_lock);

/* To save some cycles from a linear search */
static unsigned int cpu_lut_match(unsigned int power,
		struct tegra_system_edp_entry *lut, unsigned int lutlen)
{
	unsigned int fv;
	unsigned int lv;
	unsigned int step;
	unsigned int i;

	if (lutlen == 1)
		return 0;

	fv = lut[0].power_limit_100mW * 100;
	lv = lut[lutlen - 1].power_limit_100mW * 100;
	step = (lv - fv) / (lutlen - 1);

	i = (power - fv + step - 1) / step;
	i = min_t(unsigned int, i, lutlen - 1);
	if (lut[i].power_limit_100mW * 100 >= power)
		return i;

	/* Didn't work, search back from the end */
	return lutlen - 1;
}

static unsigned int get_cpufreq_lim(unsigned int power)
{
	struct tegra_system_edp_entry *p;
	int i;

	i = cpu_lut_match(power, core_platdata->cpufreq_lim,
			core_platdata->cpufreq_lim_size);
	p = core_platdata->cpufreq_lim + i;

	for (; i > 0; i--, p--) {
		if (p->power_limit_100mW * 100 <= power)
			break;
	}

	WARN_ON(p->power_limit_100mW > power);
	return p->freq_limits[online_cpu_count - 1];
}

static void pr_caps(struct freqcap *old, struct freqcap *new,
		unsigned int cpu_power)
{
	if (!IS_ENABLED(CONFIG_DEBUG_KERNEL))
		return;

	if (new->cpu == old->cpu &&
			new->gpu == old->gpu &&
			new->emc == old->emc)
		return;

	pr_debug("sysedp: ncpus %u, gpupri %d, core %5u mW, "
			"cpu %5u mW %u kHz, gpu %u kHz, emc %u kHz\n",
			online_cpu_count, gpu_busy, cur_corecap->power,
			cpu_power, new->cpu, new->gpu, new->emc);
}

static void apply_caps(struct tegra_sysedp_devcap *devcap)
{
	struct freqcap new;
	int r;

	core_policy.cpu = get_cpufreq_lim(devcap->cpu_power +
			cpu_power_balance +
			cpu_power_offset);
	core_policy.gpu = devcap->gpufreq;
	core_policy.emc = devcap->emcfreq;

	new.cpu = forced_caps.cpu ?: core_policy.cpu;
	new.gpu = forced_caps.gpu ?: core_policy.gpu;
	new.emc = forced_caps.emc ?: core_policy.emc;

	if (new.cpu != cur_caps.cpu)
		pm_qos_update_request(&cpufreq_qos, new.cpu);

	if (new.emc != cur_caps.emc) {
		r = clk_set_rate(emc_cap_clk, new.emc * 1000);
		WARN_ON(r);
	}

	if (new.gpu != cur_caps.gpu) {
		r = clk_set_rate(gpu_cap_clk, new.gpu * 1000);
		WARN_ON(r);
	}

	pr_caps(&cur_caps, &new, devcap->cpu_power);
	cur_caps = new;
}

static inline bool gpu_priority(void)
{
	return gpu_busy || force_gpu_pri;
}

static inline struct tegra_sysedp_devcap *get_devcap(void)
{
	return gpu_priority() ? &cur_corecap->gpupri : &cur_corecap->cpupri;
}

static void __do_cap_control(void)
{
	struct tegra_sysedp_devcap *cap;

	if (!cur_corecap)
		return;

	cap = get_devcap();
	apply_caps(cap);
}

static void do_cap_control(void)
{
	mutex_lock(&core_lock);
	__do_cap_control();
	mutex_unlock(&core_lock);
}

static void update_cur_corecap(void)
{
	struct tegra_sysedp_corecap *cap;
	unsigned int power;
	int i;

	if (!core_platdata)
		return;

	power = core_edp_states[core_state] * core_platdata->core_gain / 100;
	power += core_loan;
	i = core_platdata->corecap_size - 1;
	cap = core_platdata->corecap + i;

	for (; i >= 0; i--, cap--) {
		if (cap->power <= power) {
			cur_corecap = cap;
			cpu_power_balance = power - cap->power;
			return;
		}
	}

	WARN_ON(1);
	cur_corecap = core_platdata->corecap;
}

static void state_change_cb(unsigned int new_state, void *priv_data)
{
	mutex_lock(&core_lock);
	core_state = new_state;
	update_cur_corecap();
	__do_cap_control();
	mutex_unlock(&core_lock);
}

static unsigned int loan_update_cb(unsigned int new_size,
		struct edp_client *lender, void *priv_data)
{
	mutex_lock(&core_lock);
	core_loan = new_size;
	update_cur_corecap();
	__do_cap_control();
	mutex_unlock(&core_lock);
	return new_size;
}

static void loan_close_cb(struct edp_client *lender, void *priv_data)
{
	loan_update_cb(0, lender, priv_data);
}

static void core_worker(struct work_struct *work)
{
	if (!gpu_busy)
		do_cap_control();
}

/*
 * Return true if load was above threshold for at least
 * gpu_high_count number of notifications
 */
static bool calc_gpu_busy(unsigned int load)
{
	unsigned int mask;

	mask = (1 << gpu_high_count) - 1;

	gpu_high_hist <<= 1;
	if (load >= gpu_high_threshold)
		gpu_high_hist |= 1;

	return (gpu_high_hist & mask) == mask;
}

void tegra_edp_notify_gpu_load(unsigned int load)
{
	bool old;

	old = gpu_busy;
	gpu_busy = calc_gpu_busy(load);

	if (gpu_busy == old || force_gpu_pri || !core_platdata)
		return;

	cancel_delayed_work(&core_work);

	if (gpu_busy)
		do_cap_control();
	else
		schedule_delayed_work(&core_work,
				msecs_to_jiffies(gpu_window));
}

static int tegra_edp_cpu_notify(struct notifier_block *nb,
		unsigned long action, void *data)
{
	switch (action) {
	case CPU_UP_PREPARE:
		online_cpu_count = num_online_cpus() + 1;
		break;
	case CPU_DEAD:
		online_cpu_count = num_online_cpus();
		break;
	default:
		return NOTIFY_OK;
	}

	do_cap_control();
	return NOTIFY_OK;
}

static struct notifier_block tegra_edp_cpu_nb = {
	.notifier_call = tegra_edp_cpu_notify
};

static ssize_t core_request_store(struct edp_client *c,
		struct edp_client_attribute *attr, const char *s, size_t count)
{
	unsigned int id;
	unsigned int approved;
	int r;

	if (sscanf(s, "%u", &id) != 1)
		return -EINVAL;

	mutex_lock(&core_lock);

	r = edp_update_client_request(c, id, &approved);
	if (r)
		goto out;

	core_state = approved;
	update_cur_corecap();
	__do_cap_control();

out:
	mutex_unlock(&core_lock);
	return r ?: count;
}

struct edp_client_attribute core_attrs[] = {
	__ATTR(set_request, 0200, NULL, core_request_store),
	__ATTR_NULL
};

static struct edp_client core_client = {
	.name = "core",
	.priority = EDP_MIN_PRIO,
	.throttle = state_change_cb,
	.attrs = core_attrs,
	.notify_promotion = state_change_cb,
	.notify_loan_update = loan_update_cb,
	.notify_loan_close = loan_close_cb
};

#ifdef CONFIG_DEBUG_FS
static int core_set(void *data, u64 val)
{
	unsigned int *pdata = data;
	unsigned int old;

	old = *pdata;
	*pdata = val;

	if (old != *pdata) {
		/* Changes to core_gain require corecap update */
		if (pdata == &core_platdata->core_gain)
			update_cur_corecap();
		do_cap_control();
	}

	return 0;
}

static int core_get(void *data, u64 *val)
{
	unsigned int *pdata = data;
	*val = *pdata;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(core_fops, core_get, core_set, "%lld\n");

static void create_attr(const char *name, unsigned int *data)
{
	struct dentry *d;

	d = debugfs_create_file(name, S_IRUGO | S_IWUSR, core_client.dentry,
			data, &core_fops);
	WARN_ON(IS_ERR_OR_NULL(d));
}

static inline void edp_show_2core_cpucaps(struct seq_file *file)
{
	int i;
	struct tegra_system_edp_entry *p = core_platdata->cpufreq_lim;

	seq_printf(file, "%5s %10s %10s\n",
			"Power", "1-core", "2-cores");

	for (i = 0; i < core_platdata->cpufreq_lim_size; i++, p++) {
		seq_printf(file, "%5d %10u %10u\n",
				p->power_limit_100mW * 100,
				p->freq_limits[0],
				p->freq_limits[1]);
	}
}

static inline void edp_show_4core_cpucaps(struct seq_file *file)
{
	int i;
	struct tegra_system_edp_entry *p = core_platdata->cpufreq_lim;

	seq_printf(file, "%5s %10s %10s %10s %10s\n",
			"Power", "1-core", "2-cores", "3-cores", "4-cores");

	for (i = 0; i < core_platdata->cpufreq_lim_size; i++, p++) {
		seq_printf(file, "%5d %10u %10u %10u %10u\n",
				p->power_limit_100mW * 100,
				p->freq_limits[0],
				p->freq_limits[1],
				p->freq_limits[2],
				p->freq_limits[3]);
	}
}

static int cpucaps_show(struct seq_file *file, void *data)
{
	unsigned int max_nr_cpus = num_possible_cpus();

	if (core_platdata ? !core_platdata->cpufreq_lim : true)
		return -ENODEV;

	if (max_nr_cpus == 2)
		edp_show_2core_cpucaps(file);
	else if (max_nr_cpus == 4)
		edp_show_4core_cpucaps(file);

	return 0;
}

static int corecaps_show(struct seq_file *file, void *data)
{
	int i;
	struct tegra_sysedp_corecap *p;
	struct tegra_sysedp_devcap *c;
	struct tegra_sysedp_devcap *g;

	if (core_platdata ? !core_platdata->corecap : true)
		return -ENODEV;

	p = core_platdata->corecap;

	seq_printf(file, "%s %s { %s %9s %9s } %s { %s %9s %9s }\n",
			"E-state",
			"CPU-pri", "CPU-mW", "GPU-kHz", "EMC-kHz",
			"GPU-pri", "CPU-mW", "GPU-kHz", "EMC-kHz");

	for (i = 0; i < core_platdata->corecap_size; i++, p++) {
		c = &p->cpupri;
		g = &p->gpupri;
		seq_printf(file, "%7u %16u %9u %9u %18u %9u %9u\n",
				p->power,
				c->cpu_power, c->gpufreq, c->emcfreq,
				g->cpu_power, g->gpufreq, g->emcfreq);
	}

	return 0;
}

static int status_show(struct seq_file *file, void *data)
{
	mutex_lock(&core_lock);

	seq_printf(file, "cpus online : %u\n", online_cpu_count);
	seq_printf(file, "gpu priority: %u\n", gpu_priority());
	seq_printf(file, "E-state     : %u\n", core_edp_states[core_state]);
	seq_printf(file, "gain        : %u\n", core_platdata->core_gain);
	seq_printf(file, "loan        : %u\n", core_loan);
	seq_printf(file, "core cap    : %u\n", cur_corecap->power);
	seq_printf(file, "cpu balance : %u\n", cpu_power_balance);
	seq_printf(file, "cpu offset  : %u\n", cpu_power_offset);
	seq_printf(file, "cpu power   : %u\n", get_devcap()->cpu_power +
			cpu_power_balance + cpu_power_offset);
	seq_printf(file, "cpu cap     : %u kHz\n", cur_caps.cpu);
	seq_printf(file, "gpu cap     : %u kHz\n", cur_caps.gpu);
	seq_printf(file, "emc cap     : %u kHz\n", cur_caps.emc);

	mutex_unlock(&core_lock);
	return 0;
}

static int longattr_open(struct inode *inode, struct file *file)
{
	return single_open(file, inode->i_private, NULL);
}

static const struct file_operations longattr_fops = {
	.open = longattr_open,
	.read = seq_read,
};

static void create_longattr(const char *name,
		int (*show)(struct seq_file *, void *))
{
	struct dentry *d;

	d = debugfs_create_file(name, S_IRUGO, core_client.dentry, show,
			&longattr_fops);
	WARN_ON(IS_ERR_OR_NULL(d));
}

static void init_debug(void)
{
	if (!core_client.dentry) {
		WARN_ON(1);
		return;
	}

	create_attr("cpu_offset", &cpu_power_offset);
	create_attr("favor_gpu", &force_gpu_pri);
	create_attr("gpu_threshold", &gpu_high_threshold);
	create_attr("force_cpu", &forced_caps.cpu);
	create_attr("force_gpu", &forced_caps.gpu);
	create_attr("force_emc", &forced_caps.emc);
	create_attr("gpu_window", &gpu_window);
	create_attr("gain", &core_platdata->core_gain);
	create_attr("gpu_high_count", &gpu_high_count);

	create_longattr("corecaps", corecaps_show);
	create_longattr("cpucaps", cpucaps_show);
	create_longattr("status", status_show);
}
#else
static inline void init_debug(void) {}
#endif

static void register_loan(struct tegra_sysedp_platform_data *pdata)
{
	struct edp_client *c;
	int r;

	if (!pdata->bbc)
		return;

	c = edp_get_client(pdata->bbc);
	if (!c) {
		pr_info("Could not access modem EDP client\n");
		return;
	}

	r = edp_register_loan(c, &core_client);
	WARN_ON(r && r != -EEXIST);
}

/* Power without gain */
static unsigned int to_base_power(unsigned int power,
		struct tegra_sysedp_platform_data *pdata)
{
	return (power * 100 + pdata->core_gain - 1) / pdata->core_gain;
}

static unsigned int get_num_states(
		struct tegra_sysedp_platform_data *pdata)
{
	unsigned int power = 0;
	unsigned int num = 0;
	unsigned int i;

	for (i = 0; i < pdata->corecap_size; i++) {
		if (pdata->corecap[i].power != power) {
			power = pdata->corecap[i].power;
			num++;
		}
	}

	return num;
}

static void get_states(struct tegra_sysedp_platform_data *pdata,
		unsigned int num, unsigned int *states)
{
	unsigned int power = 0;
	unsigned int e0i = 0;
	unsigned int i;

	for (i = 0; i < pdata->corecap_size; i++) {
		if (pdata->corecap[i].power == power)
			continue;

		power = to_base_power(pdata->corecap[i].power, pdata);
		states[num - e0i - 1] = power;
		e0i++;
	}
}

static unsigned int initial_req(struct edp_client *client,
		struct tegra_sysedp_platform_data *pdata)
{
	int i;
	unsigned int watts;

	watts = to_base_power(pdata->init_req_watts, pdata);

	for (i = 0; i < client->num_states; i++) {
		if (client->states[i] == watts)
			return i;
	}

	WARN_ON(1);
	return 0;
}

static int init_client(struct tegra_sysedp_platform_data *pdata)
{
	struct edp_manager *m;
	unsigned int cnt;
	unsigned int ei;
	int r;

	m = edp_get_manager("battery");
	if (!m)
		return -ENODEV;

	cnt = get_num_states(pdata);
	if (!cnt)
		return -EINVAL;

	core_edp_states = kzalloc(sizeof(*core_edp_states) * cnt, GFP_KERNEL);
	if (!core_edp_states)
		return -ENOMEM;

	get_states(pdata, cnt, core_edp_states);

	core_client.states = core_edp_states;
	core_client.num_states = cnt;
	core_client.e0_index = cnt - 1;
	core_client.private_data = &core_client;

	r = edp_register_client(m, &core_client);
	if (r)
		goto fail;

	ei = initial_req(&core_client, pdata);
	r = edp_update_client_request(&core_client, ei, &core_state);
	if (r)
		return r;

	register_loan(pdata);
	return 0;

fail:
	kfree(core_edp_states);
	core_edp_states = NULL;
	return r;
}

static int init_clks(void)
{
	emc_cap_clk = clk_get_sys("battery_edp", "emc");
	if (IS_ERR(emc_cap_clk))
		return -ENODEV;

	gpu_cap_clk = clk_get_sys("battery_edp", "gpu");
	if (IS_ERR(gpu_cap_clk)) {
		clk_put(emc_cap_clk);
		return -ENODEV;
	}

	return 0;
}

static int tegra_sysedp_probe(struct platform_device *pdev)
{
	int r;

	if (!pdev->dev.platform_data)
		return -EINVAL;

	online_cpu_count = num_online_cpus();
	INIT_DELAYED_WORK(&core_work, core_worker);
	pm_qos_add_request(&cpufreq_qos, PM_QOS_CPU_FREQ_MAX,
			PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);

	r = register_cpu_notifier(&tegra_edp_cpu_nb);
	if (r)
		return r;

	r = init_clks();
	if (r)
		return r;

	r = init_client(pdev->dev.platform_data);
	if (r)
		return r;

	mutex_lock(&core_lock);
	core_platdata = pdev->dev.platform_data;
	update_cur_corecap();
	__do_cap_control();
	mutex_unlock(&core_lock);

	init_debug();

	return 0;
}

static struct platform_driver tegra_sysedp_driver = {
	.probe = tegra_sysedp_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra_sysedp"
	}
};

static __init int tegra_sysedp_init(void)
{
	return platform_driver_register(&tegra_sysedp_driver);
}
late_initcall(tegra_sysedp_init);
