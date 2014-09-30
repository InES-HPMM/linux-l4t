/*
 * app.c
 *
 * ADSP OS App management
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/tegra_nvadsp.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/dma-buf.h>
#include <linux/slab.h>
#include <linux/elf.h>

#include "aram_manager.h"
#include "os.h"

#define APP_LOADER_MBOX_ID		1

#define ADSP_OS_LOAD_TIMEOUT		5000 /* 5000 ms */

#define OS_LOAD_COMPLETE		0x01
#define LOAD_APP			0x03
#define APP_LOAD_RET_STATUS		0x04
#define APP_INIT			0x05
#define APP_INIT_STATUS			0x06
#define APP_START			0x07
#define APP_START_STATUS		0x08
#define APP_DEINIT			0x09
#define APP_DEINIT_STATUS		0x0A
#define APP_COMPLETE			0x0B
#define APP_UNLOAD			0x0C
#define APP_UNLOAD_STATUS		0x0D

struct nvadsp_app_service {
	char name[NVADSP_NAME_SZ];
	char file[NVADSP_NAME_SZ];
	struct list_head node;
	int instance;
	spinlock_t lock;
	struct list_head app_head;
	const uint32_t token;
	const struct app_mem_size *mem_size;
	int generated_instance_id;
	struct adsp_module *mod;
};

struct nvadsp_app_priv_struct {
	struct platform_device *pdev;
};

struct app_load_data {
	struct app_mem_size mem_size;
#if CONFIG_USE_STATIC_APP_LOAD
	int8_t service_name[NVADSP_NAME_SZ];
#else
	uint32_t adsp_mod_ptr;
	uint64_t adsp_mod_size;
#endif
	uint32_t ser;
#if RECORD_STATS
	uint64_t map_time;
	uint64_t app_load_time;
	uint64_t adsp_send_status_time;
	uint64_t timestamp;
	uint64_t receive_timestamp;
#endif
} __packed;

struct app_init_data {
	uint32_t ser;
	uint64_t host_ref;
	uint32_t app_token; /* holds the address of the app structure */
	uint32_t dram_data_ptr;
	uint32_t dram_shared_ptr;
	uint32_t dram_shared_wc_ptr;
	uint32_t aram_ptr;
	uint32_t aram_flag;
	uint32_t aram_x_ptr;
	uint32_t aram_x_flag;
	nvadsp_app_args_t app_args;
#if RECORD_STATS
	uint64_t app_init_time;
	uint64_t app_mem_instance_map;
	uint64_t app_init_call;
	uint64_t adsp_send_status_time;
	uint64_t timestamp;
	uint64_t receive_timestamp;
#endif
} __packed;

struct app_deinit_data {
	uint32_t ptr;
	uint32_t status;
} __packed;

struct app_start_data {
	uint32_t ptr;
	uint32_t stack_size;
	uint32_t status;
#if RECORD_STATS
	uint64_t app_start_time;
	uint64_t app_thread_creation_time;
	uint64_t app_thread_detach_time;
	uint64_t app_thread_resume_time;
	uint64_t insert_queue_head_time;
	uint64_t thread_yield_time;
	uint64_t thread_resched_time;
	uint64_t kevlog_thread_switch_time;
	uint64_t thread_context_switch_time;
	uint64_t adsp_send_status_time;
	uint64_t timestamp;
	uint64_t receive_timestamp;
#endif
} __packed;

struct app_complete_data {
	uint64_t host_ref;
	int32_t app_status;
	int32_t copy_complete;
} __packed;

struct app_unload_data {
	uint32_t	ser;
	int32_t		status;
} __packed;

struct shared_mem_struct {
	struct app_load_data		app_load;
	struct app_init_data		app_init;
	struct app_start_data		app_start;
	struct app_deinit_data		app_deinit;
	struct app_complete_data	app_complete;
	struct app_unload_data		app_unload;
} __packed;

static struct nvadsp_app_priv_struct priv;
static struct work_struct mailbox_work;
static struct nvadsp_mbox mbox;
static struct list_head service_list;
#if RECORD_STATS
static u64 ns_time_native_load_complete;
#endif

DECLARE_COMPLETION(os_load);
DECLARE_COMPLETION(load_app_status);
DECLARE_COMPLETION(app_init_status);
DECLARE_COMPLETION(app_start_status);
DECLARE_COMPLETION(app_deinit_status);
DECLARE_COMPLETION(app_unload_status);

DEFINE_MUTEX(load_app_mutex);
DEFINE_MUTEX(app_init_mutex);
DEFINE_MUTEX(app_start_mutex);
DEFINE_MUTEX(app_deinit_mutex);
DEFINE_MUTEX(app_unload_mutex);

/* app states need to be changed atomically */
static DEFINE_SPINLOCK(state_lock);

static DEFINE_MUTEX(service_lock_list);

struct shared_mem_struct *shared;

static inline void print_load_stats(const char *appfile,
	struct app_load_stats *stats, struct device *dev)
{
#if RECORD_STATS
	dev_info(dev, "%s total load time %lld us\n",
		appfile, stats->ns_time_load / 1000);
	dev_info(dev, "%s parse load time %lld us\n",
		appfile, stats->ns_time_service_parse / 1000);
	dev_info(dev, "%s module load time %lld us\n",
		appfile, stats->ns_time_module_load / 1000);
	dev_info(dev, "\t request firmware %lld us\n",
		stats->ns_time_req_firmware / 1000);
	dev_info(dev, "\t layout and allocate  %lld us\n",
		stats->ns_time_layout / 1000);
	dev_info(dev, "%s native load time %lld us\n",
		appfile, stats->ns_time_native_load / 1000);
	dev_info(dev, "\t load mbox_send time %lld us\n",
		stats->ns_time_load_mbox_send_time / 1000);
	dev_info(dev, "\t load native wait time %lld us\n",
		stats->ns_time_load_wait_time / 1000);
	dev_info(dev, "\t\t load native load complete %lld us\n",
		stats->ns_time_native_load_complete / 1000);
	dev_info(dev, "\t\t\t adsp map %llu us\n",
		stats->ns_time_adsp_map);
	dev_info(dev, "\t\t\t adsp app load %llu us\n",
		stats->ns_time_adsp_app_load);
	dev_info(dev, "\t\t\t adsp app load send ack %llu us\n",
		stats->ns_time_adsp_send_status);
	dev_info(dev, "\t\t\t latency in receiveing the mbox on adsp %llu\n",
		stats->adsp_receive_timestamp - stats->host_send_timestamp);
	dev_info(dev, "\t\t\t latency in receiveing the mbox on host %llu\n",
		stats->host_receive_timestamp - stats->adsp_receive_timestamp);
	dev_info(dev, "%s timestamp before send %llu\n",
		appfile, stats->host_send_timestamp);
	dev_info(dev, "%s timestamp on adsp receive %llu\n",
		appfile, stats->adsp_receive_timestamp);
	dev_info(dev, "%s timestamp on adsp receive %llu\n",
		appfile, stats->host_receive_timestamp);
#endif
}

static inline void print_init_stats(const char *appfile,
	struct app_init_stats *stats, struct device *dev)
{
#if RECORD_STATS
	dev_info(dev, "%s app takes %lld us to init\n",
		appfile, stats->ns_time_app_init / 1000);
	dev_info(dev, "\t app instance allocation %lld us\n",
		stats->ns_time_app_alloc / 1000);
	dev_info(dev, "\t app instance memory allocation %lld us\n",
		stats->ns_time_instance_memory / 1000);
	dev_info(dev, "\t app instance native init call %lld us\n",
		stats->ns_time_native_call / 1000);
	dev_info(dev, "\t\t app init call on adsp %llu us\n",
		stats->ns_time_adsp_app_init);
	dev_info(dev, "\t\t\t app instance map on adsp %llu us\n",
		stats->ns_time_adsp_mem_instance_map);
	dev_info(dev, "\t\t\t app actual init call on adsp %llu us\n",
		stats->ns_time_adsp_init_call);
	dev_info(dev, "\t\t app send init status ack %llu us\n",
		stats->ns_time_adsp_send_status);
#endif
}

static inline void print_start_stats(const char *appfile,
	struct app_start_stats *stats, struct device *dev)
{
#if RECORD_STATS
	dev_info(dev, "%s app takes %lld us to start\n",
		appfile, stats->ns_time_app_start / 1000);
	dev_info(dev, "\t app instance native init call %lld us\n",
		stats->ns_time_native_call / 1000);
	dev_info(dev, "\t\t app start call on adsp %llu us\n",
		stats->ns_time_adsp_app_start);
	dev_info(dev, "\t\t\t app thread creation %llu us\n",
		stats->ns_time_app_thread_creation);
	dev_info(dev, "\t\t\t app thread detach %llu us\n",
		stats->ns_time_app_thread_detach);
	dev_info(dev, "\t\t\t app threaad resume %llu us\n",
		stats->ns_time_app_thread_resume);
	dev_info(dev, "\t\t app send start status ack %llu us\n",
		stats->ns_time_adsp_send_status);
#endif
}

static void app_complete_notifier(struct work_struct *work)
{
	nvadsp_app_info_t *app = container_of(work,
			struct nvadsp_app_info, complete_work);

	wait_for_nvadsp_app_complete(app);
	app->complete_status_notifier(app, app->return_status);
}

static void notify_update_nvadsp_app_complete(struct app_complete_data *data)
{
	nvadsp_app_info_t *app = (nvadsp_app_info_t *)data->host_ref;
	int *state;
	unsigned long flags;

	app->return_status = data->app_status;

	/* notify adsp can process next data */
	data->copy_complete = 0;

	/* set the app instance as initialized to be reused or deinit */
	state = (int *)&app->state;
	spin_lock_irqsave(&state_lock, flags);
	*state = NVADSP_APP_STATE_INITIALIZED;
	spin_unlock_irqrestore(&state_lock, flags);

	/* notify app instance has completed */
	complete_all(&app->wait_for_app_complete);
}

static void nvadsp_app_receive_thread(struct work_struct *work)
{
	struct device *dev = &priv.pdev->dev;
	struct app_load_data *load;
	int data = -1;

	while (1) {
		nvadsp_mbox_recv(&mbox, &data, true, ~0);
		load = &shared->app_load;
		RECORD_TIMESTAMP(load->receive_timestamp);
		switch (data) {
		case OS_LOAD_COMPLETE:
			dev_dbg(dev, "in OS_LOAD_COMPLETE\n");
			complete(&os_load);
			break;

		case APP_LOAD_RET_STATUS:
			dev_dbg(dev, "in APP_LOAD_RET_STATUS\n");
			RECORD_STAT(ns_time_native_load_complete);
			complete(&load_app_status);
			break;

		case APP_UNLOAD_STATUS:
			dev_dbg(dev, "in APP_LOAD_RET_STATUS\n");
			complete(&app_unload_status);
			break;

		case APP_INIT_STATUS:
			dev_dbg(dev, "in APP_INIT_STATUS\n");
			complete(&app_init_status);
			break;

		case APP_DEINIT_STATUS:
			dev_dbg(dev, "in APP_DEINIT_STATUS\n");
			complete(&app_deinit_status);
			break;

		case APP_START_STATUS:
			dev_dbg(dev, "in APP_START_STATUS\n");
			complete(&app_start_status);
			break;

		case APP_COMPLETE:
			dev_dbg(dev, "in APP_COMPLETE\n");
			notify_update_nvadsp_app_complete(
					&shared->app_complete);
			break;

		default:
			dev_err(dev, "received wrong data\n");
		}
	}
}

int wait_for_adsp_os_load_complete(void)
{
	struct device *dev = &priv.pdev->dev;
	unsigned long timeout;
	int ret;

	timeout = msecs_to_jiffies(ADSP_OS_LOAD_TIMEOUT);
	ret = wait_for_completion_timeout(&os_load, timeout);
	if (!ret) {
		dev_err(dev, "ADSP OS loading timed out\n");
		ret = -ETIMEDOUT;
	} else
		ret = 0;

	return ret;
}
static inline void native_adsp_app_start_stats(struct app_start_stats *stats,
	struct app_start_data *data)
{
	EQUATE_STAT(stats->ns_time_adsp_app_start, data->app_start_time);
	EQUATE_STAT(stats->ns_time_app_thread_creation,
		data->app_thread_creation_time);
	EQUATE_STAT(stats->ns_time_app_thread_detach,
		data->app_thread_detach_time);
	EQUATE_STAT(stats->ns_time_app_thread_resume,
		data->app_thread_resume_time);
	EQUATE_STAT(stats->ns_time_adsp_send_status,
		data->adsp_send_status_time);
	EQUATE_STAT(stats->adsp_receive_timestamp, data->timestamp);
}

static int native_adsp_app_start(nvadsp_app_info_t *app,
	struct app_start_stats *stats)
{
	struct app_start_data *data = &shared->app_start;
	int32_t status;

	data->ptr = app->token;
	data->stack_size = app->stack_size;

	nvadsp_mbox_send(&mbox, APP_START, NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_start_status);
	native_adsp_app_start_stats(stats, data);

	status = data->status;
	memset(data, 0, sizeof(struct app_start_data));

	return status;
}

static struct nvadsp_app_service
*get_loaded_service(const char *appname)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser;

	mutex_lock(&service_lock_list);
	list_for_each_entry(ser, &service_list, node) {
		if (!strcmp(appname, ser->name)) {
			dev_dbg(dev, "module %s already loaded\n", appname);
			mutex_unlock(&service_lock_list);
			return ser;
		}
	}
	mutex_unlock(&service_lock_list);
	dev_dbg(dev, "module %s can be loaded\n", appname);
	return NULL;
}

static inline void native_adsp_load_stats(struct app_load_stats *stats,
	struct app_load_data *data)
{
	EQUATE_STAT(stats->ns_time_native_load_complete,
		ns_time_native_load_complete);
	EQUATE_STAT(stats->ns_time_adsp_map, data->map_time);
	EQUATE_STAT(stats->ns_time_adsp_app_load, data->app_load_time);
	EQUATE_STAT(stats->ns_time_adsp_send_status,
		data->adsp_send_status_time);
	EQUATE_STAT(stats->adsp_receive_timestamp, data->timestamp);
	EQUATE_STAT(stats->host_receive_timestamp, data->receive_timestamp);
	EQUATE_STAT(ns_time_native_load_complete, 0);
}

uint32_t native_adsp_load_app(struct nvadsp_app_service *ser,
	struct app_load_stats *stats)
{
	uint32_t status;
	struct app_load_data *data = &shared->app_load;

	memcpy(&data->mem_size, ser->mem_size,
					sizeof(struct app_mem_size));
#if !CONFIG_USE_STATIC_APP_LOAD
	data->adsp_mod_ptr = ser->mod->adsp_module_ptr;
	data->adsp_mod_size = ser->mod->size;
#else
	strncpy(data->service_name, ser->name, NVADSP_NAME_SZ);
#endif

	RECORD_TIMESTAMP(stats->host_send_timestamp);
	RECORD_STAT(stats->ns_time_load_mbox_send_time);
	nvadsp_mbox_send(&mbox, LOAD_APP, NVADSP_MBOX_SMSG, true, 100);
	RECORD_STAT(stats->ns_time_load_mbox_send_time);

	RECORD_STAT(stats->ns_time_load_wait_time);
	RECORD_STAT(ns_time_native_load_complete);
	wait_for_completion(&load_app_status);
	RECORD_STAT(stats->ns_time_load_wait_time);
	native_adsp_load_stats(stats, data);
	status = data->ser;
	memset(data, 0, sizeof(struct app_load_data));

	return status;
}

void update_nvadsp_app_shared_ptr(void *ptr)
{
	shared = (struct shared_mem_struct *)ptr;
}

nvadsp_app_handle_t
nvadsp_app_load(const char *appname, const char *appfile)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser = NULL;
	struct app_load_stats stats = { };
	struct adsp_module *mod;
	uint32_t *token;

	RECORD_STAT(stats.ns_time_load);
	RECORD_STAT(stats.ns_time_service_parse);
	ser = get_loaded_service(appname);
	RECORD_STAT(stats.ns_time_service_parse);
	if (!ser) {
		dev_dbg(dev, "loading app %s\n", appname);

		RECORD_STAT(stats.ns_time_module_load);
		mod = load_adsp_module(appname, appfile, dev, &stats);
		if (IS_ERR_OR_NULL(mod))
			goto end;
		RECORD_STAT(stats.ns_time_module_load);

		ser = kzalloc(sizeof(struct nvadsp_app_service), GFP_KERNEL);
		strncpy(ser->name, appname, NVADSP_NAME_SZ);
		strncpy(ser->file, appfile, NVADSP_NAME_SZ);
		ser->mod = mod;

		ser->mem_size = &mod->mem_size;
		token = (void *)&ser->token;
		RECORD_STAT(stats.ns_time_native_load);
		mutex_lock(&load_app_mutex);
		*token = native_adsp_load_app(ser, &stats);
		mutex_unlock(&load_app_mutex);
		RECORD_STAT(stats.ns_time_native_load);
		if (!ser->token) {
			dev_err(dev, "unable to load app %s\n", appname);
			kfree(ser);
			goto end;
		}
		spin_lock_init(&ser->lock);
		INIT_LIST_HEAD(&ser->app_head);

		/* add the app instance service to the list */
		mutex_lock(&service_lock_list);
		list_add_tail(&ser->node, &service_list);
		mutex_unlock(&service_lock_list);

		dev_dbg(dev, "loaded app %s\n", ser->name);
	}

	RECORD_STAT(stats.ns_time_load);
	print_load_stats(appfile, &stats, dev);
end:
	return ser;
}
EXPORT_SYMBOL(nvadsp_app_load);

static int
create_instance_memory(nvadsp_app_info_t *app, const struct app_mem_size *sz)
{
	struct device *dev = &priv.pdev->dev;
	adsp_app_mem_t *mem = &app->mem;
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;
	dma_addr_t da;
	void *aram_handle;
	char name[NVADSP_NAME_SZ];

	snprintf(name, NVADSP_NAME_SZ, "%s:%d", app->name, app->instance_id);

	if (sz->dram) {
		mem->dram = nvadsp_alloc_coherent(sz->dram, &da, GFP_KERNEL);
		if (!mem->dram) {
			dev_err(dev,
			"cannot allocate dram memory for app %s instance\n",
			name);
			goto end;
		}
		iova_mem->dram = (uint32_t)da;
		dev_dbg(dev, "%s :: mem.dram %p 0x%x\n",
				name, mem->dram, iova_mem->dram);
	}

	if (sz->dram_shared) {
		mem->shared = nvadsp_alloc_coherent(sz->dram_shared,
							&da, GFP_KERNEL);
		if (!mem->shared) {
			dev_err(dev,
			"cannot allocate dram shared for app %s instance\n",
			name);
			goto free_dram;
		}
		iova_mem->shared = (uint32_t)da;
		dev_dbg(dev, "%s :: mem.shared %p 0x%x\n",
				name, mem->shared, iova_mem->shared);
	}

	if (sz->dram_shared_wc) {
		mem->shared_wc = nvadsp_alloc_coherent(
				sz->dram_shared_wc, &da, GFP_KERNEL);
		if (!mem->shared_wc) {
			dev_err(dev,
			"cannot allocate dram shared for app %s instance\n",
						name);
			goto free_dram_shared;
		}
		iova_mem->shared_wc = (uint32_t)da;
		dev_dbg(dev, "%s: :: mem.shared_wc %p 0x%x\n",
				name, mem->shared_wc, iova_mem->shared_wc);
	}

	if (sz->aram) {
		aram_handle = aram_request(name, sz->aram);
		if (!IS_ERR_OR_NULL(aram_handle)) {
			iova_mem->aram = aram_get_address(aram_handle);
			mem->aram = aram_handle;
			iova_mem->aram_flag = mem->aram_flag = 1;
			dev_dbg(dev, "%s aram %x\n", name, iova_mem->aram);
		} else {
			dev_info(dev,
			"No ARAM memory avialable ! allocating from DRAM for app %s instance\n",
			name);
			mem->aram = nvadsp_alloc_coherent(sz->aram,
					&da, GFP_KERNEL);
			if (!mem->aram) {
				dev_err(dev,
				"cannot allocate aram memory from dram for app %s instance\n",
				name);
				goto free_dram_shared_wc;
			}
			iova_mem->aram = (uint32_t)da;
			iova_mem->aram_flag = mem->aram_flag = 0;
			dev_dbg(dev, "mem.aram %p 0x%x\n",
					mem->aram, iova_mem->aram);
		}
	}

	if (sz->aram_x) {
		aram_handle = aram_request(name, sz->aram);
		if (!IS_ERR_OR_NULL(aram_handle)) {
			iova_mem->aram_x = aram_get_address(aram_handle);
			mem->aram_x = aram_handle;
			iova_mem->aram_x_flag = mem->aram_x_flag = 1;
			dev_dbg(dev, "aram_x %x\n", iova_mem->aram_x);
		} else {
			iova_mem->aram_x = 0;
			iova_mem->aram_x_flag = mem->aram_x_flag = 0;
			dev_err(dev,
			"unable to allocate exclusive memory for app instance %s\n",
			name);
		}
	}
	return 0;

free_dram_shared_wc:
	if (sz->dram_shared_wc)
		nvadsp_free_coherent(sz->dram_shared_wc, mem->shared_wc,
				iova_mem->shared_wc);
free_dram_shared:
	if (sz->dram_shared)
		nvadsp_free_coherent(sz->dram_shared, mem->shared,
				iova_mem->shared);
free_dram:
	if (sz->dram)
		nvadsp_free_coherent(sz->dram, mem->dram, iova_mem->dram);
end:
	return -ENOMEM;
}

static void free_instance_memory(nvadsp_app_info_t *app,
		const struct app_mem_size *sz)
{
	adsp_app_mem_t *mem = &app->mem;
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;

	if (sz->dram)
		nvadsp_free_coherent(sz->dram, mem->dram, iova_mem->dram);

	if (sz->dram_shared)
		nvadsp_free_coherent(sz->dram_shared, mem->shared,
				iova_mem->shared);
	if (sz->dram_shared_wc)
		nvadsp_free_coherent(sz->dram_shared_wc, mem->shared_wc,
				iova_mem->shared_wc);
	if (sz->aram) {
		if (mem->aram_flag)
			aram_release(mem->aram);
		else
			nvadsp_free_coherent(sz->aram,
					mem->aram, iova_mem->aram);
	}

	if (sz->aram_x && mem->aram_x_flag)
		aram_release(mem->aram_x);

}

static inline void native_adsp_init_stats(struct app_init_stats *stats,
	struct app_init_data *data)
{
	EQUATE_STAT(stats->ns_time_adsp_app_init, data->app_init_time);
	EQUATE_STAT(stats->ns_time_adsp_mem_instance_map,
		data->app_mem_instance_map);
	EQUATE_STAT(stats->ns_time_adsp_init_call, data->app_init_call);
	EQUATE_STAT(stats->ns_time_adsp_send_status,
		data->adsp_send_status_time);
	EQUATE_STAT(stats->adsp_receive_timestamp, data->timestamp);
}

static uint32_t
native_adsp_app_init(nvadsp_app_info_t *app,
	const struct nvadsp_app_service *ser, nvadsp_app_args_t *app_args,
	struct app_init_stats *stats)
{
	uint32_t token;
	struct app_init_data *data = &shared->app_init;
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;

	/* copy the iova address to adsp so that adsp can access the memory */
	data->dram_data_ptr = iova_mem->dram;
	data->dram_shared_ptr = iova_mem->shared;
	data->dram_shared_wc_ptr = iova_mem->shared_wc;
	data->aram_ptr = iova_mem->aram;
	data->aram_flag = iova_mem->aram_flag;
	data->aram_x_ptr = iova_mem->aram_x;
	data->aram_x_flag = iova_mem->aram_x_flag;

	if (app_args)
		memcpy(&data->app_args, app_args, sizeof(nvadsp_app_args_t));
	/*
	 * app on adsp needs to know the service from which it should instance
	 */
	data->ser = ser->token;
	/*
	 * app on adsp holds the reference of host app instance to communicate
	 * back when completed. This way we do not need to iterate through the
	 * list to find the instance.
	 */
	data->host_ref = (uint64_t)app;

	/* call the adsp app init function on ADSP */
	nvadsp_mbox_send(&mbox, APP_INIT, NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_init_status);
	native_adsp_init_stats(stats, data);
	/*
	 * store the app instance structure and clearing the value in the shared
	 * memory.
	 */
	token = data->app_token;

	/* clear shared app init data structure */
	memset(data, 0, sizeof(struct app_init_data));
	return token;
}

nvadsp_app_info_t *nvadsp_app_init(nvadsp_app_handle_t handle,
					nvadsp_app_args_t *app_args)
{
	struct nvadsp_app_service *ser = (void *)handle;
	struct device *dev = &priv.pdev->dev;
	nvadsp_app_info_t *app;
	unsigned long flags;
	uint32_t *token;
	int *state;
	int *id;
	struct app_init_stats stats = { };

	RECORD_STAT(stats.ns_time_app_init);
	EQUATE_STAT(stats.ns_time_app_alloc, stats.ns_time_app_init);
	app = kzalloc(sizeof(nvadsp_app_info_t), GFP_KERNEL);
	if (unlikely(!app)) {
		dev_err(dev, "cannot allocate memory for app %s instance\n",
				ser->name);
		goto err_value;
	}
	RECORD_STAT(stats.ns_time_app_alloc);

	/* set the instance name with the app name */
	app->name = ser->name;
	/* associate a unique id */
	id = (int *)&app->instance_id;
	*id = ser->generated_instance_id++;

	/* create the instance memory required by the app instance */
	RECORD_STAT(stats.ns_time_instance_memory);
	if (create_instance_memory(app, ser->mem_size)) {
		dev_err(dev, "instance creation failed for app %s:%d\n",
				app->name, app->instance_id);
		goto free_app;
	}
	RECORD_STAT(stats.ns_time_instance_memory);

	/* token holds the app instance pointer created on adsp */
	token = (void *)&app->token;
	RECORD_STAT(stats.ns_time_native_call);
	mutex_lock(&app_init_mutex);
	*token = native_adsp_app_init(app, ser, app_args, &stats);
	mutex_unlock(&app_init_mutex);
	RECORD_STAT(stats.ns_time_native_call);

	if (!app->token) {
		dev_err(dev, "app failed to initilize %s\n", app->name);
		goto free_instance_memory;
	}

	/*
	 * hold the pointer to the service, to dereference later during deinit
	 */
	app->handle = ser;

	/*
	 * Initilize the complete work which returns the return value after
	 * execution
	 */
	INIT_WORK(&app->complete_work, app_complete_notifier);

	/* set the state to INITIALIZED. No need to do it in a spin lock */
	state = (int *)&app->state;
	*state = NVADSP_APP_STATE_INITIALIZED;

	/* increment instance count and add the app instance to service list */
	spin_lock_irqsave(&ser->lock, flags);
	list_add_tail(&app->node, &ser->app_head);
	ser->instance++;
	spin_unlock_irqrestore(&ser->lock, flags);

	dev_dbg(dev, "app %s instance %d initilized\n",
			app->name, app->instance_id);
	dev_dbg(dev, "app %s has %d instances\n", ser->name, ser->instance);
	goto end;

free_instance_memory:
	free_instance_memory(app, ser->mem_size);
free_app:
	kfree(app);
err_value:
	app = ERR_PTR(-ENOMEM);
end:

	RECORD_STAT(stats.ns_time_app_init);
	print_init_stats(ser->file, &stats, dev);

	return app;
}
EXPORT_SYMBOL(nvadsp_app_init);

static int native_adsp_app_deinit(nvadsp_app_info_t *app)
{
	struct app_deinit_data *data = &shared->app_deinit;
	int32_t status;

	data->ptr = app->token;
	nvadsp_mbox_send(&mbox, APP_DEINIT, NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_deinit_status);
	status = data->status;
	memset(data, 0, sizeof(struct app_deinit_data));
	return status;
}

int nvadsp_app_deinit(nvadsp_app_info_t *app)
{
	struct device *dev = &priv.pdev->dev;
	unsigned long flags;
	int ret = -1;
	int *state;

	state = (int *)&app->state;
	/* check and update state of app atomically */
	spin_lock_irqsave(&state_lock, flags);
	if (*state == NVADSP_APP_STATE_INITIALIZED) {
		*state = NVADSP_APP_STATE_UNKNOWN;
		spin_unlock_irqrestore(&state_lock, flags);

		/* free the adsp instance from app side */
		mutex_lock(&app_deinit_mutex);
		ret = native_adsp_app_deinit(app);
		mutex_unlock(&app_deinit_mutex);

		if (likely(!ret)) {
			struct nvadsp_app_service *ser =
				(struct nvadsp_app_service *)app->handle;
			dev_dbg(dev, "%s:freeing app %s:%d\n",
					__func__, app->name, app->instance_id);

			/* free instance memory */
			free_instance_memory(app, ser->mem_size);
			kfree(app);

			/* update the service app instance manager atomically */
			spin_lock_irqsave(&ser->lock, flags);
			ser->instance--;
			list_del(&app->node);
			dev_dbg(dev, "%s: the app %s has instance %d\n",
					__func__, ser->name, ser->instance);
			spin_unlock_irqrestore(&ser->lock, flags);
		} else {
			/*
			 * this should not happen, This means there is something
			 * wrong.
			 */
			WARN(1, "invalid states maintained on ADSP and HOST for app %s:%d",
					app->name, app->instance_id);
			spin_lock_irqsave(&state_lock, flags);
			*state = NVADSP_APP_STATE_INITIALIZED;
			spin_unlock_irqrestore(&state_lock, flags);
		}
	} else {
		spin_unlock_irqrestore(&state_lock, flags);
	}
	return ret;
}
EXPORT_SYMBOL(nvadsp_app_deinit);

static int native_adsp_app_unload(struct nvadsp_app_service *ser)
{
	struct app_unload_data *data = &shared->app_unload;
	int32_t status;

	data->ser = ser->token;
	nvadsp_mbox_send(&mbox, APP_UNLOAD, NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_unload_status);
	status = data->status;
	memset(data, 0, sizeof(struct app_deinit_data));
	return status;
}

void nvadsp_app_unload(nvadsp_app_handle_t handle)
{
	struct nvadsp_app_service *ser = (struct nvadsp_app_service *)handle;
	struct device *dev = &priv.pdev->dev;
	int ret;

	if (ser->instance) {
		dev_err(dev, "cannot unload app %s, has instances %d\n",
				ser->name, ser->instance);
		return;
	}

	mutex_lock(&service_lock_list);
	list_del(&ser->node);
	mutex_unlock(&service_lock_list);

	mutex_lock(&app_unload_mutex);
	ret = native_adsp_app_unload(ser);
	mutex_unlock(&app_unload_mutex);

	if (ret)
		WARN(1, "%s:invalid counts maintained %s", __func__, ser->name);

	unload_adsp_module(ser->mod);
	kfree(ser);
}
EXPORT_SYMBOL(nvadsp_app_unload);

int nvadsp_app_start(nvadsp_app_info_t *app)
{
	struct nvadsp_app_service *ser = (void *)app->handle;
	struct device *dev = &priv.pdev->dev;
	struct app_start_stats stats = { };
	unsigned long flags;
	int ret = -EINVAL;
	int *state;

	state = (int *)&app->state;

	/*
	 * possiblity is that after deinit start is called ! need to check app
	 * pointer
	 */
	spin_lock_irqsave(&state_lock, flags);
	if (IS_ERR_OR_NULL(app)) {
		dev_err(dev, "unable to start app instance %s:%d\n",
				app->name, app->instance_id);
		spin_unlock_irqrestore(&state_lock, flags);
	} else if (*state == NVADSP_APP_STATE_INITIALIZED) {
		*state = NVADSP_APP_STATE_STARTED;
		spin_unlock_irqrestore(&state_lock, flags);

		/* initilize the complete structure */
		init_completion(&app->wait_for_app_complete);

		/* start the app from adsp side by creating a thread */
		RECORD_STAT(stats.ns_time_native_call);
		mutex_lock(&app_start_mutex);
		ret = native_adsp_app_start(app, &stats);
		mutex_unlock(&app_start_mutex);
		RECORD_STAT(stats.ns_time_native_call);

		if (ret) {
			dev_err(dev,
				"Unable to start app instance adsp thread %s:%d\n",
				app->name, app->instance_id);
			spin_lock_irqsave(&state_lock, flags);
			*state = NVADSP_APP_STATE_INITIALIZED;
			spin_unlock_irqrestore(&state_lock, flags);
			goto end;
		}
		/*
		 * schedule the work if there is notifier call back and adsp
		 * app instance has started
		 */
		if (app->complete_status_notifier)
			schedule_work(&app->complete_work);
	}
end:
	print_start_stats(ser->file, &stats, dev);
	return ret;
}
EXPORT_SYMBOL(nvadsp_app_start);

int nvadsp_app_stop(nvadsp_app_info_t *app)
{
	return -ENOENT;
}

int nvadsp_app_module_probe(struct platform_device *pdev)
{
	int ret;
	uint16_t mbox_id = APP_LOADER_MBOX_ID;

	dev_info(&pdev->dev, "%s\n", __func__);

	priv.pdev = pdev;
	ret = nvadsp_mbox_open(&mbox, &mbox_id, "app_service", NULL, NULL);
	if (ret) {
		pr_info("unable to open mailbox\n");
		goto end;
	}
	INIT_LIST_HEAD(&service_list);
	INIT_WORK(&mailbox_work, nvadsp_app_receive_thread);
	schedule_work(&mailbox_work);
end:
	return ret;
}
