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

#include <linux/tegra_nvadsp.h>
#include <linux/workqueue.h>
#include <linux/elf.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>

#include "os.h"
#include "aram_manager.h"

#define APP_LOADER_MBOX_ID		1

#define OS_LOAD_COMPLETE		0x01
#define LOAD_APP			0x03
#define APP_LOAD_RET_STATUS		0x04
#define APP_INIT			0x05
#define APP_INIT_STATUS			0x06
#define APP_START			0x07
#define APP_START_STATUS		0x08

struct nvadsp_app_service {
	char name[NVADSP_NAME_SZ];
	struct list_head node;
	struct list_head app_head;
	const uint32_t token;
	const struct app_mem_size *mem_size;
	int instance;
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
} __packed;

struct app_init_data {
	uint32_t ser;
	uint32_t app_token; /* holds the address of the app structure */
	uint32_t dram_data_ptr;
	uint32_t dram_shared_ptr;
	uint32_t dram_shared_wc_ptr;
	uint32_t aram_ptr;
	uint32_t aram_flag;
	uint32_t aram_x_ptr;
	uint32_t aram_x_flag;
	nvadsp_app_args_t app_args;
} __packed;


struct app_start_data {
	uint32_t ptr;
	uint32_t stack_size;
	uint32_t status;
} __packed;

struct shared_mem_struct {
	struct app_load_data app_load;
	struct app_init_data app_init;
	struct app_start_data app_start;
} __packed;

enum {
	LOADED,
	INITIALIZED,
	STARTED,
	STOPPED
};

static struct nvadsp_app_priv_struct priv;
static struct work_struct mailbox_work;
static struct nvadsp_mbox mbox;
static struct list_head service_list;

DECLARE_COMPLETION(os_load);
DECLARE_COMPLETION(load_app_status);
DECLARE_COMPLETION(app_init_status);
DECLARE_COMPLETION(app_start_status);
DECLARE_COMPLETION(app_alloc_status);

DEFINE_MUTEX(load_app_mutex);
DEFINE_MUTEX(app_init_mutex);
DEFINE_MUTEX(app_start_mutex);
DEFINE_MUTEX(app_alloc_mutex);

struct shared_mem_struct *shared;

static void nvadsp_app_receive_thread(struct work_struct *work)
{
	int data = -1;
	struct device *dev = &priv.pdev->dev;
	while (1) {
		nvadsp_mbox_recv(&mbox, &data, true, ~0);
		switch (data) {
		case OS_LOAD_COMPLETE:
			dev_dbg(dev, "in OS_LOAD_COMPLETE\n");
			complete(&os_load);
			break;
		case APP_LOAD_RET_STATUS:
			dev_dbg(dev, "in APP_LOAD_RET_STATUS\n");
			complete(&load_app_status);
			break;
		case APP_INIT_STATUS:
			dev_dbg(dev, "in APP_INIT_STATUS\n");
			complete(&app_init_status);
			break;
		case APP_START_STATUS:
			dev_dbg(dev, "in APP_START_STATUS\n");
			complete(&app_start_status);
			break;
		default:
			dev_err(dev, "received wrong data\n");
		}
	}
}

void wait_for_adsp_os_load_complete(void)
{
	wait_for_completion(&os_load);
}

static int native_adsp_app_start(nvadsp_app_info_t *app)
{
	struct app_start_data *data = &shared->app_start;
	data->ptr = app->token;
	data->stack_size = app->stack_size;
	nvadsp_mbox_send(&mbox, APP_START, NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_start_status);
	return data->status;
}

static struct nvadsp_app_service
*get_loaded_service(const char *appname)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser;

	list_for_each_entry(ser, &service_list, node) {
		if (!strcmp(appname, ser->name)) {
			dev_info(dev, "module %s already loaded\n",
							appname);
			return ser;
		}
	}
	dev_info(dev, "module %s can be loaded\n", appname);
	return NULL;
}

uint32_t native_adsp_load_app(struct nvadsp_app_service *ser)
{
	int status;
	struct app_load_data *data = &shared->app_load;
	memcpy(&data->mem_size, ser->mem_size,
					sizeof(struct app_mem_size));
#if !CONFIG_USE_STATIC_APP_LOAD
	data->adsp_mod_ptr =
			ser->mod->adsp_module_ptr;
	data->adsp_mod_size =
			ser->mod->size;
#else
	strncpy(data->service_name, ser->name,
					NVADSP_NAME_SZ);
#endif

	nvadsp_mbox_send(&mbox, LOAD_APP,
				NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&load_app_status);
	status = data->ser;
	data->ser = 0;
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
	struct adsp_module *mod;
	struct nvadsp_app_service *ser;
	uint32_t *token;

	ser = get_loaded_service(appname);
	if (!ser) {
		dev_info(dev, "loading app %s\n", appname);

		mod = load_adsp_module(appname, appfile, dev);
		if (IS_ERR_OR_NULL(mod))
			goto end;

		ser = kzalloc(sizeof(struct nvadsp_app_service), GFP_KERNEL);
		strncpy(ser->name, appname, NVADSP_NAME_SZ);
		ser->mod = mod;

		ser->mem_size = &mod->mem_size;
		token = (void *)&ser->token;
		mutex_lock(&load_app_mutex);
		*token = native_adsp_load_app(ser);
		mutex_unlock(&load_app_mutex);
		if (!ser->token) {
			dev_err(dev, "unable to load app %s\n", appname);
			kfree(ser);
			goto end;
		}
		INIT_LIST_HEAD(&ser->app_head);
		list_add_tail(&ser->node, &service_list);
		dev_info(dev, "loaded app %s\n", ser->name);
	}
end:
	return ser;
}
EXPORT_SYMBOL(nvadsp_app_load);

static void
populate_memory(const char *name, adsp_app_mem_t *mem,
	const struct app_mem_size *sz, struct app_init_data *data)
{
	struct device *dev = &priv.pdev->dev;
	dma_addr_t da;
	void *aram_handle;

	mem->dram = nvadsp_alloc_coherent(sz->dram,
						&da, GFP_KERNEL);
	data->dram_data_ptr =
			!mem->dram ? 0 : (int)da;
	dev_dbg(dev, "mem.dram %p 0x%x", mem->dram, data->dram_data_ptr);

	mem->shared = nvadsp_alloc_coherent(sz->dram_shared, &da, GFP_KERNEL);
	data->dram_shared_ptr =
			!mem->shared ? 0 : (int)da;
	dev_dbg(dev, "mem.shared %p 0x%x", mem->shared, data->dram_shared_ptr);

	mem->shared_wc =
		nvadsp_alloc_coherent(sz->dram_shared_wc, &da, GFP_KERNEL);
	data->dram_shared_wc_ptr =
			!mem->shared_wc ? 0 : (int)da;
	dev_dbg(dev, "mem.shared_wc %p 0x%x",
			mem->shared_wc, data->dram_shared_wc_ptr);

	if (sz->aram) {
		aram_handle = aram_request(name, sz->aram);
		if (!IS_ERR_OR_NULL(aram_handle)) {
			data->aram_ptr = (u32)aram_get_address(aram_handle);
			mem->aram = aram_handle;
			data->aram_flag = mem->aram_flag = 1;
			dev_dbg(dev, "aram %x\n", data->aram_ptr);
		} else {
			dev_info(dev,
			"No ARAM memory avialable ! allocating from DRAM\n");
			mem->aram = nvadsp_alloc_coherent(sz->aram,
					&da, GFP_KERNEL);
			data->aram_ptr = !mem->aram ? 0 : (uint32_t)da;
			data->aram_flag = mem->aram_flag = 0;
			dev_dbg(dev, "mem.aram %p 0x%x",
					mem->aram, data->aram_ptr);
		}
	}

	if (sz->aram_x) {
		aram_handle = aram_request(name, sz->aram);
		if (!IS_ERR_OR_NULL(aram_handle)) {
			data->aram_x_ptr = (u32)aram_get_address(aram_handle);
			mem->aram_x = aram_handle;
			data->aram_x_flag = mem->aram_x_flag = 1;
			dev_dbg(dev, "data->aram_x %x\n", data->aram_x_ptr);
		} else {
			data->aram_x_ptr = 0;
			data->aram_x_flag = mem->aram_x_flag = 0;
			dev_err(dev,
			"unable to allocate exclusive memory for app %s\n",
									name);
		}
	}
}

static uint32_t
native_adsp_app_init(nvadsp_app_info_t *app,
				const struct nvadsp_app_service *ser,
					nvadsp_app_args_t *app_args)
{
	uint32_t token;
	struct app_init_data *data = &shared->app_init;
	populate_memory(app->name, &app->mem, ser->mem_size, data);
	if (app_args)
		memcpy(&data->app_args, app_args,
					sizeof(nvadsp_app_args_t));
	data->ser = ser->token;

	nvadsp_mbox_send(&mbox, APP_INIT,
				NVADSP_MBOX_SMSG, true, 100);
	wait_for_completion(&app_init_status);
	token = data->app_token;
	data->app_token = 0;
	return token;
}

nvadsp_app_info_t *nvadsp_app_init(nvadsp_app_handle_t handle,
					nvadsp_app_args_t *app_args)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser = (void *)handle;
	int *state;
	uint32_t *token;
	nvadsp_app_info_t *app =
		kzalloc(sizeof(nvadsp_app_info_t),
						GFP_KERNEL);
	if (unlikely(!app)) {
		dev_err(dev,
		"cannot allocate memory for app %s instance\n",
							ser->name);
		app = ERR_PTR(-ENOMEM);
		goto end;
	}
	state = (int *)&app->state;
	app->name = ser->name;
	token = (void *)&app->token;
	mutex_lock(&app_init_mutex);
	*token = native_adsp_app_init(app, ser, app_args);
	mutex_unlock(&app_init_mutex);
	if (!app->token) {
		dev_err(dev,
			"app failed to initilize %s\n",
						app->name);
		kfree(app);
		app = ERR_PTR(-EINVAL);
		goto end;
	}
	list_add_tail(&app->node, &ser->app_head);
	ser->instance++;
	*state = INITIALIZED;
	dev_info(dev,
		"app %s instance %d initilized\n", app->name,
						ser->instance);
end:
	return app;
}
EXPORT_SYMBOL(nvadsp_app_init);

void nvadsp_app_unload(nvadsp_app_handle_t handle)
{
	return;
}

int nvadsp_app_start(nvadsp_app_info_t *app)
{
	/*pass token to adsp appp to start the app */
	int ret = -1;
	int *state;
	state = (int *)&app->state;
	if (*state == INITIALIZED) {
		mutex_lock(&app_start_mutex);
		ret = native_adsp_app_start(app);
		mutex_unlock(&app_start_mutex);
		if (!ret)
			*state = STARTED;
	}
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
