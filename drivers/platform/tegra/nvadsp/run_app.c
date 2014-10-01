/*
 * run_app.c
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
#include "dev.h"

#define APP_LOADER_MBOX_ID		10

#define APP_START			0x07

struct nvadsp_app_service {
	char name[NVADSP_NAME_SZ];
	struct list_head node;
	int instance;
	spinlock_t lock;
	struct list_head app_head;
	const uint32_t token;
	const struct app_mem_size *mem_size;
	int generated_instance_id;
	struct adsp_module *mod;
};

static struct nvadsp_drv_data *drv_data;
static struct device *dev;

struct nvadsp_app_priv_struct {
	struct platform_device *pdev;
};

struct run_app_instance_data {
	uint32_t adsp_mod_ptr;
	uint64_t host_ref;
	uint32_t dram_data_ptr;
	uint32_t dram_shared_ptr;
	uint32_t dram_shared_wc_ptr;
	uint32_t aram_ptr;
	uint32_t aram_flag;
	uint32_t aram_x_ptr;
	uint32_t aram_x_flag;
	struct app_mem_size mem_size;
	nvadsp_app_args_t app_args;
	uint32_t stack_size;
} __packed;

struct app_loader_data {
	int32_t header[MSGQ_MESSAGE_HEADER_WSIZE];
	struct run_app_instance_data	app_init;
} __packed;

#pragma pack(4)
union app_loader_message {
	msgq_message_t msgq_msg;
	struct app_loader_data data;
};

struct app_complete_status_data {
	int32_t header[MSGQ_MESSAGE_HEADER_WSIZE];
	uint64_t host_ref;
	int32_t status;
} __packed;

#pragma pack(4)
union app_complete_status_message {
	msgq_message_t msgq_msg;
	struct app_complete_status_data data;
};

static struct nvadsp_app_priv_struct priv;
static struct nvadsp_mbox mbox;
static struct list_head service_list;

static DEFINE_MUTEX(service_lock_list);

static struct nvadsp_app_service *get_loaded_service(const char *appfile)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser;

	list_for_each_entry(ser, &service_list, node) {
		if (!strcmp(appfile, ser->name)) {
			dev_dbg(dev, "module %s already loaded\n", appfile);
			return ser;
		}
	}
	dev_dbg(dev, "module %s will be loaded\n", appfile);
	return NULL;
}

static nvadsp_app_handle_t app_load(const char *appfile)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser;
	struct app_load_stats stats;

	mutex_lock(&service_lock_list);
	ser = get_loaded_service(appfile);
	if (!ser) {
		dev_dbg(dev, "loading app %s\n", appfile);

		ser = devm_kzalloc(dev, sizeof(*ser), GFP_KERNEL);
		if (!ser)
			goto err;
		strncpy(ser->name, appfile, NVADSP_NAME_SZ);

		/*load the module in to memory */
		ser->mod = load_adsp_module(appfile, appfile, dev, &stats);
		if (IS_ERR_OR_NULL(ser->mod))
			goto err_free_service;
		ser->mem_size = &ser->mod->mem_size;

		spin_lock_init(&ser->lock);
		INIT_LIST_HEAD(&ser->app_head);

		/* add the app instance service to the list */
		list_add_tail(&ser->node, &service_list);
		dev_dbg(dev, "loaded app %s\n", ser->name);
	}
	mutex_unlock(&service_lock_list);

	return ser;

err_free_service:
	devm_kfree(dev, ser);
err:
	mutex_unlock(&service_lock_list);
	return NULL;
}

static void free_instance_memory(nvadsp_app_info_t *app,
		const struct app_mem_size *sz)
{
	adsp_app_mem_t *mem = &app->mem;
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;

	if (mem->dram) {
		nvadsp_free_coherent(sz->dram, mem->dram, iova_mem->dram);
		mem->dram = NULL;
		iova_mem->dram = 0;
	}

	if (mem->shared) {
		nvadsp_free_coherent(sz->dram_shared, mem->shared,
				iova_mem->shared);
		mem->shared = NULL;
		iova_mem->shared = 0;
	}

	if (mem->shared_wc) {
		nvadsp_free_coherent(sz->dram_shared_wc, mem->shared_wc,
				iova_mem->shared_wc);
		mem->shared_wc = NULL;
		iova_mem->shared_wc = 0;
	}

	if (mem->aram_flag)
		aram_release(mem->aram);
	else if (mem->aram)
		nvadsp_free_coherent(sz->aram, mem->aram, iova_mem->aram);
	mem->aram = NULL;
	iova_mem->aram = 0;
	mem->aram_flag = 0;

	if (mem->aram_x_flag) {
		aram_release(mem->aram_x);
		mem->aram_x = NULL;
		iova_mem->aram_x = 0;
		mem->aram_flag = 0;
	}

}

static int create_instance_memory(nvadsp_app_info_t *app,
		const struct app_mem_size *sz)
{
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;
	struct device *dev = &priv.pdev->dev;
	adsp_app_mem_t *mem = &app->mem;
	char name[NVADSP_NAME_SZ];
	void *aram_handle;
	dma_addr_t da;

	snprintf(name, NVADSP_NAME_SZ, "%s:%d", app->name, app->instance_id);

	if (sz->dram) {
		mem->dram = nvadsp_alloc_coherent(sz->dram, &da, GFP_KERNEL);
		iova_mem->dram = (uint32_t)da;
		if (!mem->dram) {
			dev_err(dev, "app %s dram alloc failed\n",
				name);
			goto end;
		}
		dev_dbg(dev, "%s :: mem.dram %p 0x%x\n", name,
			mem->dram, iova_mem->dram);
	}

	if (sz->dram_shared) {
		mem->shared = nvadsp_alloc_coherent(sz->dram_shared,
				&da, GFP_KERNEL);
		if (!mem->shared) {
			dev_err(dev, "app %s shared dram alloc failed\n",
				name);
			goto end;
		}
		iova_mem->shared = (uint32_t)da;
		dev_dbg(dev, "%s :: mem.shared %p 0x%x\n", name,
			mem->shared, iova_mem->shared);
	}

	if (sz->dram_shared_wc) {
		mem->shared_wc = nvadsp_alloc_coherent(sz->dram_shared_wc,
					&da, GFP_KERNEL);
		if (!mem->shared_wc) {
			dev_err(dev, "app %s shared dram wc alloc failed\n",
				name);
			goto end;
		}
		iova_mem->shared_wc = (uint32_t)da;
		dev_dbg(dev, "%s :: mem.shared_wc %p 0x%x\n", name,
			mem->shared_wc, iova_mem->shared_wc);
	}

	if (sz->aram) {
		aram_handle = aram_request(name, sz->aram);
		if (!IS_ERR_OR_NULL(aram_handle)) {
			iova_mem->aram = aram_get_address(aram_handle);
			mem->aram = aram_handle;
			iova_mem->aram_flag = mem->aram_flag = 1;
			dev_dbg(dev, "%s aram %x\n", name, iova_mem->aram);
		} else {
			dev_dbg(dev, "app %s no ARAM memory ! using DRAM\n",
				name);
			mem->aram = nvadsp_alloc_coherent(sz->aram,
					&da, GFP_KERNEL);
			if (!mem->aram) {
				iova_mem->aram_flag = mem->aram_flag = 0;
				dev_err(dev,
					"app %s aram memory alloc failed\n",
					name);
				goto end;
			}
			iova_mem->aram = (uint32_t)da;
			dev_dbg(dev, "%s :: mem.aram %p 0x%x\n", name,
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
			dev_err(dev, "app %s aram x memory alloc failed\n",
				name);
		}
	}
	return 0;

end:
	free_instance_memory(app, sz);
	return -ENOMEM;
}

static void fill_app_instance_data(nvadsp_app_info_t *app,
	struct nvadsp_app_service *ser, nvadsp_app_args_t *app_args,
	struct run_app_instance_data *data, uint32_t stack_sz)
{
	adsp_app_iova_mem_t *iova_mem = &app->iova_mem;

	data->adsp_mod_ptr = ser->mod->adsp_module_ptr;
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
	 * app on adsp holds the reference of host app instance to communicate
	 * back when completed. This way we do not need to iterate through the
	 * list to find the instance.
	 */
	data->host_ref = (uint64_t)app;

	/* copy instance mem_size */
	memcpy(&data->mem_size, ser->mem_size, sizeof(struct app_mem_size));
}

static nvadsp_app_info_t *create_app_instance(nvadsp_app_handle_t handle,
	nvadsp_app_args_t *app_args, struct run_app_instance_data *data,
	app_complete_status_notifier notifier, uint32_t stack_size)
{
	struct nvadsp_app_service *ser = (void *)handle;
	struct device *dev = &priv.pdev->dev;
	nvadsp_app_info_t *app;
	unsigned long flags;
	int *state;
	int *id;

	app = kzalloc(sizeof(*app), GFP_KERNEL);
	if (unlikely(!app)) {
		dev_err(dev, "cannot allocate memory for app %s instance\n",
				ser->name);
		goto err_value;
	}
	/* set the instance name with the app name */
	app->name = ser->name;
	/* associate a unique id */
	id = (int *)&app->instance_id;
	*id = ser->generated_instance_id++;
	/*
	 * hold the pointer to the service, to dereference later during deinit
	 */
	app->handle = ser;

	/* create the instance memory required by the app instance */
	if (create_instance_memory(app, ser->mem_size)) {
		dev_err(dev, "instance creation failed for app %s:%d\n",
				app->name, app->instance_id);
		goto free_app;
	}

	/* assign the stack that is needed by the app */
	data->stack_size  = stack_size;

	/* set the state to INITIALIZED. No need to do it in a spin lock */
	state = (int *)&app->state;
	*state = NVADSP_APP_STATE_INITIALIZED;

	/* increment instance count and add the app instance to service list */
	spin_lock_irqsave(&ser->lock, flags);
	list_add_tail(&app->node, &ser->app_head);
	ser->instance++;
	spin_unlock_irqrestore(&ser->lock, flags);

	fill_app_instance_data(app, ser, app_args, data, stack_size);

	init_completion(&app->wait_for_app_start);
	init_completion(&app->wait_for_app_complete);
	set_app_complete_notifier(app, notifier);

	dev_dbg(dev, "app %s instance %d initilized\n",
			app->name, app->instance_id);
	dev_dbg(dev, "app %s has %d instances\n", ser->name, ser->instance);
	goto end;

free_app:
	kfree(app);
err_value:
	app = ERR_PTR(-ENOMEM);
end:
	return app;
}

static void start_app_on_adsp(nvadsp_app_info_t *app,
	union app_loader_message *message, bool block)
{
	struct shared_mem_struct *shared_struct = drv_data->shared_adsp_os_data;
	msgq_t *msgq_send = &shared_struct->app_loader_send_message.msgq;
	int *state;

	message->msgq_msg.size = MSGQ_MSG_PAYLOAD_WSIZE(*message);
	msgq_queue_message(msgq_send, &message->msgq_msg);

	if (app->return_status) {
		state = (int *)&app->state;
		*state = NVADSP_APP_STATE_STARTED;
	}

	nvadsp_mbox_send(&mbox, APP_START, NVADSP_MBOX_SMSG, false, 0);

	if (block) {
		wait_for_completion(&app->wait_for_app_start);
		if (app->return_status) {
			state = (int *)&app->state;
			*state = NVADSP_APP_STATE_INITIALIZED;
		}
	}
}

nvadsp_app_info_t *nvadsp_run_app(nvadsp_os_handle_t os_handle,
	const char *appfile, nvadsp_app_args_t *app_args,
	app_complete_status_notifier notifier, uint32_t stack_sz, bool block)
{
	union app_loader_message message = {};
	nvadsp_app_handle_t service_handle;
	nvadsp_app_info_t *info =  NULL;
	struct app_loader_data *data;
	struct device *dev;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		info = ERR_PTR(-EINVAL);
		goto end;
	}

	dev = &priv.pdev->dev;
	data = &message.data;
	service_handle = app_load(appfile);
	if (!service_handle) {
		dev_err(dev, "unable to load the app %s\n", appfile);
		goto end;
	}

	info = create_app_instance(service_handle, app_args,
		&data->app_init, notifier, stack_sz);
	if (IS_ERR_OR_NULL(info)) {
		dev_err(dev, "unable to create instance for app %s\n", appfile);
		goto end;
	}

	start_app_on_adsp(info, &message, block);
end:
	return info;
}
EXPORT_SYMBOL(nvadsp_run_app);

static void delete_app_instance(nvadsp_app_info_t *app)
{
	struct nvadsp_app_service *ser =
		(struct nvadsp_app_service *)app->handle;
	struct device *dev = &priv.pdev->dev;
	unsigned long flags;

	dev_dbg(dev, "%s:freeing app %s:%d\n",
		__func__, app->name, app->instance_id);

	/* update the service app instance manager atomically */
	spin_lock_irqsave(&ser->lock, flags);
	ser->instance--;
	list_del(&app->node);
	spin_unlock_irqrestore(&ser->lock, flags);

	/* free instance memory */
	free_instance_memory(app, ser->mem_size);
	kfree(app);
}

void nvadsp_exit_app(nvadsp_app_info_t *app, bool terminate)
{
	int *state;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	if (IS_ERR_OR_NULL(app))
		return;

	/* TODO: add termination if possible to kill thread on adsp */
	wait_for_completion(&app->wait_for_app_complete);
	state = (int *)&app->state;
	*state = NVADSP_APP_STATE_INITIALIZED;
	delete_app_instance(app);
}
EXPORT_SYMBOL(nvadsp_exit_app);

static status_t nvadsp_app_receive_handler(uint32_t msg, void *hdata)
{
	union app_complete_status_message message = { };
	struct shared_mem_struct *shared_adsp_struct;
	struct app_complete_status_data *data;
	struct nvadsp_drv_data *drv_data;
	struct platform_device *pdev;
	nvadsp_app_info_t *app;
	struct device *dev;
	msgq_t *msgq_recv;

	pdev = hdata;
	dev = &pdev->dev;
	drv_data = platform_get_drvdata(pdev);
	shared_adsp_struct = drv_data->shared_adsp_os_data;
	msgq_recv = &shared_adsp_struct->app_loader_recv_message.msgq;
	data = &message.data;

	message.msgq_msg.size = MSGQ_MSG_PAYLOAD_WSIZE(*data);
	if (msgq_dequeue_message(msgq_recv, &message.msgq_msg)) {
		dev_err(dev, "unable to dequeue app status message\n");
		return 0;
	}
	app = (nvadsp_app_info_t *)data->host_ref;
	app->return_status = data->status;
	app->status_msg = msg;

	if (app->complete_status_notifier) {
		app->complete_status_notifier(app,
			app->status_msg, app->return_status);
	}

	switch (msg) {
	case ADSP_APP_START_STATUS:
		complete_all(&app->wait_for_app_start);
		break;
	case ADSP_APP_COMPLETE_STATUS:
		complete_all(&app->wait_for_app_complete);
		break;
	}

	return 0;
}

int nvadsp_run_app_module_probe(struct platform_device *pdev)
{
	int ret;
	uint16_t mbox_id = APP_LOADER_MBOX_ID;

	dev_info(&pdev->dev, "%s\n", __func__);

	dev = &pdev->dev;
	priv.pdev = pdev;
	ret = nvadsp_mbox_open(&mbox, &mbox_id,
		"app_service", nvadsp_app_receive_handler, pdev);
	if (ret) {
		dev_err(dev, "unable to open mailbox\n");
		goto end;
	}
	INIT_LIST_HEAD(&service_list);
	drv_data = platform_get_drvdata(pdev);

end:
	return ret;
}
