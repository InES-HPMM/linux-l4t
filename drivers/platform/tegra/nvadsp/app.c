/*
 * run_app.c
 *
 * ADSP OS App management
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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

#define APP_LOADER_MBOX_ID		1

#define ADSP_OS_LOAD_TIMEOUT		5000 /* 5000 ms */
#define APP_FRAME_WAIT_TIMEOUT		5000 /* 5000 ms */

/*ADSP message pool structure */
union app_loader_msgq {
	msgq_t msgq;
	struct {
		int32_t header[MSGQ_HEADER_WSIZE];
		int32_t queue[MSGQ_MAX_QUEUE_WSIZE];
	};
};

struct adsp_app_shared_msg_pool {
	union app_loader_msgq		app_loader_send_message;
	union app_loader_msgq		app_loader_recv_message;
} __packed;

/* ADSP app loader message queue */
struct run_app_instance_data {
	uint32_t adsp_mod_ptr;
	uint64_t host_ref;
	uint32_t adsp_ref;
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
	uint32_t message;
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
	uint32_t adsp_ref;
	int32_t status;
	uint32_t message;
} __packed;

#pragma pack(4)
union app_complete_status_message {
	msgq_message_t msgq_msg;
	struct app_complete_status_data data;
};

/*
 * structure to hold the list of app binaries loaded and
 * its associated instances.
*/
struct nvadsp_app_service {
	char name[NVADSP_NAME_SZ];
	struct list_head node;
	int instance;
	struct mutex lock;
	struct list_head app_head;
	const uint32_t token;
	const struct app_mem_size *mem_size;
	int generated_instance_id;
	struct adsp_module *mod;
};

/* nvadsp app loader private structure */
struct nvadsp_app_priv_struct {
	struct platform_device *pdev;
	struct completion os_load_complete;
	struct nvadsp_mbox mbox;
	struct list_head service_list;
	struct mutex service_lock_list;
};

static struct nvadsp_app_priv_struct priv;

static struct nvadsp_app_service *get_loaded_service(const char *appfile)
{
	struct device *dev = &priv.pdev->dev;
	struct nvadsp_app_service *ser;

	list_for_each_entry(ser, &priv.service_list, node) {
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

	mutex_lock(&priv.service_lock_list);
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

		mutex_init(&ser->lock);
		INIT_LIST_HEAD(&ser->app_head);

		/* add the app instance service to the list */
		list_add_tail(&ser->node, &priv.service_list);
		dev_dbg(dev, "loaded app %s\n", ser->name);
	}
	mutex_unlock(&priv.service_lock_list);

	return ser;

err_free_service:
	devm_kfree(dev, ser);
err:
	mutex_unlock(&priv.service_lock_list);
	return NULL;
}


nvadsp_app_handle_t nvadsp_app_load(const char *appname, const char *appfile)
{
	struct nvadsp_drv_data *drv_data;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		return NULL;
	}

	drv_data = platform_get_drvdata(priv.pdev);

	if (!drv_data->adsp_os_running)
		return NULL;

	return app_load(appfile);
}
EXPORT_SYMBOL(nvadsp_app_load);

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
	mutex_lock(&ser->lock);
	list_add_tail(&app->node, &ser->app_head);
	ser->instance++;
	mutex_unlock(&ser->lock);

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

nvadsp_app_info_t __must_check *nvadsp_app_init(nvadsp_app_handle_t handle,
	nvadsp_app_args_t *args)
{
	struct adsp_app_shared_msg_pool *msg_pool;
	union app_loader_message *message;
	struct nvadsp_drv_data *drv_data;
	struct app_loader_data *data;
	nvadsp_app_info_t *app;
	msgq_t *msgq_send;
	int *state;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		goto err;
	}

	drv_data = platform_get_drvdata(priv.pdev);

	if (!drv_data->adsp_os_running)
		goto err;

	message = kzalloc(sizeof(*message), GFP_KERNEL);
	if (!message)
		goto err;

	msg_pool = drv_data->shared_adsp_os_data;
	msgq_send = &msg_pool->app_loader_send_message.msgq;
	data = &message->data;

	app = create_app_instance(handle, args, &data->app_init, NULL, 0);
	if (IS_ERR_OR_NULL(app)) {
		kfree(message);
		goto err;
	}
	app->priv = data;
	data->app_init.message = ADSP_APP_INIT;

	message->msgq_msg.size = MSGQ_MSG_PAYLOAD_WSIZE(*message);
	msgq_queue_message(msgq_send, &message->msgq_msg);

	if (app->return_status) {
		state = (int *)&app->state;
		*state = NVADSP_APP_STATE_STARTED;
	}

	nvadsp_mbox_send(&priv.mbox, 0, NVADSP_MBOX_SMSG, false, 0);

	wait_for_completion(&app->wait_for_app_start);
	init_completion(&app->wait_for_app_start);
	return app;
err:
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(nvadsp_app_init);

static void start_app_on_adsp(nvadsp_app_info_t *app,
	union app_loader_message *message, bool block)
{
	struct adsp_app_shared_msg_pool *msg_pool;
	struct nvadsp_drv_data *drv_data;
	msgq_t *msgq_send;
	int *state;

	drv_data = platform_get_drvdata(priv.pdev);
	msg_pool = drv_data->shared_adsp_os_data;
	msgq_send = &msg_pool->app_loader_send_message.msgq;

	message->msgq_msg.size = MSGQ_MSG_PAYLOAD_WSIZE(*message);
	msgq_queue_message(msgq_send, &message->msgq_msg);

	state = (int *)&app->state;
	*state = NVADSP_APP_STATE_STARTED;

	nvadsp_mbox_send(&priv.mbox, 0, NVADSP_MBOX_SMSG, false, 0);

	if (block) {
		wait_for_completion(&app->wait_for_app_start);
		if (app->return_status) {
			pr_info("app instance failed to start\n");
			state = (int *)&app->state;
			*state = NVADSP_APP_STATE_INITIALIZED;
		}
	}
}

int nvadsp_app_start(nvadsp_app_info_t *app)
{
	union app_loader_message *message = app->priv;
	struct app_loader_data *data = &message->data;
	struct nvadsp_drv_data *drv_data;

	if (IS_ERR_OR_NULL(app))
		return -EINVAL;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		goto err;
	}

	drv_data = platform_get_drvdata(priv.pdev);

	if (!drv_data->adsp_os_running)
		goto err;

	data->app_init.message = ADSP_APP_START;
	data->app_init.adsp_ref = app->token;
	data->app_init.stack_size = app->stack_size;
	start_app_on_adsp(app, app->priv, true);
err:
	return (app->state == NVADSP_APP_STATE_STARTED) ? 0 : -EINVAL;
}
EXPORT_SYMBOL(nvadsp_app_start);

nvadsp_app_info_t *nvadsp_run_app(nvadsp_os_handle_t os_handle,
	const char *appfile, nvadsp_app_args_t *app_args,
	app_complete_status_notifier notifier, uint32_t stack_sz, bool block)
{
	union app_loader_message message = {};
	nvadsp_app_handle_t service_handle;
	struct nvadsp_drv_data *drv_data;
	nvadsp_app_info_t *info =  NULL;
	struct app_loader_data *data;
	struct device *dev;

	if (IS_ERR_OR_NULL(priv.pdev)) {
		pr_err("ADSP Driver is not initialized\n");
		info = ERR_PTR(-EINVAL);
		goto end;
	}

	drv_data = platform_get_drvdata(priv.pdev);
	dev = &priv.pdev->dev;

	if (!drv_data->adsp_os_running)
		goto end;

	if (IS_ERR_OR_NULL(appfile))
		goto end;

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
	data->app_init.message = RUN_ADSP_APP;

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

	dev_dbg(dev, "%s:freeing app %s:%d\n",
		__func__, app->name, app->instance_id);

	/* update the service app instance manager atomically */
	mutex_lock(&ser->lock);
	ser->instance--;
	list_del(&app->node);
	mutex_unlock(&ser->lock);

	/* free instance memory */
	free_instance_memory(app, ser->mem_size);
	kfree(app->priv);
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

int nvadsp_app_deinit(nvadsp_app_info_t *app)
{
	nvadsp_exit_app(app, false);
	return 0;
}
EXPORT_SYMBOL(nvadsp_app_deinit);

int nvadsp_app_stop(nvadsp_app_info_t *app)
{
	return -ENOENT;
}
EXPORT_SYMBOL(nvadsp_app_stop);

void nvadsp_app_unload(nvadsp_app_handle_t handle)
{
	struct nvadsp_drv_data *drv_data;
	struct nvadsp_app_service *ser;
	struct device *dev;

	if (!priv.pdev) {
		pr_err("ADSP Driver is not initialized\n");
		return;
	}

	drv_data = platform_get_drvdata(priv.pdev);
	dev = &priv.pdev->dev;

	if (!drv_data->adsp_os_running)
		return;

	if (IS_ERR_OR_NULL(handle))
		return;

	mutex_lock(&priv.service_lock_list);
	ser = (struct nvadsp_app_service *)handle;
	if (ser->instance) {
		dev_err(dev, "cannot unload app %s, has instances %d\n",
				ser->name, ser->instance);
		return;
	}

	list_del(&ser->node);
	unload_adsp_module(ser->mod);
	devm_kfree(dev, ser);
	mutex_unlock(&priv.service_lock_list);
}
EXPORT_SYMBOL(nvadsp_app_unload);

static status_t nvadsp_app_receive_handler(uint32_t msg, void *hdata)
{
	union app_complete_status_message message = { };
	struct adsp_app_shared_msg_pool *msg_pool;
	struct app_complete_status_data *data;
	struct nvadsp_drv_data *drv_data;
	struct platform_device *pdev;
	nvadsp_app_info_t *app;
	struct device *dev;
	msgq_t *msgq_recv;
	uint32_t *token;

	pdev = hdata;
	dev = &pdev->dev;
	drv_data = platform_get_drvdata(pdev);
	msg_pool = drv_data->shared_adsp_os_data;
	msgq_recv = &msg_pool->app_loader_recv_message.msgq;
	data = &message.data;

	message.msgq_msg.size = MSGQ_MSG_PAYLOAD_WSIZE(*data);
	if (msgq_dequeue_message(msgq_recv, &message.msgq_msg)) {
		dev_err(dev, "unable to dequeue app status message\n");
		return 0;
	}

	if (unlikely(data->message == OS_LOAD_COMPLETE)) {
		complete(&priv.os_load_complete);
		return 0;
	}

	app = (nvadsp_app_info_t *)data->host_ref;
	app->return_status = data->status;
	app->status_msg = data->message;
	token = (uint32_t *)&app->token;
	*token = data->adsp_ref;

	if (app->complete_status_notifier) {
		app->complete_status_notifier(app,
			app->status_msg, app->return_status);
	}

	switch (data->message) {
	case ADSP_APP_START_STATUS:
		complete_all(&app->wait_for_app_start);
		break;
	case ADSP_APP_COMPLETE_STATUS:
		complete_all(&app->wait_for_app_complete);
		break;
	}

	return 0;
}

int wait_for_adsp_os_load_complete(void)
{
	struct device *dev = &priv.pdev->dev;
	unsigned long timeout;
	int ret;

	timeout = msecs_to_jiffies(ADSP_OS_LOAD_TIMEOUT);
	ret = wait_for_completion_timeout(&priv.os_load_complete, timeout);
	if (!ret) {
		dev_err(dev, "ADSP OS loading timed out\n");
		ret = -ETIMEDOUT;
	} else
		ret = 0;

	return ret;
}

int nvadsp_app_module_probe(struct platform_device *pdev)
{
	int ret;
	uint16_t mbox_id = APP_LOADER_MBOX_ID;
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s\n", __func__);

	ret = nvadsp_mbox_open(&priv.mbox, &mbox_id,
		"app_service", nvadsp_app_receive_handler, pdev);
	if (ret) {
		dev_err(dev, "unable to open mailbox\n");
		goto end;
	}
	priv.pdev = pdev;
	INIT_LIST_HEAD(&priv.service_list);
	init_completion(&priv.os_load_complete);
	mutex_init(&priv.service_lock_list);
end:
	return ret;
}
