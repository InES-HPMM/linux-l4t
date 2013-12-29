/*
 * nvadsp_app.c
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

nvadsp_app_info_t *nvadsp_app_load(char *appname, char *appfile)
{
	return NULL;
}

void nvadsp_app_init(nvadsp_app_info_t *appinfo)
{
	return;
}

void nvadsp_app_exit(nvadsp_app_info_t *appinfo)
{
	return;
}

void nvadsp_app_unload(nvadsp_app_info_t *info)
{
	return;
}

int nvadsp_app_start(char *app_name, void *arg)
{
	return -ENOENT;
}

int nvadsp_app_stop(char *app_name)
{
	return -ENOENT;
}
