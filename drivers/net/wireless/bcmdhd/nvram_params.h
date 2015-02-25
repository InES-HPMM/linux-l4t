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
 */

#ifndef _NVRAM_PARAMS_H_
#define _NVRAM_PARAMS_H_

#define COUNTRY_CODE_LEN 2
#define MAX_NVRAMBUF_SIZE 4096
#define MAX_ALL_NVRAMBUF_SIZE (4096*7)

extern unsigned char nvram_params[];

extern char current_ccode[];
extern char current_nvram_code[];

#define WIFI_COUNTRY_FILE "/data/misc/wifi/country.txt"
#define WIFI_NVRAM_FILE "/etc/nvram_bcm43241.txt"
#define WIFI_NVRAM_FILE_BIN "/system/vendor/firmware/bcm43241/nvram_bcm43241.bin"

int nv_dhd_set_nvram_params(char *country_code, struct net_device *net);
char *country_to_nvram_code(char *country_code);
int get_country_code_fs(char *ccode);
int set_country_code_fs(char *ccode);
int get_nvram_plain(char *nvram_plain);
int get_nvram_param(char *nvram_code);
#endif /* _NVRAM_PARAMS_H_ */
