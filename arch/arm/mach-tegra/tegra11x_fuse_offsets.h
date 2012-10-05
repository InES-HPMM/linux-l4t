/*
 * copyright (c) 2012, nvidia corporation.
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license as published by
 * the free software foundation; either version 2 of the license, or
 * (at your option) any later version.
 *
 * this program is distributed in the hope that it will be useful, but without
 * any warranty; without even the implied warranty of merchantability or
 * fitness for a particular purpose.  see the gnu general public license for
 * more details.
 *
 * you should have received a copy of the gnu general public license along
 * with this program; if not, write to the free software foundation, inc.,
 * 51 franklin street, fifth floor, boston, ma  02110-1301, usa.
 */

#ifndef __TEGRA11x_FUSE_OFFSETS_H
#define __TEGRA11x_FUSE_OFFSETS_H

#define DEVKEY_START_OFFSET 0x2C
#define DEVKEY_START_BIT    0x07

#define JTAG_START_OFFSET 0x0
#define JTAG_START_BIT    0x3

#define ODM_PROD_START_OFFSET 0x0
#define ODM_PROD_START_BIT    0x4

#define SB_DEVCFG_START_OFFSET 0x2E
#define SB_DEVCFG_START_BIT    0x07

#define SB_DEVSEL_START_OFFSET 0x2E
#define SB_DEVSEL_START_BIT    0x23

#define SBK_START_OFFSET 0x24
#define SBK_START_BIT    0x07

#define SW_RESERVED_START_OFFSET 0x2E
#define SW_RESERVED_START_BIT    0x07

#define IGNORE_DEVSEL_START_OFFSET 0x2E
#define IGNORE_DEVSEL_START_BIT    0x26

#define ODM_RESERVED_DEVSEL_START_OFFSET 0X30
#define ODM_RESERVED_START_BIT    0X0

#endif /* __TEGRA11x_FUSE_OFFSETS_H */
