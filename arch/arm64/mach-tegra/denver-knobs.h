/*
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
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

#ifndef _MACH_DENVER_KNOBS_H_
#define _MACH_DENVER_KNOBS_H_

bool denver_get_bg_allowed(int cpu);
void denver_set_bg_allowed(int cpu, bool enable);

#endif /* _MACH_DENVER_KNOBS_H_ */
