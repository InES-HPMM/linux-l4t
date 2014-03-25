/*
 * USB nonswitch table
 *
 * Copyright (c) 2014, NVIDIA Corporation. All Rights Reserved.
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

/*
 * Some USB Sound device depends on other conditions to prempt sound
 * play/acquire.
 * Prevent them from sending uevent upon connection/disconnection
 */
{
	/*This is the VID and PID for Blake device*/
	USB_DEVICE(0x0955, 0x7210),
},
