/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __UAPI_CORESIGHT_STM_H_
#define __UAPI_CORESIGHT_STM_H_

enum {
	OST_ENTITY_NONE			= 0x00,
	OST_ENTITY_FTRACE_EVENTS	= 0x01,
	OST_ENTITY_TRACE_PRINTK		= 0x02,
	OST_ENTITY_TRACE_MARKER		= 0x04,
	OST_ENTITY_DEV_NODE		= 0x08,
	OST_ENTITY_DIAG			= 0xEE,
	OST_ENTITY_QVIEW		= 0xFE,
	OST_ENTITY_MAX			= 0xFF,
};

enum {
	STM_OPTION_NONE			= 0x0,
	STM_OPTION_TIMESTAMPED		= 0x08,
	STM_OPTION_GUARANTEED		= 0x80,
};

#endif
