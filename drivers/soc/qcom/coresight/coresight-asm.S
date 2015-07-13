/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/linkage.h>

/* u64 __invoke_coresight_fn_smc(u64 function_id, u64 arg0, u64 arg1, u64 arg2) */
ENTRY(__invoke_coresight_fn_smc)
        smc     #0
        ret
ENDPROC(__invoke_coresight_fn_smc)
