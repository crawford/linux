/*
 * Copyright (c) 2015-2016 The Linux Foundation. All rights reserved.
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

#ifndef __QCOM_PERF_EVENT_CPU_H
#define __QCOM_PERF_EVENT_CPU_H

#define pmresr0_el0         S3_5_c11_c3_0
#define pmresr1_el0         S3_5_c11_c3_2
#define pmresr2_el0         S3_5_c11_c3_4
#define pmxevcntcr_el0      S3_5_c11_c0_3

#define RESR_ENABLE         0x8000000000000000ULL

#define ARMV8_PMCR_P        0x00000002 /* Reset counters */
#define ARMV8_PMCR_C        0x00000004 /* Reset cycle counter */

#define QC_EVT_PREFIX       1
#define QC_EVT_MASK         GENMASK(19, 0)
#define QC_EVT_PFX_MASK     GENMASK(19, 16)
#define QC_EVT_REG_MASK     GENMASK(15, 12)
#define QC_EVT_CODE_MASK    GENMASK(11, 4)
#define QC_EVT_GRP_MASK     GENMASK(3, 0)
#define QC_EVT_PFX_SHIFT    16
#define QC_EVT_REG_SHIFT    12
#define QC_EVT_CODE_SHIFT   4
#define QC_EVT_GRP_SHIFT    0
#define QC_MODE_EXCL_MASK   GENMASK(31, 30)
#define QC_EVT_PFX(event)   (((event) & QC_EVT_PFX_MASK)   >> QC_EVT_PFX_SHIFT)
#define QC_EVT_REG(event)   (((event) & QC_EVT_REG_MASK)   >> QC_EVT_REG_SHIFT)
#define QC_EVT_CODE(event)  (((event) & QC_EVT_CODE_MASK)  >> QC_EVT_CODE_SHIFT)
#define QC_EVT_GROUP(event) (((event) & QC_EVT_GRP_MASK)   >> QC_EVT_GRP_SHIFT)

#define QC_MAX_L1_REG       2
#define QC_MAX_GROUP        7

#endif
