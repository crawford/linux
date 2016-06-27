/*
 * Copyright (c) 2015, 2016 The Linux Foundation. All rights reserved.
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

#ifndef __QCOM_PERF_EVENT_L2_H
#define __QCOM_PERF_EVENT_L2_H

#define MAX_L2_CTRS             17
#define MAX_L2_EVENTS           32

#define PMCR_NUM_EV_SHIFT       11
#define PMCR_NUM_EV_MASK        0x1f

#define L2PMCR                  0x400
#define L2PMCNTENCLR            0x403
#define L2PMCNTENSET            0x404
#define L2PMINTENCLR            0x405
#define L2PMINTENSET            0x406
#define L2PMOVSCLR              0x407
#define L2PMOVSSET              0x408
#define L2PMCCNTCR              0x409
#define L2PMCCNTR               0x40A
#define L2PMCCNTSR              0x40C
#define L2PMRESR                0x410
#define IA_L2PMXEVCNTCR_BASE    0x420
#define IA_L2PMXEVCNTR_BASE     0x421
#define IA_L2PMXEVFILTER_BASE   0x423
#define IA_L2PMXEVTYPER_BASE    0x424

#define IA_L2_REG_OFFSET        0x10

#define L2PMXEVFILTER_SUFILTER_ALL      0x000E0000
#define L2PMXEVFILTER_ORGFILTER_IDINDEP 0x00000004
#define L2PMXEVFILTER_ORGFILTER_ALL     0x00000003

#define L2PM_CC_ENABLE          0x80000000

#define L2EVTYPER_REG_SHIFT     3

#define L2CYCLE_CTR_BIT         31
#define L2CYCLE_CTR_RAW_CODE    0xFE

#define L2PMCR_RESET_ALL        0x6
#define L2PMCR_GLOBAL_ENABLE    0x1
#define L2PMCR_GLOBAL_DISABLE   0x0

#define L2PMRESR_EN             ((u64)1 << 63)

#define L2_EVT_MASK             0xFFFFF
#define L2_EVT_PFX_MASK         0xF0000
#define L2_EVT_REG_MASK         0x0F000
#define L2_EVT_CODE_MASK        0x00FF0
#define L2_EVT_GROUP_MASK       0x0000F
#define L2_EVT_REG_GROUP_MASK   (L2_EVT_REG_MASK | L2_EVT_GROUP_MASK)
#define L2_EVT_PFX_SHIFT        16
#define L2_EVT_REG_SHIFT        12
#define L2_EVT_CODE_SHIFT        4
#define L2_EVT_GROUP_SHIFT       0
#define L2_EVT_PREFIX(event) ((event & L2_EVT_PFX_MASK) >> L2_EVT_PFX_SHIFT)
#define L2_EVT_REG(event)    ((event & L2_EVT_REG_MASK) >> L2_EVT_REG_SHIFT)
#define L2_EVT_CODE(event)   ((event & L2_EVT_CODE_MASK) >> L2_EVT_CODE_SHIFT)
#define L2_EVT_GROUP(event)  ((event & L2_EVT_GROUP_MASK) >> L2_EVT_GROUP_SHIFT)

#define L2_EVT_GROUP_MAX         7

#define L2_TRACECTR_PREFIX       5

#define L2_MAX_PERIOD           U32_MAX
#define L2_CNT_PERIOD           (U32_MAX - 0x07FFFFFF)

#define MAX_CPUS_IN_CLUSTER     16

#endif
