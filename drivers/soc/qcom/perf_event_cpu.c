/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * Copyright (C) 2012 ARM Limited
 * Author: Will Deacon <will.deacon@arm.com>
 *
 * ARMv7 support: Jean Pihet <jpihet@mvista.com>
 * 2010 (c) MontaVista Software, LLC.
 *
 * Based on the ARMv8 and ARMv7 Performance Events handling code.
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

#define pr_fmt(fmt) "qc perfevents: " fmt

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/perf_event.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/soc/qcom/perf_event_cpu.h>
#include <linux/perf/arm_pmu.h>
#include <asm/sysreg.h>

#define ARMV8_IDX_CYCLE_COUNTER                 0
#define ARMV8_IDX_COUNTER0                      1
#define ARMV8_PMUV3_PERFCTR_CLOCK_CYCLES        0x11
#define COUNT_MASK                              GENMASK(31, 0)
#define ARMV8_IDX_COUNTER_LAST(cpu_pmu) \
	(ARMV8_IDX_CYCLE_COUNTER + cpu_pmu->num_events - 1)

static const u8 evt_type_base[3] = {0xd8, 0xe0, 0xe8};

struct q_evt {
	/*
	 * The pmresr_setval field corresponds to the value that the pmresr
	 * register needs to be set to. This value is calculated from the code
	 * and group that the event belongs to in the event table.
	 */
	u64 pmresr_setval;
	/* The PMRESR reg that the event belongs to */
	u8 reg;
	u8 group;
	/* The armv8 defined event code that the events map to */
	u32 armv8_evt_type;
};

static void get_evtinfo(unsigned int evtype, struct q_evt *evtinfo)
{
	u8 reg =    QC_EVT_REG(evtype);
	u8 code =   QC_EVT_CODE(evtype);
	u8 group =  QC_EVT_GROUP(evtype);

	evtinfo->pmresr_setval = ((u64)code << (group * 8)) | RESR_ENABLE;
	evtinfo->reg = reg;
	evtinfo->group = group;
	evtinfo->armv8_evt_type = evt_type_base[reg] | group;
}

static void write_pmxevcntcr(u32 val)
{
	write_sysreg(val, pmxevcntcr_el0);
}

static void write_pmresr(int reg, u64 val)
{
	switch (reg) {
	case 0:
		write_sysreg(val, pmresr0_el0);
		break;
	case 1:
		write_sysreg(val, pmresr1_el0);
		break;
	case 2:
		write_sysreg(val, pmresr2_el0);
		break;
	default:
		pr_err("Invalid write to RESR reg %d\n", reg);

	}
}

static u64 read_pmresr(int reg)
{
	u64 val = 0;

	switch (reg) {
	case 0:
		val = read_sysreg(pmresr0_el0);
		break;
	case 1:
		val = read_sysreg(pmresr1_el0);
		break;
	case 2:
		val = read_sysreg(pmresr2_el0);
		break;
	default:
		pr_err("Invalid read of RESR reg %d\n", reg);
	}

	return val;
}

static inline u64 get_columnmask(u32 group)
{
	u64 mask;
	u32 shift = 8 * group;

	mask = ~(GENMASK_ULL(shift + 7, shift));
	if (group == QC_MAX_GROUP)
		mask |= RESR_ENABLE;

	return mask;
}

static void set_resr(struct q_evt *evtinfo)
{
	u64 val;

	val = read_pmresr(evtinfo->reg) & get_columnmask(evtinfo->group);
	val |= evtinfo->pmresr_setval;
	write_pmresr(evtinfo->reg, val);
}

static void clear_resrs(void)
{
	unsigned int i;

	for (i = 0; i <= QC_MAX_L1_REG; i++)
		write_pmresr(i, 0);
}

static void clear_resr(struct q_evt *evtinfo)
{
	u64 val;

	val = read_pmresr(evtinfo->reg) &
		get_columnmask(evtinfo->group);
	write_pmresr(evtinfo->reg, val);
}

static void pmu_disable_event(struct perf_event *event)
{
	unsigned long flags;
	struct hw_perf_event *hwc = &event->hw;
	u32 evtype = hwc->config_base & QC_EVT_MASK;
	struct q_evt evtinfo;
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);
	struct pmu_hw_events *events = this_cpu_ptr(cpu_pmu->hw_events);
	int idx = hwc->idx;

	/* Disable counter and interrupt */
	raw_spin_lock_irqsave(&events->pmu_lock, flags);

	armv8pmu_disable_counter(idx);

	/*
	 * Clear pmresr code
	 * Don't need to set the event if it's a cycle count
	 */
	if (idx != ARMV8_IDX_CYCLE_COUNTER) {
		if (QC_EVT_PFX(evtype)) {
			get_evtinfo(evtype, &evtinfo);
			clear_resr(&evtinfo);
		}
	}
	/* Disable interrupt for this counter */
	armv8pmu_disable_intens(idx);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static unsigned int event_to_bit(struct perf_event *event, unsigned int reg,
			unsigned int group)
{
	unsigned int bit;
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);

	bit = evt_type_base[reg] - evt_type_base[0] + group;
	/* Lower bits are reserved for use by the counters */
	bit += ARMV8_IDX_COUNTER_LAST(cpu_pmu) + 1;

	return bit;
}

static int filter_match(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	u32 evtype = hwc->config_base & QC_EVT_MASK;
	u8 prefix  = QC_EVT_PFX(evtype);
	u8 reg     = QC_EVT_REG(evtype);
	u8 group   = QC_EVT_GROUP(evtype);

	/* Always match ARM-architected events */
	if (prefix == 0)
		return 1;

	if ((group > QC_MAX_GROUP) || (reg > QC_MAX_L1_REG) ||
	    (prefix != QC_EVT_PREFIX))
		return 0;

	return 1;
}

static int get_event_idx(struct pmu_hw_events *cpuc,
			 struct perf_event *event)
{
	unsigned int idx;
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	unsigned long evtype = hwc->config_base & QC_EVT_MASK;
	struct q_evt evtinfo;
	int bit = -1;

	/* Always place a cycle count into the cycle counter. */
	if (evtype == ARMV8_PMUV3_PERFCTR_CLOCK_CYCLES) {
		if (test_and_set_bit(ARMV8_IDX_CYCLE_COUNTER, cpuc->used_mask))
			return -EAGAIN;

		return ARMV8_IDX_CYCLE_COUNTER;
	}

	/* check for column exclusion */
	if (QC_EVT_PFX(evtype)) {
		get_evtinfo(evtype, &evtinfo);
		bit = event_to_bit(event, evtinfo.reg, evtinfo.group);
		if (test_and_set_bit(bit, cpuc->used_mask)) {
			pr_err("column exclusion error for evt %lx\n", evtype);
			event->state = PERF_EVENT_STATE_OFF;
			return -EINVAL;
		}
	}

	/*
	 * For anything other than a cycle counter, try to use
	 * the event counters
	 */
	for (idx = ARMV8_IDX_COUNTER0; idx < cpu_pmu->num_events; ++idx) {
		if (!test_and_set_bit(idx, cpuc->used_mask))
			return idx;
	}

	/* The counters are all in use. */
	if (bit >= 0)
		clear_bit(bit, cpuc->used_mask);

	return -EAGAIN;
}

static void clear_event_idx(struct pmu_hw_events *cpuc,
			    struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	unsigned long evtype = hwc->config_base & QC_EVT_MASK;
	struct q_evt evtinfo;

	if (QC_EVT_PFX(evtype)) {
		get_evtinfo(evtype, &evtinfo);
		clear_bit(event_to_bit(event, evtinfo.reg, evtinfo.group),
			  cpuc->used_mask);
	}
}

static void pmu_enable_event(struct perf_event *event)
{
	unsigned long flags;
	struct hw_perf_event *hwc = &event->hw;
	struct q_evt evtinfo;
	unsigned long long prev_count = local64_read(&hwc->prev_count);
	struct arm_pmu *cpu_pmu = to_arm_pmu(event->pmu);
	struct pmu_hw_events *events = this_cpu_ptr(cpu_pmu->hw_events);
	unsigned long evtype = hwc->config_base & QC_EVT_MASK;
	int idx = hwc->idx;
	int ev_num;

	/*
	 * Enable counter and interrupt, and set the counter to count
	 * the event that we're interested in.
	 */
	raw_spin_lock_irqsave(&events->pmu_lock, flags);

	armv8pmu_disable_counter(idx);

	/* set event for ARM-architected events and CC */
	if (!QC_EVT_PFX(evtype) || (idx == ARMV8_IDX_CYCLE_COUNTER)) {
		armv8pmu_write_evtype(idx, hwc->config_base);
	} else {
		get_evtinfo(evtype, &evtinfo);

		/* Restore Mode-exclusion bits */
		ev_num = evtinfo.armv8_evt_type |
			(hwc->config_base & QC_MODE_EXCL_MASK);

		armv8pmu_write_evtype(idx, ev_num);
		write_pmxevcntcr(0);
		set_resr(&evtinfo);
	}

	/* Enable interrupt for this counter */
	armv8pmu_enable_intens(idx);

	/* Restore prev val */
	cpu_pmu->write_counter(event, prev_count & COUNT_MASK);

	armv8pmu_enable_counter(idx);
	raw_spin_unlock_irqrestore(&events->pmu_lock, flags);
}

static void pmu_reset(void *info)
{
	struct arm_pmu *cpu_pmu = (struct arm_pmu *)info;
	u32 idx, nb_cnt = cpu_pmu->num_events;

	/* Stop all counters and their interrupts */
	for (idx = ARMV8_IDX_CYCLE_COUNTER; idx < nb_cnt; ++idx) {
		armv8pmu_disable_counter(idx);
		armv8pmu_disable_intens(idx);
	}

	/* Clear all pmresrs */
	clear_resrs();

	/* Reset irq status reg */
	armv8pmu_getreset_flags();

	/* Reset all counters */
	armv8pmu_pmcr_write(ARMV8_PMCR_P | ARMV8_PMCR_C);
}

/* NRCCG format for perf RAW codes. */
PMU_FORMAT_ATTR(prefix, "config:16-19");
PMU_FORMAT_ATTR(reg,    "config:12-15");
PMU_FORMAT_ATTR(code,   "config:4-11");
PMU_FORMAT_ATTR(grp,    "config:0-3");

static struct attribute *ev_formats[] = {
	&format_attr_prefix.attr,
	&format_attr_reg.attr,
	&format_attr_code.attr,
	&format_attr_grp.attr,
	NULL,
};

/*
 * Format group is essential to access PMU from userspace
 * via its .name field.
 */
static struct attribute_group pmu_format_group = {
	.name = "format",
	.attrs = ev_formats,
};

static const struct attribute_group *pmu_attr_grps[] = {
	&pmu_format_group,
	NULL,
};

void qc_pmu_init(struct arm_pmu *cpu_pmu)
{
	cpu_pmu->enable			= pmu_enable_event;
	cpu_pmu->disable		= pmu_disable_event;
	cpu_pmu->reset			= pmu_reset;
	cpu_pmu->pmu.attr_groups	= pmu_attr_grps;
	cpu_pmu->get_event_idx		= get_event_idx;
	cpu_pmu->clear_event_idx	= clear_event_idx;
	cpu_pmu->pmu.filter_match	= filter_match;

	clear_resrs();
}
