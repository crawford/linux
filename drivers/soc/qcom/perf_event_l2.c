/* Copyright (c) 2015,2016 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "l2 perfevents: " fmt

#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/acpi.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/perf_event_l2.h>
#include <linux/soc/qcom/l2-accessors.h>

/*
 * The cache is made-up of one or more slices, each slice has its own PMU.
 * This structure represents one of the hardware PMUs.
 */
struct hml2_pmu {
	struct list_head entry;
	struct perf_event *events[MAX_L2_CTRS];
	unsigned long used_mask[BITS_TO_LONGS(MAX_L2_EVENTS)];
	unsigned int valid_cpus;
	u8 cpu[MAX_CPUS_IN_CLUSTER];
	atomic64_t prev_count[MAX_L2_CTRS];
	spinlock_t pmu_lock;
};

/*
 * Aggregate PMU. Implements the core pmu functions and manages
 * the hardware PMUs.
 */
struct l2cache_pmu {
	u32 num_pmus;
	struct list_head pmus;
	struct pmu pmu;
	int num_counters;
};

#define to_l2cache_pmu(p) (container_of(p, struct l2cache_pmu, pmu))

static DEFINE_PER_CPU(struct hml2_pmu *, cpu_to_pmu);
static struct l2cache_pmu l2cache_pmu = { 0 };
static int num_cpus_in_cluster;

static u32 l2_cycle_ctr_idx;
static u32 l2_reset_mask;
static u32 mpidr_affl1_shift;

static inline u32 idx_to_reg(u32 idx)
{
	u32 bit;

	if (idx == l2_cycle_ctr_idx)
		bit = BIT(L2CYCLE_CTR_BIT);
	else
		bit = BIT(idx);
	return bit;
}

static struct hml2_pmu *get_hml2_pmu(struct l2cache_pmu *system, int cpu)
{
	if (cpu < 0)
		cpu = smp_processor_id();

	return per_cpu(cpu_to_pmu, cpu);
}

static void hml2_pmu__reset_on_slice(void *x)
{
	/* Reset all ctrs */
	set_l2_indirect_reg(L2PMCR, L2PMCR_RESET_ALL);
	set_l2_indirect_reg(L2PMCNTENCLR, l2_reset_mask);
	set_l2_indirect_reg(L2PMINTENCLR, l2_reset_mask);
	set_l2_indirect_reg(L2PMOVSCLR, l2_reset_mask);
}

static inline void hml2_pmu__reset(struct hml2_pmu *slice)
{
	int i;

	if (per_cpu(cpu_to_pmu, smp_processor_id()) == slice) {
		hml2_pmu__reset_on_slice(NULL);
		return;
	}

	/* Call each cpu in the cluster until one works */
	for (i = 0; i < slice->valid_cpus; i++) {
		if (!smp_call_function_single(slice->cpu[i],
					      hml2_pmu__reset_on_slice,
					      NULL, 1))
			return;
	}

	pr_err("Failed to reset on cluster with cpu %d\n", slice->cpu[0]);
}

static inline void hml2_pmu__init(struct hml2_pmu *slice)
{
	hml2_pmu__reset(slice);
}

static inline void hml2_pmu__enable(void)
{
	/* Ensure all programming commands are done before proceeding */
	wmb();
	set_l2_indirect_reg(L2PMCR, L2PMCR_GLOBAL_ENABLE);
}

static inline void hml2_pmu__disable(void)
{
	set_l2_indirect_reg(L2PMCR, L2PMCR_GLOBAL_DISABLE);
	/* Ensure the basic counter unit is stopped before proceeding */
	wmb();
}

static inline void hml2_pmu__counter_set_value(u32 idx, u64 value)
{
	u32 counter_reg;

	if (idx == l2_cycle_ctr_idx) {
		set_l2_indirect_reg(L2PMCCNTR, value);
	} else {
		counter_reg = (idx * 16) + IA_L2PMXEVCNTR_BASE;
		set_l2_indirect_reg(counter_reg, (u32)(value & 0xFFFFFFFF));
	}
}

static inline u64 hml2_pmu__counter_get_value(u32 idx)
{
	u64 value;
	u32 counter_reg;

	if (idx == l2_cycle_ctr_idx) {
		value = get_l2_indirect_reg(L2PMCCNTR);
	} else {
		counter_reg = (idx * 16) + IA_L2PMXEVCNTR_BASE;
		value = get_l2_indirect_reg(counter_reg);
	}

	return value;
}

static inline void hml2_pmu__counter_enable(u32 idx)
{
	u32 reg;

	reg = get_l2_indirect_reg(L2PMCNTENSET);
	reg |= idx_to_reg(idx);
	set_l2_indirect_reg(L2PMCNTENSET, reg);
}

static inline void hml2_pmu__counter_disable(u32 idx)
{
	set_l2_indirect_reg(L2PMCNTENCLR, idx_to_reg(idx));
}

static inline void hml2_pmu__counter_enable_interrupt(u32 idx)
{
	u32 reg;

	reg = get_l2_indirect_reg(L2PMINTENSET);
	reg |= idx_to_reg(idx);
	set_l2_indirect_reg(L2PMINTENSET, reg);
}

static inline void hml2_pmu__counter_disable_interrupt(u32 idx)
{
	set_l2_indirect_reg(L2PMINTENCLR, idx_to_reg(idx));
}

static inline void hml2_pmu__set_evcntcr(u32 ctr, u32 val)
{
	u32 evtcr_reg = (ctr * IA_L2_REG_OFFSET) + IA_L2PMXEVCNTCR_BASE;

	set_l2_indirect_reg(evtcr_reg, val);
}

static inline void hml2_pmu__set_evtyper(u32 val, u32 ctr)
{
	u32 evtype_reg = (ctr * IA_L2_REG_OFFSET) + IA_L2PMXEVTYPER_BASE;

	set_l2_indirect_reg(evtype_reg, val);
}

static void hml2_pmu__set_evres(struct hml2_pmu *slice,
				u32 event_group, u32 event_reg, u32 event_cc)
{
	u64 group_val;
	u64 group_mask;
	u64 resr_val;
	u32 shift;
	unsigned long iflags;

	shift = 8 * event_group;
	group_val = ((u64)(event_cc & 0xff) << shift) | L2PMRESR_EN;
	group_mask = ~(GENMASK_ULL(shift+7, shift));

	spin_lock_irqsave(&slice->pmu_lock, iflags);

	resr_val = get_l2_indirect_reg(L2PMRESR);
	resr_val &= group_mask;
	resr_val |= group_val;
	set_l2_indirect_reg(L2PMRESR, resr_val);

	spin_unlock_irqrestore(&slice->pmu_lock, iflags);
}

static void hml2_pmu__set_evfilter_task_mode(int ctr)
{
	u32 filter_reg = (ctr * 16) + IA_L2PMXEVFILTER_BASE;
	u32 l2_orig_filter = L2PMXEVFILTER_SUFILTER_ALL |
			     L2PMXEVFILTER_ORGFILTER_IDINDEP;
	u32 filter_val = l2_orig_filter | 1 << (smp_processor_id() % 2);

	set_l2_indirect_reg(filter_reg, filter_val);
}

static void hml2_pmu__set_evfilter_sys_mode(int ctr, int cpu, bool is_tracectr)
{
	u32 filter_reg = (ctr * IA_L2_REG_OFFSET) + IA_L2PMXEVFILTER_BASE;
	u32 filter_val;
	u32 l2_orig_filter = L2PMXEVFILTER_SUFILTER_ALL |
			     L2PMXEVFILTER_ORGFILTER_IDINDEP;

	if (is_tracectr)
		filter_val = l2_orig_filter | 1 << (cpu % 2);
	else
		filter_val = l2_orig_filter | L2PMXEVFILTER_ORGFILTER_ALL;

	set_l2_indirect_reg(filter_reg, filter_val);
}

static inline u32 hml2_pmu__getreset_ovsr(void)
{
	u32 result = get_l2_indirect_reg(L2PMOVSSET);

	set_l2_indirect_reg(L2PMOVSCLR, result);
	return result;
}

static inline bool hml2_pmu__has_overflowed(u32 ovsr)
{
	return !!(ovsr & l2_reset_mask);
}

static inline bool hml2_pmu__counter_has_overflowed(u32 ovsr, u32 idx)
{
	return !!(ovsr & idx_to_reg(idx));
}

static void l2_cache__event_update_from_slice(struct perf_event *event,
					      struct hml2_pmu *slice)
{
	struct hw_perf_event *hwc = &event->hw;
	u64 delta64, prev, now;
	u32 delta;
	u32 idx = hwc->idx;

	do {
		prev = atomic64_read(&slice->prev_count[idx]);
		now = hml2_pmu__counter_get_value(idx);
	} while (atomic64_cmpxchg(&slice->prev_count[idx], prev, now) != prev);

	if (idx == l2_cycle_ctr_idx) {
		/*
		 * The cycle counter is 64-bit so needs separate handling
		 * of 64-bit delta.
		 */
		delta64 = now - prev;
		local64_add(delta64, &event->count);
		local64_sub(delta64, &hwc->period_left);
	} else {
		/*
		 * 32-bit counters need the unsigned 32-bit math to handle
		 * overflow and now < prev
		 */
		delta = now - prev;
		local64_add(delta, &event->count);
		local64_sub(delta, &hwc->period_left);
	}
}

static void l2_cache__slice_set_period(struct hml2_pmu *slice,
				       struct hw_perf_event *hwc)
{
	u64 value = L2_MAX_PERIOD - (hwc->sample_period - 1);
	u32 idx = hwc->idx;
	u64 prev = atomic64_read(&slice->prev_count[idx]);

	if (prev < value) {
		value += prev;
		atomic64_set(&slice->prev_count[idx], value);
	} else {
		value = prev;
	}

	hml2_pmu__counter_set_value(idx, value);
}

static int l2_cache__event_set_period(struct perf_event *event,
				      struct hw_perf_event *hwc)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hml2_pmu *slice = get_hml2_pmu(system, event->cpu);
	s64 left = local64_read(&hwc->period_left);
	s64 period = hwc->sample_period;
	int ret = 0;
	u32 idx;

	if (unlikely(left <= -period)) {
		left = period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	if (unlikely(left <= 0)) {
		left += period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	if (left > (s64)L2_MAX_PERIOD)
		left = L2_MAX_PERIOD;

	idx = hwc->idx;

	atomic64_set(&slice->prev_count[idx], (u64)-left);
	hml2_pmu__counter_set_value(idx, (u64)-left);
	perf_event_update_userpage(event);

	return ret;
}

static inline int event_to_bit(unsigned int group)
{
	/* Lower bits are reserved for use by the counters */
	return group + MAX_L2_CTRS + 1;
}

static int l2_cache__get_event_idx(struct hml2_pmu *slice,
				   struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	int idx;
	int bit;

	if (hwc->config_base == L2CYCLE_CTR_RAW_CODE) {
		if (test_and_set_bit(l2_cycle_ctr_idx, slice->used_mask))
			return -EAGAIN;

		return l2_cycle_ctr_idx;
	}

	if (L2_EVT_GROUP(hwc->config_base) > L2_EVT_GROUP_MAX)
		return -EINVAL;

	/* check for column exclusion */
	bit = event_to_bit(L2_EVT_GROUP(hwc->config_base));
	if (test_and_set_bit(bit, slice->used_mask)) {
		pr_err("column exclusion for evt %lx\n", hwc->config_base);
		event->state = PERF_EVENT_STATE_OFF;
		return -EINVAL;
	}

	for (idx = 0; idx < l2cache_pmu.num_counters - 1; idx++) {
		if (!test_and_set_bit(idx, slice->used_mask))
			return idx;
	}

	/* The counters are all in use. */
	clear_bit(bit, slice->used_mask);
	return -EAGAIN;
}

static void l2_cache__clear_event_idx(struct hml2_pmu *slice,
				      struct perf_event *event)
{
	int bit;
	struct hw_perf_event *hwc = &event->hw;

	bit = event_to_bit(L2_EVT_GROUP(hwc->config_base));
	clear_bit(bit, slice->used_mask);
}

static void l2_cache__event_disable(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	if (!(hwc->state & PERF_HES_STOPPED)) {
		hml2_pmu__counter_disable_interrupt(hwc->idx);
		hml2_pmu__counter_disable(hwc->idx);
	}
}

static inline bool is_sampling(struct perf_event *event)
{
	return !!event->attr.sample_type;
}

static irqreturn_t l2_cache__handle_irq(int irq_num, void *data)
{
	struct hml2_pmu *slice = data;
	u32 ovsr;
	int idx;
	struct pt_regs *regs;

	ovsr = hml2_pmu__getreset_ovsr();
	if (!hml2_pmu__has_overflowed(ovsr))
		return IRQ_NONE;

	regs = get_irq_regs();

	for (idx = 0; idx < l2cache_pmu.num_counters; idx++) {
		struct perf_event *event = slice->events[idx];
		struct hw_perf_event *hwc;
		struct perf_sample_data data;

		if (!event)
			continue;

		if (!hml2_pmu__counter_has_overflowed(ovsr, idx))
			continue;

		l2_cache__event_update_from_slice(event, slice);
		hwc = &event->hw;

		if (is_sampling(event)) {
			perf_sample_data_init(&data, 0, hwc->last_period);
			if (!l2_cache__event_set_period(event, hwc))
				continue;
			if (perf_event_overflow(event, &data, regs))
				l2_cache__event_disable(event);
		} else {
			l2_cache__slice_set_period(slice, hwc);
		}
	}

	/*
	 * Handle the pending perf events.
	 *
	 * Note: this call *must* be run with interrupts disabled. For
	 * platforms that can have the PMU interrupts raised as an NMI, this
	 * will not work.
	 */
	irq_work_run();

	return IRQ_HANDLED;
}

/*
 * Implementation of abstract pmu functionality required by
 * the core perf events code.
 */

static void l2_cache__pmu_enable(struct pmu *pmu)
{
	hml2_pmu__enable();
}

static void l2_cache__pmu_disable(struct pmu *pmu)
{
	hml2_pmu__disable();
}

static int l2_cache__event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	if (event->attr.type != l2cache_pmu.pmu.type)
		return -ENOENT;

	/* We cannot filter accurately so we just don't allow it. */
	if (event->attr.exclude_user || event->attr.exclude_kernel ||
			event->attr.exclude_hv || event->attr.exclude_idle)
		return -EINVAL;

	hwc->idx = -1;
	hwc->config_base = event->attr.config;

	/*
	 * For counting events use L2_CNT_PERIOD which allows for simplified
	 * math and proper handling of overflows in the presence of IRQs and
	 * SMP.
	 */
	if (hwc->sample_period == 0) {
		hwc->sample_period = L2_CNT_PERIOD;
		hwc->last_period   = hwc->sample_period;
		local64_set(&hwc->period_left, hwc->sample_period);
	}

	return 0;
}

static void l2_cache__event_update(struct perf_event *event)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hml2_pmu *slice;
	struct hw_perf_event *hwc = &event->hw;

	if (hwc->idx == -1)
		return;

	slice = get_hml2_pmu(system, event->cpu);
	if (likely(slice))
		l2_cache__event_update_from_slice(event, slice);
}

static void l2_cache__event_start(struct perf_event *event, int flags)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hml2_pmu *slice;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;
	u32 config;
	u32 evt_prefix, event_reg, event_cc, event_group;
	bool is_tracectr = false;

	if (idx < 0)
		return;

	hwc->state = 0;

	slice = get_hml2_pmu(system, event->cpu);
	if (unlikely(!slice))
		return;
	if (is_sampling(event))
		l2_cache__event_set_period(event, hwc);
	else
		l2_cache__slice_set_period(slice, hwc);

	if (hwc->config_base == L2CYCLE_CTR_RAW_CODE)
		goto out;

	config = hwc->config_base;
	evt_prefix  = L2_EVT_PREFIX(config);
	event_reg   = L2_EVT_REG(config);
	event_cc    = L2_EVT_CODE(config);
	event_group = L2_EVT_GROUP(config);

	/* Check if user requested any special origin filtering. */
	if (evt_prefix == L2_TRACECTR_PREFIX)
		is_tracectr = true;

	hml2_pmu__set_evcntcr(idx, 0x0);
	hml2_pmu__set_evtyper(event_group, idx);
	hml2_pmu__set_evres(slice, event_group, event_reg, event_cc);
	if (event->cpu < 0)
		hml2_pmu__set_evfilter_task_mode(idx);
	else
		hml2_pmu__set_evfilter_sys_mode(idx, event->cpu, is_tracectr);
out:
	hml2_pmu__counter_enable_interrupt(idx);
	hml2_pmu__counter_enable(idx);
}

static void l2_cache__event_stop(struct perf_event *event, int flags)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hml2_pmu *slice;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	if (idx < 0)
		return;

	if (!(hwc->state & PERF_HES_STOPPED)) {
		slice = get_hml2_pmu(system, event->cpu);
		if (unlikely(!slice))
			return;
		hml2_pmu__counter_disable_interrupt(idx);
		hml2_pmu__counter_disable(idx);

		if (flags & PERF_EF_UPDATE)
			l2_cache__event_update(event);
		hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
	}
}

/* Look for a duplicate event already configured on this cluster */
static bool config_is_dup(struct hml2_pmu *slice, struct hw_perf_event *hwc)
{
	int i;
	struct hw_perf_event *hwc_i;

	for (i = 0; i < MAX_L2_CTRS; i++) {
		if (slice->events[i] == NULL)
			continue;
		hwc_i = &slice->events[i]->hw;
		if (hwc->config_base == hwc_i->config_base)
			return true;
	}
	return false;
}

static int l2_cache__event_add(struct perf_event *event, int flags)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int idx;
	int err = 0;
	struct hml2_pmu *slice;

	/*
	 * We need to disable the pmu while adding the event, otherwise
	 * the perf tick might kick-in and re-add this event.
	 */
	perf_pmu_disable(event->pmu);

	slice = get_hml2_pmu(system, event->cpu);
	if (!slice) {
		event->state = PERF_EVENT_STATE_OFF;
		hwc->idx = -1;
		goto out;
	}

	/*
	 * This checks for a duplicate event on the same cluster, which
	 * typically occurs in non-sampling mode when using perf -a,
	 * which generates events on each CPU. In this case, we don't
	 * want to permanently disable the event by setting its state to
	 * OFF, because if the other CPU is subsequently hotplugged, etc,
	 * we want the opportunity to start collecting on this event.
	 */
	if (config_is_dup(slice, hwc)) {
		hwc->idx = -1;
		goto out;
	}

	idx = l2_cache__get_event_idx(slice, event);
	if (idx < 0) {
		err = idx;
		goto out;
	}

	hwc->idx = idx;
	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	slice->events[idx] = event;
	atomic64_set(&slice->prev_count[idx], 0ULL);

	if (flags & PERF_EF_START)
		l2_cache__event_start(event, flags);

	/* Propagate changes to the userspace mapping. */
	perf_event_update_userpage(event);

out:
	perf_pmu_enable(event->pmu);
	return err;
}

static void l2_cache__event_del(struct perf_event *event, int flags)
{
	struct l2cache_pmu *system = to_l2cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	struct hml2_pmu *slice;
	int idx = hwc->idx;

	if (idx < 0)
		return;

	slice = get_hml2_pmu(system, event->cpu);
	if (unlikely(!slice))
		return;
	l2_cache__event_stop(event, flags | PERF_EF_UPDATE);
	slice->events[idx] = NULL;
	clear_bit(idx, slice->used_mask);
	l2_cache__clear_event_idx(slice, event);

	perf_event_update_userpage(event);
}

static void l2_cache__event_read(struct perf_event *event)
{
	l2_cache__event_update(event);
}

static int dummy_event_idx(struct perf_event *event)
{
	return 0;
}

/* NRCCG format for perf RAW codes. */
PMU_FORMAT_ATTR(l2_prefix, "config:16-19");
PMU_FORMAT_ATTR(l2_reg,    "config:12-15");
PMU_FORMAT_ATTR(l2_code,   "config:4-11");
PMU_FORMAT_ATTR(l2_grp,    "config:0-3");
static struct attribute *l2_cache_pmu_formats[] = {
	&format_attr_l2_prefix.attr,
	&format_attr_l2_reg.attr,
	&format_attr_l2_code.attr,
	&format_attr_l2_grp.attr,
	NULL,
};

static struct attribute_group l2_cache_pmu_format_group = {
	.name = "format",
	.attrs = l2_cache_pmu_formats,
};

static const struct attribute_group *l2_cache_pmu_attr_grps[] = {
	&l2_cache_pmu_format_group,
	NULL,
};

/*
 * Generic device handlers
 */

static const struct acpi_device_id l2_cache_pmu_acpi_match[] = {
	{ "QCOM8130", },
	{ }
};

static const struct of_device_id l2_cache_pmu_of_match[] = {
	{ .compatible = "qcom,qcom-l2cache-pmu", },
	{}
};
MODULE_DEVICE_TABLE(of, l2_cache_pmu_of_match);

static int get_num_counters(void)
{
	int val;

	val = get_l2_indirect_reg(L2PMCR);

	/*
	 * Read bits 15:11 of the L2PMCR and add 1
	 * for the cycle counter.
	 */
	return ((val >> PMCR_NUM_EV_SHIFT) & PMCR_NUM_EV_MASK) + 1;
}

static inline int mpidr_to_cpu(u32 mpidr)
{
	return MPIDR_AFFINITY_LEVEL(mpidr, 0) |
		(MPIDR_AFFINITY_LEVEL(mpidr, 1) << mpidr_affl1_shift);
}

static int get_logical_cpu_index(int phys_cpu)
{
	int cpu;

	for (cpu = 0; cpu < nr_cpu_ids; cpu++)
		if (mpidr_to_cpu(cpu_logical_map(cpu)) == phys_cpu)
			return cpu;
	return -EINVAL;
}

static int l2_cache_pmu_probe_slice(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev->parent);
	struct platform_device *sdev = to_platform_device(dev);
	struct l2cache_pmu *l2cache_pmu = data;
	struct hml2_pmu *slice;
	struct cpumask affinity_mask;
	struct acpi_device *device;
	int irq, err;
	int phys_cpu, logical_cpu;
	int i;
	unsigned long instance;

	if (acpi_bus_get_device(ACPI_HANDLE(dev), &device))
		return -ENODEV;

	if (kstrtol(device->pnp.unique_id, 10, &instance) < 0) {
		pr_err("unable to read ACPI uid\n");
		return -ENODEV;
	}

	irq = platform_get_irq(sdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev,
			"Failed to get valid irq for slice %ld\n", instance);
		return irq;
	}

	slice = devm_kzalloc(&pdev->dev, sizeof(*slice), GFP_KERNEL);
	if (!slice)
		return -ENOMEM;

	cpumask_clear(&affinity_mask);

	for (i = 0; i < num_cpus_in_cluster; i++) {
		phys_cpu = instance * num_cpus_in_cluster + i;
		logical_cpu = get_logical_cpu_index(phys_cpu);
		if (logical_cpu >= 0) {
			slice->cpu[slice->valid_cpus++] = logical_cpu;
			per_cpu(cpu_to_pmu, logical_cpu) = slice;
			cpumask_set_cpu(logical_cpu, &affinity_mask);
		}
	}

	if (slice->valid_cpus == 0) {
		dev_err(&pdev->dev, "No CPUs found for L2 cache instance %ld\n",
			instance);
		return -ENODEV;
	}

	if (irq_set_affinity(irq, &affinity_mask)) {
		dev_err(&pdev->dev,
			"Unable to set irq affinity (irq=%d, first cpu=%d)\n",
			irq, slice->cpu[0]);
		return -ENODEV;
	}

	err = devm_request_irq(
		&pdev->dev, irq, l2_cache__handle_irq,
		IRQF_NOBALANCING, "l2-cache-pmu", slice);
	if (err) {
		dev_err(&pdev->dev,
			"Unable to request IRQ%d for L2 PMU counters\n",
			irq);
		return err;
	}

	dev_info(&pdev->dev, "Registered L2 cache PMU instance %ld with %d CPUs\n",
		 instance, slice->valid_cpus);

	slice->pmu_lock = __SPIN_LOCK_UNLOCKED(slice->pmu_lock);

	hml2_pmu__init(slice);
	list_add(&slice->entry, &l2cache_pmu->pmus);
	l2cache_pmu->num_pmus++;

	return 0;
}

static int l2_cache_pmu_probe(struct platform_device *pdev)
{
	int result;
	int err;

	INIT_LIST_HEAD(&l2cache_pmu.pmus);

	l2cache_pmu.pmu = (struct pmu) {
		.task_ctx_nr	= perf_hw_context,

		.name		= "l2cache",
		.pmu_enable	= l2_cache__pmu_enable,
		.pmu_disable	= l2_cache__pmu_disable,
		.event_init	= l2_cache__event_init,
		.add		= l2_cache__event_add,
		.del		= l2_cache__event_del,
		.start		= l2_cache__event_start,
		.stop		= l2_cache__event_stop,
		.read		= l2_cache__event_read,
		.event_idx	= dummy_event_idx,
		.attr_groups	= l2_cache_pmu_attr_grps,
	};

	l2cache_pmu.num_counters = get_num_counters();
	l2_cycle_ctr_idx = l2cache_pmu.num_counters - 1;
	l2_reset_mask = ((1 << (l2cache_pmu.num_counters - 1)) - 1) |
		L2PM_CC_ENABLE;

	err = device_property_read_u32(&pdev->dev, "qcom,num-cpus-in-cluster",
				       &num_cpus_in_cluster);
	if (err) {
		dev_err(&pdev->dev, "Failed to read number of cpus in cluster\n");
		return err;
	}
	mpidr_affl1_shift = hweight8(num_cpus_in_cluster - 1);

	/* Read slice info and initialize each slice */
	result = device_for_each_child(&pdev->dev, &l2cache_pmu,
				       l2_cache_pmu_probe_slice);

	if (result < 0)
		return -ENODEV;

	if (l2cache_pmu.num_pmus == 0) {
		dev_err(&pdev->dev, "No hardware L2 PMUs found\n");
		return -ENODEV;
	}

	result = perf_pmu_register(&l2cache_pmu.pmu,
				   l2cache_pmu.pmu.name, -1);

	if (result < 0)
		dev_err(&pdev->dev,
			"Failed to register L2 cache PMU (%d)\n",
			result);
	else
		dev_info(&pdev->dev,
			 "Registered L2 cache PMU using %d HW PMUs\n",
			 l2cache_pmu.num_pmus);

	return result;
}

static int l2_cache_pmu_remove(struct platform_device *pdev)
{
	perf_pmu_unregister(&l2cache_pmu.pmu);
	return 0;
}

static struct platform_driver l2_cache_pmu_driver = {
	.driver = {
		.name = "qcom-l2cache-pmu",
		.owner = THIS_MODULE,
		.of_match_table = l2_cache_pmu_of_match,
		.acpi_match_table = ACPI_PTR(l2_cache_pmu_acpi_match),
	},
	.probe = l2_cache_pmu_probe,
	.remove = l2_cache_pmu_remove,
};

static int __init register_l2_cache_pmu_driver(void)
{
	return platform_driver_register(&l2_cache_pmu_driver);
}
device_initcall(register_l2_cache_pmu_driver);
