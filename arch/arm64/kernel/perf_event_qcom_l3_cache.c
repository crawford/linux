/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/list.h>
#include <linux/acpi.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>

/*
 * Driver for the L3 cache PMUs in Qualcomm Technologies chips.
 *
 * The driver supports a distributed cache architecture where the overall
 * cache is comprised of multiple slices each with its own PMU. The driver
 * aggregates counts across the whole system to provide a global picture
 * of the metrics selected by the user.
 */

/*
 * General constants
 */

#define L3_NUM_COUNTERS  (8)
#define L3_MAX_EVTYPE    (0xFF)
#define L3_MAX_PERIOD    U32_MAX
#define L3_CNT_PERIOD    (U32_MAX - 0xFFFF)

/*
 * Register offsets vary depending on hardware version
 * Unsupported registers are set to -1.
 */

struct hml3_offsets {
	s32 pm_cr;
	s32 pm_evcntr;
	s32 pm_cntctl;
	s32 pm_evtype;
	s32 pm_filtra;
	s32 pm_filtram;
	s32 pm_filtrb;
	s32 pm_filtrbm;
	s32 pm_filtrc;
	s32 pm_filtrcm;
	s32 bc_cr;
	s32 bc_satroll_cr;
	s32 bc_cntenset;
	s32 bc_cntenclr;
	s32 bc_intenset;
	s32 bc_intenclr;
	s32 bc_ovsr;
	s32 bc_gang;
	s32 bc_irqctl;
};

static struct hml3_offsets hml3_pmu_v0_offsets = {
	.pm_cr =		0x000,
	.pm_evcntr =		0x040,
	.pm_cntctl =		0x200,
	.pm_evtype =		0x240,
	.pm_filtra =		0x460,
	.pm_filtram =		0x464,
	.pm_filtrb =		0x468,
	.pm_filtrbm =		0x470,
	.pm_filtrc =		0x474,
	.pm_filtrcm =		0x478,
	.bc_cr =		0x500,
	.bc_satroll_cr =	0x504,
	.bc_cntenset =		0x508,
	.bc_cntenclr =		0x50C,
	.bc_intenset =		0x510,
	.bc_intenclr =		0x514,
	.bc_ovsr =		0x740,
	.bc_gang =		-1,
	.bc_irqctl =		-1,
};

static struct hml3_offsets hml3_pmu_v1_offsets = {
	.pm_cr =		0x000,
	.pm_evcntr =		0x420,
	.pm_cntctl =		0x120,
	.pm_evtype =		0x220,
	.pm_filtra =		0x300,
	.pm_filtram =		0x308,
	.pm_filtrb =		0x310,
	.pm_filtrbm =		0x304,
	.pm_filtrc =		0x30C,
	.pm_filtrcm =		0x314,
	.bc_cr =		0x500,
	.bc_satroll_cr =	0x504,
	.bc_cntenset =		0x508,
	.bc_cntenclr =		0x50C,
	.bc_intenset =		0x510,
	.bc_intenclr =		0x514,
	.bc_ovsr =		0x740,
	.bc_gang =		0x718,
	.bc_irqctl =		0x96C,
};

#define L3_HML3_PM_CR(__pmu) ((__pmu)->regs + (__pmu)->offsets->pm_cr)
#define L3_HML3_PM_EVCNTR(__pmu, __cntr) \
	((__pmu)->regs + (__pmu)->offsets->pm_evcntr + ((__cntr) & 0x7) * 8)
#define L3_HML3_PM_CNTCTL(__pmu, __cntr) \
	((__pmu)->regs + (__pmu)->offsets->pm_cntctl + ((__cntr) & 0x7) * 8)
#define L3_HML3_PM_EVTYPE(__pmu, __cntr) \
	((__pmu)->regs + (__pmu)->offsets->pm_evtype + ((__cntr) & 0x7) * 8)
#define L3_HML3_PM_FILTRA(__pmu)  ((__pmu)->regs + (__pmu)->offsets->pm_filtra)
#define L3_HML3_PM_FILTRAM(__pmu) ((__pmu)->regs + (__pmu)->offsets->pm_filtram)
#define L3_HML3_PM_FILTRB(__pmu)  ((__pmu)->regs + (__pmu)->offsets->pm_filtrb)
#define L3_HML3_PM_FILTRBM(__pmu) ((__pmu)->regs + (__pmu)->offsets->pm_filtrbm)
#define L3_HML3_PM_FILTRC(__pmu)  ((__pmu)->regs + (__pmu)->offsets->pm_filtrc)
#define L3_HML3_PM_FILTRCM(__pmu) ((__pmu)->regs + (__pmu)->offsets->pm_filtrcm)
#define L3_HML3_BC_CR(__pmu) ((__pmu)->regs + (__pmu)->offsets->bc_cr)
#define L3_HML3_BC_SATROLL_CR(__pmu) \
			((__pmu)->regs + (__pmu)->offsets->bc_satroll_cr)
#define L3_HML3_BC_CNTENSET(__pmu) \
			((__pmu)->regs + (__pmu)->offsets->bc_cntenset)
#define L3_HML3_BC_CNTENCLR(__pmu) \
			((__pmu)->regs + (__pmu)->offsets->bc_cntenclr)
#define L3_HML3_BC_INTENSET(__pmu) \
			((__pmu)->regs + (__pmu)->offsets->bc_intenset)
#define L3_HML3_BC_INTENCLR(__pmu) \
			((__pmu)->regs + (__pmu)->offsets->bc_intenclr)
#define L3_HML3_BC_OVSR(__pmu)   ((__pmu)->regs + (__pmu)->offsets->bc_ovsr)
#define L3_HML3_BC_GANG(__pmu)   ((__pmu)->regs + (__pmu)->offsets->bc_gang)
#define L3_HML3_BC_IRQCTL(__pmu) ((__pmu)->regs + (__pmu)->offsets->bc_irqctl)

/*
 * Bit field manipulators
 */

/* L3_HML3_PM_CR */
#define PM_CR_RESET           (0)

/* L3_HML3_PM_XCNTCTL/L3_HML3_PM_CNTCTLx */
#define PMCNT_RESET           (0)

/* L3_HML3_PM_EVTYPEx */
#define EVSEL(__val)          ((u32)((__val) & 0xFF))

/* Reset value for all the filter registers */
#define PM_FLTR_RESET         (0)

/* L3_M_BC_CR */
#define BC_RESET              (((u32)1) << 1)
#define BC_ENABLE             ((u32)1)

/* L3_M_BC_SATROLL_CR */
#define BC_SATROLL_CR_RESET   (0)

/* L3_M_BC_CNTENSET */
#define PMCNTENSET(__cntr)    (((u32)1) << ((__cntr) & 0x7))

/* L3_M_BC_CNTENCLR */
#define PMCNTENCLR(__cntr)    (((u32)1) << ((__cntr) & 0x7))
#define BC_CNTENCLR_RESET     (0xFF)

/* L3_M_BC_INTENSET */
#define PMINTENSET(__cntr)    (((u32)1) << ((__cntr) & 0x7))

/* L3_M_BC_INTENCLR */
#define PMINTENCLR(__cntr)    (((u32)1) << ((__cntr) & 0x7))
#define BC_INTENCLR_RESET     (0xFF)

/* L3_M_BC_GANG */
#define GANG_EN(__cntr)       (((u32)1) << ((__cntr) & 0x7))
#define BC_GANG_RESET         (0)

/* L3_M_BC_OVSR */
#define PMOVSRCLR(__cntr)     (((u32)1) << ((__cntr) & 0x7))
#define PMOVSRCLR_RESET       (0xFF)

/* L3_M_BC_IRQCTL */
#define PMIRQONMSBEN(__cntr)  (((u32)1) << ((__cntr) & 0x7))
#define BC_IRQCTL_RESET       (0x0)

/*
 * Events
 */

#define L3_CYCLES		0x01
#define L3_READ_HIT		0x20
#define L3_READ_MISS		0x21
#define L3_READ_HIT_D		0x22
#define L3_READ_MISS_D		0x23
#define L3_WRITE_HIT		0x24
#define L3_WRITE_MISS		0x25

/*
 * The cache is made-up of one or more slices, each slice has its own PMU.
 * This structure represents one of the hardware PMUs.
 */
struct hml3_pmu {
	struct list_head	entry;
	void __iomem		*regs;
	int			irq;
	struct hml3_offsets	*offsets;
	atomic_t		prev_count[L3_NUM_COUNTERS];
};

static
void hml3_pmu__reset(struct hml3_pmu *pmu)
{
	int i;

	writel_relaxed(BC_RESET, L3_HML3_BC_CR(pmu));

	/*
	 * Use writel for the first programming command to ensure the basic
	 * counter unit is stopped before proceeding
	 */
	writel(BC_SATROLL_CR_RESET, L3_HML3_BC_SATROLL_CR(pmu));
	writel_relaxed(BC_CNTENCLR_RESET, L3_HML3_BC_CNTENCLR(pmu));
	writel_relaxed(BC_INTENCLR_RESET, L3_HML3_BC_INTENCLR(pmu));
	writel_relaxed(PMOVSRCLR_RESET, L3_HML3_BC_OVSR(pmu));
	if (pmu->offsets->bc_gang != -1)
		writel_relaxed(BC_GANG_RESET, L3_HML3_BC_GANG(pmu));
	if (pmu->offsets->bc_irqctl != -1)
		writel_relaxed(BC_IRQCTL_RESET, L3_HML3_BC_IRQCTL(pmu));
	writel_relaxed(PM_CR_RESET, L3_HML3_PM_CR(pmu));

	for (i = 0; i < L3_NUM_COUNTERS; ++i) {
		writel_relaxed(PMCNT_RESET, L3_HML3_PM_CNTCTL(pmu, i));
		writel_relaxed(EVSEL(0), L3_HML3_PM_EVTYPE(pmu, i));
	}
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRA(pmu));
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRAM(pmu));
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRB(pmu));
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRBM(pmu));
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRC(pmu));
	writel_relaxed(PM_FLTR_RESET, L3_HML3_PM_FILTRCM(pmu));
}

static inline
void hml3_pmu__init(struct hml3_pmu *pmu, void __iomem *regs, int irq,
						struct hml3_offsets *offsets)
{
	pmu->regs = regs;
	pmu->offsets = offsets;
	pmu->irq = irq;
	hml3_pmu__reset(pmu);

	/*
	 * Use writel here to ensure all programming commands are done
	 *  before proceeding
	 */
	writel(BC_ENABLE, L3_HML3_BC_CR(pmu));
}

static inline
void hml3_pmu__enable(struct hml3_pmu *pmu)
{
	writel_relaxed(BC_ENABLE, L3_HML3_BC_CR(pmu));
}

static inline
void hml3_pmu__disable(struct hml3_pmu *pmu)
{
	writel_relaxed(0, L3_HML3_BC_CR(pmu));
}

static inline
void hml3_pmu__counter_set_event(struct hml3_pmu *pmu, u8 cntr, u32 event)
{
	writel_relaxed(EVSEL(event), L3_HML3_PM_EVTYPE(pmu, cntr));
}

static inline
void hml3_pmu__counter_set_value(struct hml3_pmu *pmu, u8 cntr, u32 value)
{
	writel_relaxed(value, L3_HML3_PM_EVCNTR(pmu, cntr));
}

static inline
u32 hml3_pmu__counter_get_value(struct hml3_pmu *pmu, u8 cntr)
{
	return readl_relaxed(L3_HML3_PM_EVCNTR(pmu, cntr));
}

static inline
void hml3_pmu__counter_enable(struct hml3_pmu *pmu, u8 cntr)
{
	writel_relaxed(PMCNTENSET(cntr), L3_HML3_BC_CNTENSET(pmu));
}

static inline
void hml3_pmu__counter_reset_trigger(struct hml3_pmu *pmu, u8 cntr)
{
	writel_relaxed(PMCNT_RESET, L3_HML3_PM_CNTCTL(pmu, cntr));
}

static inline
void hml3_pmu__counter_disable(struct hml3_pmu *pmu, u8 cntr)
{
	writel_relaxed(PMCNTENCLR(cntr), L3_HML3_BC_CNTENCLR(pmu));
}

static inline
void hml3_pmu__counter_enable_interrupt(struct hml3_pmu *pmu, u8 cntr)
{
	writel_relaxed(PMINTENSET(cntr), L3_HML3_BC_INTENSET(pmu));
}

static inline
void hml3_pmu__counter_disable_interrupt(struct hml3_pmu *pmu, u8 cntr)
{
	writel_relaxed(PMINTENCLR(cntr), L3_HML3_BC_INTENCLR(pmu));
}

static inline
void hml3_pmu__counter_enable_gang(struct hml3_pmu *pmu, u8 cntr)
{
	u32 value = readl_relaxed(L3_HML3_BC_GANG(pmu));

	value |= GANG_EN(cntr);
	writel_relaxed(value, L3_HML3_BC_GANG(pmu));
}

static inline
void hml3_pmu__counter_disable_gang(struct hml3_pmu *pmu, u8 cntr)
{
	u32 value = readl_relaxed(L3_HML3_BC_GANG(pmu));

	value &= ~(GANG_EN(cntr));
	writel_relaxed(value, L3_HML3_BC_GANG(pmu));
}

static inline
void hml3_pmu__counter_enable_irq_on_msb(struct hml3_pmu *pmu, u8 cntr)
{
	u32 value = readl_relaxed(L3_HML3_BC_IRQCTL(pmu));

	value |= PMIRQONMSBEN(cntr);
	writel_relaxed(value, L3_HML3_BC_IRQCTL(pmu));
}

static inline
void hml3_pmu__counter_disable_irq_on_msb(struct hml3_pmu *pmu, u8 cntr)
{
	u32 value = readl_relaxed(L3_HML3_BC_IRQCTL(pmu));

	value &= ~(PMIRQONMSBEN(cntr));
	writel_relaxed(value, L3_HML3_BC_IRQCTL(pmu));
}

static inline
u32 hml3_pmu__getreset_ovsr(struct hml3_pmu *pmu)
{
	u32 result = readl_relaxed(L3_HML3_BC_OVSR(pmu));

	writel_relaxed(result, L3_HML3_BC_OVSR(pmu));
	return result;
}

static inline
int hml3_pmu__has_overflowed(u32 ovsr)
{
	return (ovsr & PMOVSRCLR_RESET) != 0;
}

/*
 * Hardware counter interface.
 *
 * This interface allows operations on counters to be polymorphic.
 * The hardware supports counter chaining to allow 64 bit virtual counters.
 * We expose this capability as a config option for each event, that way
 * a user can create perf events that use 32 bit counters for events that
 * increment at a slower rate, and perf events that use 64 bit counters
 * for events that increment faster and avoid IRQs.
 */
struct l3cache_pmu_hwc {
	struct perf_event	*event;
	/* Called to start event monitoring */
	void (*start)(struct perf_event *event);
	/* Called to stop event monitoring */
	void (*stop)(struct perf_event *event, int flags);
	/* Called to update the perf_event */
	void (*update)(struct perf_event *event);
};

/*
 * Decoding of settings from perf_event_attr
 *
 * The config format for perf events is:
 * - config: bits 0-7: event type
 *           bit  32:  HW counter size requested, 0: 32 bits, 1: 64 bits
 */
static inline u32 get_event_type(struct perf_event *event)
{
	return (event->attr.config) & L3_MAX_EVTYPE;
}

static inline int get_hw_counter_size(struct perf_event *event)
{
	return event->attr.config >> 32 & 1;
}

/*
 * Aggregate PMU. Implements the core pmu functions and manages
 * the hardware PMU, configuring each one in the same way and
 * aggregating events when needed.
 */

struct l3cache_pmu {
	u32			version;
	struct hml3_offsets	*offsets;
	u32			num_pmus;
	struct list_head	pmus;
	unsigned long		used_mask[BITS_TO_LONGS(L3_NUM_COUNTERS)];
	struct l3cache_pmu_hwc	counters[L3_NUM_COUNTERS];
	cpumask_t		cpu;
	struct notifier_block	cpu_nb;
	struct pmu		pmu;
};

#define to_l3cache_pmu(p) (container_of(p, struct l3cache_pmu, pmu))

static struct l3cache_pmu l3cache_pmu = { 0 };

/*
 * 64 bit counter interface implementation.
 */

static
void qcom_l3_cache__64bit_counter_start(struct perf_event *event)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;
	int idx = event->hw.idx;
	u64 value = local64_read(&event->count);

	list_for_each_entry(slice, &system->pmus, entry) {
		hml3_pmu__counter_enable_gang(slice, idx+1);

		if (value) {
			hml3_pmu__counter_set_value(slice, idx+1, value >> 32);
			hml3_pmu__counter_set_value(slice, idx, (u32)value);
			value = 0;
		} else {
			hml3_pmu__counter_set_value(slice, idx+1, 0);
			hml3_pmu__counter_set_value(slice, idx, 0);
		}

		hml3_pmu__counter_set_event(slice, idx+1, 0);
		hml3_pmu__counter_set_event(slice, idx, get_event_type(event));

		hml3_pmu__counter_enable(slice, idx+1);
		hml3_pmu__counter_enable(slice, idx);
	}
}

static
void qcom_l3_cache__64bit_counter_stop(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;
	int idx = event->hw.idx;

	list_for_each_entry(slice, &system->pmus, entry) {
		hml3_pmu__counter_disable_gang(slice, idx+1);

		hml3_pmu__counter_disable(slice, idx);
		hml3_pmu__counter_disable(slice, idx+1);
	}
}

static
u64 qcom_l3_cache__64bit_counter_get_value(struct hml3_pmu *slice, int idx)
{
	u32 hi_old, lo, hi_new;
	int i, retries = 2;

	hi_new = hml3_pmu__counter_get_value(slice, idx+1);
	for (i = 0; i < retries; i++) {
		hi_old = hi_new;
		lo = hml3_pmu__counter_get_value(slice, idx);
		hi_new = hml3_pmu__counter_get_value(slice, idx+1);
		if (hi_old == hi_new)
			break;
	}

	return ((u64)hi_new << 32) | lo;
}

static
void qcom_l3_cache__64bit_counter_update(struct perf_event *event)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;
	int idx = event->hw.idx;
	u64 new = 0;

	list_for_each_entry(slice, &system->pmus, entry)
		new += qcom_l3_cache__64bit_counter_get_value(slice, idx);

	local64_set(&event->count, new);
}

/*
 * 32 bit counter interface implementation
 */

static
void qcom_l3_cache__slice_set_period(struct hml3_pmu *slice, int idx, u32 prev)
{
	u32 value = L3_MAX_PERIOD - (L3_CNT_PERIOD - 1);

	if (prev < value) {
		value += prev;
		atomic_set(&slice->prev_count[idx], value);
	} else {
		value = prev;
	}
	hml3_pmu__counter_set_value(slice, idx, value);
}

static
void qcom_l3_cache__32bit_counter_start(struct perf_event *event)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;
	struct hw_perf_event *hwc = &event->hw;

	slice = list_first_entry(&system->pmus, struct hml3_pmu, entry);

	list_for_each_entry(slice, &system->pmus, entry) {
		/*
		 * If the hardware supports generation of IRQs on the toggling
		 * of the MSB we use that feature to avoid having to adjust
		 * the hardware counter on each IRQ. When using that feature
		 * the hardware counters are essentially left rolling.
		 */
		if (slice->offsets->bc_irqctl == -1) {
			qcom_l3_cache__slice_set_period(slice, hwc->idx, 0);
		} else {
			atomic_set(&slice->prev_count[hwc->idx], 0);
			hml3_pmu__counter_set_value(slice, hwc->idx, 0);
			hml3_pmu__counter_enable_irq_on_msb(slice, hwc->idx);
		}

		hml3_pmu__counter_set_event(slice, hwc->idx,
					    get_event_type(event));
		hml3_pmu__counter_enable_interrupt(slice, hwc->idx);
		hml3_pmu__counter_enable(slice, hwc->idx);
	}
}

static
void qcom_l3_cache__32bit_counter_stop(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;
	struct hw_perf_event *hwc = &event->hw;

	list_for_each_entry(slice, &system->pmus, entry) {
		if (slice->offsets->bc_irqctl != -1)
			hml3_pmu__counter_disable_irq_on_msb(slice, hwc->idx);
		hml3_pmu__counter_disable_interrupt(slice, hwc->idx);
		hml3_pmu__counter_disable(slice, hwc->idx);
	}
}

static
void qcom_l3_cache__32bit_counter_update_from_slice(struct perf_event *event,
						    struct hml3_pmu *slice,
						    int idx)
{
	u32 delta, prev, now;

again:
	prev = atomic_read(&slice->prev_count[idx]);
	now = hml3_pmu__counter_get_value(slice, idx);

	if (atomic_cmpxchg(&slice->prev_count[idx], prev, now) != prev)
		goto again;

	delta = now - prev;

	local64_add(delta, &event->count);
}

static
void qcom_l3_cache__32bit_counter_update(struct perf_event *event)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hml3_pmu *slice;

	list_for_each_entry(slice, &system->pmus, entry)
		qcom_l3_cache__32bit_counter_update_from_slice(event, slice,
							       event->hw.idx);
}

/*
 * Top level PMU functions.
 */

static
irqreturn_t qcom_l3_cache__handle_irq(int irq_num, void *data)
{
	struct hml3_pmu *slice = data;
	u32 status;
	int idx;

	status = hml3_pmu__getreset_ovsr(slice);
	if (!hml3_pmu__has_overflowed(status))
		return IRQ_NONE;

	while (status) {
		struct perf_event *event;

		idx = __ffs(status);
		status &= ~(1 << idx);
		event = l3cache_pmu.counters[idx].event;
		if (!event)
			continue;

		qcom_l3_cache__32bit_counter_update_from_slice(event, slice,
							       event->hw.idx);

		/*
		 * If the hardware supports generation of IRQs on the toggling
		 * of the MSB we use that feature to avoid having to adjust
		 * the hardware counter on each IRQ. When using that feature
		 * the hardware counters are essentially left rolling.
		 */
		if (slice->offsets->bc_irqctl == -1)
			qcom_l3_cache__slice_set_period(slice, event->hw.idx,
					atomic_read(&slice->prev_count[idx]));

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

static
void qcom_l3_cache__pmu_enable(struct pmu *pmu)
{
	struct l3cache_pmu *system = to_l3cache_pmu(pmu);
	struct hml3_pmu *slice;
	int idx;

	/*
	 * Re-write CNTCTL for all existing events to re-assert
	 * the start trigger.
	 */
	for (idx = 0; idx < L3_NUM_COUNTERS; idx++)
		if (system->counters[idx].event)
			list_for_each_entry(slice, &system->pmus, entry)
				hml3_pmu__counter_reset_trigger(slice, idx);

	/* Ensure all programming commands are done before proceeding */
	wmb();
	list_for_each_entry(slice, &system->pmus, entry)
		hml3_pmu__enable(slice);
}

static
void qcom_l3_cache__pmu_disable(struct pmu *pmu)
{
	struct l3cache_pmu *system = to_l3cache_pmu(pmu);
	struct hml3_pmu *slice;

	list_for_each_entry(slice, &system->pmus, entry)
		hml3_pmu__disable(slice);

	/* Ensure the basic counter unit is stopped before proceeding */
	wmb();
}

static
int qcom_l3_cache__event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	if (event->attr.type != l3cache_pmu.pmu.type)
		return -ENOENT;

	/*
	 * There are no per-counter mode filters in the PMU.
	 */
	if (event->attr.exclude_user || event->attr.exclude_kernel ||
			event->attr.exclude_hv || event->attr.exclude_idle)
		return -EINVAL;

	hwc->idx = -1;

	/*
	 * Sampling not supported since these events are not core-attributable.
	 */
	if (hwc->sample_period)
		return -EINVAL;

	/*
	 * Task mode not available, we run the counters as system counters,
	 * not attributable to any CPU and therefore cannot attribute per-task.
	 */
	if (event->cpu < 0)
		return -EINVAL;

	/*
	 * Many perf core operations (eg. events rotation) operate on a
	 * single CPU context. This is obvious for CPU PMUs, where one
	 * expects the same sets of events being observed on all CPUs,
	 * but can lead to issues for off-core PMUs, like this one, where
	 * each event could be theoretically assigned to a different CPU.
	 * To mitigate this, we enforce CPU assignment to one designated
	 * processor (the one described in the "cpumask" attribute exported
	 * by the PMU device). perf user space tools honors this and avoids
	 * opening more than one copy of the events.
	 */
	event->cpu = cpumask_first(&l3cache_pmu.cpu);

	return 0;
}

static
void qcom_l3_cache__event_start(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	hwc->state = 0;

	system->counters[hwc->idx].start(event);
}

static
void qcom_l3_cache__event_stop(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	if (!(hwc->state & PERF_HES_STOPPED)) {
		system->counters[hwc->idx].stop(event, flags);

		if (flags & PERF_EF_UPDATE)
			system->counters[hwc->idx].update(event);
		hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
	}
}

static
int qcom_l3_cache__event_add(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int idx;
	int sz;

	/*
	 * Try to allocate a counter.
	 */
	sz = system->offsets->bc_gang == -1 ? 0 : get_hw_counter_size(event);
	idx = bitmap_find_free_region(system->used_mask, L3_NUM_COUNTERS, sz);
	if (idx < 0)
		/* The counters are all in use. */
		return -EAGAIN;

	hwc->idx = idx;
	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	if (sz == 0)
		system->counters[idx] = (struct l3cache_pmu_hwc) {
			.event = event,
			.start = qcom_l3_cache__32bit_counter_start,
			.stop = qcom_l3_cache__32bit_counter_stop,
			.update = qcom_l3_cache__32bit_counter_update
		};
	else {
		system->counters[idx] = (struct l3cache_pmu_hwc) {
			.event = event,
			.start = qcom_l3_cache__64bit_counter_start,
			.stop = qcom_l3_cache__64bit_counter_stop,
			.update = qcom_l3_cache__64bit_counter_update
		};
		system->counters[idx+1] = system->counters[idx];
	}

	if (flags & PERF_EF_START)
		qcom_l3_cache__event_start(event, 0);

	/* Propagate changes to the userspace mapping. */
	perf_event_update_userpage(event);

	return 0;
}

static
void qcom_l3_cache__event_del(struct perf_event *event, int flags)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	int sz;

	qcom_l3_cache__event_stop(event,  flags | PERF_EF_UPDATE);
	sz = system->offsets->bc_gang == -1 ? 0 : get_hw_counter_size(event);
	system->counters[hwc->idx].event = NULL;
	if (sz)
		system->counters[hwc->idx+1].event = NULL;
	bitmap_release_region(system->used_mask, hwc->idx, sz);

	perf_event_update_userpage(event);
}

static
void qcom_l3_cache__event_read(struct perf_event *event)
{
	struct l3cache_pmu *system = to_l3cache_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	system->counters[hwc->idx].update(event);
}

/*
 * Add support for creating events symbolically when using the perf
 * user space tools command line. E.g.:
 *   perf stat -a -e l3cache/event=read-miss/ ls
 *   perf stat -a -e l3cache/event=0x21/ ls
 */

ssize_t l3cache_pmu_event_sysfs_show(struct device *dev,
				     struct device_attribute *attr, char *page)
{
	struct perf_pmu_events_attr *pmu_attr;

	pmu_attr = container_of(attr, struct perf_pmu_events_attr, attr);
	return sprintf(page, "event=0x%02llx,name=%s\n",
		       pmu_attr->id, attr->attr.name);
}

#define L3CACHE_EVENT_VAR(__id)	pmu_event_attr_##__id
#define L3CACHE_EVENT_PTR(__id)	(&L3CACHE_EVENT_VAR(__id).attr.attr)

#define L3CACHE_EVENT_ATTR(__name, __id)			\
	PMU_EVENT_ATTR(__name, L3CACHE_EVENT_VAR(__id), __id,	\
		       l3cache_pmu_event_sysfs_show)


L3CACHE_EVENT_ATTR(cycles, L3_CYCLES);
L3CACHE_EVENT_ATTR(read-hit, L3_READ_HIT);
L3CACHE_EVENT_ATTR(read-miss, L3_READ_MISS);
L3CACHE_EVENT_ATTR(read-hit-d-side, L3_READ_HIT_D);
L3CACHE_EVENT_ATTR(read-miss-d-side, L3_READ_MISS_D);
L3CACHE_EVENT_ATTR(write-hit, L3_WRITE_HIT);
L3CACHE_EVENT_ATTR(write-miss, L3_WRITE_MISS);

static struct attribute *qcom_l3_cache_pmu_events[] = {
	L3CACHE_EVENT_PTR(L3_CYCLES),
	L3CACHE_EVENT_PTR(L3_READ_HIT),
	L3CACHE_EVENT_PTR(L3_READ_MISS),
	L3CACHE_EVENT_PTR(L3_READ_HIT_D),
	L3CACHE_EVENT_PTR(L3_READ_MISS_D),
	L3CACHE_EVENT_PTR(L3_WRITE_HIT),
	L3CACHE_EVENT_PTR(L3_WRITE_MISS),
	NULL
};

static struct attribute_group qcom_l3_cache_pmu_events_group = {
	.name = "events",
	.attrs = qcom_l3_cache_pmu_events,
};

PMU_FORMAT_ATTR(event, "config:0-7");
PMU_FORMAT_ATTR(lc, "config:32");

static struct attribute *qcom_l3_cache_pmu_formats[] = {
	&format_attr_event.attr,
	&format_attr_lc.attr,
	NULL,
};

static struct attribute_group qcom_l3_cache_pmu_format_group = {
	.name = "format",
	.attrs = qcom_l3_cache_pmu_formats,
};

static ssize_t qcom_l3_cache_pmu_cpumask_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct l3cache_pmu *system = to_l3cache_pmu(dev_get_drvdata(dev));

	return cpumap_print_to_pagebuf(true, buf, &system->cpu);
}

static struct device_attribute qcom_l3_cache_pmu_cpumask_attr =
		__ATTR(cpumask, S_IRUGO, qcom_l3_cache_pmu_cpumask_show, NULL);

static struct attribute *qcom_l3_cache_pmu_cpumask_attrs[] = {
	&qcom_l3_cache_pmu_cpumask_attr.attr,
	NULL,
};

static struct attribute_group qcom_l3_cache_pmu_cpumask_attr_group = {
	.attrs = qcom_l3_cache_pmu_cpumask_attrs,
};


static const struct attribute_group *qcom_l3_cache_pmu_attr_grps[] = {
	&qcom_l3_cache_pmu_format_group,
	&qcom_l3_cache_pmu_events_group,
	&qcom_l3_cache_pmu_cpumask_attr_group,
	NULL,
};

/*
 * Probing functions and data.
 */

static int qcom_l3_cache_pmu_cpu_notifier(struct notifier_block *nb,
					  unsigned long action, void *hcpu)
{
	struct l3cache_pmu *sys = container_of(nb, struct l3cache_pmu, cpu_nb);
	struct hml3_pmu *slice;
	unsigned int cpu = (long)hcpu; /* for (long) see kernel/cpu.c */
	unsigned int target;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_PREPARE:
		if (!cpumask_test_and_clear_cpu(cpu, &sys->cpu))
			break;
		target = cpumask_any_but(cpu_online_mask, cpu);
		if (target >= nr_cpu_ids)
			break;
		perf_pmu_migrate_context(&sys->pmu, cpu, target);
		cpumask_set_cpu(target, &sys->cpu);
		list_for_each_entry(slice, &sys->pmus, entry)
			WARN_ON(irq_set_affinity(slice->irq, &sys->cpu) != 0);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

/*
 * In some platforms interrupt resources might not come directly from the GIC,
 * but from separate IRQ circuitry that signals a summary IRQ to the GIC and
 * is handled by a secondary IRQ controller. This is problematic under ACPI boot
 * because the ACPI core does not use the Resource Source field on the Extended
 * Interrupt Descriptor, which in theory could be used to specify an alternative
 * IRQ controller.

 * For this reason in these platforms we implement the secondary IRQ controller
 * using the gpiolib and specify the IRQs as GpioInt resources, so when getting
 * an IRQ from the device we first try platform_get_irq and if it fails we try
 * devm_gpiod_get_index/gpiod_to_irq.
 */
static
int qcom_l3_cache_pmu_get_slice_irq(struct platform_device *pdev,
				    struct platform_device *sdev)
{
	int virq = platform_get_irq(sdev, 0);
	struct gpio_desc *desc;

	if (virq >= 0)
		return virq;

	desc = devm_gpiod_get_index(&sdev->dev, NULL, 0, GPIOD_ASIS);
	if (IS_ERR(desc))
		return -ENOENT;

	return gpiod_to_irq(desc);
}

static int qcom_l3_cache_pmu_probe_slice(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev->parent);
	struct platform_device *sdev = to_platform_device(dev);
	struct l3cache_pmu *system = data;
	struct resource *slice_info;
	void __iomem *slice_mem;
	struct hml3_pmu *slice;
	int irq, err;

	slice_info = platform_get_resource(sdev, IORESOURCE_MEM, 0);
	slice = devm_kzalloc(&pdev->dev, sizeof(*slice), GFP_KERNEL);
	if (!slice)
		return -ENOMEM;

	slice_mem = devm_ioremap_resource(&pdev->dev, slice_info);
	if (IS_ERR(slice_mem)) {
		dev_err(&pdev->dev, "Can't map slice @%pa\n",
			&slice_info->start);
		return PTR_ERR(slice_mem);
	}

	irq = qcom_l3_cache_pmu_get_slice_irq(pdev, sdev);
	if (irq < 0) {
		dev_err(&pdev->dev,
			"Failed to get valid irq for slice @%pa\n",
			&slice_info->start);
		return irq;
	}

	err = devm_request_irq(&pdev->dev, irq, qcom_l3_cache__handle_irq, 0,
			       "qcom-l3-cache-pmu", slice);
	if (err) {
		dev_err(&pdev->dev, "Request for IRQ failed for slice @%pa\n",
			&slice_info->start);
		return err;
	}

	err = irq_set_affinity(irq, &system->cpu);
	if (err) {
		dev_err(&pdev->dev, "Failed to set irq aff for slice @%pa\n",
			&slice_info->start);
		return err;
	}

	hml3_pmu__init(slice, slice_mem, irq, system->offsets);
	list_add(&slice->entry, &system->pmus);
	l3cache_pmu.num_pmus++;
	return 0;
}

static int qcom_l3_cache_pmu_probe(struct platform_device *pdev)
{
	int err;

	INIT_LIST_HEAD(&l3cache_pmu.pmus);

	l3cache_pmu.pmu = (struct pmu) {
		.task_ctx_nr	= perf_invalid_context,

		.pmu_enable	= qcom_l3_cache__pmu_enable,
		.pmu_disable	= qcom_l3_cache__pmu_disable,
		.event_init	= qcom_l3_cache__event_init,
		.add		= qcom_l3_cache__event_add,
		.del		= qcom_l3_cache__event_del,
		.start		= qcom_l3_cache__event_start,
		.stop		= qcom_l3_cache__event_stop,
		.read		= qcom_l3_cache__event_read,

		.attr_groups	= qcom_l3_cache_pmu_attr_grps,
	};

	if (device_property_read_u32(&pdev->dev, "qcom,pmu-version",
				     &l3cache_pmu.version))
		dev_warn(&pdev->dev, "Error reading PMU version, using 0.\n");

	switch (l3cache_pmu.version) {
	case 0:
		l3cache_pmu.offsets = &hml3_pmu_v0_offsets;
		break;
	case 1:
		l3cache_pmu.offsets = &hml3_pmu_v1_offsets;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Designate the probing CPU as the reader and install a hook
	 * to update it in case it goes offline. Priority is increased
	 * to have a chance to migrate events before perf is notified.
	 */
	cpumask_set_cpu(smp_processor_id(), &l3cache_pmu.cpu);
	l3cache_pmu.cpu_nb = (struct notifier_block) {
		.notifier_call = qcom_l3_cache_pmu_cpu_notifier,
		.priority = CPU_PRI_PERF + 1
	};
	err = register_cpu_notifier(&l3cache_pmu.cpu_nb);
	if (err) {
		dev_err(&pdev->dev, "Failed to register CPU notifier\n");
		return err;
	}

	/* Iterate through slices and add */
	err = device_for_each_child(&pdev->dev, &l3cache_pmu,
				    qcom_l3_cache_pmu_probe_slice);

	if (err < 0)
		goto unregister_notifier;

	if (l3cache_pmu.num_pmus == 0) {
		dev_err(&pdev->dev, "No hardware HML3 PMUs found\n");
		goto unregister_notifier;
	}

	if (l3cache_pmu.offsets->bc_gang == -1) {
		/*
		 * 64 bit counters are not supported, remove the 'lc'
		 * attribute from the format group.
		 */
		struct attribute **attrs = qcom_l3_cache_pmu_formats;

		while (*attrs) {
			if (*attrs == &format_attr_lc.attr) {
				*attrs = NULL;
				break;
			}
			attrs++;
		}
	}

	err = perf_pmu_register(&l3cache_pmu.pmu, "l3cache", -1);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to register L3 cache PMU (%d)\n", err);
		goto unregister_notifier;
	}

	dev_info(&pdev->dev,
		 "Registered L3 cache PMU, type: %d, using %d HW PMUs\n",
		 l3cache_pmu.pmu.type, l3cache_pmu.num_pmus);

	return 0;

unregister_notifier:
	unregister_cpu_notifier(&l3cache_pmu.cpu_nb);
	return err;
}

static int qcom_l3_cache_pmu_remove(struct platform_device *pdev)
{
	perf_pmu_unregister(&l3cache_pmu.pmu);
	return 0;
}

static const struct acpi_device_id qcom_l3_cache_pmu_acpi_match[] = {
	{ "QCOM8080", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, qcom_l3_cache_pmu_acpi_match);

static struct platform_driver qcom_l3_cache_pmu_driver = {
	.driver = {
		.name = "qcom-l3cache-pmu",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(qcom_l3_cache_pmu_acpi_match),
	},
	.probe = qcom_l3_cache_pmu_probe,
	.remove = qcom_l3_cache_pmu_remove,
};

static int __init register_qcom_l3_cache_pmu_driver(void)
{
	return platform_driver_register(&qcom_l3_cache_pmu_driver);
}
device_initcall(register_qcom_l3_cache_pmu_driver);
