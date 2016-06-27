#include <linux/cpu_pm.h>

#include <asm/cpuidle.h>

/*
 * arm_enter_idle_state - Programs CPU to enter the specified state
 */
static int __maybe_unused arm_generic_enter_idle_state(int idx, int save_context)
{
	int ret = 0;

	if (!idx) {
		cpu_do_idle();
		return idx;
	}

	if (save_context)
		ret = cpu_pm_enter();
	if (!ret) {
		/*
		 * Pass idle state index to cpu_suspend which in turn will
		 * call the CPU ops suspend protocol with idle index as a
		 * parameter.
		 */
		ret = arm_cpuidle_suspend(idx);

		if (save_context)
			cpu_pm_exit();
	}

	return ret ? -1 : idx;
}
