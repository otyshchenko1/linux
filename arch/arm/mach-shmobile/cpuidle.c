/*
 * CPUIdle support code for SH-Mobile ARM
 *
 *  Copyright (C) 2011 Magnus Damm
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/pm.h>
#include <linux/cpuidle.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/cpuidle.h>
#include <asm/io.h>
#include "common.h"

static int shmobile_cpuidle_notifier_call(struct notifier_block *nfb,
					  unsigned long action, void *hcpu)
{
	unsigned int cpu = (long)hcpu;

	switch (action) {
	case CPU_ONLINE:
	case CPU_STARTING:
	case CPU_STARTING_FROZEN:
		/* For this particular CPU register CPUIdle boot vector */
		shmobile_smp_hook(cpu, virt_to_phys(cpu_resume), 0);
		break;
	};

	return NOTIFY_OK;
}

static struct notifier_block shmobile_cpuidle_notifier = {
	.notifier_call = shmobile_cpuidle_notifier_call,
};

static struct cpuidle_driver shmobile_cpuidle_default_driver = {
	.name			= "shmobile_cpuidle",
	.owner			= THIS_MODULE,
	.states[0]		= ARM_CPUIDLE_WFI_STATE,
	.safe_state_index	= 0, /* C1 */
	.state_count		= 1,
};

static struct cpuidle_driver *cpuidle_drv = &shmobile_cpuidle_default_driver;

void __init shmobile_cpuidle_set_driver(struct cpuidle_driver *drv)
{
	cpuidle_drv = drv;
}

static struct platform_device shmobile_cpuidle = {
	.name              = "cpuidle-shmobile",
	.dev.platform_data = shmobile_smp_apmu_enter_cpuidle,
	.id                = -1,
};

int __init shmobile_cpuidle_init(void)
{
	int cpu;

	/* Make sure boot CPU also gets CPUIdle initialized */
	for_each_online_cpu(cpu) {
		shmobile_smp_hook(cpu, virt_to_phys(cpu_resume), 0);
	}

#ifdef CONFIG_ARCH_SHMOBILE_MULTI
	/* Use CPU notifier for reset vector control */
	register_cpu_notifier(&shmobile_cpuidle_notifier);

	platform_device_register(&shmobile_cpuidle);
	return 0;
#else
	return cpuidle_register(cpuidle_drv, NULL);
#endif
}
