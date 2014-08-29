/*
 * SMP support for SoCs with APMU
 *
 * Copyright (C) 2014  Renesas Electronics Corporation
 * Copyright (C) 2013  Magnus Damm
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/threads.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cp15.h>
#include <asm/cpuidle.h>
#include <asm/proc-fns.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <mach/platsmp-apmu.h>
#include <mach/platsmp-rst.h>
#include "common.h"
#include "pm-rcar.h"

/* only enable the cluster that includes the boot CPU by default */
static bool enable_multicluster = false;

static __init int apmu_setup(char *opt)
{
	if (!opt)
		return -EINVAL;
	if (!strncmp(opt, "multicluster", 12))
		enable_multicluster = true;
	return 0;
}
early_param("apmu", apmu_setup);

static struct {
	void __iomem *iomem;
	int bit;
} apmu_cpus[NR_CPUS];

#define WUPCR_OFFS 0x10
#define PSTR_OFFS 0x40
#define CPUNCR_OFFS(n) (0x100 + (0x10 * (n)))
#define CPUCMCR 0xe6154184

void __iomem *cpucmcr;

static int __maybe_unused apmu_power_on(void __iomem *p, int bit)
{
	/* request power on */
	writel_relaxed(BIT(bit), p + WUPCR_OFFS);

	/* wait for APMU to finish */
	while (readl_relaxed(p + WUPCR_OFFS) != 0)
		cpu_relax();

	return 0;
}

static int apmu_power_off(void __iomem *p, int bit)
{
	/* request Core Standby for next WFI */
	writel_relaxed(3, p + CPUNCR_OFFS(bit));
	return 0;
}

static int __maybe_unused apmu_power_off_poll(void __iomem *p, int bit)
{
	int k;

	for (k = 0; k < 1000; k++) {
		if (((readl_relaxed(p + PSTR_OFFS) >> (bit * 4)) & 0x03) == 3)
			return 1;

		mdelay(1);
	}

	return 0;
}

static int apmu_wrap(int cpu, int (*fn)(void __iomem *p, int cpu))
{
	void __iomem *p = apmu_cpus[cpu].iomem;

	return p ? fn(p, apmu_cpus[cpu].bit) : -EINVAL;
}

static void apmu_init_cpu(struct resource *res, int cpu, int bit)
{
	if ((cpu >= ARRAY_SIZE(apmu_cpus)) || apmu_cpus[cpu].iomem)
		return;

	apmu_cpus[cpu].iomem = ioremap_nocache(res->start, resource_size(res));
	apmu_cpus[cpu].bit = bit;

	pr_debug("apmu ioremap %d %d %pr\n", cpu, bit, res);
}

static void apmu_parse_cfg(void (*fn)(struct resource *res, int cpu, int bit),
			   struct rcar_apmu_config *apmu_config, int num)
{
	u32 id;
	int k;
	int bit, index;
	bool is_allowed;

	for (k = 0; k < num; k++) {
		is_allowed = enable_multicluster;
		for (bit = 0; bit < ARRAY_SIZE(apmu_config[k].cpus); bit++) {
			id = apmu_config[k].cpus[bit];
			if (id >= 0) {
				if (id == cpu_logical_map(0))
					is_allowed = true;
			}
		}
		if (!is_allowed)
			continue;

		for (bit = 0; bit < ARRAY_SIZE(apmu_config[k].cpus); bit++) {
			id = apmu_config[k].cpus[bit];
			if (id >= 0) {
				index = get_logical_index(id);
				if (index >= 0)
					fn(&apmu_config[k].iomem, index, bit);
			}
		}
	}
}

void __init shmobile_smp_apmu_prepare_cpus(unsigned int max_cpus,
					   struct rcar_apmu_config *apmu_config,
					   int num)
{
	/* pass physical address of cpu_resume() to assembly resume code */
	cpu_resume_phys_addr = virt_to_phys(cpu_resume);

	/* install boot code shared by all CPUs */
	shmobile_boot_fn = virt_to_phys(shmobile_smp_boot);
	shmobile_boot_arg = MPIDR_HWID_BITMASK;

	/* perform per-cpu setup */
	apmu_parse_cfg(apmu_init_cpu, apmu_config, num);
}

#ifdef CONFIG_SMP
int __cpuinit shmobile_smp_apmu_boot_secondary(unsigned int cpu,
					       struct task_struct *idle)
{
	int ret;

	/* For this particular CPU register boot vector */
	shmobile_smp_hook(cpu, virt_to_phys(shmobile_invalidate_start), 0);

	ret = apmu_wrap(cpu, apmu_power_on);
	r8a779x_deassert_reset(cpu);
	return ret;
}
#endif

#if defined(CONFIG_HOTPLUG_CPU) || defined(CONFIG_SUSPEND) || \
defined(CONFIG_CPU_IDLE)
#define v7_exit_coherency_flush(level) \
	asm volatile( \
	"stmfd	sp!, {fp, ip} \n\t" \
	"mrc	p15, 0, r0, c1, c0, 0	@ get SCTLR \n\t" \
	"bic	r0, r0, #"__stringify(CR_C)" \n\t" \
	"mcr	p15, 0, r0, c1, c0, 0	@ set SCTLR \n\t" \
	"isb	\n\t" \
	"bl	v7_flush_dcache_"__stringify(level)" \n\t" \
	"clrex	\n\t" \
	"mrc	p15, 0, r0, c1, c0, 1	@ get ACTLR \n\t" \
	"bic	r0, r0, #(1 << 6)	@ disable local coherency \n\t" \
	"mcr	p15, 0, r0, c1, c0, 1	@ set ACTLR \n\t" \
	"isb	\n\t" \
	"dsb	\n\t" \
	"ldmfd	sp!, {fp, ip}" \
	: : : "r0","r1","r2","r3","r4","r5","r6","r7", \
	      "r9","r10","lr","memory" )

static inline void cpu_enter_lowpower_a15(void)
{
	v7_exit_coherency_flush(louis);
}

void __cpuinit shmobile_smp_apmu_cpu_shutdown(unsigned int cpu)
{

	/* Select next sleep mode using the APMU */
	apmu_wrap(cpu, apmu_power_off);

	/* Do ARM specific CPU shutdown */
	cpu_enter_lowpower_a15();
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile("mrc    p15, 0, %0, c1, c0, 0\n"
		     "       orr     %0, %0, %1\n"
		     "       mcr     p15, 0, %0, c1, c0, 0\n"
		     "       mrc     p15, 0, %0, c1, c0, 1\n"
		     "       orr     %0, %0, %2\n"
		     "       mcr     p15, 0, %0, c1, c0, 1\n"
		     : "=&r" (v)
		     : "Ir" (CR_C), "Ir" (0x40)
		     : "cc");
}
#endif

#if defined(CONFIG_HOTPLUG_CPU)
void __cpuinit shmobile_smp_apmu_cpu_die(unsigned int cpu)
{
	/* For this particular CPU deregister boot vector */
	shmobile_smp_hook(cpu, 0, 0);

	/* Shutdown CPU core */
	shmobile_smp_apmu_cpu_shutdown(cpu);

	/* jump to shared mach-shmobile sleep / reset code */
	shmobile_smp_sleep();
}

int shmobile_smp_apmu_cpu_kill(unsigned int cpu)
{
	int ret;

	ret = apmu_wrap(cpu, apmu_power_off_poll);
	r8a779x_assert_reset(cpu);
	return ret;
}
#endif

#if defined(CONFIG_SUSPEND) || defined(CONFIG_CPU_IDLE)
static int __cpuinit shmobile_smp_apmu_do_suspend(unsigned long cpu)
{
	shmobile_smp_apmu_cpu_shutdown(cpu);
	cpu_do_idle();
	return 1;
}
#endif

#if defined(CONFIG_SUSPEND)
static int __cpuinit shmobile_smp_apmu_enter_suspend(suspend_state_t state)
{
	/*
	 * Disable the CPU interface when a CPU core is entering L2
	 * shutdown mode, that will help us to prevent spurious CPU
	 * wakeup to happen upon WFI execution.
	 */
	gic_cpu_if_down();

	writel_relaxed(0x2, cpucmcr);
	if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A15) {
		is_a15_l2shutdown = 1;
		asm volatile("mrc p15, 1, %0, c9 , c0, 2"
			: "=r" (l2ctlr_value));
		pr_debug("%s: l2ctlr: 0x%08x\n", __func__, l2ctlr_value);
	} else {
		is_a15_l2shutdown = 0;
	}

	shmobile_smp_hook(smp_processor_id(), virt_to_phys(rcar_cpu_resume), 0);
	cpu_suspend(smp_processor_id(), shmobile_smp_apmu_do_suspend);
	cpu_leave_lowpower();

	writel_relaxed(0x0, cpucmcr);
	is_a15_l2shutdown = 0;
	rcar_sysc_clear_event_status();

	return 0;
}

void __init shmobile_smp_apmu_suspend_init(void)
{
	cpucmcr = ioremap_nocache(CPUCMCR, 0x4);
	shmobile_suspend_ops.enter = shmobile_smp_apmu_enter_suspend;
}
#endif

#if defined(CONFIG_CPU_IDLE)
void shmobile_smp_apmu_enter_cpuidle(void)
{
	cpu_pm_enter();
	cpu_suspend(smp_processor_id(), shmobile_smp_apmu_do_suspend);
	cpu_leave_lowpower();
	cpu_pm_exit();
}
#endif
