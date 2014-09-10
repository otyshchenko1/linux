/*
 * SMP support for r8a7793
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/smp_plat.h>
#include <mach/platsmp-apmu.h>

#include "common.h"
#include "pm-rcar.h"
#include "r8a7793.h"
#include "rcar-gen2.h"

#define APMU		0xe6151000
#define CA15DBGRCR	0x1180

static struct rcar_apmu_config r8a7793_apmu_config[] = {
	{
		.iomem = DEFINE_RES_MEM(0xe6152000, 0x88),
		.cpus = { 0, 1 },
	},
};

static void __init r8a7793_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *p;
	u32 val;

	/* let APMU code install data related to shmobile_boot_vector */
	shmobile_smp_apmu_prepare_cpus(max_cpus,
				       r8a7793_apmu_config,
				       ARRAY_SIZE(r8a7793_apmu_config));

	/* setup for debug mode */
	if (rcar_gen2_read_mode_pins() & MD(21)) {
		p = ioremap_nocache(APMU, 0x2000);
		val = readl_relaxed(p + CA15DBGRCR);
		writel_relaxed((val | 0x01f80000), p + CA15DBGRCR);
		iounmap(p);
	}

	r8a7793_pm_init();
}

struct smp_operations r8a7793_smp_ops __initdata = {
	.smp_prepare_cpus	= r8a7793_smp_prepare_cpus,
	.smp_boot_secondary	= shmobile_smp_apmu_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable		= shmobile_smp_cpu_disable,
	.cpu_die		= shmobile_smp_apmu_cpu_die,
	.cpu_kill		= shmobile_smp_apmu_cpu_kill,
#endif
};
