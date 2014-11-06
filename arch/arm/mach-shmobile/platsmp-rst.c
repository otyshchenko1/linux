/*
 * SMP support for SoCs with RESET(RST)
 *
 * Copyright (C) 2013  Magnus Damm
 * Copyright (C) 2013-2014  Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <asm/smp_plat.h>
#include <mach/platsmp-rst.h>

#define RST		0xe6160000

#define r8a779x_clst_id(cpu) (cpu_logical_map((cpu)) >> 8)
#define r8a779x_cpu_id(cpu) (cpu_logical_map((cpu)) & 0xff)

void __iomem *r8a779x_rst_base;
struct rcar_rst_config *r8a779x_clst;

void r8a779x_deassert_reset(unsigned int cpu)
{
	void __iomem *p, *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a779x_clst_id(cpu);

	p = r8a779x_rst_base;
	mask = BIT(3 - r8a779x_cpu_id(cpu));
	magic = r8a779x_clst[clst_id].rescnt_magic;
	rescnt = p + r8a779x_clst[clst_id].rescnt;
	writel_relaxed((readl_relaxed(rescnt) & ~mask) | magic, rescnt);
}

void r8a779x_assert_reset(unsigned int cpu)
{
	void __iomem *p, *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a779x_clst_id(cpu);

	p = r8a779x_rst_base;
	mask = BIT(3 - r8a779x_cpu_id(cpu));
	magic = r8a779x_clst[clst_id].rescnt_magic;
	rescnt = p + r8a779x_clst[clst_id].rescnt;
	writel_relaxed((readl_relaxed(rescnt) | mask) | magic, rescnt);
}

void r8a779x_init_reset(struct rcar_rst_config *rst_config)
{
	r8a779x_rst_base = ioremap_nocache(RST, 0x64);

	r8a779x_clst = rst_config;
}
