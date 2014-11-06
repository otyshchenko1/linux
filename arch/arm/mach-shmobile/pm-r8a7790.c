/*
 * r8a7790 Power management support
 *
 * Copyright (C) 2013-2014  Renesas Electronics Corporation
 * Copyright (C) 2011  Renesas Solutions Corp.
 * Copyright (C) 2011  Magnus Damm
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>

#include <linux/smp.h>
#include <asm/io.h>

#include "common.h"
#include "pm-rcar.h"
#include "r8a7790.h"

/* RST */
#define RST		0xe6160000
#define CA15BAR		0x0020
#define CA7BAR		0x0030
#define RAM		0xe63c0000

/* SYSC */
#define SYSCIER 0x0c
#define SYSCIMR 0x10

struct r8a7790_pm_domain {
	struct generic_pm_domain genpd;
	struct rcar_sysc_ch ch;
};

static inline struct rcar_sysc_ch *to_r8a7790_ch(struct generic_pm_domain *d)
{
	return &container_of(d, struct r8a7790_pm_domain, genpd)->ch;
}

#if defined(CONFIG_PM) || defined(CONFIG_SMP)

static void __init r8a7790_sysc_init(void)
{
	void __iomem *base = rcar_sysc_init(0xe6180000);

	/* enable all interrupt sources, but do not use interrupt handler */
	iowrite32(0x0131000e, base + SYSCIER);
	iowrite32(0, base + SYSCIMR);
}

#else /* CONFIG_PM || CONFIG_SMP */

static inline void r8a7790_sysc_init(void) {}

#endif /* CONFIG_PM || CONFIG_SMP */

#ifdef CONFIG_PM

static int pd_power_down(struct generic_pm_domain *genpd)
{
	return rcar_sysc_power_down(to_r8a7790_ch(genpd));
}

static int pd_power_up(struct generic_pm_domain *genpd)
{
	return rcar_sysc_power_up(to_r8a7790_ch(genpd));
}

static bool pd_active_wakeup(struct device *dev)
{
	return true;
}

static struct notifier_block platform_nb;

static void r8a7790_init_pm_domain(struct r8a7790_pm_domain *r8a7790_pd)
{
	struct generic_pm_domain *genpd = &r8a7790_pd->genpd;

	pm_genpd_init(genpd, NULL, true);
	genpd->dev_ops.stop = pm_clk_suspend;
	genpd->dev_ops.start = pm_clk_resume;
	genpd->dev_ops.active_wakeup = pd_active_wakeup;
	genpd->dev_irq_safe = true;
	genpd->power_off = pd_power_down;
	genpd->power_on = pd_power_up;

	bus_register_notifier(&platform_bus_type, &platform_nb);
}

static struct r8a7790_pm_domain r8a7790_pm_domains[] = {
	{
		.genpd.name = "pvrsrvkm",
		.ch = {
			.chan_offs = 0xc0, /* PWRSR2 .. PWRER2 */
			.isr_bit = 20, /* RGX */
		},
	},
};

void __init r8a7790_init_pm_domains(void)
{
	int j;

	for (j = 0; j < ARRAY_SIZE(r8a7790_pm_domains); j++)
		r8a7790_init_pm_domain(&r8a7790_pm_domains[j]);
}

static int r8a7790_pm_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct device *dev = data;
	struct r8a7790_pm_domain *pd;
	int j;

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		for (j = 0; j < ARRAY_SIZE(r8a7790_pm_domains); j++) {
			pd = &r8a7790_pm_domains[j];
			if (!strcmp(pd->genpd.name, dev_name(dev))) {
				pm_genpd_add_device(&pd->genpd, dev);
				if (pm_clk_no_clocks(dev))
					pm_clk_add(dev, NULL);
			}
		}
		break;

	case BUS_NOTIFY_UNBOUND_DRIVER:
		for (j = 0; j < ARRAY_SIZE(r8a7790_pm_domains); j++) {
			pd = &r8a7790_pm_domains[j];
			if (!strcmp(pd->genpd.name, dev_name(dev)))
				pm_genpd_remove_device(&pd->genpd, dev);

		}
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = r8a7790_pm_notifier_call,
};

#endif /* CONFIG_PM */

void __init r8a7790_pm_init(void)
{
	void __iomem *p;
	u32 bar;
	static int once;

	if (once++)
		return;

	/* RAM for jump stub, because BAR requires 256KB aligned address */
	p = ioremap_nocache(RAM, shmobile_boot_size);
	memcpy_toio(p, shmobile_boot_vector, shmobile_boot_size);
	iounmap(p);

	/* setup reset vectors */
	p = ioremap_nocache(RST, 0x63);
	bar = (RAM >> 8) & 0xfffffc00;
	writel_relaxed(bar, p + CA15BAR);
	writel_relaxed(bar, p + CA7BAR);
	writel_relaxed(bar | 0x10, p + CA15BAR);
	writel_relaxed(bar | 0x10, p + CA7BAR);
	iounmap(p);

	r8a7790_sysc_init();
	shmobile_smp_apmu_suspend_init();
}
