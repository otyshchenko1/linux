/*
 * PTP 1588 clock using the Renesas Ethernet AVB
 *
 * Copyright (C) 2010 OMICRON electronics GmbH
 * Copyright (C) 2013-2015 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/ptp_clock_kernel.h>

#include "ravb.h"

/* ravb ptp registers */
/* Bit definitions for the CSR.OPS */
#define OPS_RESET	(1<<0)
#define OPS_CONFIG	(1<<1)
#define OPS_OPERATION	(1<<2)
#define OPS_STANDBY	(1<<3)

/* Bit definitions for the GCCR.TCSS */
#define TCSS_SHIFT	(8)
#define TCSS_GPTP	(0) /* gPTP timer value */
#define TCSS_CORR_GPTP	(1) /* Corrected gPTP timer value */
#define TCSS_AVTP	(2) /* AVTP presentation time value */

/* Bit definitions for the GCCR.TCR */
#define TCR_NOREQ	(0) /* No request */
#define TCR_RESET	(1) /* Reset gPTP and AVTP presentation timer */
#define TCR_CAPTURE	(3) /* Capture values as selected
			     * by GCCR.TCSS to GCTt.CTV
			     */

/* Bit definitions for the GTI register */
#define GTI_TIV		(0x0fffffff)

#define U32_MAX		((u32)~0U)

#define DRIVER		"ravb_ptp"
#define N_EXT_TS	1
#define N_PER_OUT	1

static inline void ravb_ptp_tcr_request(struct ravb_ptp *ravb_ptp,
		int request)
{
	struct net_device *ndev = ravb_ptp->ndev;

	if ((ravb_read(ndev, CSR) & CSR_OPS) & OPS_OPERATION) {
		while ((ravb_read(ndev, GCCR) & GCCR_TCR) != TCR_NOREQ)
			;
		ravb_write(ndev,
			ravb_read(ndev, GCCR) | request, GCCR);
		while ((ravb_read(ndev, GCCR) & GCCR_TCR) != TCR_NOREQ)
			;
	}
}

static inline bool ravb_ptp_is_config(struct ravb_ptp *ravb_ptp)
{
	struct net_device *ndev = ravb_ptp->ndev;

	if ((ravb_read(ndev, CSR) & CSR_OPS) & OPS_CONFIG)
		return true;
	else
		return false;
}

static int ravb_ptp_avtp_capture_gpio_irq_control(struct ravb_ptp *ravb_ptp,
		int on)
{
	int gpio = ravb_ptp->avtp_capture_gpio;

	if (!gpio)
		return -1;

	if (on)
		enable_irq(gpio_to_irq(gpio));
	else
		disable_irq(gpio_to_irq(gpio));

	return 0;
}

static u64 ravb_ptp_capture_cnt_read(struct ravb_ptp *ravb_ptp)
{
	return ravb_read(ravb_ptp->ndev, GCPT);
}

/* Caller must hold lock */
static void ravb_ptp_time_read(struct ravb_ptp *ravb_ptp,
		struct timespec *ts)
{
	struct net_device *ndev = ravb_ptp->ndev;

	ravb_ptp_tcr_request(ravb_ptp, TCR_CAPTURE);

	ts->tv_nsec = ravb_read(ndev, GCT0);
	ts->tv_sec = ravb_read(ndev, GCT1);
}

/* Caller must hold lock */
static u64 ravb_ptp_cnt_read(struct ravb_ptp *ravb_ptp)
{
	struct timespec ts;
	ktime_t kt;

	ravb_ptp_time_read(ravb_ptp, &ts);
	kt = timespec_to_ktime(ts);

	return ktime_to_ns(kt);
}

/* Caller must hold lock */
static void ravb_ptp_time_write(struct ravb_ptp *ravb_ptp,
		const struct timespec *ts)
{
	struct net_device *ndev = ravb_ptp->ndev;

	ravb_ptp_tcr_request(ravb_ptp, TCR_RESET);

	ravb_write(ndev, ts->tv_nsec, GTO0);
	ravb_write(ndev, ts->tv_sec, GTO1);
	ravb_write(ndev,
			ravb_read(ndev, GCCR) | GCCR_LTO, GCCR);
	if ((ravb_read(ndev, CSR) & CSR_OPS) & OPS_OPERATION)
		while (ravb_read(ndev, GCCR) & GCCR_LTO)
			;
}

/* Caller must hold lock */
static void ravb_ptp_cnt_write(struct ravb_ptp *ravb_ptp, u64 ns)
{
	struct timespec ts;

	ts = ns_to_timespec(ns);

	ravb_ptp_time_write(ravb_ptp, &ts);
}

/* Caller must hold lock */
static void ravb_ptp_select_counter(struct ravb_ptp *ravb_ptp, u16 sel)
{
	struct net_device *ndev = ravb_ptp->ndev;
	u32 val;

	while ((ravb_read(ndev, GCCR) & GCCR_TCR) != TCR_NOREQ)
		;
	val = ravb_read(ndev, GCCR) & ~GCCR_TCSS;
	ravb_write(ndev, val | (sel<<TCSS_SHIFT), GCCR);
}

/* Caller must hold lock */
static void ravb_ptp_update_addend(struct ravb_ptp *ravb_ptp, u32 addend)
{
	struct net_device *ndev = ravb_ptp->ndev;

	ravb_ptp->current_addend = addend;

	ravb_write(ndev, addend & GTI_TIV, GTI);
	ravb_write(ndev,
			ravb_read(ndev, GCCR) | GCCR_LTI, GCCR);
	if ((ravb_read(ndev, CSR) & CSR_OPS) & OPS_OPERATION)
		while (ravb_read(ndev, GCCR) & GCCR_LTI)
			;
}

/* Caller must hold lock */
static void ravb_ptp_update_compare(struct ravb_ptp *ravb_ptp, u32 ns)
{
	struct net_device *ndev = ravb_ptp->ndev;

	/**
	 * When the comparison value (GPTC.PTCV) is in range of
	 * [x-1 to x+1] (x is the configured increment value in
	 * GTI.TIV), it may happend that a comparison match is
	 * not detected when Timer wraps around.
	 */
	u32 gti_ns_plus1 = (ravb_ptp->current_addend >> 20) + 1;
	if (ns < gti_ns_plus1)
		ns = gti_ns_plus1;
	else if (ns > 0 - gti_ns_plus1)
		ns = 0 - gti_ns_plus1;

	ravb_write(ndev, ns, GPTC);
	ravb_write(ndev, ravb_read(ndev, GCCR) | GCCR_LPTC, GCCR);
	if ((ravb_read(ndev, CSR) & CSR_OPS) & OPS_OPERATION)
		while (ravb_read(ndev, GCCR) & GCCR_LPTC)
			;
}

/* Caller must hold lock */
static void ravb_ptp_toggle_avtp_match_gpio(struct ravb_ptp *ravb_ptp)
{
	int gpio = ravb_ptp->avtp_match_gpio;

	if (!gpio)
		return;

	gpio_set_value(gpio, 1);
	gpio_set_value(gpio, 0);
}

/* Interrupt service routine */
static irqreturn_t isr(int irq, void *priv)
{
	struct ravb_ptp *ravb_ptp = priv;
	struct net_device *ndev = ravb_ptp->ndev;
	struct ptp_clock_event event;
	u32 ack = 0, val;

	val = ravb_read(ndev, ISS);
	if (val & ISS_CGIS) {
		val = ravb_read(ndev, GIS) & ravb_read(ndev, GIC);
		if (val & GIS_PTCF) {
			ack |= GIS_PTCF;
			event.type = PTP_CLOCK_EXTTS;
			event.index = 0;
			event.timestamp = ravb_ptp_capture_cnt_read(ravb_ptp);
			ptp_clock_event(ravb_ptp->clock, &event);
		}
		if (val & GIS_PTMF) {
			struct ravb_ptp_perout *perout;

			ack |= GIS_PTMF;
			perout = &ravb_ptp->perout[0];
			if (perout->period) {
				perout->target += perout->period;
				ravb_ptp_update_compare(ravb_ptp,
						perout->target);
				ravb_ptp_toggle_avtp_match_gpio(ravb_ptp);
			}
		}
	}

	if (ack) {
		ravb_write(ndev, ~ack, GIS);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static irqreturn_t isr_avtp_capture_gpio(int irq, void *priv)
{
	struct ravb_ptp *ravb_ptp = priv;
	struct ptp_clock_event event;

	event.type = PTP_CLOCK_EXTTS;
	event.index = 0;
	event.timestamp = ravb_ptp_cnt_read(ravb_ptp);
	ptp_clock_event(ravb_ptp->clock, &event);

	return IRQ_HANDLED;
}

/* PTP clock operations */
static int ravb_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, addend;
	int neg_adj = 0;
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	addend = ravb_ptp->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, NSEC_PER_SEC);

	addend = neg_adj ? addend - diff : addend + diff;

	spin_lock_irqsave(&ravb_ptp->lock, flags);
	ravb_ptp_update_addend(ravb_ptp, addend);
	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ravb_ptp_is_config(ravb_ptp))
		return -EBUSY;

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	now = ravb_ptp_cnt_read(ravb_ptp);
	now += delta;
	ravb_ptp_cnt_write(ravb_ptp, now);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ravb_ptp_is_config(ravb_ptp))
		return -EBUSY;

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	ravb_ptp_time_read(ravb_ptp, ts);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_settime(struct ptp_clock_info *ptp,
		const struct timespec *ts)
{
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	ravb_ptp_time_write(ravb_ptp, ts);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_perout_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);
	struct net_device *ndev = ravb_ptp->ndev;
	struct ravb_ptp_perout *perout;
	unsigned long flags;
	u32 mask;

	if (on) {
		u64 start_ns;
		u64 period_ns;

		start_ns = rq->perout.start.sec * NSEC_PER_SEC +
			rq->perout.start.nsec;
		period_ns = rq->perout.period.sec * NSEC_PER_SEC +
			rq->perout.period.nsec;

		if (start_ns > U32_MAX) {
			netdev_warn(ndev,
					"ptp: Start value (nsec) is over limit. Maximum size of start is only 32 bits\n");
			return -ERANGE;
		}

		if (period_ns > U32_MAX) {
			netdev_warn(ndev,
					"ptp: Period value (nsec) is over limit. Maximum size of period is only 32 bits\n");
			return -ERANGE;
		}

		spin_lock_irqsave(&ravb_ptp->lock, flags);
		perout = &ravb_ptp->perout[rq->perout.index];
		perout->target = (u32)start_ns;
		perout->period = (u32)period_ns;
		ravb_ptp_update_compare(ravb_ptp, (u32)start_ns);

		/* interrupt unmask */
		mask = ravb_read(ndev, GIC);
		mask |= GIC_PTME;
		ravb_write(ndev, mask, GIC);

		spin_unlock_irqrestore(&ravb_ptp->lock, flags);
	} else {
		spin_lock_irqsave(&ravb_ptp->lock, flags);
		perout = &ravb_ptp->perout[rq->perout.index];
		perout->period = 0;

		/* interrupt mask */
		mask = ravb_read(ndev, GIC);
		mask &= ~GIC_PTME;
		ravb_write(ndev, mask, GIC);

		spin_unlock_irqrestore(&ravb_ptp->lock, flags);
	}

	return 0;
}

static int ravb_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);
	struct net_device *ndev = ravb_ptp->ndev;
	unsigned long flags;
	u32 bit, mask;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		if (ravb_ptp->extts[rq->extts.index] == on)
			return 0;
		ravb_ptp->extts[rq->extts.index] = on;

		switch (rq->extts.index) {
		case 0:
			bit = GIC_PTCE;
			ravb_ptp_avtp_capture_gpio_irq_control(ravb_ptp, on);
			break;
		default:
			return -EINVAL;
		}

		spin_lock_irqsave(&ravb_ptp->lock, flags);
		mask = ravb_read(ndev, GIC);
		if (on)
			mask |= bit;
		else
			mask &= ~bit;
		ravb_write(ndev, mask, GIC);
		spin_unlock_irqrestore(&ravb_ptp->lock, flags);
		return 0;

	case PTP_CLK_REQ_PEROUT:
		return ravb_ptp_perout_enable(ptp, rq, on);

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static struct ptp_clock_info ravb_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "ravb clock",
	.max_adj	= 50000000,
	.n_ext_ts	= N_EXT_TS,
	.n_per_out	= N_PER_OUT,
	.adjfreq	= ravb_ptp_adjfreq,
	.adjtime	= ravb_ptp_adjtime,
	.gettime	= ravb_ptp_gettime,
	.settime	= ravb_ptp_settime,
	.enable		= ravb_ptp_enable,
};

#ifdef CONFIG_OF
static void ravb_ptp_parse_dt(struct device *dev, struct ravb_ptp *ravb_ptp)
{
	struct device_node *np = dev->of_node;
	int gpio;

	gpio = of_get_named_gpio(np, "avtp-capture-gpio", 0);
	if (gpio_is_valid(gpio)) {
		gpio_request_one(gpio,
				 GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED,
				 "avtp_capture_gpio");
		ravb_ptp->avtp_capture_gpio = gpio;
		dev_info(dev, "ptp: use GPIO%d instead of avtp_capture.\n",
				gpio);
	}
	gpio = of_get_named_gpio(np, "avtp-match-gpio", 0);
	if (gpio_is_valid(gpio)) {
		gpio_request_one(gpio,
				 GPIOF_OUT_INIT_LOW | GPIOF_EXPORT_DIR_FIXED,
				 "avtp_match_gpio");
		ravb_ptp->avtp_match_gpio = gpio;
		dev_info(dev, "ptp: use GPIO%d instead of avtp_match.\n", gpio);
	}

	return;
}

#else
static inline void ravb_plat_data *ravb_ptp_parse_dt(struct device *dev,
		struct ravb_ptp *ravb_ptp)
{
	return;
}
#endif

int ravb_ptp_init(struct net_device *ndev,
			   struct platform_device *pdev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ptp *ravb_ptp;
	int err = -ENOMEM;
	unsigned long flags;

	ravb_ptp = kzalloc(sizeof(*ravb_ptp), GFP_KERNEL);
	if (!ravb_ptp)
		goto no_memory;

	err = -ENODEV;

	ravb_ptp->ndev = ndev;
	ravb_ptp->caps = ravb_ptp_caps;

	if (pdev->dev.of_node)
		ravb_ptp_parse_dt(&pdev->dev, ravb_ptp);

	if (N_EXT_TS+N_PER_OUT) {
		ravb_ptp->irq = platform_get_irq(pdev, 0);

		if (ravb_ptp->irq == NO_IRQ) {
			dev_err(&pdev->dev, "ptp: irq not in platfrom\n");
			goto no_node;
		}
		if (request_irq(ravb_ptp->irq, isr, IRQF_SHARED,
					DRIVER, ravb_ptp)) {
			dev_err(&pdev->dev, "ptp: request_irq failed\n");
			goto no_node;
		}

		if (ravb_ptp->avtp_capture_gpio) {
			request_irq(gpio_to_irq(ravb_ptp->avtp_capture_gpio),
					isr_avtp_capture_gpio,
					IRQ_TYPE_EDGE_RISING,
					"avtp_capture", ravb_ptp);
			disable_irq(gpio_to_irq(ravb_ptp->avtp_capture_gpio));
		}
	}

	ravb_ptp->default_addend = ravb_read(ndev, GTI);
	ravb_ptp->current_addend = ravb_ptp->default_addend;

	spin_lock_init(&ravb_ptp->lock);

	spin_lock_irqsave(&ravb_ptp->lock, flags);
	ravb_ptp_select_counter(ravb_ptp, TCSS_CORR_GPTP);
	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	ravb_ptp->clock = ptp_clock_register(&ravb_ptp->caps, &pdev->dev);
	if (IS_ERR(ravb_ptp->clock)) {
		err = PTR_ERR(ravb_ptp->clock);
		goto no_clock;
	}

	mdp->ptp = ravb_ptp;

	return 0;

no_clock:
	if (ravb_ptp->avtp_capture_gpio) {
		free_irq(gpio_to_irq(ravb_ptp->avtp_capture_gpio), ravb_ptp);
		gpio_free(ravb_ptp->avtp_capture_gpio);
	}
	if (ravb_ptp->avtp_match_gpio)
		gpio_free(ravb_ptp->avtp_match_gpio);
	if (ravb_ptp->irq)
		free_irq(ravb_ptp->irq, ravb_ptp);
no_node:
	kfree(ravb_ptp);
no_memory:
	return err;
}

int ravb_ptp_stop(struct net_device *ndev,
			   struct platform_device *pdev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ptp *ravb_ptp = mdp->ptp;

	ravb_write(ndev, 0, GIC);
	ravb_write(ndev, 0, GIS);

	ptp_clock_unregister(ravb_ptp->clock);
	if (ravb_ptp->avtp_capture_gpio) {
		free_irq(gpio_to_irq(ravb_ptp->avtp_capture_gpio), ravb_ptp);
		gpio_free(ravb_ptp->avtp_capture_gpio);
	}
	if (ravb_ptp->avtp_match_gpio)
		gpio_free(ravb_ptp->avtp_match_gpio);
	if (ravb_ptp->irq)
		free_irq(ravb_ptp->irq, ravb_ptp);
	kfree(ravb_ptp);
	return 0;
}
