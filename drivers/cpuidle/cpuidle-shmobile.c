/*
 * CPUIdle support code for SH-Mobile ARM
 *
 *  Copyright (C) 2014 Renesas Electronics Corporation
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpuidle.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/suspend.h>

#include <asm/cpuidle.h>
#include <asm/proc-fns.h>

static void (*shmobile_enter_corestandby)(void);

int shmobile_enter_cpuidle(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv,
			   int index)
{
	shmobile_enter_corestandby();
	return index;
}

static struct cpuidle_driver renesas_cpuidle_driver = {
	.name = "shmobile_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		ARM_CPUIDLE_WFI_STATE,
		{
			.name			= "C1",
			.desc			= "Core Standby Mode",
			.exit_latency		= 500,
			.target_residency	= 100000,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.enter			= shmobile_enter_cpuidle,
		},
	},
	.state_count = 2,
};

static int shmobile_cpuidle_probe(struct platform_device *pdev)
{
	int ret;

	shmobile_enter_corestandby = (void *)(pdev->dev.platform_data);

	ret = cpuidle_register(&renesas_cpuidle_driver, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to register cpuidle driver\n");
		return ret;
	}

	return 0;
}

static struct platform_driver shmobile_cpuidle_driver = {
	.probe	= shmobile_cpuidle_probe,
	.driver	= {
		.name = "cpuidle-shmobile",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(shmobile_cpuidle_driver);
