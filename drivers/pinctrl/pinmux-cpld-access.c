/*
 * r8a7790-stout board CPLD access driver
 *
 * Copyright (C) 2015 Renesas Electronics Europe GmbH
 * Copyright (C) 2015 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>

#include "pinmux-cpld-access.h"

//#define CPLD_DEBUG

#define CPLD_ADDR_MODE		0x00 /* RW */
#define CPLD_ADDR_MUX		0x01 /* RW */
#define CPLD_ADDR_HDMI		0x02 /* RW */
#define CPLD_ADDR_DIPSW		0x08 /* R */
#define CPLD_ADDR_RESET		0x80 /* RW */
#define CPLD_ADDR_VERSION	0xFF /* R */

struct pinmux_cpld_access_priv {
	struct delayed_work work;

	unsigned sclk_gpio;
	unsigned sstbz_gpio;
	unsigned mosi_gpio;
	unsigned miso_gpio;

	u8 cpld_addr;

	struct notifier_block restart_handler;

	struct pinctrl *pinctrl;
	struct pinctrl_state **states;
	struct pinctrl_state *state_idle;
};

static u32 cpld_read(struct pinmux_cpld_access_priv *priv, u8 addr)
{
	int i;
	u32 data = 0;

	for (i = 0; i < 8; i++) {
		/* MSB first */
		gpio_set_value(priv->mosi_gpio, addr & 0x80);
		gpio_set_value(priv->sclk_gpio, 1);
		addr <<= 1;
		gpio_set_value(priv->sclk_gpio, 0);
	}

	gpio_set_value(priv->mosi_gpio, 0); /* READ */
	gpio_set_value(priv->sstbz_gpio, 0);
	gpio_set_value(priv->sclk_gpio, 1);
	gpio_set_value(priv->sclk_gpio, 0);
	gpio_set_value(priv->sstbz_gpio, 1);

	for (i = 0; i < 32; i++) {
		gpio_set_value(priv->sclk_gpio, 1);
		data <<= 1;
		/* MSB first */
		data |= (!!gpio_get_value(priv->miso_gpio));
		gpio_set_value(priv->sclk_gpio, 0);
	}

	return data;
}

static void cpld_write(struct pinmux_cpld_access_priv *priv, u8 addr, u32 data)
{
	int i;

	for (i = 0; i < 32; i++) {
		 /* MSB first */
		gpio_set_value(priv->mosi_gpio, data & (1 << 31));
		gpio_set_value(priv->sclk_gpio, 1);
		data <<= 1;
		gpio_set_value(priv->sclk_gpio, 0);
	}

	for (i = 0; i < 8; i++) {
		/* MSB first */
		gpio_set_value(priv->mosi_gpio, addr & 0x80);
		gpio_set_value(priv->sclk_gpio, 1);
		addr <<= 1;
		gpio_set_value(priv->sclk_gpio, 0);
	}

	gpio_set_value(priv->mosi_gpio, 1); /* WRITE */
	gpio_set_value(priv->sstbz_gpio, 0);
	gpio_set_value(priv->sclk_gpio, 1);
	gpio_set_value(priv->sclk_gpio, 0);
	gpio_set_value(priv->sstbz_gpio, 1);
}

static void cpld_init(struct pinmux_cpld_access_priv *priv)
{
	gpio_request_one(priv->sclk_gpio, GPIOF_OUT_INIT_LOW, NULL);
	gpio_request_one(priv->sstbz_gpio, GPIOF_OUT_INIT_HIGH, NULL);
	gpio_request_one(priv->mosi_gpio, GPIOF_OUT_INIT_LOW, NULL);
	gpio_request_one(priv->miso_gpio, GPIOF_IN, NULL);

	/* dummy read */
	cpld_read(priv, CPLD_ADDR_VERSION);

#if defined(CPLD_DEBUG)
	printk("CPLD version:              0x%08x\n",
		cpld_read(priv, CPLD_ADDR_VERSION));
	printk("H2 Mode setting (MD0..28): 0x%08x\n",
		cpld_read(priv, CPLD_ADDR_MODE));
	printk("Multiplexer settings:      0x%08x\n",
		cpld_read(priv, CPLD_ADDR_MUX));
	printk("HDMI setting:              0x%08x\n",
		cpld_read(priv, CPLD_ADDR_HDMI));
	printk("DIPSW (SW3):               0x%08x\n",
		cpld_read(priv, CPLD_ADDR_DIPSW));
#endif
}

static struct pinmux_cpld_access_priv *pinmux_cpld_access;

u32 cpld_mux_read(void)
{
	return cpld_read(pinmux_cpld_access, CPLD_ADDR_MUX);
}
EXPORT_SYMBOL(cpld_mux_read);

void cpld_mux_write(u32 data)
{
	cpld_write(pinmux_cpld_access, CPLD_ADDR_MUX, data);
}
EXPORT_SYMBOL(cpld_mux_write);

static ssize_t cpld_sysfs_read_reg(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct pinmux_cpld_access_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", cpld_read(priv, priv->cpld_addr));
}
static ssize_t cpld_sysfs_write_reg(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct pinmux_cpld_access_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 16, &val);
	if (ret)
		return ret;

#if 0
	if (priv->cpld_addr == CPLD_ADDR_MUX) {
		/* never mask SCIFA0 console */
		val &= ~MUX_MSK_SCIFA0_USB;
		val |= MUX_VAL_SCIFA0_USB;
	}
#endif

	cpld_write(priv, priv->cpld_addr, val);

	return len;
}

static ssize_t cpld_sysfs_get_addr(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct pinmux_cpld_access_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", priv->cpld_addr);
}

static ssize_t cpld_sysfs_set_addr(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct pinmux_cpld_access_priv *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 16, &val);
	if (ret)
		return ret;

	if (!(val == CPLD_ADDR_VERSION || val == CPLD_ADDR_MODE ||
	      val == CPLD_ADDR_MUX || val == CPLD_ADDR_HDMI ||
	      val == CPLD_ADDR_DIPSW || val == CPLD_ADDR_RESET)) {
		return -EINVAL;
	}

	priv->cpld_addr = val;

	return len;
}

static DEVICE_ATTR(val, S_IRUGO | S_IWUGO, cpld_sysfs_read_reg, cpld_sysfs_write_reg);
static DEVICE_ATTR(adr, S_IRUGO | S_IWUGO, cpld_sysfs_get_addr, cpld_sysfs_set_addr);

static int pinmux_cpld_access_restart_handler(struct notifier_block *this,
				      unsigned long mode, void *cmd)
{
	struct pinmux_cpld_access_priv *priv = container_of(this,
						    struct pinmux_cpld_access_priv,
						    restart_handler);

	cpld_write(priv, CPLD_ADDR_RESET, 1);

	return NOTIFY_DONE;
}

static struct pinmux_cpld_access_priv *pinmux_cpld_access_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct pinmux_cpld_access_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	priv->sclk_gpio = of_get_named_gpio(np, "pinmux-cpld-access,sclk-gpio", 0);
	if (priv->sclk_gpio < 0)
		return NULL;

	priv->sstbz_gpio = of_get_named_gpio(np, "pinmux-cpld-access,sstbz-gpio", 0);
	if (priv->sstbz_gpio < 0)
		return NULL;

	priv->mosi_gpio = of_get_named_gpio(np, "pinmux-cpld-access,mosi-gpio", 0);
	if (priv->mosi_gpio < 0)
		return NULL;

	priv->miso_gpio = of_get_named_gpio(np, "pinmux-cpld-access,miso-gpio", 0);
	if (priv->miso_gpio < 0)
		return NULL;

	return priv;
}

static int pinmux_cpld_access_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinmux_cpld_access_priv *priv;
	int ret;

	priv = pinmux_cpld_access_parse_dt(dev);
	if (!priv)
		return -EINVAL;

	cpld_init(priv);

	dev_set_drvdata(dev, priv);

	ret = device_create_file(dev, &dev_attr_val);
	ret += device_create_file(dev, &dev_attr_adr);
	if (ret) {
		dev_err(dev, "cannot create attribute\n");
		return ret;
	}

	/* register reboot handler */
	priv->restart_handler.notifier_call = pinmux_cpld_access_restart_handler;
	priv->restart_handler.priority = 254;
	ret = register_restart_handler(&priv->restart_handler);
	if (ret)
		dev_err(dev,
			"Failed to register restart handler (err = %d)\n", ret);

	priv->cpld_addr = CPLD_ADDR_MUX;
	pinmux_cpld_access = priv;

	return 0;
}

static int pinmux_cpld_access_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinmux_cpld_access_priv *priv = dev_get_drvdata(dev);

	unregister_restart_handler(&priv->restart_handler);

	device_remove_file(dev, &dev_attr_val);
	device_remove_file(dev, &dev_attr_adr);

	return 0;
}

static const struct of_device_id pinmux_cpld_access_dt_ids[] = {
	{ .compatible = "pinmux-cpld-access" },
	{},
};
MODULE_DEVICE_TABLE(of, pinmux_cpld_access_dt_ids);

static struct platform_driver pinmux_cpld_access_driver = {
	.driver	= {
		.name		= "pinmux-cpld-access",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(pinmux_cpld_access_dt_ids),
	},
	.probe	= pinmux_cpld_access_probe,
	.remove	= pinmux_cpld_access_remove,
};

static int __init pinmux_cpld_access_init(void)
{
	return platform_driver_register(&pinmux_cpld_access_driver);
}

static void __exit pinmux_cpld_access_exit(void)
{
	platform_driver_unregister(&pinmux_cpld_access_driver);
}

subsys_initcall_sync(pinmux_cpld_access_init);
module_exit(pinmux_cpld_access_exit);
