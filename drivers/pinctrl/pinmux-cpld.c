/*
 * r8a7790-stout board CPLD MUX driver
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
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>

#include "pinmux-cpld-access.h"

#define CPLD_MUX_MSK(x)		(2*x)
#define CPLD_MUX_VAL(x)		(1+2*x)

struct pinmux_cpld_priv {
	const char **pinctrl_states;
	u32 *cpldmux_states;
	int states_count;
	int states_current;

	struct pinctrl *pinctrl;
	struct pinctrl_state **states;
};

static ssize_t cpld_sysfs_read_states(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct pinmux_cpld_priv *priv = dev_get_drvdata(dev);
	char *out = buf;
	int i;

	for (i = 0; i < priv->states_count; i++)
		out += sprintf(out, "%s ", priv->pinctrl_states[i]);

	out += sprintf(out, "\n");

	return out - buf;
}

static ssize_t cpld_sysfs_read_state(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct pinmux_cpld_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", priv->pinctrl_states[priv->states_current]);
}

static ssize_t cpld_sysfs_write_state(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct pinmux_cpld_priv *priv = dev_get_drvdata(dev);
	int i, ret;
	u32 reg;

	for (i = 0; i < priv->states_count; i++) {
		if (!strncmp(priv->pinctrl_states[i], buf, 20)) {
			ret = pinctrl_select_state(priv->pinctrl, priv->states[i]);
			if (!ret) {
				priv->states_current = i;
				reg = cpld_mux_read();
				reg &= ~priv->cpldmux_states[CPLD_MUX_MSK(i)];
				reg |= priv->cpldmux_states[CPLD_MUX_VAL(i)];
				cpld_mux_write(reg);
			}
			return len;
		}
	}

	return -EINVAL;
}

static DEVICE_ATTR(states, S_IRUGO, cpld_sysfs_read_states, NULL);
static DEVICE_ATTR(state, S_IRUGO | S_IWUGO, cpld_sysfs_read_state, cpld_sysfs_write_state);

static int pinmux_cpld_initialize(struct device *dev)
{
	struct pinmux_cpld_priv *priv = dev_get_drvdata(dev);
	int i, ret;
	u32 reg;

	/* read the bootloader cpld mux state and setup SoC pinmux correspondingly */
	reg = cpld_mux_read();
	priv->states_current = -1;

	for (i = 0; i < priv->states_count; i++) {
		if ((reg & priv->cpldmux_states[CPLD_MUX_MSK(i)]) ==
					priv->cpldmux_states[CPLD_MUX_VAL(i)]) {
			ret = pinctrl_select_state(priv->pinctrl, priv->states[i]);
			if (!ret) {
				priv->states_current = i;
				dev_info(dev, "setup cpldmux state %s\n",
					 priv->pinctrl_states[i]);
				break;
			}
		}
	}

	if (priv->states_current == -1) {
		dev_info(dev, "state not found for CPLD_MUX=0x%x, cpldmux not set\n", reg);
#if 0
		ret = pinctrl_select_state(priv->pinctrl, priv->states[0]);
		if (!ret) {
			priv->states_current = 0;
			reg = cpld_mux_read();
			reg &= ~priv->cpldmux_states[CPLD_MUX_MSK(0)];
			reg |= priv->cpldmux_states[CPLD_MUX_VAL(0)];
			cpld_mux_write(reg);
		}

		dev_info(dev, "fall to cpldmux-0 state 0x%x\n", reg);
#endif
	}

	return 0;
}

static struct pinmux_cpld_priv *pinmux_cpld_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct pinmux_cpld_priv *priv;
	int num_names, i, ret;
	char name[10];

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	num_names = of_property_count_strings(np, "pinctrl-names");
	if (num_names < 0) {
		dev_err(dev, "Cannot parse pinctrl-names: %d\n", num_names);
		return NULL;
	}

	priv->pinctrl_states = devm_kzalloc(dev,
			sizeof(*priv->pinctrl_states) * num_names, GFP_KERNEL);
	if (!priv->pinctrl_states) {
		dev_err(dev, "Cannot allocate pinctrl_states\n");
		return NULL;
	}

	priv->cpldmux_states = devm_kzalloc(dev,
			sizeof(*priv->cpldmux_states) * num_names * 2, GFP_KERNEL);
	if (!priv->cpldmux_states) {
		dev_err(dev, "Cannot allocate cpldmux_states\n");
		return NULL;
	}

	for (i = 0; i < num_names; i++) {
		ret = of_property_read_string_index(np, "pinctrl-names", i,
			&priv->pinctrl_states[i]);
		if (ret < 0) {
			dev_err(dev, "Cannot parse pinctrl-names: %d\n", ret);
			return NULL;
		}

		snprintf(name, 10, "cpldmux-%d", i);
		ret = of_property_read_u32_index(np, name, 0, &priv->cpldmux_states[CPLD_MUX_MSK(i)]);
		if (ret < 0) {
			dev_err(dev, "Cannot parse cpldmux-%d: %d\n",i, ret);
			return NULL;
		}

		ret = of_property_read_u32_index(np, name, 1, &priv->cpldmux_states[CPLD_MUX_VAL(i)]);
		if (ret < 0) {
			dev_err(dev, "Cannot parse cpldmux-%d: %d\n", i, ret);
			return NULL;
		}

		priv->states_count++;
	}

	return priv;
}

static int pinmux_cpld_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinmux_cpld_priv *priv;
	int ret, i;

	priv = pinmux_cpld_parse_dt(dev);
	if (!priv)
		return -EINVAL;

	dev_set_drvdata(dev, priv);

	ret = device_create_file(dev, &dev_attr_states);
	ret += device_create_file(dev, &dev_attr_state);
	if (ret) {
		dev_err(dev, "cannot create attribute\n");
		return ret;
	}

	priv->states = devm_kzalloc(dev, sizeof(*priv->states) * priv->states_count, GFP_KERNEL);
	if (!priv->states) {
		dev_err(dev, "Cannot allocate states\n");
		ret = -ENOMEM;
		goto err;
	}

	priv->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(priv->pinctrl)) {
		ret = PTR_ERR(priv->pinctrl);
		dev_err(dev, "Cannot get pinctrl: %d\n", ret);
		goto err;
	}

	for (i = 0; i < priv->states_count; i++) {
		priv->states[i] = pinctrl_lookup_state(priv->pinctrl,
						priv->pinctrl_states[i]);
		if (IS_ERR(priv->states[i])) {
			ret = PTR_ERR(priv->states[i]);
			dev_err(dev, "Cannot look up pinctrl state %s: %d\n",
				priv->pinctrl_states[i], ret);
			goto err;
		}
	}

	pinmux_cpld_initialize(dev);

	return 0;

err:
	device_remove_file(dev, &dev_attr_states);
	device_remove_file(dev, &dev_attr_state);
	return ret;
}

static int pinmux_cpld_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	device_remove_file(dev, &dev_attr_states);
	device_remove_file(dev, &dev_attr_state);

	return 0;
}

static const struct of_device_id pinmux_cpld_dt_ids[] = {
	{ .compatible = "pinmux-cpld" },
	{},
};
MODULE_DEVICE_TABLE(of, pinmux_cpld_dt_ids);

static struct platform_driver pinmux_cpld_driver = {
	.driver	= {
		.name		= "pinmux-cpld",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(pinmux_cpld_dt_ids),
	},
	.probe	= pinmux_cpld_probe,
	.remove	= pinmux_cpld_remove,
};

static int __init pinmux_cpld_init(void)
{
	return platform_driver_register(&pinmux_cpld_driver);
}

static void __exit pinmux_cpld_exit(void)
{
	platform_driver_unregister(&pinmux_cpld_driver);
}

subsys_initcall_sync(pinmux_cpld_init);
module_exit(pinmux_cpld_exit);
