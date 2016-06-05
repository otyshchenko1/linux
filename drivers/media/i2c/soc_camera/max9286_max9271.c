/*
 * MAXIM R-Car H3 Demo board setup driver
 *
 * Copyright (C) 2015-2016 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

#include "max9286_max9271.h"

#define MAXIM_I2C_I2C_SPEED_837KHZ	(0x7 << 2) /* 837kbps */
#define MAXIM_I2C_I2C_SPEED_533KHZ	(0x6 << 2) /* 533kbps */
#define MAXIM_I2C_I2C_SPEED_339KHZ	(0x5 << 2) /* 339 kbps */
#define MAXIM_I2C_I2C_SPEED_173KHZ	(0x4 << 2) /* 174kbps */
#define MAXIM_I2C_I2C_SPEED_105KHZ	(0x3 << 2) /* 105 kbps */
#define MAXIM_I2C_I2C_SPEED_085KHZ	(0x2 << 2) /* 84.7 kbps */
#define MAXIM_I2C_I2C_SPEED_028KHZ	(0x1 << 2) /* 28.3 kbps */
#define MAXIM_I2C_I2C_SPEED		MAXIM_I2C_I2C_SPEED_339KHZ

struct max9286_max9271_priv {
	int		des_addr;
	int		links;
	int		offset;
	const char	*fsync_mode;
	int		fsync_period;
	char		pclk_rising_edge;
	int		gpio_resetb;
	int		active_low_resetb;
	int		timeout;
};

static void max9286_max9271_preinit(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	switch (priv->des_addr) {
	case DES0:
		client->addr = DES0;			/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x0a, 0x00);	/* disable reverse control for all cams */
		maxim_reg8_write(client, 0x00, 0x00);	/* disable all GMSL links [0:3] */

		client->addr = DES1;			/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x0a, 0x00);	/* disable reverse control for all cams */
		maxim_reg8_write(client, 0x00, 0x00);	/* disable all GMSL links [0:3] */
		usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */

		priv->offset = 0;
		break;
	case DES1:
		priv->offset = 4;
		break;
	default:
		dev_err(&client->dev, "invalid deserializer address\n");
	}
}

static void max9286_max9271_reverse_channel_setup(struct i2c_client *client, int idx)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	u8 val = 0;
	int timeout = priv->timeout;

	/* Reverse channel setup */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x3f, 0x4f);			/* enable custom reverse channel & first pulse length */
	maxim_reg8_write(client, 0x34, 0xa2 | MAXIM_I2C_I2C_SPEED); /* enable artificial ACKs, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */
	maxim_reg8_write(client, 0x00, 0x1 << (idx - priv->offset)); /* enable GMSL link for CAMx */
	maxim_reg8_write(client, 0x0a, 0x11 << (idx - priv->offset)); /* enable reverse control only for CAMx */
	maxim_reg8_write(client, 0x69, (0x1 << (idx - priv->offset)) ^ 0x0f); /* unmask link only for CAMx */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

again:
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x3b, 0x1e);			/* first pulse length rise time changed from 300ns to 200ns, amplitude 100mV */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	i2c_smbus_read_byte(client);				/* ping to wake-up */
	maxim_reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
//	usleep_range(5000, 5500);				/* wait 5ms for conf_link to establish */
	maxim_reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x3b, 0x19);			/* reverse channel increase amplitude 170mV to compensate high threshold enabled */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	i2c_smbus_read_byte(client);				/* ping to wake-up */
	maxim_reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
//	usleep_range(5000, 5500);				/* wait 5ms for conf_link to establish */
	maxim_reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	maxim_reg8_read(client, 0x1e, &val);			/* read max9271 ID */
	if (val != MAX9271_ID && --timeout > 0)
		goto again;

	if (!timeout)
		dev_info(&client->dev, "timeout GMSL link establish, max9271 not found\n");
}

static void max9286_max9271_initial_setup(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x15, 0x13);			/* disable CSI output, VC is set accordingly to Link number */
	if (priv->links == 1)
		maxim_reg8_write(client, 0x12, 0x33);		/* enable CSI-2 Lane D0 only, DBL mode, YUV422 8-bit*/
	else
		maxim_reg8_write(client, 0x12, 0xf3);		/* enable CSI-2 Lanes D[0:3], DBL mode, YUV422 8-bit*/

	if (strcmp(priv->fsync_mode, "manual") == 0) {
		maxim_reg8_write(client, 0x06, priv->fsync_period & 0xff);
		maxim_reg8_write(client, 0x07, (priv->fsync_period >> 8) & 0xff);
		maxim_reg8_write(client, 0x08, priv->fsync_period >> 16);
		maxim_reg8_write(client, 0x01, 0x00);		/* manual: FRAMESYNC set manually via [0x06:0x08] regs */
	} else if (strcmp(priv->fsync_mode, "automatic") == 0) {
		maxim_reg8_write(client, 0x01, 0x02);		/* automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "semi-automatic") == 0) {
		maxim_reg8_write(client, 0x01, 0x01);		/* semi-automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "external") == 0) {
		maxim_reg8_write(client, 0x01, 0xc0);		/* ECU (aka MCU) based FrameSync using GPI-to-GPO */
	}

	maxim_reg8_write(client, 0x63, 0);			/* disable overlap window */
	maxim_reg8_write(client, 0x64, 0);
	maxim_reg8_write(client, 0x0c, 0x89);			/* enable HS/VS encoding, use D14/15 for HS/VS, invert VS */
}

static void max9286_max9271_gmsl_link_setup(struct i2c_client *client, int idx)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* GMSL setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	maxim_reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_I2C_SPEED); /* disable artificial ACK, I2C speed set */
	maxim_reg8_write(client, 0x07, 0x84 | (priv->pclk_rising_edge ? 0x10 : 0)); /* RAW/YUV, PCLK edge, HS/VS encoding enabled */
	usleep_range(2000, 2500);				/* wait 2ms */
	maxim_reg8_write(client, 0x02, 0xff);			/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */
	usleep_range(2000, 2500);				/* wait 2ms */

	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x34, 0x22 | MAXIM_I2C_I2C_SPEED); /* disable artificial ACK, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms */

	/* I2C translator setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	maxim_reg8_write(client, 0x09, maxim_map[2][idx] << 1);	/* SENSOR I2C new */
	maxim_reg8_write(client, 0x0A, 0x30 << 1);		/* SENSOR I2C address - must be set by sensor driver */
	maxim_reg8_write(client, 0x0B, BROADCAST << 1);		/* broadcast I2C */
	maxim_reg8_write(client, 0x0C, maxim_map[1][idx] << 1);	/* MAX9271-CAMx I2C new */
	/* I2C addresse change */
	maxim_reg8_write(client, 0x01, maxim_map[0][idx] << 1);	/* MAX9286-CAM0 I2C */
	maxim_reg8_write(client, 0x00, maxim_map[1][idx] << 1);	/* MAX9271-CAM0 I2C new */
	usleep_range(2000, 2500);				/* wait 2ms */
#ifdef MAXIM_DUMP
	client->addr = maxim_map[0][idx];			/* MAX9286-CAMx I2C */
	maxim_max927x_dump_regs(client);
	client->addr = maxim_map[1][idx];			/* MAX9271-CAMx I2C new */
	maxim_max927x_dump_regs(client);
	client->addr = maxim_map[2][idx];			/* SENSOR-CAMx I2C */
	maxim_ovsensor_dump_regs(client);
#endif
	if (priv->gpio_resetb > 0) {
		/* get out from sensor reset */
		client->addr = maxim_map[1][idx];		/* MAX9271-CAMx I2C new */
		maxim_reg8_write(client, 0x0f, (0xfe & ~BIT(priv->gpio_resetb)) |
				 (priv->active_low_resetb ? BIT(priv->gpio_resetb) : 0)); /* set GPIOn value to un-reset */
		usleep_range(2000, 2500);			/* wait 2ms */
		maxim_reg8_write(client, 0x0e, 0x42 | BIT(priv->gpio_resetb)); /* set GPIOn direction output */
		usleep_range(2000, 2500);			/* wait 2ms */
	}
}

static void max9286_max9271_gmsl_enable(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* Reverse channel setup */
	client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x1b, 0x0f);		/* enable equalizer for all links */
	if (priv->links == 1) {
		maxim_reg8_write(client, 0x00, 0xe1);	/* enable GMSL link 0, auto detect link used for CSI clock source */
		maxim_reg8_write(client, 0x0a, 0xf1);	/* enable reverse control only for link0 */
		maxim_reg8_write(client, 0x69, 0x0e);	/* mask Links 1 2 3, unmask link 0 */
	} else {
		maxim_reg8_write(client, 0x00, 0xef);	/* enable GMSL links [0:3], auto detect link used for CSI clock source */
		maxim_reg8_write(client, 0x0a, 0xff);	/* enable reverse control for all links */
		maxim_reg8_write(client, 0x69, 0x00);	/* unmask all links */
	}
	usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */
}

static int max9286_max9271_initialize(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	int idx;

	max9286_max9271_preinit(client);
	max9286_max9271_initial_setup(client);

	for (idx = priv->offset; idx < priv->links + priv->offset; idx++) {
		/* SETUP CAMx (MAX9286/MAX9271/SENSOR) I2C */
		dev_info(&client->dev, "link%d (MAX9286/MAX9271/SENSOR) i2c: 0x%x<->0x%x<->0x%x\n", idx,
				       maxim_map[0][idx], maxim_map[1][idx], maxim_map[2][idx]);

		max9286_max9271_reverse_channel_setup(client, idx);
		max9286_max9271_gmsl_link_setup(client, idx);
	}

	max9286_max9271_gmsl_enable(client);

	return 0;
}

static int max9286_max9271_parse_dt(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	int err, pwen;
	int sensor_delay;
	char fsync_mode_default[20] = "manual"; /* manual, automatic, semi-automatic, external */

	if (of_property_read_u32(np, "maxim,links", &priv->links))
		priv->links = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = gpio_request(pwen, dev_name(&client->dev));
		if (err) {
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
		} else {
			gpio_direction_output(pwen, 1);
			mdelay(250);
		}
		/*
		 * Powered MCU IMI cameras need delay between power-on and R-Car access to avoid
		 * i2c bus conflicts since linux kernel does not support i2c multi-mastering,
		 * IMI MCU is master and R-Car is also master.
		 * The i2c bus conflict results in R-Car i2c IP stall.
		 */
		if (!of_property_read_u32(np, "maxim,sensor_delay", &sensor_delay))
			mdelay(sensor_delay);
	}

	if (of_property_read_u32(np, "maxim,resetb-gpio", &priv->gpio_resetb)) {
		priv->gpio_resetb = -1;
	} else {
		if (priv->gpio_resetb > 5 || priv->gpio_resetb < 1) {
			dev_err(&client->dev, "gpio_resetb=%d invalid, max9271 gpio range [1..5]\n", priv->gpio_resetb);
			priv->gpio_resetb = -1;
		}
		if (of_property_read_bool(np, "maxim,resetb-active-high"))
			priv->active_low_resetb = false;
		else
			priv->active_low_resetb = true;
	}
		err = of_property_read_string(np, "maxim,fsync-mode", &priv->fsync_mode);
	if (err)
		priv->fsync_mode = fsync_mode_default;

	if (of_property_read_u32(np, "maxim,fsync-period", &priv->fsync_period))
		priv->fsync_period = 3200000; /* 96MHz/30fps */
		priv->pclk_rising_edge = true;
	if (of_property_read_bool(np, "maxim,pclk-falling-edge"))
		priv->pclk_rising_edge = false;
		if (of_property_read_u32(np, "maxim,timeout", &priv->timeout))
		priv->timeout = 100;

	dev_info(&client->dev, "LINKs=%d, RESETB GPIO=%d, FSYNC mode=%s, FSYNC period=%d, PCLK edge=%s\n",
			       priv->links, priv->gpio_resetb, priv->fsync_mode, priv->fsync_period,
			       priv->pclk_rising_edge ? "rising" : "falling");

	return 0;
}

static int max9286_max9271_probe(struct i2c_client *client,
				 const struct i2c_device_id *did)
{
	struct max9286_max9271_priv *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;

	err = max9286_max9271_parse_dt(client);
	if (err)
		goto out;

	err = max9286_max9271_initialize(client);
	if (err < 0)
		goto out;

	/* release des_addr to use it in sensor subdevice */
	client->addr = priv->des_addr - 0x40;

out:
	return err;
}

static int max9286_max9271_remove(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* revert des_addr - needed when used as module */
	client->addr = priv->des_addr;

	return 0;
}

static const struct of_device_id max9286_max9271_dt_ids[] = {
	{ .compatible = "maxim,max9286-max9271" },
	{},
};
MODULE_DEVICE_TABLE(of, max9286_max9271_dt_ids);

static const struct i2c_device_id max9286_max9271_id[] = {
	{ "max9286_max9271", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9286_max9271_id);

static struct i2c_driver max9286_max9271_i2c_driver = {
	.driver	= {
		.name		= "max9286_max9271",
		.of_match_table	= of_match_ptr(max9286_max9271_dt_ids),
	},
	.probe		= max9286_max9271_probe,
	.remove		= max9286_max9271_remove,
	.id_table	= max9286_max9271_id,
};

module_i2c_driver(max9286_max9271_i2c_driver);

MODULE_DESCRIPTION("Setup driver for GMSL Cameras MAX9286<->MAX9271<->SENSOR");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
