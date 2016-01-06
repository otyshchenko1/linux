/*
 * OmniVision ov10635 sensor camera driver
 *
 * Copyright (C) 2016 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#define OV10635_PID			0x300a
#define OV10635_VER			0x300b
#define OV10635_VERSION_REG		0xa635
#define OV10635_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))

#define OV10635_SENSOR_WIDTH		1312
#define OV10635_SENSOR_HEIGHT		814

#define OV10635_WIDTH			1280
#define OV10635_HEIGHT			800

//#define WRITE_VERIFY
//#define OV10635_DISPLAY_PATTERN

#include "ov10635_wizard_1280x800.h"

struct ov10635_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	int				width;
	int				height;
};

static inline struct ov10635_priv *to_ov10635(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov10635_priv, sd);
}

static inline int ov10635_reg16_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	ret = i2c_master_send(client, buf, 2);
	if (ret == 2)
		ret = i2c_master_recv(client, buf, 1);

	if (ret < 0) {
		dev_err(&client->dev,
			"read fail: chip 0x%x register 0x%x\n", client->addr,
								reg);
	} else {
		*val = buf[0];
	}

	return ret < 0 ? ret : 0;
}

static inline int ov10635_reg16_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 buf[3] = {reg >> 8, reg & 0xff, val};

	ret = i2c_master_send(client, buf, 3);

	if (ret < 0) {
		dev_err(&client->dev,
			"write fail: chip 0x%x register 0x%x\n", client->addr,
								 reg);
	} else {
#ifdef WRITE_VERIFY
		u8 val2;
		ov10635_reg16_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
				"write verify mismatch: chip 0x%x reg=0x%x "
				"0x%x->0x%x\n", client->addr, reg, val, val2);
#endif
	}

	return ret < 0 ? ret : 0;
}

static int ov10635_set_regs(struct i2c_client *client,
			    const struct ov10635_reg *regs, int nr_regs)
{
	int i, ret;

	for (i = 0; i < nr_regs; i++) {
		ret = ov10635_reg16_write(client, regs[i].reg, regs[i].val);
#if 0 /* Do not stop on write fail .... */
		if (ret)
			return ret;
#endif
	}

	return 0;
}

static int ov10635_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ov10635_reg16_write(client, 0x0100, enable); /* stream on/off */

	return 0;
}

static int ov10635_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	mf->width	= OV10635_WIDTH;
	mf->height	= OV10635_HEIGHT;
#ifdef CONFIG_SOC_CAMERA_OV10635_10BIT
	mf->code	= V4L2_MBUS_FMT_YUYV10_2X10;
#else
	mf->code	= V4L2_MBUS_FMT_YUYV8_2X8;
#endif
	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov10635_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
#ifdef CONFIG_SOC_CAMERA_OV10635_10BIT
	mf->code = V4L2_MBUS_FMT_YUYV10_2X10;
#else
	mf->code = V4L2_MBUS_FMT_YUYV8_2X8;
#endif
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov10635_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

#ifdef CONFIG_SOC_CAMERA_OV10635_10BIT
	*code = V4L2_MBUS_FMT_YUYV10_2X10;
#else
	*code = V4L2_MBUS_FMT_YUYV8_2X8;
#endif

	return 0;
}

static int ov10635_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_MASTER | V4L2_MBUS_PCLK_SAMPLE_RISING |
		     V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_BT656;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov10635_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	ret = ov10635_reg16_read(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int ov10635_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov10635_reg16_write(client, (u16)reg->reg, (u8)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ov10635_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov10635_g_register,
	.s_register = ov10635_s_register,
#endif
};

static int ov10635_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ov10635_ctrl_ops = {
	.s_ctrl = ov10635_s_ctrl,
};

static struct v4l2_subdev_video_ops ov10635_video_ops = {
	.s_stream	= ov10635_s_stream,
	.g_mbus_fmt	= ov10635_g_fmt,
	.try_mbus_fmt	= ov10635_try_fmt,
	.enum_mbus_fmt	= ov10635_enum_fmt,
	.g_mbus_config	= ov10635_g_mbus_config,
};

static struct v4l2_subdev_ops ov10635_subdev_ops = {
	.core	= &ov10635_core_ops,
	.video	= &ov10635_video_ops,
};

static int ov10635_initialize(struct i2c_client *client)
{
	u8 pid, ver;
	int ret;

	/* check and show product ID and manufacturer ID */
	ret = ov10635_reg16_read(client, OV10635_PID, &pid);
	if (ret)
		return ret;
	ret = ov10635_reg16_read(client, OV10635_VER, &ver);
	if (ret)
		return ret;

	if (OV10635_VERSION(pid, ver) != OV10635_VERSION_REG) {
		dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	dev_info(&client->dev, "ov10635 Product ID %x Manufacturer ID %x\n",
		 pid, ver);

	/* s/w reset sensor */
	ov10635_reg16_write(client, 0x103, 0x1);
	udelay(100);

	/* Program wizard registers */
	ret = ov10635_set_regs(client, ov10635_regs_wizard,
			       ARRAY_SIZE(ov10635_regs_wizard));
	if (ret)
		return ret;

	return 0;
}

static int ov10635_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov10635_priv *priv;
	int ret;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	if (!ssdd) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov10635_subdev_ops);

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	ret = ov10635_initialize(client);
	if (ret < 0)
		goto cleanup;

	return 0;

cleanup:
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
	return ret;
}

static int ov10635_remove(struct i2c_client *client)
{
	struct ov10635_priv *priv = i2c_get_clientdata(client);

	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static void ov10635_shutdown(struct i2c_client *client)
{
	struct ov10635_priv *priv = i2c_get_clientdata(client);

	/* make sure stream off during shutdown (reset/reboot) */
	ov10635_s_stream(&priv->sd, 0);
}

static const struct i2c_device_id ov10635_id[] = {
	{ "ov10635", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov10635_id);

static struct i2c_driver ov10635_i2c_driver = {
	.driver	= {
		.name	= "ov10635",
	},
	.probe		= ov10635_probe,
	.remove		= ov10635_remove,
	.shutdown	= ov10635_shutdown,
	.id_table	= ov10635_id,
};

module_i2c_driver(ov10635_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
