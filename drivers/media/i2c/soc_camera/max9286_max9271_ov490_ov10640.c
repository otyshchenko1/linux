/*
 * MAXIM max9286-max9271 with OmniVision ov490-ov10640 sensor camera driver
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

#include "max9286_max9271.h"
#include "max9286_max9271_ov490_ov10640.h"

#define OV490_I2C_ADDR		0x24

#define OV490_PID		0x300a
#define OV490_VER		0x300b
#define OV490_VERSION_REG	0x0490
#define OV490_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))
#define OV490_REV		0x0007

#define OV490_ISP_HSIZE_LOW	0x60
#define OV490_ISP_HSIZE_HIGH	0x61
#define OV490_ISP_VSIZE_LOW	0x62
#define OV490_ISP_VSIZE_HIGH	0x63

struct ov490_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	int				width;
	int				height;
	char				is_fixed_sensor;
	int				dvp_width;
};

static inline struct ov490_priv *to_ov490(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov490_priv, sd);
}

static int ov490_set_regs(struct i2c_client *client,
			  const struct ov490_reg *regs, int nr_regs)
{
	int i, ret;

	for (i = 0; i < nr_regs; i++) {
		ret = maxim_reg16_write(client, regs[i].reg, regs[i].val);
#if 0 /* Do not stop on write fail .... */
		if (ret)
			return ret;
#endif
	}

	return 0;
}

static int ov490_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int cam_idx = client->addr - CAM;
	int tmp_addr;

	if (!(client->addr == (CAM + 0) || client->addr == (CAM + 4)))
		return 0;

	tmp_addr = client->addr;
	client->addr = maxim_map[0][cam_idx];		/* MAX9286-CAMx */
	maxim_reg8_write(client, 0x15, 0x13);	/* disable CSI output, VC is set accordingly to Link number */
	if (enable)
		maxim_reg8_write(client, 0x15, 0x9b);	/* enable CSI output, VC is set accordingly to Link number, BIT7 magic must be set */
	client->addr = tmp_addr;

	return 0;
}

static int ov490_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->width;
	mf->height = priv->height;
	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov490_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	if (priv->width != mf->width || priv->height != mf->height) {
		dev_info(&client->dev, "update firmware to use res %dx%d (current res %dx%d)\n",
				       mf->width, mf->height, priv->width, priv->height);

		mf->width = priv->width;
		mf->height = priv->height;
	}

	return 0;
}

static int ov490_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_2X8;

	return 0;
}

static int ov490_g_mbus_config(struct v4l2_subdev *sd,
			       struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov490_g_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	ret = maxim_reg16_read(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int ov490_s_register(struct v4l2_subdev *sd,
			    const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return maxim_reg16_write(client, (u16)reg->reg, (u8)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ov490_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov490_g_register,
	.s_register = ov490_s_register,
#endif
};

static int ov490_s_ctrl(struct v4l2_ctrl *ctrl)
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

static const struct v4l2_ctrl_ops ov490_ctrl_ops = {
	.s_ctrl = ov490_s_ctrl,
};

static struct v4l2_subdev_video_ops ov490_video_ops = {
	.s_stream	= ov490_s_stream,
	.g_mbus_config	= ov490_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov490_subdev_pad_ops = {
	.enum_mbus_code	= ov490_enum_mbus_code,
	.get_fmt	= ov490_get_fmt,
	.set_fmt	= ov490_set_fmt,
};

static struct v4l2_subdev_ops ov490_subdev_ops = {
	.core	= &ov490_core_ops,
	.video	= &ov490_video_ops,
	.pad	= &ov490_subdev_pad_ops,
};

static int ov490_initialize(struct i2c_client *client)
{
	struct ov490_priv *priv = to_ov490(client);
	u8 val = 0;
	u8 pid = 0, ver = 0, rev = 0;
	int ret;
	int tmp_addr, cam_idx = client->addr - CAM;

	if (priv->is_fixed_sensor) {
		dev_info(&client->dev, "ov490/ov10640 fixed-sensor res %dx%d\n", priv->width, priv->height);
		goto streamon;
	}

	/* setup I2C translator address */
	tmp_addr = client->addr;
	client->addr = maxim_map[1][cam_idx];		/* MAX9271-CAMx */
	ret = maxim_reg8_write(client, 0x0A, OV490_I2C_ADDR << 1); /* Set sensor I2C address */
	usleep_range(2000, 2500);			/* wait 2ms after changing reverse_control */
	client->addr = tmp_addr;

	/* check and show product ID and manufacturer ID */
	ret = maxim_reg16_write(client, 0xFFFD, 0x80);
	if (ret)
		return ret;
	ret = maxim_reg16_write(client, 0xFFFE, 0x80);
	if (ret)
		return ret;
	ret = maxim_reg16_read(client, OV490_PID, &pid);
	if (ret)
		return ret;
	ret = maxim_reg16_read(client, OV490_VER, &ver);
	if (ret)
		return ret;
	ret = maxim_reg16_read(client, OV490_REV, &rev);
	if (ret)
		return ret;

	if (OV490_VERSION(pid, ver) != OV490_VERSION_REG) {
		dev_dbg(&client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	/* read resolution used by current firmware */
	maxim_reg16_write(client, 0xFFFD, 0x80);
	maxim_reg16_write(client, 0xFFFE, 0x82);
	maxim_reg16_read(client, OV490_ISP_HSIZE_HIGH, &val);
	priv->width = val;
	maxim_reg16_read(client, OV490_ISP_HSIZE_LOW, &val);
	priv->width = (priv->width << 8) | val;

	maxim_reg16_read(client, OV490_ISP_VSIZE_HIGH, &val);
	priv->height = val;
	maxim_reg16_read(client, OV490_ISP_VSIZE_LOW, &val);
	priv->height = (priv->height << 8) | val;

	maxim_reg16_write(client, 0xFFFD, 0x80);
	maxim_reg16_write(client, 0xFFFE, 0x28);
	switch (priv->dvp_width) {
	case 12:
		maxim_reg16_write(client, 0x6009, 0x10);
		break;
	case 10:
		maxim_reg16_write(client, 0x6009, 0x20);
		break;
	case 8:
		maxim_reg16_write(client, 0x6009, 0x30);
		break;
	default:
		return -EINVAL;
	}

	dev_info(&client->dev, "ov490/ov10640 Product ID %x Manufacturer ID %x, rev 1%x, res %dx%d, dvp %dbit\n",
		 pid, ver, 0xa + rev, priv->width, priv->height, priv->dvp_width);

	ret = ov490_set_regs(client, ov490_regs_wizard,
			     ARRAY_SIZE(ov490_regs_wizard));
	if (ret)
		return ret;

streamon:
	/* switch to GMSL serial_link for streaming video */
	tmp_addr = client->addr;
	client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
	maxim_reg8_write(client, 0x04, 0x83);	/* enable reverse_control/serial_link */
	usleep_range(2000, 2500);		/* wait 2ms after changing reverse_control */
	client->addr = tmp_addr;

	return 0;
}


static int ov490_parse_dt(struct device_node *np, struct ov490_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int err;
	const char *fixed_sensor;

	if (of_property_read_u32(np, "maxim,dvp-width", &priv->dvp_width))
		priv->dvp_width = 12;

	err = of_property_read_string(np, "maxim,fixed-sensor", &fixed_sensor);
	if (err)
		return 0;

	if (strcmp(fixed_sensor, "ov490") == 0) {
		err = of_property_read_u32(np, "maxim,width", &priv->width);
		if (err) {
			dev_err(&client->dev, "maxim,width must be set for fixed-sensor\n");
			goto out;
		}

		err = of_property_read_u32(np, "maxim,height", &priv->height);
		if (err) {
			dev_err(&client->dev, "maxim,height must be set for fixed-sensor\n");
			goto out;
		}

		priv->is_fixed_sensor = true;
	}

out:
	return err;
}

static int ov490_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct ov490_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov490_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto cleanup;

	ret = ov490_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ov490_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	dev_dbg(&client->dev, "failed to probe ov490 @ 0x%02x\n", OV490_I2C_ADDR);
	return ret;
}

static int ov490_remove(struct i2c_client *client)
{
	struct ov490_priv *priv = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static void ov490_shutdown(struct i2c_client *client)
{
	struct ov490_priv *priv = i2c_get_clientdata(client);

	/* make sure stream off during shutdown (reset/reboot) */
	ov490_s_stream(&priv->sd, 0);
}

static const struct i2c_device_id ov490_id[] = {
	{ "ov490-ov10640", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov490_id);

static const struct of_device_id ov490_of_ids[] = {
	{ .compatible = "maxim,max9286-max9271-ov490-ov10640", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov490_of_ids);

static struct i2c_driver ov490_i2c_driver = {
	.driver	= {
		.name		= "ov490-ov10640",
		.of_match_table	= ov490_of_ids,
	},
	.probe		= ov490_probe,
	.remove		= ov490_remove,
	.shutdown	= ov490_shutdown,
	.id_table	= ov490_id,
};

module_i2c_driver(ov490_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for MAX9286<->MAX9271<->OV490<->OV10640");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
