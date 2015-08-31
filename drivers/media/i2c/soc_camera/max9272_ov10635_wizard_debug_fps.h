/*
 * OmniVision ov10635 sensor camera wizard debug fps
 *
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG_FPS			25
#define OV10635_XVCLK			24000000
#define OV10635_VFLIP_SUBSAMPLE		0x1
#define OV10635_VFLIP			0x381c


/*
 * Get the best pixel clock (pclk) that meets minimum hts/vts requirements.
 * xvclk => pre-divider => clk1 => multiplier => clk2 => post-divider => pclk
 * We try all valid combinations of settings for the 3 blocks to get the pixel
 * clock, and from that calculate the actual hts/vts to use. The vts is
 * extended so as to achieve the required frame rate. The function also returns
 * the PLL register contents needed to set the pixel clock.
 */
static int ov10635_get_pclk(int xvclk, int *htsmin, int *vtsmin,
			    int fps_numerator, int fps_denominator,
			    u8 *r3003, u8 *r3004)
{
	int pre_divs[] = { 2, 3, 4, 6, 8, 10, 12, 14 };
	int pclk;
	int best_pclk = INT_MAX;
	int best_hts = 0;
	int i, j, k;
	int best_i = 0, best_j = 0, best_k = 0;
	int clk1, clk2;
	int hts;

	/* Pre-div, reg 0x3004, bits 6:4 */
	for (i = 0; i < ARRAY_SIZE(pre_divs); i++) {
		clk1 = (xvclk / pre_divs[i]) * 2;

		if ((clk1 < 3000000) || (clk1 > 27000000))
			continue;

		/* Mult = reg 0x3003, bits 5:0 */
		for (j = 1; j < 32; j++) {
			clk2 = (clk1 * j);

			if ((clk2 < 200000000) || (clk2 > 500000000))
				continue;

			/* Post-div, reg 0x3004, bits 2:0 */
			for (k = 0; k < 8; k++) {
				pclk = clk2 / (2*(k+1));

				if (pclk > 96000000)
					continue;

				hts = *htsmin + 200 + pclk/300000;

				/* 2 clock cycles for every YUV422 pixel */
				if (pclk < (((hts * *vtsmin)/fps_denominator)
					* fps_numerator * 2))
					continue;

				if (pclk < best_pclk) {
					best_pclk = pclk;
					best_hts = hts;
					best_i = i;
					best_j = j;
					best_k = k;
				}
			}
		}
	}

	/* register contents */
	*r3003 = (u8)best_j;
	*r3004 = ((u8)best_i << 4) | (u8)best_k;

	/* Did we get a valid PCLK? */
	if (best_pclk == INT_MAX)
		return -1;

	*htsmin = best_hts;

	/* Adjust vts to get as close to the desired frame rate as we can */
	*vtsmin = best_pclk / ((best_hts/fps_denominator) * fps_numerator * 2);

	return best_pclk;
}

static int ov10635_set_params(struct i2c_client *client)
{
	struct ov10635_priv *priv = to_ov10635(client);
	int ret = -EINVAL;
	int pclk;
	int hts, vts;
	u8 r3003, r3004;
	int tmp;
	u32 height_pre_subsample;
	u32 width_pre_subsample;
	u8 horiz_crop_mode;
	int nr_isp_pixels;
	int vert_sub_sample = 0;
	int horiz_sub_sample = 0;
	int sensor_width;

	priv->width = OV10635_MAX_WIDTH;
	priv->height = OV10635_MAX_HEIGHT;

	/* Vertical sub-sampling? */
	height_pre_subsample = priv->height;
	if (priv->height <= 400) {
		vert_sub_sample = 1;
		height_pre_subsample <<= 1;
	}

	/* Horizontal sub-sampling? */
	width_pre_subsample = priv->width;
	if (priv->width <= 640) {
		horiz_sub_sample = 1;
		width_pre_subsample <<= 1;
	}

	/* Horizontal cropping */
	if (width_pre_subsample > 768) {
		sensor_width = OV10635_SENSOR_WIDTH;
		horiz_crop_mode = 0x63;
	} else if (width_pre_subsample > 656) {
		sensor_width = 768;
		horiz_crop_mode = 0x6b;
	} else {
		sensor_width = 656;
		horiz_crop_mode = 0x73;
	}

	/* minimum values for hts and vts */
	hts = sensor_width;
	vts = height_pre_subsample + 50;
	dev_dbg(&client->dev, "fps=(%d/%d), hts=%d, vts=%d\n", DEBUG_FPS, 1, hts, vts);

	/* Get the best PCLK & adjust hts,vts accordingly */
	pclk = ov10635_get_pclk(OV10635_XVCLK, &hts, &vts, DEBUG_FPS, 1, &r3003, &r3004);
	if (pclk < 0)
		return ret;
	dev_dbg(&client->dev, "pclk=%d, hts=%d, vts=%d\n", pclk, hts, vts);
	dev_dbg(&client->dev, "r3003=0x%X r3004=0x%X\n", r3003, r3004);

	/* Disable ISP & program all registers that we might modify */
	ret = ov10635_set_regs(client, ov10635_regs_change_mode,
		ARRAY_SIZE(ov10635_regs_change_mode));
	if (ret)
		return ret;

	/* Set PLL */
	ret = maxim_reg16_write(client, 0x3003, r3003);
	if (ret)
		return ret;
	ret = maxim_reg16_write(client, 0x3004, r3004);
	if (ret)
		return ret;

	/* Set format to UYVY */
	ret = maxim_reg16_write(client, 0x4300, 0x3a);
	if (ret)
		return ret;

	/* Set mode to 8-bit BT.656 */
	ret = ov10635_set_regs(client, ov10635_regs_bt656,
		ARRAY_SIZE(ov10635_regs_bt656));
	if (ret)
		return ret;

	/* Set output to 8-bit yuv */
	ret = maxim_reg16_write(client, 0x4605, 0x08);
	if (ret)
		return ret;

	/* Horizontal cropping */
	ret = maxim_reg16_write(client, 0x3621, horiz_crop_mode);
	if (ret)
		return ret;

	ret = maxim_reg16_write(client, 0x3702, (pclk+1500000)/3000000);
	if (ret)
		return ret;
	ret = maxim_reg16_write(client, 0x3703, (pclk+666666)/1333333);
	if (ret)
		return ret;
	ret = maxim_reg16_write(client, 0x3704, (pclk+961500)/1923000);
	if (ret)
		return ret;

	/* Vertical cropping */
	tmp = ((OV10635_SENSOR_HEIGHT - height_pre_subsample) / 2) & ~0x1;
	ret = maxim_reg16_write16(client, 0x3802, tmp);
	if (ret)
		return ret;
	tmp = tmp + height_pre_subsample + 3;
	ret = maxim_reg16_write16(client, 0x3806, tmp);
	if (ret)
		return ret;

	/* Output size */
	ret = maxim_reg16_write16(client, 0x3808, priv->width);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0x380a, priv->height);
	if (ret)
		return ret;

	ret = maxim_reg16_write16(client, 0x380c, hts);
	if (ret)
		return ret;

	ret = maxim_reg16_write16(client, 0x380e, vts);
	if (ret)
		return ret;

#if 0
	if (vert_sub_sample) {
		ret = ov10635_reg_rmw(client, OV10635_VFLIP,
				      OV10635_VFLIP_SUBSAMPLE, 0);
		if (ret)
			return ret;

		ret = ov10635_set_regs(client, ov10635_regs_vert_sub_sample,
			ARRAY_SIZE(ov10635_regs_vert_sub_sample));
		if (ret)
			return ret;
	}
#endif

	ret = maxim_reg16_write16(client, 0x4606, 2*hts);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0x460a, 2*(hts-width_pre_subsample));
	if (ret)
		return ret;

	tmp = (vts - 8) * 16;
	ret = maxim_reg16_write16(client, 0xc488, tmp);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0xc48a, tmp);
	if (ret)
		return ret;

	nr_isp_pixels = sensor_width * (priv->height + 4);
	ret = maxim_reg16_write16(client, 0xc4cc, nr_isp_pixels/256);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0xc4ce, nr_isp_pixels/256);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0xc512, nr_isp_pixels/16);
	if (ret)
		return ret;

	/* Horizontal sub-sampling */
	if (horiz_sub_sample) {
		ret = maxim_reg16_write(client, 0x5005, 0x9);
		if (ret)
			return ret;

		ret = maxim_reg16_write(client, 0x3007, 0x2);
		if (ret)
			return ret;
	}

	ret = maxim_reg16_write16(client, 0xc518, vts);
	if (ret)
		return ret;
	ret = maxim_reg16_write16(client, 0xc51a, hts);
	if (ret)
		return ret;

#ifdef MAXIM_DUMP
	maxim_ov10635_dump_format_regs(client);
#endif

	/* Enable ISP blocks */
	ret = ov10635_set_regs(client, ov10635_regs_enable,
		ARRAY_SIZE(ov10635_regs_enable));
	if (ret)
		return ret;

	return 0;
}
