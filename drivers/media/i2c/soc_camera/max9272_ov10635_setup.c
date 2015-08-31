/*
 * MAXIM R-Car H2 Demo board setup driver
 *
 * Copyright (C) 2015 Cogent Embedded, Inc.
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

#include "max9272_ov10635.h"

static int maxim_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	int err;
	int pwen;
#ifdef MAX9275_PRESENT
	u8 val;
#endif
	int cam_idx, tmp_addr;

	if (client->dev.of_node) {
		pwen = of_get_gpio(client->dev.of_node, 0);
		if (pwen > 0) {
			err = gpio_request(pwen, dev_name(&client->dev));
			if (err) {
				dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
			} else {
				gpio_direction_output(pwen, 1);
				mdelay(250);
			}
		}
	}

	/*
	 * Powered MCU IMI cameras need delay between power-on and R-Car access to avoid
	 * i2c bus conflicts since linux kernel does not support i2c multi-mastering,
	 * IMI MCU is master and R-Car is also master.
	 * The i2c bus conflict results in R-Car i2c IP stall.
	 */
	mdelay(MAXIM_IMI_MCU_DELAY);

	tmp_addr = client->addr;

	/* power down cascaded MAX9272 chips */
	client->addr = 0x48;						/* MAX9272-CAM0 I2C */
	maxim_reg8_write(client, 0x0e, 0x02);				/* GP0 high */

#ifdef MAX9275_PRESENT
	/*
	 * SETUP MAX9275 I2C
	 */
	dev_info(&client->dev, "SETUP MAX9275 I2C to 0x46<->04e\n");

	client->addr = 0x48;						/* MAX9272-CAM0 I2C */
	maxim_reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_SPEED);		/* disable artificial ACK, I2C speed set */
	maxim_reg8_write(client, 0x04, 0x00);				/* disable reverse_control/serial_link/conf_link - this is to disable access to max9271-CAM0 */

	client->addr = 0x40 + 6;					/* MAX9275 new I2C */
	maxim_reg8_read(client, 0x1e, &val);				/* read ID and check if we come from reboot or power down */
	if (val != 0x21) {
		client->addr = 0x40;					/* MAX9275-CAM0 I2C after poweroff */
		i2c_smbus_read_byte(client);				/* ping to wake-up */
		maxim_reg8_write(client, 0x04, 0x00);			/* wake up, disable reverse_control/serial_link/conf_link */

		/* make sure that it is a MAX9275 */
		maxim_reg8_read(client, 0x1e, &val);			/* read ID */
		if (val == 0x21) {
			maxim_reg8_write(client, 0x13, 0x36);		/* disable artificial ACK */
			maxim_reg8_write(client, 0x04, 0x00);		/* disable reverse_control/serial_link/conf_link */
			maxim_reg8_write(client, 0x01, (0x48 + 6) << 1); /* set MAX9276 I2C pair to 0x48+6 */
			maxim_reg8_write(client, 0x00, (0x40 + 6) << 1); /* set MAX9275 I2C to 0x40+6 */
		}
	}
#endif

	for (cam_idx = 0; cam_idx < MAXIM_NUM; cam_idx++) {
		/*
		 * SETUP CAMx (MAX9272/MAX9271/OV10635) I2C
		 */
		dev_info(&client->dev, "SETUP CAM%d (MAX9272/MAX9271/OV10635) I2C: 0x%x<->0x%x<->0x%x\n", cam_idx,
				       maxim_map[0][cam_idx], maxim_map[1][cam_idx], maxim_map[2][cam_idx]);

		/* Reverse channel setup */
		client->addr = 0x48;					/* MAX9272-CAMx I2C */
		maxim_reg8_write(client, 0x04, 0x43);			/* enable reverse_control/conf_link */
		maxim_reg8_write(client, 0x0d, 0xa2 | MAXIM_I2C_SPEED);	/* enable artificial ACKs - this is to wake up MAX9271-CAMx, I2C speed set */
		maxim_reg8_write(client, 0x15, 0x17);			/* reverse channel pulse length 200ns, amplitude 100mV */
		mdelay(2);						/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;					/* MAX9271-CAMx I2C */
		i2c_smbus_read_byte(client);				/* ping to wake-up */
		maxim_reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		mdelay(5);						/* wait 5ms for conf_link to establish */
		maxim_reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
		mdelay(2);						/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x48;					/* MAX9272-CAMx I2C */
		maxim_reg8_write(client, 0x15, 0x1f);			/* reverse channel increase amplitude 160mV to compensate high threshold enabled */
		mdelay(2);						/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;					/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		mdelay(5);						/* wait 5ms for conf_link to establish */
		maxim_reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
		mdelay(2);						/* wait 2ms after any change of reverse channel settings */

		/* GMSL setup */
		client->addr = 0x40;					/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
		maxim_reg8_write(client, 0x07, 0x94);			/* RAW/YUV, PCLK rising edge, HS/VS encoding enabled */
		maxim_reg8_write(client, 0x02 , 0xff);			/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */

		client->addr = 0x48;					/* MAX9272-CAMx I2C */
		maxim_reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
		maxim_reg8_write(client, 0x07, 0x84);			/* RAW/YUV, 24-bit, PCLK rising edge, HS/VS encoding enabled */
		maxim_reg8_write(client, 0x05, 0x7B);			/* boosted current, full hs/vs tracking, equalizer enable, 13dB boost gain */
		maxim_reg8_write(client, 0x02, 0xdf);			/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */
		mdelay(2);						/* wait 2ms */

#if 1
		/* I2C translator setup */
		client->addr = 0x40;					/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x09, maxim_map[2][cam_idx] << 1); /* OV10635 I2C new */
		maxim_reg8_write(client, 0x0A, 0x30 << 1);		/* OV10635 I2C */
		maxim_reg8_write(client, 0x0B, 0x45 << 1);		/* broadcast I2C */
		maxim_reg8_write(client, 0x0C, maxim_map[1][cam_idx] << 1); /* MAX9271-CAMx I2C new */
#else
		client->addr = 0x30;					/* OV10635-CAM0 I2C */
		maxim_reg16_write(client, 0x300C, (maxim_map[2][cam_idx] << 1) | 0x1); /* OV10635 new */
#endif

		/* I2C addresses change */
		client->addr = 0x40;					/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x01, maxim_map[0][cam_idx] << 1); /* MAX9272-CAM0 I2C new */
		maxim_reg8_write(client, 0x00, maxim_map[1][cam_idx] << 1); /* MAX9271-CAM0 I2C new */

		client->addr = 0x48;					/* MAX9272-CAM0 I2C */
		maxim_reg8_write(client, 0x00, maxim_map[1][cam_idx] << 1); /* MAX9272-CAMx I2C new */
		maxim_reg8_write(client, 0x01, maxim_map[0][cam_idx] << 1); /* MAX9271-CAMx I2C new */

#ifdef MAXIM_DUMP
		client->addr = maxim_map[0][cam_idx];			/* MAX9272-CAMx I2C new */
		maxim_max927x_dump_regs(client);
		client->addr = maxim_map[1][cam_idx];			/* MAX9271-CAMx I2C new */
		maxim_max927x_dump_regs(client);
		client->addr = maxim_map[2][cam_idx];			/* OV10635-CAMx I2C */
		maxim_ov10635_dump_regs(client);
#endif

		/* make sure that the conf_link enabled - needed for reset/reboot, due to I2C runtime changeing */
		client->addr = maxim_map[1][cam_idx];			/* MAX9271-CAMx I2C new */
		maxim_reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		mdelay(5);						/* wait 5ms for conf_link to establish */

		/* Reverse channel disable */
		client->addr = maxim_map[0][cam_idx];			/* MAX9272-CAMx I2C new */
		maxim_reg8_write(client, 0x04, 0x40);			/* disable reverse control/serial_link/conf_link */
		/* Enable cascaded MAX9272 */
		maxim_reg8_write(client, 0x0e, 0x00);			/* GP0 low */
	}

	/* NOTE: I2C addr tmp_addr will be occupied as UU */
	client->addr = tmp_addr;

	return 0;
}

static int maxim_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id maxim_dt_ids[] = {
	{ .compatible = "maxim,max9272-ov10635-setup" },
	{},
};
MODULE_DEVICE_TABLE(of, maxim_dt_ids);

static const struct i2c_device_id maxim_id[] = {
	{ "maxim_setup", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, maxim_id);

static struct i2c_driver maxim_i2c_driver = {
	.driver	= {
		.name		= "maxim_setup",
		.of_match_table	= of_match_ptr(maxim_dt_ids),
	},
	.probe		= maxim_probe,
	.remove		= maxim_remove,
	.id_table	= maxim_id,
};

module_i2c_driver(maxim_i2c_driver);

MODULE_DESCRIPTION("Setup driver for 4 SoC Cameras MAX9272<->MAX9271<->OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
