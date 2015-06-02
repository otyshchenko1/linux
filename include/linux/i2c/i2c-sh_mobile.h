#ifndef __I2C_SH_MOBILE_H__
#define __I2C_SH_MOBILE_H__

#include <linux/platform_device.h>

enum i2c_sh_mobile_clk_calc_type {
	I2C_SHMOBILE_CLK_CALC_TYPE_LEGACY,
	I2C_SHMOBILE_CLK_CALC_TYPE_GEN2,
};

struct i2c_sh_mobile_platform_data {
	unsigned long bus_speed;
	unsigned int clks_per_count;
	enum i2c_sh_mobile_clk_calc_type clk_type;
};

#endif /* __I2C_SH_MOBILE_H__ */
