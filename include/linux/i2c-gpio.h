/*
 * i2c-gpio interface to platform code
 *
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_I2C_GPIO_H
#define _LINUX_I2C_GPIO_H

/**
 * struct i2c_gpio_platform_data - Platform-dependent data for i2c-gpio
 * @sda_pin: GPIO pin ID to use for SDA
 * @scl_pin: GPIO pin ID to use for SCL
 * @sda_is_open_drain: SDA is configured as open drain, i.e. the pin
 *	isn't actively driven high when setting the output value high.
 * @scl_is_open_drain: SCL is set up as open drain.
 * @scl_is_output_only: SCL output drivers cannot be turned off.
 */
struct i2c_gpio_platform_data {
	unsigned int sda_pin;
	unsigned int scl_pin;
	unsigned int sda_is_open_drain:1;
	unsigned int scl_is_open_drain:1;
	unsigned int scl_is_output_only:1;
};

#endif /* _LINUX_I2C_GPIO_H */
