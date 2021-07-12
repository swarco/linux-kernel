// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip KSZ8795 series register access through SPI
 *
 * Copyright (C) 2017 Microchip Technology Inc.
 *	Tristram Ha <Tristram.Ha@microchip.com>
 */

#include <asm/unaligned.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "ksz_common.h"

#define SPI_ADDR_SHIFT			12
#define SPI_ADDR_ALIGN			3
#define SPI_TURNAROUND_SHIFT		1

KSZ_REGMAP_TABLE(ksz8795, 16, SPI_ADDR_SHIFT,
		 SPI_TURNAROUND_SHIFT, SPI_ADDR_ALIGN);
#define SPI_CMD_READ	0x60
#define SPI_CMD_WRITE	0x40

static inline int ksz_spi_read_raw(struct ksz_device *kszd, u8 reg, u8 *data,
				   unsigned int len)
{
	struct spi_device *spi = kszd->priv;
	u8 txbuf[2];

	/* Command (RD=011, 4 dummy bits, 8 address bits, 1 turnaround bit */
	txbuf[0] = SPI_CMD_READ | (reg >> 7);
	txbuf[1] = reg << 1;

	return spi_write_then_read(spi, txbuf, 2, data, len);
}
static int ksz_spi_read8(struct ksz_device *kszd, u8 reg, u8 *val)
{
	int ret;
	
	mutex_lock(&kszd->regmap_mutex);
	ret = ksz_spi_read_raw(kszd, reg, val, 1);
	mutex_unlock(&kszd->regmap_mutex);
	
	return ret;
}

static int ksz8795_spi_probe(struct spi_device *spi)
{
	struct regmap_config rc;
	struct ksz_device *dev;
	int i, ret;
	printk(KERN_INFO "-- ksz8795_spi_probe() #1\n");

	dev = ksz_switch_alloc(&spi->dev, spi);
	printk(KERN_INFO "-- ksz8795_spi_probe() #2\n");
	if (!dev)
		return -ENOMEM;
	{
		u8 val = 0;

#define REG_CHIP_ID0			0x00

#define FAMILY_ID			0x87
		printk(KERN_INFO "-- read_spi_id() %d %02x\n", ksz_spi_read8(dev, REG_CHIP_ID0, &val), val);;
	}
		
	for (i = 0; i < ARRAY_SIZE(ksz8795_regmap_config); i++) {
		rc = ksz8795_regmap_config[i];
		rc.lock_arg = &dev->regmap_mutex;
		dev->regmap[i] = devm_regmap_init_spi(spi, &rc);
		if (IS_ERR(dev->regmap[i])) {
			ret = PTR_ERR(dev->regmap[i]);
			dev_err(&spi->dev,
				"Failed to initialize regmap%i: %d\n",
				ksz8795_regmap_config[i].val_bits, ret);
			return ret;
		}
	}
	printk(KERN_INFO "-- ksz8795_spi_probe() #3\n");
	{
		u8 val = 0;

		printk(KERN_INFO "-- read_id() %d %02x\n", ksz_read8(dev, REG_CHIP_ID0, &val), val);;
	}
		

	if (spi->dev.platform_data)
		dev->pdata = spi->dev.platform_data;

	ret = ksz8795_switch_register(dev);
	printk(KERN_INFO "-- ksz8795_spi_probe() #4 ret=%d\n", ret);
	{
		u8 val = 0;

		printk(KERN_INFO "-- read_spi_id#2() %d %02x\n", ksz_spi_read8(dev, REG_CHIP_ID0, &val), val);;
	}
	{
		u8 val = 0;

		printk(KERN_INFO "-- read_id()#2 %d %02x\n", ksz_read8(dev, REG_CHIP_ID0, &val), val);;
	}

	/* Main DSA driver may not be started yet. */
	if (ret)
		return ret;
	printk(KERN_INFO "-- ksz8795_spi_probe() #5\n");

	spi_set_drvdata(spi, dev);

	return 0;
}

static int ksz8795_spi_remove(struct spi_device *spi)
{
	struct ksz_device *dev = spi_get_drvdata(spi);

	if (dev)
		ksz_switch_remove(dev);

	return 0;
}

static void ksz8795_spi_shutdown(struct spi_device *spi)
{
	struct ksz_device *dev = spi_get_drvdata(spi);

	if (dev && dev->dev_ops->shutdown)
		dev->dev_ops->shutdown(dev);
}

static const struct of_device_id ksz8795_dt_ids[] = {
	{ .compatible = "microchip,ksz8765" },
	{ .compatible = "microchip,ksz8794" },
	{ .compatible = "microchip,ksz8795" },
	{},
};
MODULE_DEVICE_TABLE(of, ksz8795_dt_ids);

static struct spi_driver ksz8795_spi_driver = {
	.driver = {
		.name	= "ksz8795-switch",
		.bus = &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ksz8795_dt_ids),
	},
	.probe	= ksz8795_spi_probe,
	.remove	= ksz8795_spi_remove,
	.shutdown = ksz8795_spi_shutdown,
};

module_spi_driver(ksz8795_spi_driver);

MODULE_AUTHOR("Tristram Ha <Tristram.Ha@microchip.com>");
MODULE_DESCRIPTION("Microchip KSZ8795 Series Switch SPI Driver");
MODULE_LICENSE("GPL");
