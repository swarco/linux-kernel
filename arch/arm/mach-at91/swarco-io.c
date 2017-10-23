/*
 * Digital inputs, digital output, analog inputs for Swarco SCC-3G.
 * 
 * Copyright (C) 2009-12 Micro Technic A/S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/at91_tc.h>
#include <asm/mach-types.h>


static struct class *swarcoio_class;
static struct input_dev *swarcoio_input_dev;


#define GPI	0	/* Active high GPIO input */
#define GPINOT	1	/* Active low GPIO input */
#define GPO	2	/* Active high GPIO output */
#define GPONOT	3	/* Active low GPIO output */
#define PWM	4	/* The PWM output */
#define AI1100	5	/* The analog input on ADS1100 */
#define AI1112	6	/* An analog input on ADS1112 */

struct swarcoio_drvdata {
	struct device *dev;
	int type;
	const char *name;
	int pin;
	int eventcode;
};


#define PIN_MPPSET_PWM		AT91_PIN_PA26

static void __iomem *at91tcb0_base;


static inline void write_tcb0(u32 regoffset, u32 value)
{
	__raw_writel(value, at91tcb0_base + regoffset);
}


static inline u32 read_tcb0(u32 regoffset)
{
	return __raw_readl(at91tcb0_base + regoffset);
}


/* Bits in ADS1100 configuration/status register */
#define ADS1100_ST_BSY		0x80
#define ADS1100_CONV_CONTINUOUS	0x00
#define ADS1100_CONV_SINGLE	0x10
#define ADS1100_PGA_1		0x00
#define ADS1100_PGA_2		0x01
#define ADS1100_PGA_4		0x02
#define ADS1100_PGA_8		0x03
#define ADS1100_RATE_128SPS	0x00
#define ADS1100_RATE_32SPS	0x04
#define ADS1100_RATE_16SPS	0x08
#define ADS1100_RATE_8SPS	0x0C

#define ADS1100_CONFIG	(ADS1100_ST_BSY | ADS1100_CONV_SINGLE | ADS1100_PGA_8 | ADS1100_RATE_8SPS)

static struct i2c_client *ads1100_client;
static DEFINE_MUTEX(ads1100_lock);


/* Bits in ADS1112 configuration/status register */
#define ADS1112_START		0x80
#define ADS1112_BUSY		0x80
#define ADS1112_INP_AIN0_AIN1	0x00
#define ADS1112_INP_AIN2_AIN3	0x20
#define ADS1112_INP_AIN0_AIN3	0x40
#define ADS1112_INP_AIN1_AIN3	0x60
#define ADS1112_CONTINUOUS	0x00
#define ADS1112_SINGLE		0x10
#define ADS1112_SPEED_240SPS	0x00
#define ADS1112_SPEED_60SPS	0x04
#define ADS1112_SPEED_30SPS	0x08
#define ADS1112_SPEED_15SPS	0x0C
#define ADS1112_PGA_1		0x00
#define ADS1112_PGA_2		0x01
#define ADS1112_PGA_4		0x02
#define ADS1112_PGA_8		0x03

#define ADS1112_CFG_BATVOLT	(ADS1112_START | ADS1112_INP_AIN0_AIN3 | ADS1112_SINGLE | ADS1112_SPEED_240SPS | ADS1112_PGA_1)
#define ADS1112_CFG_MAINSVOLT	(ADS1112_START | ADS1112_INP_AIN1_AIN3 | ADS1112_SINGLE | ADS1112_SPEED_240SPS | ADS1112_PGA_1)
#define ADS1112_CFG_MAINSCURR	(ADS1112_START | ADS1112_INP_AIN2_AIN3 | ADS1112_SINGLE | ADS1112_SPEED_15SPS  | ADS1112_PGA_8)

static struct i2c_client *ads1112_client;
static DEFINE_MUTEX(ads1112_lock);


/* --- Platform driver --- */

static struct swarcoio_drvdata ios[] = {
	{ .type = GPONOT, .name = "rs485_fullduplex1", .pin = AT91_PIN_PB20 },
	{ .type = GPONOT, .name = "rs485_fullduplex2", .pin = AT91_PIN_PB29 },
	{ .type = GPONOT, .name = "bluetooth_hwreset", .pin = AT91_PIN_PB18 },
	{ .type = GPONOT, .name = "bluetooth_freset",  .pin = AT91_PIN_PB19 },
	{ .type = GPINOT, .name = "modem_pwrmon",      .pin = AT91_PIN_PB2  },
	{ .type = GPO,    .name = "aux_power",         .pin = AT91_PIN_PC2  },
	{ .type = GPO,    .name = "aux_power_12v",     .pin = AT91_PIN_PC3  },
	{ .type = GPO,    .name = "modem_reset",       .pin = AT91_PIN_PC4  },
	{ .type = GPO,    .name = "modem_onoff",       .pin = AT91_PIN_PC5  },
	{ .type = GPO,    .name = "modem_vbus",        .pin = AT91_PIN_PC6  },
	{ .type = GPINOT, .name = "powerfreq",         .pin = AT91_PIN_PC12 },
	{ .type = GPINOT, .name = "modem_2g",          .pin = AT91_PIN_PC19 },
	{ .type = GPINOT, .name = "modem_3g",          .pin = AT91_PIN_PC18 },
	{ .type = GPO,    .name = "5v_off",            .pin = AT91_PIN_PB17 },
	{ .type = GPINOT, .name = "onoff_button",      .pin = AT91_PIN_PA29, .eventcode = KEY_POWER },
	{ .type = GPINOT, .name = "chg_stat1",         .pin = AT91_PIN_PA27 },
	{ .type = GPINOT, .name = "chg_stat2",         .pin = AT91_PIN_PA22 },	
	{ .type = PWM,    .name = "chg_mppset",                             },
	{ .type = AI1100, .name = "batcurr",                                },
	{ .type = AI1112, .name = "batvolt",           .pin = 0             },
	{ .type = AI1112, .name = "mainsvolt",         .pin = 1             },
	{ .type = AI1112, .name = "mainscurr",         .pin = 2             },
};


static int conv_adc(struct i2c_client *client, int cfg, int msecs, int *raw)
{
	char buf[2];
	int rc;
	
	/* Start the conversion */
	buf[0] = cfg;
	rc = i2c_master_send(client, buf, 1);
	if (rc < 0) {
		printk(KERN_ERR "conv_adc (0x%02x): i2c_master_send error %d\n", cfg, rc);
		return rc;
	}
	
	/* Wait for the conversion to complete */
	msleep(msecs);
	
	/* Read the result */
	rc = i2c_master_recv(client, buf, 2);
	if (rc < 0) {
		printk(KERN_ERR "conv_adc (0x%02x): i2c_master_recv error %d\n", cfg, rc);
		return rc;
	}
	
	*raw = ((signed char)buf[0] << 8) | buf[1];
	return 0;
}


static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct swarcoio_drvdata *drvdata = dev_get_drvdata(dev);
	int raw;
	int rc;
	
	switch (drvdata->type)
	{
	case GPI:
	case GPO:
		return sprintf(buf, "%d\n", at91_get_gpio_value(drvdata->pin));
		
	case GPINOT:
	case GPONOT:
		return sprintf(buf, "%d\n", !at91_get_gpio_value(drvdata->pin));
		
	case PWM:
		return sprintf(buf, "%d\n", 32768 - read_tcb0(AT91_TC_RA));

	case AI1100:
		/* Nominal speed 8 SPS, actual speed 6.5-11.5 SPS, duration
		 * 87.0-153.8 msec.  Wait 154 msec for the result */
		mutex_lock(&ads1100_lock);
		rc = conv_adc(ads1100_client, ADS1100_CONFIG, 154, &raw);
		mutex_unlock(&ads1100_lock);
		if (rc)
			return rc;
		return sprintf(buf, "%d\n", raw);
		
	case AI1112:
		mutex_lock(&ads1112_lock);
		switch (drvdata->pin)
		{
		case 0: /* batvolt: AIN0 - AIN3.
			 * Nominal speed 240 SPS, actual speed 180-308 SPS,
			 * actual duration 3.25-5.56 msec */
			rc = conv_adc(ads1112_client, ADS1112_CFG_BATVOLT, 6, &raw);
			break;
			
		case 1: /* mainsvolt: AIN1 - AIN3.
			 * Nominal speed 240 SPS, actual speed 180-308 SPS,
			 * actual duration 3.25-5.56 msec */
			rc = conv_adc(ads1112_client, ADS1112_CFG_MAINSVOLT, 6, &raw);
			break;
			
		case 2: /* mainscurr: AIN2 - AIN3.
			 * Nominal speed 15 SPS, actual speed 11-20 SPS,
			 * actual duration 50-91 msec */
			rc = conv_adc(ads1112_client, ADS1112_CFG_MAINSCURR, 91, &raw);
			break;
			
		default:
			rc = -EINVAL;
			break;
		}
		mutex_unlock(&ads1112_lock);
		if (rc)
			return rc;
		return sprintf(buf, "%d\n", raw);
		
	}
	return -EINVAL;
}


static ssize_t value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct swarcoio_drvdata *drvdata = dev_get_drvdata(dev);
	int value = simple_strtol(buf, NULL, 10);
	
	switch (drvdata->type)
	{
	case GPO:
		at91_set_gpio_value(drvdata->pin, !!value);
		break;
		
	case GPONOT:
		at91_set_gpio_value(drvdata->pin, !value);
		break;
		
	case PWM:	
		if (value > 32768)
			return -EINVAL;
		write_tcb0(AT91_TC_RA, 32768 - value);
		break;
		
	case AI1100:
	case AI1112:
		/* Never happens, they are read only */
		return -EACCES;
	}
	return count;
}


static struct device_attribute dev_attr_value_ro = __ATTR(value, 0444, value_show, NULL);
static struct device_attribute dev_attr_value_rw = __ATTR(value, 0644, value_show, value_store);


static irqreturn_t swarcoio_di_isr(int irq, void *dev_id)
{
	struct swarcoio_drvdata *drvdata = (struct swarcoio_drvdata *)dev_id;
	
	input_report_key(swarcoio_input_dev, drvdata->eventcode, 
		at91_get_gpio_value(drvdata->pin) ^ (drvdata->type == GPINOT));
	input_sync(swarcoio_input_dev);
	return IRQ_HANDLED;
}


static int __devinit swarcoio_platform_probe(struct platform_device *pdev)
{
	struct device *parent = &pdev->dev;
	int i;
	struct clk *at91tc0_clk;
	
	/* Create /dev/event0 input device used for certain digital inputs */
	swarcoio_input_dev = input_allocate_device();
	if (!swarcoio_input_dev) {
		printk(KERN_ERR "swarcoio_platform_probe: Cannot allocate input device\n");
		return PTR_ERR(swarcoio_input_dev);
	}
	swarcoio_input_dev->name = pdev->name;
	swarcoio_input_dev->phys = "swarcoio/input0";
	swarcoio_input_dev->id.bustype = BUS_HOST;
	swarcoio_input_dev->id.vendor = 0x0001;
	swarcoio_input_dev->id.product = 0x0001;
	swarcoio_input_dev->id.version = 0x0100;
	swarcoio_input_dev->dev.parent = &pdev->dev;
		
	/* Create the "/sys/class/swarcoio" class if one of the other probe
	 * functions haven't done it already */
	if (!swarcoio_class) {
		swarcoio_class = class_create(THIS_MODULE, "swarcoio");
		if (IS_ERR(swarcoio_class)) {
			printk(KERN_ERR "swarcoio_platform_probe: Cannot create class /sys/class/swarcoio\n");
			return PTR_ERR(swarcoio_class);
		}
	}
		
	/* PWM output for MPPSET voltage setpoint */
	at91tcb0_base = ioremap(AT91SAM9260_BASE_TCB0, 256);
	if (at91tcb0_base == NULL)
	{
		printk(KERN_ERR "swarcoio_probe: ioremap(tcb0) failed\n");
		return -EIO;
	}
	at91tc0_clk = clk_get(NULL, "tc0_clk");
	if (at91tc0_clk == NULL)
	{
		printk(KERN_ERR "swarcoio_probe: clk_get(tc0_clk) failed\n");
		return -EIO;
	}
	clk_enable(at91tc0_clk);
	
	/* TCB0 counts 0-32767 at 66.624 MHz, giving a PWM frequency = 2034 Hz.
	 * RA defines the pulsewidth.  At RC match TIOA0 is set so that the
	 * output goes high at TC_CV=0.  At RA match TIOA0 is cleared so that
	 * the output goes low at TC_CV = RA+1.  The output is high for RA+1
	 * clocks out of 32768. */
	write_tcb0(AT91_TC_RA, 16384);	/* Initial PWM value: Upwm=1.65V => */
	write_tcb0(AT91_TC_RC, 32767);	/* PWM period: Count 0-32767 = 2034 Hz */
	write_tcb0(AT91_TC_CMR,
		AT91_TC_TIMER_CLOCK1	|	/* MCK/2 = 66.6624 MHz */
		AT91_TC_WAVESEL_UP_AUTO	|	/* Count up from 0 to RC */
		AT91_TC_WAVE		|	/* Waveform, not capture mode */
		AT91_TC_ACPA_CLEAR	|	/* Clear TIOA0 output at RC match */
		AT91_TC_ACPC_SET);		/* Set TIOA0 output at RC match */
	write_tcb0(AT91_TC_CCR, AT91_TC_CLKEN);	/* Start TCB0 */
	write_tcb0(AT91_TC_CCR, AT91_TC_SWTRG);	/* Trig the timer */
	at91_set_A_periph(PIN_MPPSET_PWM, 0);	/* PA26 = TIOA0, not gpio */
	
	/* Create our /sys/class/swarcoio/ entries */
	for (i=0; i < ARRAY_SIZE(ios); i++)
	{
		struct swarcoio_drvdata *drvdata = &ios[i];
		int active_low = 0;
		
		switch (drvdata->type)
		{
		case GPINOT:
			active_low = 1;
			/* fall through */
		case GPI:
			at91_set_gpio_input(drvdata->pin, 0);
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_ro);
			if (drvdata->eventcode) {
				/* Register this event type, and set initial state of the "key" */
				input_set_capability(swarcoio_input_dev, EV_KEY, drvdata->eventcode);
				input_report_key(swarcoio_input_dev,
					drvdata->eventcode,
					at91_get_gpio_value(drvdata->pin) ^ active_low);
				request_irq(gpio_to_irq(drvdata->pin), swarcoio_di_isr,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"swarcoio", drvdata);
			}
			break;
		
		case GPO:
			at91_set_gpio_output(drvdata->pin, 0);
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_rw);
			break;
		
		case GPONOT:
			at91_set_gpio_output(drvdata->pin, 1);
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_rw);
			break;
			
		case PWM:
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_rw);
			break;
		}
	}
	
	/* All possible input event codes have been registered */
	input_register_device(swarcoio_input_dev);

	return 0;
}


static int __devexit swarcoio_platform_remove(struct platform_device *pdev)
{
	class_destroy(swarcoio_class);
	return 0;
}


#ifdef CONFIG_PM

static int swarcoio_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* @@@ Not implemented yet */
	return 0;
}

static int swarcoio_platform_resume(struct platform_device *pdev)
{
	/* @@@ Not implemented yet */
	return 0;
}

#else
#define swarcoio_platform_suspend	NULL
#define swarcoio_platform_resume	NULL
#endif


struct platform_driver swarcoio_platform_driver = {
	.probe		= swarcoio_platform_probe,
	.remove		= __devexit_p(swarcoio_platform_remove),
	.suspend	= swarcoio_platform_suspend,
	.resume		= swarcoio_platform_resume,
	.driver		= {
		.name	= "swarcoio-platform",
	},
};


/* --- ADS1100 I2C --- */



static int __devinit swarcoio_ads1100_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct device *parent = &client->dev;
	char buf[3];
	int rc;
	int i;
	
	printk(KERN_INFO "swarcoio_ads1100_probe\n");
	
	/* Reset default: Continuous conversion, speed=8SPS, PGA=1.
	 * We want: Single conversion, speed=8SPS, PGA=8.
	 * Start one conversion.  When the current continuous conversion is
	 * complete, the ADC should power down. */
	ads1100_client = client;
	buf[0] = ADS1100_CONFIG;
	rc = i2c_master_send(client, buf, 1);
	if (rc < 0) {
		printk(KERN_ERR "swarcoio_ads1100_probe: Error %d sending config byte\n", rc);
		return rc;
	}
	
	/* Read output and config register */
	buf[2] = 0x00;
	rc = i2c_master_recv(client, buf, 3);
	if (rc < 0) {
		printk(KERN_ERR "swarcoio_ads1100_probe: Error %d reading back config byte\n", rc);
		return rc;
	}
	
	/* Verify the config register read back, except bit 7 (ST/BSY) which
	 * may clear at any time */
	if ((buf[2] & 0x7F) != (ADS1100_CONFIG & 0x7F)) {
		printk(KERN_ERR "swarcoio_ads1100_probe: config readback error: Got 0x%02X, expected 0x%02X\n",
			buf[2] & 0x7F, ADS1100_CONFIG & 0x7F);
		return -EIO;
	}
	/* ADS1100 appears to be OK. */

	/* Create the "/sys/class/swarcoio" class if one of the other probe
	 * functions haven't done it already */
	if (!swarcoio_class) {
		swarcoio_class = class_create(THIS_MODULE, "swarcoio");
		if (IS_ERR(swarcoio_class)) {
			printk(KERN_ERR "swarcoio_platform_probe: Cannot create class /sys/class/swarcoio\n");
			return PTR_ERR(swarcoio_class);
		}
	}
	
	/* Create our /sys/class/swarcoio/ entry */
	for (i=0; i < ARRAY_SIZE(ios); i++) {
		struct swarcoio_drvdata *drvdata = &ios[i];
		
		switch (drvdata->type)
		{
		case AI1100:
			/* No per-input hardware configuration necessary */
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_ro);
			break;
		}
	}
	
	return 0;
}


static const struct i2c_device_id swarcoio_ads1100_id[] = {
	{ "swarcoio-ads1100", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, swarcoio_ads1100_id);

static struct i2c_driver swarcoio_ads1100_driver = {
	.probe		= swarcoio_ads1100_probe,
	.id_table	= swarcoio_ads1100_id,
	.driver = {
		.name	= "swarcoio-ads1100",
		.owner	= THIS_MODULE,
	},
};


/* --- ADS1112 I2C --- */

static int __devinit swarcoio_ads1112_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct device *parent = &client->dev;
	char buf[3];
	int i;
	int rc;
	
	printk(KERN_INFO "swarcoio_ads1112_probe\n");
	ads1112_client = client;
	
	/* ADS1112 config register reset default: Busy, AIN0-AIN1, continuous,
	 * 15SPS, PGA x1.  Do a test where we start a batvolt conversion, wait
	 * for it to complete, and read back the config register.  We expect
	 * the busy bit to be clear.  The conversion takes at most 5556 usec. */
	buf[0] = ADS1112_CFG_BATVOLT;
	rc = i2c_master_send(client, buf, 1);
	if (rc < 0) {
		printk(KERN_ERR "swarcoio_ads1112_probe: i2c_master_send error %d\n", rc);
		return rc;
	}
	msleep(6);
	buf[2] = 0x00;
	rc = i2c_master_recv(client, buf, 3);
	if (rc < 0) {
		printk(KERN_ERR "swarcoio_ads1112_probe: i2c_master_recv error %d\n", rc);
		return rc;
	}
	if ((buf[2] & 0x7F) != (ADS1112_CFG_BATVOLT & 0x7F)) {
		printk(KERN_ERR "swarcoio_ads1112_probe: Verify error. Expected 0x%02x, got 0x%02x\n",
			ADS1112_CFG_BATVOLT & 0x7F, buf[2] & 0x7F);
		return -EIO;
	}
	/* The ADS1112 appears to be OK. */
	
	/* Create the "/sys/class/swarcoio" class if one of the other probe
	 * functions haven't done it already */
	if (!swarcoio_class) {
		swarcoio_class = class_create(THIS_MODULE, "swarcoio");
		if (IS_ERR(swarcoio_class)) {
			printk(KERN_ERR "swarcoio_platform_probe: Cannot create class /sys/class/swarcoio\n");
			return PTR_ERR(swarcoio_class);
		}
	}	

	/* Create our /sys/class/swarcoio/ entries */
	for (i=0; i < ARRAY_SIZE(ios); i++) {
		struct swarcoio_drvdata *drvdata = &ios[i];
		
		switch (drvdata->type)
		{
		case AI1112:
			/* No per-input hardware configuration necessary */
			drvdata->dev = device_create(swarcoio_class, parent, 0, drvdata, "%s", drvdata->name);
			device_create_file(drvdata->dev, &dev_attr_value_ro);
			break;
		}
	}
	
	return 0;
}


static const struct i2c_device_id swarcoio_ads1112_id[] = {
	{ "swarcoio-ads1112", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, swarcoio_ads1112_id);

static struct i2c_driver swarcoio_ads1112_driver = {
	.probe		= swarcoio_ads1112_probe,
	.id_table	= swarcoio_ads1112_id,
	.driver = {
		.name	= "swarcoio-ads1112",
		.owner	= THIS_MODULE,
	},
};


/* --- */

static int __init swarcoio_init(void)
{
	int rc;
	
	/* Platform driver for GPIOs etc. */
	rc = platform_driver_register(&swarcoio_platform_driver);
	if (rc) {
		printk(KERN_ERR "swarcoio_init: Error %d registering platform driver\n", rc);
		return rc;
	}
	
	/* I2C driver for ADS1100 */
	rc = i2c_add_driver(&swarcoio_ads1100_driver);
	if (rc) {
		printk(KERN_ERR "swarcoio_init: Error %d adding ads1100 driver.\n", rc);
		return -ENODEV;
	}
	
	/* I2C driver for ADS1112 */
	rc = i2c_add_driver(&swarcoio_ads1112_driver);
	if (rc) {
		printk(KERN_ERR "swarcoio_init: Error %d adding ads1112 driver.\n", rc);
		return -ENODEV;
	}
	
	return 0;
}

static void __exit swarcoio_exit(void)
{
	platform_driver_unregister(&swarcoio_platform_driver);
	i2c_del_driver(&swarcoio_ads1100_driver);
	i2c_del_driver(&swarcoio_ads1112_driver);
}


module_init(swarcoio_init);
module_exit(swarcoio_exit);

MODULE_AUTHOR("Karl Olsen");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Swarco I/O class interface");
