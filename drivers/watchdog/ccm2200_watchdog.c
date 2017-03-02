/*
 *	SWARCO CCM2200 Single Board Computer Watchdog Timer driver
 *
 *	Based on wdt.c. Original copyright messages:
 *
 *	(c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>,
 *						All Rights Reserved.
 *
 * Copyright (C) 2007 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Trigger CCM2200 board specific watchdog chip MAX6751.
 *              This watchdog is started direct after rest and has a timeout
 *              period of 64 seconds. The application can reduce the watchdog
 *              timeout period to 0.5 second.
 *
 *              The AT91RM9200 internal watchdog can be used together
 *              with this driver to get shorter watchdog periods using
 *              the at91_wdt.c driver.
 * @references:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  @par Modification History:
 *     2011-01-13 gc: adopted to platform_driver framework from linux-2.6.37
 *     2007-02-01 gc: initial version
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>

#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/at91-util.h>
#include <mach/board-ccm2200.h>
#include <linux/ccm2200_gpio.h>

/* Module information */
#define DRV_NAME "ccm2200_wdt"
#define PFX DRV_NAME ": "
#define WATCHDOG_NAME "CCM2200 WDT"

/* internal variables */
/* the watchdog platform device */
static struct platform_device *ccm2200_wdt_platform_device;
static unsigned long ccm2200_wdt_is_open;
#define WDT_DEFAULT_TIME 64	/* 64 seconds */
static int ccm2200_wdt_time = WDT_DEFAULT_TIME;
static u32 ccm2200_watchdog_led = 0;


/*
 *	Watchdog Operations
 */

/*************************************************************************
 *  hw_watchdog_reset
 *
 *	This routine is called to reset (keep alive) the watchdog timer
 *
 ************************************************************************/

static struct at91_pio_pins watchdog_trigger = {
	AT91_PIO_BASE(AT91_PIOA), 1<<21
};

static struct at91_pio_pins watchdog_speed = {
	AT91_PIO_BASE(AT91_PIOA), 1<<29
};

static void ccm2200_wdt_keepalive(void)
{

	static unsigned long last_reset = 0;
	static unsigned led = 0;

	/* do reset only once all 500msec */
	if ((unsigned long) (jiffies - last_reset)
	    > (unsigned long) (HZ / 2)) {

		/* assert reset signal to external watchdog */
		at91_pio_clear_all_pins(&watchdog_trigger);
		//udelay(1);
		at91_pio_set_all_pins(&watchdog_trigger);
		/* blink with LED to indicate the running watchdog */
		led ^= ccm2200_watchdog_led;
		ccm2200_set_frontpanel_leds(led, ccm2200_watchdog_led);
		last_reset = jiffies;
	}
}

/*
 *	/dev/watchdog handling
 */

static ssize_t ccm2200_wdt_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	if (count) {
		ccm2200_wdt_keepalive();
	}
	return count;
}

static long ccm2200_wdt_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	unsigned int new_value;

	static const struct watchdog_info info = {
		identity: "CCM2200 MAX6751 watchdog",
		options:  WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	};

	switch (cmd) {
	case WDIOC_KEEPALIVE:
		ccm2200_wdt_keepalive();
		return 0;

	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &info, sizeof(info));

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		if ((new_value <= 0) || (new_value > WDT_DEFAULT_TIME))
			return -EINVAL;

		if (new_value <= 1) {
			at91_pio_clear_all_pins(&watchdog_speed);
			ccm2200_wdt_time = 0;
		} else {
			at91_pio_set_all_pins(&watchdog_speed);
			ccm2200_wdt_time = WDT_DEFAULT_TIME;
		}
		/* Return current value */
		return put_user(ccm2200_wdt_time, p);

	case WDIOC_GETTIMEOUT:
		return put_user(ccm2200_wdt_time, p);

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case CCM2200_WDIOC_SETLED:
		if (get_user(new_value, (int *)arg))
			return -EFAULT;
		ccm2200_watchdog_led = new_value;
		return 0;
		/* case WDIOC_SETOPTIONS: */

	default:
		return -ENOIOCTLCMD;
	}
}

static int ccm2200_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &ccm2200_wdt_is_open))
		return -EBUSY;

	/* Activate */
	ccm2200_wdt_keepalive();

	return nonseekable_open(inode, file);
}

static int ccm2200_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &ccm2200_wdt_is_open);
	return 0;
}

/*
 *	Kernel Interfaces
 */

static const struct file_operations ccm2200_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= ccm2200_wdt_write,
	.unlocked_ioctl	= ccm2200_wdt_ioctl,
	.open		= ccm2200_wdt_open,
	.release	= ccm2200_wdt_close,
};

static struct miscdevice ccm2200_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR+1,
	.name	= "ccm2200_watchdog",
	.fops	= &ccm2200_wdt_fops,
};

/*
 *	Init & exit routines
 */

static int __devinit ccm2200_wdt_probe(struct platform_device *dev)
{
	int ret;

	ret = misc_register(&ccm2200_wdt_miscdev);
	if (ret != 0) {
		printk(KERN_ERR PFX
			"cannot register miscdev on minor=%d (err=%d)\n",
							WATCHDOG_MINOR, ret);
		return 0;
	}
	return ret;
}

static int __devexit ccm2200_wdt_remove(struct platform_device *dev)
{
	misc_deregister(&ccm2200_wdt_miscdev);
	return 0;
}

static void ccm2200_wdt_shutdown(struct platform_device *dev)
{
	/* we can not turn of our watchdog after it is once started... */;
}

static struct platform_driver ccm2200_wdt_driver = {
	.probe		= ccm2200_wdt_probe,
	.remove		= __devexit_p(ccm2200_wdt_remove),
	.shutdown	= ccm2200_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRV_NAME,
	},
};

static int __init ccm2200_wdt_init(void)
{
	int err;

	err = platform_driver_register(&ccm2200_wdt_driver);
	if (err)
		return err;

	ccm2200_wdt_platform_device = platform_device_register_simple(DRV_NAME,
								-1, NULL, 0);
	if (IS_ERR(ccm2200_wdt_platform_device)) {
		err = PTR_ERR(ccm2200_wdt_platform_device);
		goto unreg_platform_driver;
	}

	at91_pio_set_all_pins(&watchdog_trigger);
	at91_pio_config_output_pins(&watchdog_trigger);
	at91_pio_set_all_pins(&watchdog_speed);
	at91_pio_config_output_pins(&watchdog_speed);
	printk(KERN_INFO "CCM2200 Watchdog Timer enabled (%d seconds)\n",
	       WDT_DEFAULT_TIME);

	return 0;

unreg_platform_driver:
	platform_driver_unregister(&ccm2200_wdt_driver);
	return err;
}

static void __exit ccm2200_wdt_exit(void)
{
	platform_device_unregister(ccm2200_wdt_platform_device);
	platform_driver_unregister(&ccm2200_wdt_driver);
	printk(KERN_INFO PFX "Watchdog Module Unloaded.\n");
}

module_init(ccm2200_wdt_init);
module_exit(ccm2200_wdt_exit);

MODULE_AUTHOR("Guido Classen");
MODULE_DESCRIPTION("Driver for CCM2200 board Watchdog (MAX6751 Chip)");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_LICENSE("GPL");
