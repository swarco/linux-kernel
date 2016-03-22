/*
 * drivers/char/watchdog/ccm2200_watchdog.c
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
 *     2007-02-01 gc: initial version
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <asm/bitops.h> 
#include <asm/uaccess.h>
#include <linux/init.h>
#include <asm/arch/at91-util.h>
#include <asm/arch/board-ccm2200.h>
#include <linux/ccm2200_gpio.h>

#define WDT_DEFAULT_TIME 64	/* 64 seconds */

static int ccm2200_wdt_time = WDT_DEFAULT_TIME;
static unsigned long ccm2200_wdt_busy;
static u32 ccm2200_watchdog_led = 0;

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

void hw_watchdog_reset(void)
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


/* ......................................................................... */

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int ccm2200_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, &ccm2200_wdt_busy))
		return -EBUSY;

	return 0;
}

/*
 * Close the watchdog device.
 * If CONFIG_WATCHDOG_NOWAYOUT is NOT defined then the watchdog is also
 *  disabled.
 */
static int ccm2200_wdt_close(struct inode *inode, struct file *file)
{
	ccm2200_wdt_busy = 0;
	return 0;
}

/*
 * Handle commands from user-space.
 */
static int ccm2200_wdt_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned int new_value;
	static struct watchdog_info info = {
		identity: "CCM2200 MAX6751 watchdog",
		options:  WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	};

	switch(cmd) {
	case WDIOC_KEEPALIVE:
		hw_watchdog_reset();
		return 0;

	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *)arg, 
				    &info, sizeof(info));

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, (int *)arg))
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
		return put_user(ccm2200_wdt_time, (int *)arg);

	case WDIOC_GETTIMEOUT:
		return put_user(ccm2200_wdt_time, (int *)arg);

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, (int *)arg);

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

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t ccm2200_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
        hw_watchdog_reset();
	return len;
}

/* ......................................................................... */

static struct file_operations ccm2200_wdt_fops =
{
	.owner		= THIS_MODULE,
	.ioctl		= ccm2200_wdt_ioctl,
	.open		= ccm2200_wdt_open,
	.release	= ccm2200_wdt_close,
	.write		= ccm2200_wdt_write,
};

static struct miscdevice ccm2200_wdt_miscdev =
{
	.minor		= WATCHDOG_MINOR+1,
	.name		= "ccm2200_watchdog",
	.fops		= &ccm2200_wdt_fops,
};

static int __init ccm2200_wdt_init(void)
{
	int res;

        at91_pio_set_all_pins(&watchdog_trigger);
        at91_pio_config_output_pins(&watchdog_trigger);
        at91_pio_set_all_pins(&watchdog_speed);
        at91_pio_config_output_pins(&watchdog_speed);

	res = misc_register(&ccm2200_wdt_miscdev);
	if (res)
		return res;

	printk("CCM2200 Watchdog Timer enabled (%d seconds)\n", 
               WDT_DEFAULT_TIME);
	return 0;
}

static void __exit ccm2200_wdt_exit(void)
{
	misc_deregister(&ccm2200_wdt_miscdev);
}

module_init(ccm2200_wdt_init);
module_exit(ccm2200_wdt_exit);

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Guido Classen")
MODULE_DESCRIPTION("Watchdog driver for CCM2200 board (MAX6751 Chip)")
