/*
 * linux/drivers/leds/leds-ccm2200.c
 *
 * Copyright (C) 2007 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Userspace access to CCM2200 frontpanel indicator LEDs
 *              using Linux kernel LED framework
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
 *     2007-09-10 gc: initial version (derived from leds-at91
 *                    written by David Brownell)
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

struct ccm2200_led {
	struct led_classdev	cdev;
	unsigned		mask;
	char			name[8]; // "ledxx"
};


#define CCM2200_NUM_LEDS	16
static struct ccm2200_led ccm2200_leds[CCM2200_NUM_LEDS];



/*
 * Change the state of the LED.
 */
static void ccm2200_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	struct ccm2200_led	*led = container_of(cdev, struct ccm2200_led, cdev);


	ccm2200_set_frontpanel_leds(
		(value == LED_OFF) ? 0 : led->mask, led->mask);
}

static int __devexit ccm2200_led_remove(struct platform_device *pdev)
{
	struct ccm2200_led		*led;

	for (led = ccm2200_leds; led < ccm2200_leds+CCM2200_NUM_LEDS; ++led) {
		
		led_classdev_unregister(&led->cdev);
	}
	return 0;
}

static int __init ccm2200_led_probe(struct platform_device *pdev)
{
	int			status = 0;
	int			i;
	unsigned		nr_leds;
	struct ccm2200_led	*led = ccm2200_leds;

	for (i=0; i < CCM2200_NUM_LEDS; ++i) {
		memset(led, 0, sizeof(*led));
		led->mask	= 1 << i;
		snprintf(led->name, sizeof(led->name), "led%d", i);
		led->cdev.name = led->name;
		led->cdev.brightness_set = ccm2200_led_set,
		led->cdev.default_trigger = NULL;

		status = led_classdev_register(&pdev->dev, &led->cdev);
		if (status < 0) {
			dev_err(&pdev->dev, "led_classdev_register failed - %d\n", status);
cleanup:
			ccm2200_led_remove(pdev);
			break;
		}
		++led;
	}
	return status;
}

#ifdef CONFIG_PM
static int ccm2200_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct ccm2200_led	*led;

	for (led = ccm2200_leds; led < ccm2200_leds+CCM2200_NUM_LEDS; ++led) {
		
		led_classdev_suspend(&led->cdev);
	}

	return 0;
}

static int ccm2200_led_resume(struct platform_device *dev)
{
	struct ccm2200_led	*led;

	for (led = ccm2200_leds; led < ccm2200_leds+CCM2200_NUM_LEDS; ++led) {
		
		led_classdev_resume(&led->cdev);
	}

	return 0;
}
#else
#define	ccm2200_led_suspend	NULL
#define	ccm2200_led_resume		NULL
#endif

static struct platform_driver ccm2200_led_driver = {
	.probe		= ccm2200_led_probe,
	.remove		= __devexit_p(ccm2200_led_remove),
	.suspend	= ccm2200_led_suspend,
	.resume		= ccm2200_led_resume,
	.driver		= {
		.name	= "ccm2200_leds",
		.owner	= THIS_MODULE,
	},
};

static int __init ccm2200_led_init(void)
{
	return platform_driver_register(&ccm2200_led_driver);
}
module_init(ccm2200_led_init);

static void __exit ccm2200_led_exit(void)
{
	platform_driver_unregister(&ccm2200_led_driver);
}
module_exit(ccm2200_led_exit);

MODULE_DESCRIPTION("CCM2200 LED driver");
MODULE_AUTHOR("Guido Classen");
MODULE_LICENSE("GPL");
