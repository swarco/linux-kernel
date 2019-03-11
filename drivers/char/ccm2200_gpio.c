/*
 * linux/drivers/char/ccm2200_gpio.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Userspace access to CCM2200 digital in-/output lines
 *              and indicator LEDs
 * @todo:       sophisticated, interrupt driven IO support
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
 *     2006-05-04 gc: initial version (partly derived from ite_gpio.c
 *                    written by Hai-Pao Fan <haipao@mvista.com>)
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>

#include <linux/ccm2200_gpio.h>

#include <asm/io.h>       		/* ioremap */
#include <asm/uaccess.h>		/* copy_to_user */
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/at91-util.h>
#include <mach/at91rm9200_smc.h>
#include <mach/board-ccm2200.h>
#include <mach/at91_pio.h>
#include <mach/at91rm9200_mc.h>
#include <mach/at91_pmc.h>


//#define debug(format, ...) printk(format, ## __VA_ARGS__)
#define debug(format, ...) 

#define CCM2200_PIOB_IN0_7_MASK         0x00000ff0
#define CCM2200_PIOB_IN0_7_SHIFT        4

#define CCM2200_PIOD_IN8_9_MASK         (1<<23 | 1<<24)
#define CCM2200_PIOD_IN8_9_SHIFT        (23-8)

#define CCM2200_PIOD_IN10_11_MASK       (1<<26 | 1<<27)
#define CCM2200_PIOD_IN10_11_SHIFT      (26-10)

#define CCM2200_PIOD_LED_MASK           0x0000ffff
#define CCM2200_PIOD_LED_SHIFT          0


#define CCM2200_BUS_OUT0_7_MASK         0x000000ff
#define CCM2200_PIOD_OUT8_11_MASK       0x000f0000
#define CCM2200_PIOD_OUT8_11_SHIFT      (16-8)


/* SCONF0: PIOA27 */
/* SCONF1: PIOA28 */
#define CCM2200_PIOA_SCONF0_1_MASK      (1<<27 | 1<<28)
#define CCM2200_PIOA_SCONF0_1_SHIFT     27

/* preliminary for testing */
/* SCONF2: PIOC5 */
#define CCM2200_PIOC_SCONF2_MASK        (1<<5)
#define CCM2200_PIOC_SCONF2_SHIFT       (5-2)

/* preliminary for testing */
/* SCONF3: PB28 */
#define CCM2200_PIOB_SCONF3_MASK        (1<<28)
#define CCM2200_PIOB_SCONF3_SHIFT       (28-3)


static volatile u8 *ccm2200_digital_out_virt = NULL;
static u32 current_output = 0xf00;
static u32 current_leds = 0x000;


/* 2012-11-05 gc: todo: use new kernel infrastructure for this */
#define at91_sys_write(addr, val) __raw_writel((val), (addr))
#define at91_sys_read(addr)  __raw_readl((addr))

__u32 ccm2200_get_digital_input(void)
{
        
        return ( (at91_sys_read(AT91C_VA_BASE_PIOB + PIO_PDSR)&CCM2200_PIOB_IN0_7_MASK) 
                 >> CCM2200_PIOB_IN0_7_SHIFT )
                | ( (at91_sys_read(AT91C_VA_BASE_PIOD + PIO_PDSR) & CCM2200_PIOD_IN8_9_MASK) 
                   >> CCM2200_PIOD_IN8_9_SHIFT )
                | ( (at91_sys_read(AT91C_VA_BASE_PIOD + PIO_PDSR) & CCM2200_PIOD_IN10_11_MASK) 
                   >> CCM2200_PIOD_IN10_11_SHIFT );
}

__u32 ccm2200_get_sconf_input(void)
{
        return ( (at91_sys_read(AT91C_VA_BASE_PIOA + PIO_PDSR) & CCM2200_PIOA_SCONF0_1_MASK) 
                 >> CCM2200_PIOA_SCONF0_1_SHIFT )
                | ( (at91_sys_read(AT91C_VA_BASE_PIOC + PIO_PDSR) & CCM2200_PIOC_SCONF2_MASK) 
                   >> CCM2200_PIOC_SCONF2_SHIFT )
                | ( (at91_sys_read(AT91C_VA_BASE_PIOB + PIO_PDSR) & CCM2200_PIOB_SCONF3_MASK) 
                   >> CCM2200_PIOB_SCONF3_SHIFT );
}

void ccm2200_set_digital_output(register __u32 data, register __u32 mask)
{
        current_output = (current_output & ~mask) | (data & mask);

        /* write lower 8 bits over external bus in latch */
        *ccm2200_digital_out_virt = current_output & CCM2200_BUS_OUT0_7_MASK;
        /* write upper bit 8...11 to PD16...19 over AT91RM9200 GPIO */
        {
                register __u32 value = 
                        current_output << CCM2200_PIOD_OUT8_11_SHIFT;
                at91_sys_write(AT91C_VA_BASE_PIOD + PIO_SODR, value & CCM2200_PIOD_OUT8_11_MASK);
                at91_sys_write(AT91C_VA_BASE_PIOD + PIO_CODR, ~value & CCM2200_PIOD_OUT8_11_MASK);
/*                 AT91_SYS->PIOD_SODR = value & CCM2200_PIOD_OUT8_11_MASK; */
/*                 AT91_SYS->PIOD_CODR = ~value & CCM2200_PIOD_OUT8_11_MASK; */
        }
}

void ccm2200_set_frontpanel_leds(register __u32 data, register __u32 mask)
{
        current_leds = (current_leds & ~mask) | (data & mask);
        /* invert LED port here! */
        at91_sys_write(AT91C_VA_BASE_PIOD + PIO_SODR, ~current_leds & CCM2200_PIOD_LED_MASK);
        at91_sys_write(AT91C_VA_BASE_PIOD + PIO_CODR, current_leds  & CCM2200_PIOD_LED_MASK);
/*         AT91_SYS->PIOD_SODR = ~current_leds & CCM2200_PIOD_LED_MASK; */
/*         AT91_SYS->PIOD_CODR = current_leds  & CCM2200_PIOD_LED_MASK; */
}



static inline int ccm2200_gpio_out(__u32 device, __u32 mask, __u32 data)
{
        switch (device) {
        case CCM2200_GPIO_OUTPUT:
                ccm2200_set_digital_output(data, mask);
                break;

        case CCM2200_GPIO_LED:
                ccm2200_set_frontpanel_leds(data, mask);
                break;
        default:
                return -EIO;
        }
        return 0;
}


static inline int ccm2200_gpio_in(__u32 device, __u32 mask, __u32 *data)
{
        switch (device) {
        case CCM2200_GPIO_INPUT:
                *data = ccm2200_get_digital_input() & mask;
                break;
        case CCM2200_GPIO_SCONF:
                *data = ccm2200_get_sconf_input() & mask;
                break;
        case CCM2200_GPIO_OUTPUT:
                *data = current_output & mask;
                break;
        case CCM2200_GPIO_LED:
                *data = current_leds & mask;
                break;
        default:
                return -EIO;
        }
        return 0;
}

static int ccm2200_gpio_open(struct inode *inode, struct file *file)
{
	return 0;
}


static int ccm2200_gpio_release(struct inode *inode, struct file *file)
{
	return 0;
}


static long ccm2200_gpio_ioctl(struct file *file, unsigned int cmd, 
                               unsigned long arg)
{
	struct ccm2200_gpio_ioctl_data ioctl_data;

	if (copy_from_user(&ioctl_data,
                           (struct ccm2200_gpio_ioctl_data __user *)arg,
                           sizeof(ioctl_data)))
		return -EFAULT;

	switch(cmd) {
        case CCM2200_GPIO_IN:
                if (ccm2200_gpio_in(ioctl_data.device, ioctl_data.mask,
                		   &ioctl_data.data))
                	return -EFAULT;
                if (copy_to_user((struct ccm2200_gpio_ioctl_data __user *)arg,
                                 &ioctl_data, sizeof(ioctl_data)))
                        return -EFAULT;
                break;

        case CCM2200_GPIO_OUT:
                debug("CCM2200_GPIO_OUT: %d, 0x%08x, 0x%08x\n",
                      ioctl_data.device,
                      ioctl_data.mask, ioctl_data.data);
                return ccm2200_gpio_out(ioctl_data.device,
                                        ioctl_data.mask, ioctl_data.data);
                break;

        default:
                return -ENOIOCTLCMD;
                
	}

	return 0;
}

static struct file_operations ccm2200_gpio_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= ccm2200_gpio_ioctl,
	.open		= ccm2200_gpio_open,
	.release	= ccm2200_gpio_release,
};

static struct miscdevice ccm2200_gpio_miscdev = {
	MISC_DYNAMIC_MINOR,
	"ccm2200_gpio",
	&ccm2200_gpio_fops
};

int __init ccm2200_gpio_init(void)
{
        /* prepare digital outputs over bus and static memory controller */


        /*
         * Setup static memory controller chip select for digital output
         *
         */
        static const struct __initdata  at91rm9200_smc_config 
          dig_out_cs_config = {
                .wait_states          = 2, /* enough wait states... */
                .data_float_time      = 0,
                .byte_access_type     = AT91RM9200_BAT_8_BIT,
                .data_bus_width       = AT91RM9200_DATA_BUS_WIDTH_8,
                .data_read_protocol   = AT91RM9200_DRP_STANDARD,
                .address_to_cs_setup  = AT91RM9200_ACSS_1_CYCLE,
                .rw_setup             = 2, /* 2 cycle rw_setup! */
                .rw_hold              = 2  /* 2 cycle rw_hold! */
        };

        if ( at91rm9200_smc_configure(CCM2200_DIG_OUT_CS, 
                                      &dig_out_cs_config ) != 0 ) {
	    printk( KERN_ERR 
                    "Unable to configure digital out chip select signal\n" );
            return -EIO;
	}

	ccm2200_digital_out_virt = ioremap_nocache(CCM2200_DIG_OUT_PHYS, 
                                                   CCM2200_DIG_OUT_SIZE );
	if (!ccm2200_digital_out_virt) {
		printk("Failed to ioremap CCM2200 digital output\n");
		return -EIO;
	}


        /* prepare LED outputs and FG6 outputs on PIOD */
        {
                static const struct at91_pio_pins ccm2200_out_pio_d = { 
                        AT91_PIO_BASE(AT91_PIOD), 
                        CCM2200_PIOD_LED_MASK | CCM2200_PIOD_OUT8_11_MASK 
                };
                /* reset extern latch pin */
                static const struct at91_pio_pins ccm2200_n_ext_reset = 
                        { AT91_PIO_BASE(AT91_PIOB), 1<<3 }; 

                /* 2019-03-11 gc: preserve reset state of the digital
                 * outputs (port is logical '1' => the optocoupler's
                 * LED ore turned ON (and also MOSFET output) => This
                 * ensures no change of port state during booting process
                 */
                at91_sys_write(AT91C_VA_BASE_PIOD + PIO_SODR,
                               CCM2200_PIOD_OUT8_11_MASK);
                at91_pio_enable_open_drain_pins(&ccm2200_out_pio_d);  
                at91_pio_config_output_pins(&ccm2200_out_pio_d);
                /* assert reset signal to external latch */
                at91_pio_clear_all_pins(&ccm2200_n_ext_reset);
                at91_pio_config_output_pins(&ccm2200_n_ext_reset);
                udelay(10);
                at91_pio_set_all_pins(&ccm2200_n_ext_reset);
        }

	if (misc_register(&ccm2200_gpio_miscdev)) {
                iounmap((void *) ccm2200_digital_out_virt);
		return -ENODEV;
        }

	printk("CCM2200 GPIO driver initialized\n");

	return 0;
}	

static void __exit ccm2200_gpio_exit(void)
{
	misc_deregister(&ccm2200_gpio_miscdev);
        if (ccm2200_digital_out_virt) {
                iounmap((void *) ccm2200_digital_out_virt);
                ccm2200_digital_out_virt = NULL;
        }
}

module_init(ccm2200_gpio_init);
module_exit(ccm2200_gpio_exit);

MODULE_AUTHOR("Guido Classen");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CCM2200 GPIO driver");

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
