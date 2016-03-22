/*
 * linux/arch/arm/mach-at91rm9200/board-ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Configuation settings for the CCM2200 board.
 * @references: [1] board-unc90.c
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
 *     2006-04-24 gc: initial version (taken from board-unc90.c)
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>

#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/hardware.h>
//#include <asm/mach/serial_at91rm9200.h> /* must be here, why? */
#include <asm/arch/board.h>

#include <asm/arch/at91-util.h>

#include <asm/arch/gpio.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/at91rm9200_mc.h>
#include <asm/arch/at91_pmc.h>


#include <asm/arch/board-ccm2200.h>

#include "../generic.h"



/* //#define UART_BASE AT91RM9200_BASE_US2 */
/* #define UART_BASE AT91_IO_P2V(AT91RM9200_BASE_US2) */
/* #define UART_SR   (UART_BASE+0x14) */
/* #define UART_THR   (UART_BASE+0x1c) */
/* #include <asm/arch/at91_dbgu.h> */

/* static void putc(int c) */
/* { */
/* 	while (!(__raw_readl(UART_SR) & AT91_DBGU_TXRDY)) */
/* 		barrier(); */
/* 	__raw_writel(c, UART_THR); */
/* } */
/* static inline void flush(void) */
/* { */
/* 	/\* wait for transmission to complete *\/ */
/* 	while (!(__raw_readl(UART_SR) & AT91_DBGU_TXEMPTY)) */
/* 		barrier(); */
/* } */
/* static void putstr(const char *ptr) */
/* { */
/* 	char c; */

/* 	while ((c = *ptr++) != '\0') { */
/* 		if (c == '\n') */
/* 			putc('\r'); */
/* 		putc(c); */
/* 	} */

/* 	flush(); */
/* } */


static inline void dbg_led(unsigned led)
{

/*                 ldr     r0,=0xfffffa30 */
/*                 ldr     r1,=0x0000ffff */
/*                 str     r1,[r0] */
/*                 ldr     r0,=0xfffffa34 */
/*                 ldr     r1,=(1<<0) */
/*                 str     r1,[r0] */
/* 1:              b       1b */
        *(volatile unsigned *)AT91_IO_P2V(0xfffffa30) = ~led & 0xffff;
        *(volatile unsigned *)AT91_IO_P2V(0xfffffa34) = led & 0xffff;
}

static void __init ccm2200_init_irq(void)
{
/* 	putstr("init_irq\n"); */

	/* Initialize AIC controller */
	at91rm9200_init_interrupts(NULL);

	/* Set up the GPIO interrupts (2007-02-07 gc: is this necessary???) */
	//at91_gpio_irq_setup(BGA_GPIO_BANKS);
}

/* 
 * Configure the static memory controller chip select register using
 * values in struct at91_smc_cs_info
 */
int at91_config_smc_cs( register const struct at91_smc_cs_info *info )
{
  unsigned int mask;

  /* use #if 0 to disable memory controller reprogramming in kernel */
#if 1
        /* check parameters */
        if ( info->chip_select < 0 || info->chip_select > 7 
             || info->wait_states < 0 || info->wait_states > 128  
             || info->data_float_time < 0 || info->data_float_time > 15
             || (info->byte_access_type != AT91_BAT_TWO_8_BIT
                 && info->byte_access_type != AT91_BAT_16_BIT)
             || (info->data_read_protocol != AT91_DRP_STANDARD
                 && info->data_read_protocol != AT91_DRP_EARLY)
             || info->address_to_cs_setup < AT91_ACSS_STANDARD
             || info->address_to_cs_setup > AT91_ACSS_3_CYCLES
             || info->rw_setup < 0 || info->rw_setup > 7
             || info->rw_hold  < 0 || info->rw_hold > 7)
        {
                return -1;
        }

        // configure gpios, if necessary
        if( info->chip_select > 3 )
        {
#define AT91C_PC10_NCS4_CFCS  (1<<10)
#define AT91C_PC11_NCS5_CFCE1 (1<<11)
#define AT91C_PC12_NCS6_CFCE2 (1<<12)
#define AT91C_PC13_NCS7	      (1<<13)
                switch( info->chip_select )
                {
                case 4:	mask = AT91C_PC10_NCS4_CFCS;	break;
                case 5:	mask = AT91C_PC11_NCS5_CFCE1;	break;
                case 6:	mask = AT91C_PC12_NCS6_CFCE2;	break;
                case 7:	mask = AT91C_PC13_NCS7;		break;
                default: mask = 0;
                }
                /* select peripheral a function */
                //AT91_SYS->PIOC_ASR = mask;
                at91_sys_write(AT91_PIOC + PIO_ASR, mask);

                /* disable pio controller and enable peripheral */
                //AT91_SYS->PIOC_PDR = mask;
                at91_sys_write(AT91_PIOC + PIO_PDR, mask);
        }

        /* write the new configuration to SMC chip select register */
	at91_sys_write(AT91_SMC_CSR(info->chip_select), 
                       (info->wait_states > 0 
                        ? (AT91_SMC_NWS & (info->wait_states-1)) 
                        | AT91_SMC_WSEN
                        : 0)
                       | (info->data_float_time << 8) 
                       | (info->byte_access_type == AT91_BAT_16_BIT
                   ? AT91_SMC_BAT : 0) 
                | ((int)info->data_bus_width << 13) 
                | (info->data_read_protocol == AT91_DRP_EARLY 
                   ? AT91_SMC_DPR : 0) 
                | ((int)info->address_to_cs_setup << 16)
                | (info->rw_setup << 24)
                | (info->rw_hold  << 28)
                );
#else

  printk(KERN_WARNING "CCM2200 at91_config_smc_cs(cs=%d): "
         "NOT reprogramming memory controller (DISABLED)\n",
         info->chip_select);

#endif
        return 0;
}

EXPORT_SYMBOL( at91_config_smc_cs );


/* Macros for PLL setup (maybe to hardware.h) */
#define AT91_PLL_VALUE_FAST(div,mult)	((div) | (1024 << 8) | ((mult-1) << 16) | (1 << 15) | (1 << 29))
#define AT91_PLLB_VALUE_USB(div,mult)	((div) | (63 << 8) | ((mult-1) << 16) | (1 << 28))


#define AT91_SLOW_CLOCK         32768
#define AT91_MAIN_CLOCK         18432000
static void measure_and_set_clocks(void)
{
	unsigned long mainf_counter = at91_sys_read(AT91_CKGR_MCFR) & AT91_PMC_MAINF;
	unsigned long main_clock = mainf_counter * (AT91_SLOW_CLOCK / 16);
	unsigned long main_clock_measured;
	unsigned long plla_clock;
	unsigned long pllb_clock;

	unsigned long master_clock;
	unsigned long cpu_clock;

	main_clock_measured = main_clock;

	if (main_clock_measured > (unsigned long) (AT91_MAIN_CLOCK * 0.99)
	    && main_clock_measured < (unsigned long) (AT91_MAIN_CLOCK * 1.01)) {
	    main_clock = AT91_MAIN_CLOCK;
	}


	{
			unsigned div_a = at91_sys_read(AT91_CKGR_PLLAR) & AT91_PMC_DIV;
			unsigned mul_a = (at91_sys_read(AT91_CKGR_PLLAR) & AT91_PMC_MUL) >> 16;
			
			if (div_a == 0 || mul_a == 0) {
				plla_clock = 0;
			} else {
				plla_clock = (((unsigned /*long*/ long) main_clock)
					      *  (mul_a+1) / div_a);
			}
	}
			
	{
                unsigned div_b = at91_sys_read(AT91_CKGR_PLLBR) & AT91_PMC_DIV;
                unsigned mul_b = (at91_sys_read(AT91_CKGR_PLLBR) & AT91_PMC_MUL) >> 16;
		
		if (div_b == 0 || mul_b == 0) {
			pllb_clock = 0;
		} else {
			pllb_clock = (((unsigned /*long*/ long) main_clock)
				      *  (mul_b+1) / div_b);
		}
	}

	switch (at91_sys_read(AT91_PMC_MCKR) & AT91_PMC_CSS) {
	case AT91_PMC_CSS_SLOW:
		master_clock = AT91_SLOW_CLOCK;
		break;

	case AT91_PMC_CSS_MAIN:
		master_clock = main_clock;
		break;

	case AT91_PMC_CSS_PLLA:
		master_clock = plla_clock;
		break;

	case AT91_PMC_CSS_PLLB:
		master_clock = pllb_clock;
		break;

	default:
		master_clock = 0;
	}


	switch (at91_sys_read(AT91_PMC_MCKR) & AT91_PMC_PRES) {
	default:
	case AT91_PMC_PRES_1:
		break;
	case AT91_PMC_PRES_2:
		master_clock >>= 1;
		break;

	case AT91_PMC_PRES_4:
		master_clock >>= 2;
		break;

	case AT91_PMC_PRES_8:
		master_clock >>= 3;
		break;

	case AT91_PMC_PRES_16:
		master_clock >>= 4;
		break;

	case AT91_PMC_PRES_32:
		master_clock >>= 5;
		break;

	case AT91_PMC_PRES_64:
		master_clock >>= 6;
		break;
	}

	cpu_clock = master_clock;

	switch (at91_sys_read(AT91_PMC_MCKR) & AT91_PMC_MDIV) {
	default:
	case AT91_PMC_MDIV_1:
		break;

	case AT91_PMC_MDIV_2:
		master_clock >>= 1;
		break;

	case AT91_PMC_MDIV_3:
		master_clock /= 3;
		break;

	case AT91_PMC_MDIV_4:
		master_clock >>= 2;
		break;
	}

  /* 2006-10-17 gc: save measured clock values in global variables */
//  at91_master_clock = master_clock;
//  at91_main_clock   = cpu_clock;

  printk("CCM2200 Main   Clock: %9lu Hz (measured: %9lu Hz)\n", 
	 main_clock, main_clock_measured);
  printk("        PLLA   Clock: %9lu Hz\n", plla_clock);
  printk("        PLLB   Clock: %9lu Hz\n", pllb_clock);
  printk("        CPU    Clock: %9lu Hz\n", cpu_clock);
  printk("        Master Clock: %9lu Hz\n", master_clock);
  printk("Kernel timer frequency: %6u HZ\n", (unsigned) HZ);

}

static void __init ccm2200_map_io(void)
{
/* 	putstr("map_io\n"); */

	//at91rm9200_map_io();

        /* 2006-04-28 gc: kernel will not reprogramm clocks yet, they are 
         * initialized only by the bootloader!
         *
         * 2006-10-17 gc: use messure_and_set_clocks() instead of hard coded
	 *                clock values
	 */
	//at91_pllb_clock = AT91_PLLB_VALUE_USB(14, 73);	// (18.432 / 14 * 73) /2 = 47.9714


	/* Initialize processor: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_BGA);

	/* Setup the LEDs */
	//at91_init_leds(AT91_PIN_PB2, AT91_PIN_PB2);

	//measure_and_set_clocks();

        /* early initialization to use AT91 UART as serial console!  */
        //dbg_led(1<<5);
        ccm2200_at91_uart_init();
        //dbg_led(1<<8);

	/* Setup the LEDs */
	at91_init_leds(AT91_PIN_PD10, AT91_PIN_PD11);

}

static struct at91_eth_data __initdata ccm2200_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 0,
};


#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct at91_usbh_data __initdata ccm2200_usbh_data = {
	.ports		= 2,
};
#endif


#if defined(CONFIG_USB_GADGET_AT91)
static struct at91_udc_data __initdata ccm2200_udc_data = {
/* 	.vbus_pin	= -1,	/\* Not used on CCM2200 *\/ */
        /* 
         * 2010-06-09 gc: 
         * We use the SCONF3 signal. This signal can be accessed on
         * DIP-Switch 8
         */
	.pullup_pin	= AT91_PIN_PB28,
};
#endif

static struct spi_board_info ccm2200_spi_devices[] = {
	{ /* User accessable spi - cs0 (10Mhz) */
		.modalias = "spi-cs0",
		.chip_select  = 0,
		.max_speed_hz = 10 * 1000 * 1000,
	},
	{ /* User accessable spi - cs1 (250KHz) */
		.modalias = "spi-cs1",
		.chip_select  = 1,
		.max_speed_hz = 250 *  1000,
	},
	{ /* User accessable spi - cs2 (1MHz) */
		.modalias = "spi-cs2",
		.chip_select  = 2,
		.max_speed_hz = 1 * 1000 *  1000,
	},
	{ /* User accessable spi - cs3 (10MHz) */
		.modalias = "spi-cs3",
		.chip_select  = 3,
		.max_speed_hz = 10 * 1000 *  1000,
	},
};

#undef CCM2200_USE_AT91_LEDS

#ifdef CCM2200_USE_AT91_LEDS

/* 2007-09-10 gc: make usage of new Linux kernel LED framework! */
static struct at91_gpio_led ccm2200_leds[] = {
	{
		.name		= "led0",
		.gpio		= AT91_PIN_PD0,
		//.trigger	= "heartbeat",
	},
	{
		.name		= "led1",
		.gpio		= AT91_PIN_PD1,
		//.trigger	= "timer",
	},
	{
		.name		= "led2",
		.gpio		= AT91_PIN_PD2,
	},
	{
		.name		= "led3",
		.gpio		= AT91_PIN_PD3,
	},
	{
		.name		= "led4",
		.gpio		= AT91_PIN_PD4,
	},
	{
		.name		= "led5",
		.gpio		= AT91_PIN_PD5,
	},
	{
		.name		= "led6",
		.gpio		= AT91_PIN_PD6,
	},
	{
		.name		= "led7",
		.gpio		= AT91_PIN_PD7,
	},
	{
		.name		= "led8",
		.gpio		= AT91_PIN_PD8,
	},
	{
		.name		= "led9",
		.gpio		= AT91_PIN_PD9,
	},
	{
		.name		= "led10",
		.gpio		= AT91_PIN_PD10,
	},
	{
		.name		= "led11",
		.gpio		= AT91_PIN_PD11,
	},
	{
		.name		= "led12",
		.gpio		= AT91_PIN_PD12,
	},
	{
		.name		= "led13",
		.gpio		= AT91_PIN_PD13,
	},
	{
		.name		= "led14",
		.gpio		= AT91_PIN_PD14,
	},
	{
		.name		= "led15",
		.gpio		= AT91_PIN_PD15,
	}
};

#else
static struct platform_device ccm2200_leds = {
	.name		= "ccm2200_leds",
	.id		= -1,
};
#endif


static struct i2c_gpio_platform_data ccm2200_i2c_gpio_data = {
     .sda_pin    = AT91_PIN_PA25,
     .scl_pin    = AT91_PIN_PA26,
};

static struct platform_device ccm2200_i2c_gpio_controller = {
     .name       = "i2c-gpio",
     .id     = 0,
     .dev        = {
         .platform_data  = &ccm2200_i2c_gpio_data,
     },
};

static void ccm2200_add_device_soft_i2c(void)
{
        /* i2c GPIO bit bang */
        at91_set_gpio_input(ccm2200_i2c_gpio_data.sda_pin, 1);
        at91_set_gpio_input(ccm2200_i2c_gpio_data.scl_pin, 1);
        platform_device_register(&ccm2200_i2c_gpio_controller);
}


static void __init ccm2200_board_init(void)
{
/* 	putstr("board_init\n"); */

        //ccm2200_set_frontpanel_leds(1<<0, 0xffff);
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&ccm2200_eth_data);

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/* USB Host */
	at91_add_device_usbh(&ccm2200_usbh_data);
#endif

#if defined(CONFIG_USB_GADGET_AT91)
	/* USB Device */
	at91_add_device_udc(&ccm2200_udc_data);
#endif

	/* I2C */
        /* 2007-11-13 gc: use software (bit bang) i2c, because hardware
         * two wire controler on AT91RM9200 dosn't work reliable!
         */
	/* at91_add_device_i2c(); */
        ccm2200_add_device_soft_i2c();

	/* SPI */
	at91_add_device_spi(ccm2200_spi_devices, 
			    ARRAY_SIZE(ccm2200_spi_devices));

        /* 2007-09-10 gc: LEDs: standard Linux LED driver */
#ifdef CCM2200_USE_AT91_LEDS
	at91_gpio_leds(ccm2200_leds, ARRAY_SIZE(ccm2200_leds));
#else
	platform_device_register(&ccm2200_leds);
#endif

}

MACHINE_START(CCM2200, "CCM2200")
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer	= &at91rm9200_timer,

	.map_io		= ccm2200_map_io,
	.init_irq	= ccm2200_init_irq,
	.init_machine	= ccm2200_board_init,
MACHINE_END


/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
