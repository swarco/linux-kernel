/*
 * linux/arch/arm/mach-at91rm9200/board-ccm2200.c
 *
 * Copyright (C) 2006-2010 by Weiss-Electronic GmbH.
 * Copyright (C) 2010-2012 by SWARCO Traffic Systems GmbH.
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
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/plat-ram.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/mach/serial_at91.h> /* must be here, why? */
#include <mach/board.h>
#include <mach/cpu.h>

#include <mach/at91-util.h>

#include <mach/gpio.h>

#include <mach/at91_pio.h>
#include <mach/at91rm9200_mc.h>
#include <mach/at91_pmc.h>
#include <mach/at91rm9200_smc.h>
#include <mach/at91_ramc.h>


#include <mach/board-ccm2200.h>

#include "../generic.h"



//#define UART_BASE AT91RM9200_BASE_US2
#define UART_BASE AT91_IO_P2V(AT91RM9200_BASE_US2)
#define UART_SR   (UART_BASE+0x14)
#define UART_THR   (UART_BASE+0x1c)
#include <mach/at91_dbgu.h>

static void uputc(int c)
{
	while (!(__raw_readl(UART_SR) & AT91_DBGU_TXRDY))
		barrier();
	__raw_writel(c, UART_THR);
}
static inline void flush(void)
{
	/* wait for transmission to complete */
	while (!(__raw_readl(UART_SR) & AT91_DBGU_TXEMPTY))
		barrier();
}
static void putstr(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			uputc('\r');
		uputc(c);
	}

	flush();
}


/* 
 * static inline void dbg_led(unsigned led)
 * {
 * 
 * /\*                 ldr     r0,=0xfffffa30 *\/
 * /\*                 ldr     r1,=0x0000ffff *\/
 * /\*                 str     r1,[r0] *\/
 * /\*                 ldr     r0,=0xfffffa34 *\/
 * /\*                 ldr     r1,=(1<<0) *\/
 * /\*                 str     r1,[r0] *\/
 * /\* 1:              b       1b *\/
 *         *(volatile unsigned *)AT91_IO_P2V(0xfffffa30) = ~led & 0xffff;
 *         *(volatile unsigned *)AT91_IO_P2V(0xfffffa34) = led & 0xffff;
 * }
 */


/* Macros for PLL setup (maybe to hardware.h) */
#define AT91_PLL_VALUE_FAST(div,mult)	((div) | (1024 << 8) | ((mult-1) << 16) | (1 << 15) | (1 << 29))
#define AT91_PLLB_VALUE_USB(div,mult)	((div) | (63 << 8) | ((mult-1) << 16) | (1 << 28))


#define AT91_SLOW_CLOCK         32768
#define AT91_MAIN_CLOCK         18432000
static void measure_and_set_clocks(void)
{
	unsigned long mainf_counter = at91_pmc_read(AT91_CKGR_MCFR) & AT91_PMC_MAINF;
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
			unsigned div_a = at91_pmc_read(AT91_CKGR_PLLAR) & AT91_PMC_DIV;
			unsigned mul_a = (at91_pmc_read(AT91_CKGR_PLLAR) & AT91_PMC_MUL) >> 16;
			
			if (div_a == 0 || mul_a == 0) {
				plla_clock = 0;
			} else {
				plla_clock = (((unsigned /*long*/ long) main_clock)
					      *  (mul_a+1) / div_a);
			}
	}
			
	{
                unsigned div_b = at91_pmc_read(AT91_CKGR_PLLBR) & AT91_PMC_DIV;
                unsigned mul_b = (at91_pmc_read(AT91_CKGR_PLLBR) & AT91_PMC_MUL) >> 16;
		
		if (div_b == 0 || mul_b == 0) {
			pllb_clock = 0;
		} else {
			pllb_clock = (((unsigned /*long*/ long) main_clock)
				      *  (mul_b+1) / div_b);
		}
	}

	switch (at91_pmc_read(AT91_PMC_MCKR) & AT91_PMC_CSS) {
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


	switch (at91_pmc_read(AT91_PMC_MCKR) & AT91_PMC_PRES) {
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

	switch (at91_pmc_read(AT91_PMC_MCKR) & AT91_PMC_MDIV) {
	default:
	case AT91RM9200_PMC_MDIV_1:
		break;

	case AT91RM9200_PMC_MDIV_2:
		master_clock >>= 1;
		break;

	case AT91RM9200_PMC_MDIV_3:
		master_clock /= 3;
		break;

	case AT91RM9200_PMC_MDIV_4:
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

static void __init ccm2200_init_early(void)
{
	/* Set cpu type: BGA */
	at91rm9200_set_type(ARCH_REVISON_9200_BGA);

	/* Initialize processor: 18.432 MHz crystal */
	at91_initialize(18432000);

	/* Setup the LEDs */
	at91_set_gpio_output(AT91_PIN_PD8, 1);
	at91_set_gpio_output(AT91_PIN_PD9, 1);
	at91_set_gpio_output(AT91_PIN_PD10, 1);
	at91_set_gpio_output(AT91_PIN_PD11, 1);
	at91_set_gpio_value(AT91_PIN_PD8, 1);
	at91_set_gpio_value(AT91_PIN_PD9, 0);
	at91_set_gpio_value(AT91_PIN_PD10, 1);
	at91_set_gpio_value(AT91_PIN_PD11, 0);


        ccm2200_at91_uart_init();
        /* 
         * /\*
         *  * Serial port configuration.
         *  *    ttyS0 .. ttyS3 = USART0 .. USART3
         *  *    ttyS4      = DBGU
         *  *\/
	 * /\* USART0 on ttyS0. (Rx, Tx, CTS, RTS) *\/
	 * at91_register_uart(AT91RM9200_ID_US0, 0, ATMEL_UART_CTS |
	 * 	ATMEL_UART_RTS);
	 * 
	 * /\* USART1 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) *\/
	 * at91_register_uart(AT91RM9200_ID_US1, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
	 * 		   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
	 * 		   | ATMEL_UART_RI);
	 * 
	 * /\* USART2 on ttyS2 (Rx, Tx, CTS, RTS) *\/
	 * at91_register_uart(AT91RM9200_ID_US2, 2, ATMEL_UART_CTS |
         *                    ATMEL_UART_RTS);
	 * 
	 * /\* USART3 on ttyS3 (Rx, Tx, CTS, RTS) *\/
	 * at91_register_uart(AT91RM9200_ID_US3, 3, ATMEL_UART_CTS |
         *                    ATMEL_UART_RTS);
	 * 
	 * 
	 * /\* DBGU on ttyS4. (Rx & Tx only) *\/
	 * at91_register_uart(0, 4, 0);
	 * 
	 * /\* set serial console to ttyS2 *\/
	 * at91_set_serial_console(2);
         */
	
	//measure_and_set_clocks();
}


static struct macb_platform_data __initdata ccm2200_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 0,
};


#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct at91_usbh_data __initdata ccm2200_usbh_data = {
	.ports		= 2,
	.vbus_pin	= {-EINVAL, -EINVAL},
	.overcurrent_pin= {-EINVAL, -EINVAL},
};
#endif


#if defined(CONFIG_USB_AT91) || defined(CONFIG_USB_AT91_MODULE)
static struct at91_udc_data __initdata ccm2200_udc_data = {
	.vbus_pin	= -EINVAL,
        /* 
         * 2010-06-09 gc: in newer kernels we must set a USB device pullup
         * pin
         *
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


static struct i2c_board_info __initdata ccm2200_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c64", 0x50),
	},
	{
		I2C_BOARD_INFO("ds1337", 0x68),
	}
};


static struct mtd_partition ccm2200_nand_partition[] = {
        {
                .name   = "nand-all", /* for Memory Validation Test */
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        }
};


static struct atmel_nand_data __initdata ccm2200_nand_data = {
	.ale		= 13,
	.cle		= 14,
	.det_pin	= -EINVAL,
	.rdy_pin	= AT91_PIN_PA19,
	.enable_pin	= -EINVAL,
	.bus_width_16	= 0,
	.ecc_mode	= NAND_ECC_SOFT,
	/// ????   .on_flash_bbt	= 1,
	.parts		= ccm2200_nand_partition,
	.num_parts	= ARRAY_SIZE(ccm2200_nand_partition),
};


/*
 * Setup static memory controller chip select for NAND Flash
 *
 */
static /* const */ 
struct at91rm9200_smc_config __initdata ccm2200_nand_cs_config = {
  .wait_states          = 5,
  .data_float_time      = 2,
  .byte_access_type     = AT91RM9200_BAT_8_BIT,
  .data_bus_width       = AT91RM9200_DATA_BUS_WIDTH_8,
  .data_read_protocol   = AT91RM9200_DRP_STANDARD,
  .address_to_cs_setup  = AT91RM9200_ACSS_STANDARD,
  .rw_setup             = 1,
  .rw_hold              = 1
};


#ifdef CONFIG_MTD_PHYSMAP

#define CCM2200_FLASH_BASE	AT91_CHIPSELECT_0
#define CCM2200_FLASH_SIZE	SZ_2M

/* 2006-04-24 gc: SWARCO Traffic Systems CCM2200 mapping */
static struct mtd_partition ccm2200_nor_flash_partitions[] = {
	{
		.name       = "all-ccm2200",
		.offset     = 0x00000000,
	},
	{
		.name       = "u-boot",
		.offset     = 0x00000000,
		.size       = 4 * SZ_64K,	// 4 * 64kB for u-boot and boot.bin 
	},
	{
		.name       = "kernel",		// default kernel image (1.75MB)
		.offset     = 4 * SZ_64K,
		.size       = 0x1c0000,
	},
	{
		.name       = "dummy",		// dummy partion to get nand at MTD5
		.offset     = SZ_2M,
		.size       = 0x00000000,
	},
};

static struct physmap_flash_data ccm2200_nor_flash_data = {
	.width		= 2,
	.parts		= ccm2200_nor_flash_partitions,
	.nr_parts	= ARRAY_SIZE(ccm2200_nor_flash_partitions)
};

static struct resource ccm2200_nor_flash_resource = {
	.start		= CCM2200_FLASH_BASE,
	.end		= CCM2200_FLASH_BASE + CCM2200_FLASH_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ccm2200_nor_flash = {
	.name		= "physmap-flash", 
	.id		= 0,
	.dev		= {
				.platform_data	= &ccm2200_nor_flash_data,
			},
	.resource	= &ccm2200_nor_flash_resource,
	.num_resources	= 1,
};
#endif /* CONFIG_MTD_PHYSMAP */

#ifdef CONFIG_MTD_PLATRAM
/*
 * Setup static memory controller chip select for SRAM
 *
 */
static struct  at91rm9200_smc_config __initdata sram_cs_config = {
	.wait_states          = 15, /* SRAM needs at least 3 wait states! */
	.data_float_time      = 2,
	.byte_access_type     = AT91RM9200_BAT_16_BIT,
	.data_bus_width       = AT91RM9200_DATA_BUS_WIDTH_16,
	.data_read_protocol   = AT91RM9200_DRP_EARLY, //AT91_DRP_STANDARD,
	.address_to_cs_setup  = AT91RM9200_ACSS_STANDARD,
	.rw_setup             = 2, /* SRAM needs 1 cycle rw_setup! */
	.rw_hold              = 2  /* SRAM needs 1 cycle rw_hold! */
};

struct platdata_mtd_ram ccm2200_sram_pdata = {
	.mapname	= "ccm2200-SRAM",
	.bankwidth	= 4,	/* use 32bit access for more speed! */
};

static struct resource ccm2200_sram_resource[] = {
	[0] = {
		.start = CCM2200_SRAM_PHYS,
		.end   = CCM2200_SRAM_PHYS + CCM2200_SRAM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ccm2200_sram = {
	.name		= "mtd-ram",
	.id		= 0,
	.resource	= ccm2200_sram_resource,
	.num_resources	= ARRAY_SIZE(ccm2200_sram_resource),
	.dev	= {
		.platform_data = &ccm2200_sram_pdata,
	},
};
#endif /* MTD_PLATRAM */


/* we use ccm2200 specific LED driver (drivers/leds/leds-ccm2200.c) which
 * allows intermixing of old led API (ccm2200_set_frontpanel_leds) and
 * Linux led API (/sys/class/led/...) 
 */
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


/* 
 * static struct i2c_gpio_platform_data ccm2200_i2c_gpio_data = {
 *      .sda_pin    = AT91_PIN_PA25,
 *      .scl_pin    = AT91_PIN_PA26,
 * };
 * 
 * static struct platform_device ccm2200_i2c_gpio_controller = {
 *      .name       = "i2c-gpio",
 *      .id     = 0,
 *      .dev        = {
 *          .platform_data  = &ccm2200_i2c_gpio_data,
 *      },
 * };
 * 
 * static void ccm2200_add_device_soft_i2c(void)
 * {
 *         /\* i2c GPIO bit bang *\/
 *         at91_set_gpio_input(ccm2200_i2c_gpio_data.sda_pin, 1);
 *         at91_set_gpio_input(ccm2200_i2c_gpio_data.scl_pin, 1);
 *         platform_device_register(&ccm2200_i2c_gpio_controller);
 * }
 */


static void __init ccm2200_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&ccm2200_eth_data);

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/* USB Host */
	at91_add_device_usbh(&ccm2200_usbh_data);
#endif

#if defined(CONFIG_USB_AT91) || defined(CONFIG_USB_AT91_MODULE)
	/* USB Device */
	at91_add_device_udc(&ccm2200_udc_data);
#endif

	/* I2C */
        /* 2007-11-13 gc: use software (bit bang) i2c, because hardware
         * two wire controller on AT91RM9200 dosn't work reliable!
         */
	at91_add_device_i2c(ccm2200_i2c_devices, 
                            ARRAY_SIZE(ccm2200_i2c_devices));

	/* SPI */
	at91_add_device_spi(ccm2200_spi_devices, 
			    ARRAY_SIZE(ccm2200_spi_devices));

#ifdef CONFIG_MTD_PHYSMAP
	/* NOR Flash */
	platform_device_register(&ccm2200_nor_flash);
#endif /* CONFIG_MTD_PHYSMAP */

#ifdef CONFIG_MTD_PLATRAM
	/* battery backuped SRAM */
        if ( at91rm9200_smc_configure( CCM2200_SRAM_CS, 
				       &sram_cs_config ) != 0 ) {
	    printk( KERN_ERR 
                    "Unable to configure SRAM chip select signal\n" );
            return -EIO;
	} else {
		platform_device_register(&ccm2200_sram);
	}
#endif

	/* NAND */
        if ( at91rm9200_smc_configure( CCM2200_NAND_FLASH_CS, 
				       &ccm2200_nand_cs_config ) != 0 ) {
	    printk( KERN_ERR 
                    "Unable to configure NAND flash chip select signal\n" );
	} else {
          at91_add_device_nand(&ccm2200_nand_data);
        }

        /* 2007-09-10 gc: LEDs: standard Linux LED driver */
#ifdef CCM2200_USE_AT91_LEDS
	at91_gpio_leds(ccm2200_leds, ARRAY_SIZE(ccm2200_leds));
#else
	platform_device_register(&ccm2200_leds);
#endif

}

#include <linux/console.h>
static int __init ccm2200_console_init(void)
{
	if (!machine_is_ccm2200())
		return 0;

	return add_preferred_console("ttyAT", 2, "38400");
}
console_initcall(ccm2200_console_init);

MACHINE_START(CCM2200, "SWARCO CCM2200")
	/* Maintainer: SWARCO Traffic Systems GmbH */
	.timer		= &at91rm9200_timer,
	.map_io		= at91_map_io,
	.init_early	= ccm2200_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= ccm2200_board_init,
MACHINE_END


/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
