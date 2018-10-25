/*
 * linux/arch/arm/mach-at91/board-swarcoscc3g.c
 *
 * Copyright (C)2011 Micro Technic A/S
 *
 * Board support for Swarco SCC-3G (External Modem) by Karl Olsen
 * <karl@micro-technic.com>
 *
 * Before the board got its official name, SCC-3G, it was registered on
 * http://www.arm.linux.org.uk/developer/machines/ as "Swarco External Modem"
 * (machine ID 3362), CONFIG_MACH_SWARCOEXTMODEM. Too bad.
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
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/at91sam9_smc.h>

#include "generic.h"


static void swarcoscc3g_poweroff(void)
{
	/* Power off: Activate 5v_off (PB17) pin. */
	at91_set_gpio_value(AT91_PIN_PB17, 1);	
}


/*
 * Serial port configuration
 *
 *  Linux   CPU   Electrical Signals         Used for
 *  -------------------------------------------------------------------------
 *  ttyAT0  DBGU  RS232      RxD+TxD         Console port in 10p box header
 *  ttyAT1  US0   (TTL)      All modem       Directly to Telit 2G modem
 *  ttyAT2  US2   RS232      RxD,TxD,RTS,CTS RS232 port in DB9 male connector
 *  ttyAT3  US5   (TTL)      RxD,TxD         Directly to Bluetooth module
 *  ttyAT4  US3   RS422/485  RxD,TxD,RTS     RS488/RS485 port #1
 *  ttyAT5  US1   RS422/485  RxD,TxD,RTS     RS488/RS485 port #2
 *
 * US4 is unused (PA30 and PA31).
 */

		
static void __init swarcoscc3g_init_early(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91_initialize(18432000);

	/* Replace the poweroff function. */
	pm_power_off = swarcoscc3g_poweroff;
	
	/* ttyAT0 using DBGU.
	 * Console port in 10p box header. RXD,TXD. */
	at91_register_uart(0, 0, 0);
	 
	 /* ttyAT1 using USART0.
	  * Telit 2G modem. RXD,TXD,RTS,CTS,DSR,DTR,DCD,RI. */
	at91_register_uart(AT91SAM9260_ID_US0, 1,
		ATMEL_UART_CTS | ATMEL_UART_RTS | ATMEL_UART_DTR |
		ATMEL_UART_DSR | ATMEL_UART_DCD | ATMEL_UART_RI);
	
	/* ttyAT2 using USART2.
	 * This port was not used in PCB rev.1 but is on rev.2. */
	at91_register_uart(AT91SAM9260_ID_US2, 2, ATMEL_UART_RTS | ATMEL_UART_CTS);
	
	/* ttyAT3 using USART5.
	 * Sena ESD-1000 Bluetooth module. RXD,TXD. */
	at91_register_uart(AT91SAM9260_ID_US5, 3, 0);
	
	/* ttyAT4 using USART3.
	 * External RS422/RS485 #1. RXD,TXD,RTS.
	 * Note: Some hacking has been done to drivers/serial/atmel_serial.c
	 * so that RS485 mode is always enabled for this port. This has been
	 * done instead of using the mainline RS485 support for these reasons:
	 *  - Mainline RS485 requires calling an ioctl() to set RS485 mode
	 *  - When a port is closed, the USART is not in RS485 mode, and the
	 *    CPU RTS pin is high (normal RTS function inactive state). This
	 *    enables our RS485 driver(!), and this can disturb other traffic
	 *    on the RS485 bus. Our RS485 driver should always be disabled
	 *    (CPU RTS pin low) unless we are actively transmitting.
	 *  - Mainline RS485 only supports half duplex, and always ignores
	 *    incoming bytes while transmitting. Our hardware has a jumper
	 *    that selects whether incoming data should be ignored or not
	 *    while transmitting.
	 */ 
	at91_register_uart(AT91SAM9260_ID_US3, 4, 0);
	
	/* ttyAT5 using USART1.
	 * External RS422/RS485 #2. RXD,TXD,RTS.
	 * Note: Some hacking has been done to drivers/serial/atmel_serial.c
	 * so that RS485 mode is always enabled for this port. See above. */
	at91_register_uart(AT91SAM9260_ID_US1, 5, 0);
	
	/* Set serial console to ttyAT0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static struct at91_eth_data __initdata swarcoscc3g_macb_data = {
	.is_rmii	= 1,
};


static struct at91_usbh_data __initdata swarcoscc3g_usbh_data = {
	.ports		= 2,
};


static struct at91_udc_data __initdata swarcoscc3g_udc_data = {
	.vbus_pin	= AT91_PIN_PB16,
	.pullup_pin	= 0,
};


static struct mci_platform_data __initdata swarcoscc3g_mmc_atmel_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PB3,		/* Was PA5 on pcbrev.1 */
		.wp_pin		= AT91_PIN_PA25,	/* Was PA4 on pcbrev.1 */
	},
};


static struct i2c_board_info __initdata swarcoscc3g_i2c_devices[] = {
	{
		/* "swarcoio-ads1100" is implemented in swarcoio.c */
		I2C_BOARD_INFO("swarcoio-ads1100", 0x49),
	},
	{
		/* "swarcoio-ads1112" is implemented in swarcoio.c */
		I2C_BOARD_INFO("swarcoio-ads1112", 0x48),
	},
}
 ;

/* SPI device: Microchip MCP2515 CAN controller */
static struct mcp251x_platform_data swarcoscc3g_mcp2515_info = {
    .oscillator_frequency	= 8000000,
    .irq_flags			= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
    .board_specific_setup	= NULL,
    .transceiver_enable		= NULL,
    .power_enable		= NULL
};

static struct spi_board_info swarcoscc3g_spi_devices[] = {
	{	    /* Microchip MCP2515 CAN controller */
		.modalias	= "mcp2515",
		.platform_data  = &swarcoscc3g_mcp2515_info,
//		.controller_data= (void *)PIN_CAN_CS_NOT, /* ??? */
		.irq		= AT91_PIN_PC16,
		.max_speed_hz	= 4*1000*1000,
		.chip_select	= 1
	},
};


static struct mtd_partition __initdata swarcoscc3g_nand_partition[] = {
	{
		/* Boot partition contains the kernel (zImage), no filesystem.
		 * Small page flash: 256 blocks of 16KB each
		 * Large page flash: 32 blocks of 128KB each
		 */
		.name	= "kernel",
		.offset	= 0,
		.size	= 4 * SZ_1M,
	},
	{
		/* UBI partition, containing the root volume, and possibly
		 * other volumes. */
		.name	= "ubi0",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct atmel_nand_data __initdata swarcoscc3g_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC7,
	.enable_pin	= AT91_PIN_PC14,
	.parts   	= swarcoscc3g_nand_partition,
    .num_parts  = ARRAY_SIZE(swarcoscc3g_nand_partition)
};


static void __init swarcoscc3g_add_device_nand(void)
{
	/* Configure chip select 3 (NAND).
	 * This has actually already been done by the bootloader. We configure
	 * it here again with the same parameters.
	 * If we knew that we use large-page flash we could use faster timings
	 * giving max 26.6 MB/s instead of these which give max 16.6 MB/s. */
	at91_sys_write (AT91_SMC_SETUP(3),
		AT91_SMC_NWESETUP_(0)		|
		AT91_SMC_NCS_WRSETUP_(0)	|
		AT91_SMC_NRDSETUP_(0)		|
		AT91_SMC_NCS_RDSETUP_(0));
	at91_sys_write (AT91_SMC_PULSE(3), 
		AT91_SMC_NWEPULSE_(5)		|
		AT91_SMC_NCS_WRPULSE_(5)	|
		AT91_SMC_NRDPULSE_(5)		|
		AT91_SMC_NCS_RDPULSE_(5));
	at91_sys_write (AT91_SMC_CYCLE(3),
		AT91_SMC_NWECYCLE_(8)		|
		AT91_SMC_NRDCYCLE_(8));
	at91_sys_write (AT91_SMC_MODE(3),
		AT91_SMC_READMODE		|
		AT91_SMC_WRITEMODE		|
		AT91_SMC_TDF_(2));
	
	at91_add_device_nand(&swarcoscc3g_nand_data);
}


static struct gpio_led swarcoscc3g_leds[] = {
	{
		.name		= "led1",
		.gpio		= AT91_PIN_PC1,
		.active_low	= 0,
	},
};


/* Swarco /sys/class/swarcoio/ */
static struct platform_device swarcoio_device = {
	.name		= "swarcoio-platform",
	.id		= 1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= NULL,
	},
};


static void __init swarcoscc3g_board_init(void)
{
	/* Internal serial ports */
	at91_add_device_serial();
	
	/* Ethernet */
	at91_add_device_eth (&swarcoscc3g_macb_data);
	
	/* USB Host */
	at91_add_device_usbh (&swarcoscc3g_usbh_data);
	
	/* USB Device */
	at91_add_device_udc (&swarcoscc3g_udc_data);
	
	/* SD/MMC */
	at91_add_device_mci(0, &swarcoscc3g_mmc_atmel_data);
	
	/* I2C */
	at91_add_device_i2c(swarcoscc3g_i2c_devices, ARRAY_SIZE(swarcoscc3g_i2c_devices));
	
	/* NAND */
	swarcoscc3g_add_device_nand();
	
#if defined(CONFIG_SPI_ATMEL) || defined(CONFIG_SPI_ATMEL_MODULE)
	/* SPI */
	at91_add_device_spi(swarcoscc3g_spi_devices, ARRAY_SIZE(swarcoscc3g_spi_devices));
#endif

	/* LEDs */
	at91_gpio_leds (swarcoscc3g_leds, ARRAY_SIZE(swarcoscc3g_leds));

	/* SCC-3G platform device */
	platform_device_register(&swarcoio_device);
}

MACHINE_START(SWARCOEXTMODEM, "Swarco SCC-3G")
	/* Maintainer: Karl Olsen <karl@micro-technic.com> */
	.timer		= &at91sam926x_timer,
	.map_io		= at91_map_io,
    .init_early = swarcoscc3g_init_early,
	.init_irq	= at91_init_irq_default,
	.init_machine	= swarcoscc3g_board_init,
MACHINE_END
