/*
 * linux/arch/arm/mach-at91rm9200/quad-uart-ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      Support for external Quad UART SC16C754 on CCM2200 board.
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
 *     2006-04-29 gc: removed wrong AT91C_ID_IRQ0 in ds1337.c, now we have it
 *     2006-04-28 gc: initial version (partly derived from mach-a9m2410.c
 *                    written by Jonas Dietsche, Joachim Jaeger)
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>


#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/irq.h>


#include <asm/io.h>		/* ioremap */
#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/at91-util.h>
#include <asm/arch/board-ccm2200.h>

#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <asm/arch/irqs.h>
#include <asm/hardware.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/gpio.h>

#include <linux/clk.h>          /* clk_get */


static struct 
plat_serial8250_port serial_platform_data[CCM2200_QUAD_UART_NUM_PORTS+1];
static struct platform_device serial_device = {
        .name			= "serial8250",
        .id			= 0,
        .dev			= {
                .platform_data	= serial_platform_data,
        }
};



#ifndef CONFIG_AT91_PROGRAMMABLE_CLOCKS

/** 2006-04-28 gc: configure an at91rm9200 programable clock output 
 * <br>
 * @param pck_number    number of pck pin (0...3 for AT91RM9200)
 * @return              Errorcode: 
 *                        0  ok (Service terminated)
 *                        -1 error occurred
 */
enum at91rm9200_clock_source {
        AT91RM9200_SLOW_CLOCK = 0,
        AT91RM9200_MAIN_CLOCK = 1,
        AT91RM9200_PLLA_CLOCK = 2,
        AT91RM9200_PLLB_CLOCK = 3
};

enum at91rm9200_pck_prescaler {
        PCK_SELECTED_CLOCK = 0,
        PCK_SELECTED_CLOCK_DIV_2 = 1,
        PCK_SELECTED_CLOCK_DIV_4 = 2,
        PCK_SELECTED_CLOCK_DIV_8 = 3,
        PCK_SELECTED_CLOCK_DIV_16 = 4,
        PCK_SELECTED_CLOCK_DIV_32 = 5,
        PCK_SELECTED_CLOCK_DIV_64 = 6
};

int at91rm9200_config_prg_clk_generator(int pck_number,
                                        enum at91rm9200_clock_source source,
                                        enum at91rm9200_pck_prescaler prescaler)
{
        /* program source and prescaler */
        //AT91_SYS->PMC_PCKR[pck_number] = source | (prescaler << 2);
        at91_sys_write(AT91_PMC_PCKR(pck_number), source | (prescaler << 2));
  
        /* enable programmable Clock Output */
        switch (pck_number) {
#define		AT91_PMC_PCK4		(1 << 12)		/* Programmable Clock 4 */
#define		AT91_PMC_PCK5		(1 << 13)		/* Programmable Clock 5 */
#define		AT91_PMC_PCK6		(1 << 14)		/* Programmable Clock 6 */
#define		AT91_PMC_PCK7		(1 << 15)		/* Programmable Clock 7 */

        case 0: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK0); break;
        case 1: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK1); break;
        case 2: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK2); break;
        case 3: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK3); break;
        case 4: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK4); break;
        case 5: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK5); break;
        case 6: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK6); break;
        case 7: at91_sys_write(AT91_PMC_SCER, AT91_PMC_PCK7); break;
        }
        return 0;
}

#endif

static int __init ccm2200_quad_uart_init(void) 
{
	void __iomem *quad_uart_virt_base;
        struct plat_serial8250_port *port_iter;
        int i;

        /*
         * Setup static memory controller chip select for external quad UART
         *
         */
        static const struct __initdata at91_smc_cs_info quad_uart_cs_config = {
                .chip_select          = CCM2200_QUAD_UART_CS,
                .wait_states          = 16,
                .data_float_time      = 0,
                .byte_access_type     = AT91_BAT_8_BIT,
                .data_bus_width       = AT91_DATA_BUS_WIDTH_8,
                .data_read_protocol   = AT91_DRP_STANDARD,
                .address_to_cs_setup  = AT91_ACSS_STANDARD,
                .rw_setup             = 2,
                .rw_hold              = 2
        };

        if ( at91_config_smc_cs( &quad_uart_cs_config ) != 0 ) {
                printk( KERN_ERR 
                        "Unable to configure external quad UART chip select "
                        "signal\n" );
                return -EIO;
	}

	quad_uart_virt_base = ioremap_nocache( CCM2200_QUAD_UART_PHYS, 
                                               CCM2200_QUAD_UART_SIZE);
	if (!quad_uart_virt_base ) {
		printk( "ioremap CCM2200 quad UART failed\n" );
		return -EIO;
	}

        memset(serial_platform_data, 0, sizeof(serial_platform_data));
        /* insert virtual base address in serial_platform_data */
        for (port_iter = serial_platform_data, i = 0;
             i < CCM2200_QUAD_UART_NUM_PORTS;
             ++port_iter, ++i) {
                port_iter->membase      = quad_uart_virt_base
                        + i * CCM2200_QUAD_UART_PORT_OFFSET;
                
                port_iter->iobase       = (unsigned) port_iter->membase;
                port_iter->mapbase      = (unsigned) port_iter->membase;
                port_iter->irq          = CCM2200_QUAD_UART_IRQ;
                port_iter->iotype       = UPIO_MEM;
                port_iter->uartclk      = CCM2200_QUAD_UART_CLOCK_RATE;
                port_iter->regshift     = 0;
                port_iter->flags        = UPF_BOOT_AUTOCONF;
        }

        /* configure 18.432 MHz crystal main clock divided by 2 for
         * UART over PCK1 */
#ifdef CONFIG_AT91_PROGRAMMABLE_CLOCKS
        /* 2007-06-12 gc: Support for "A91 Programmable Clocks" feature
         * in Kernels since 2.6.21.
         * Subject for testing! Then remove old code!
         */
        {
                struct clk *clk_pck1 = clk_get(NULL, "pck1");
                struct clk *clk_main = clk_get(NULL, "main");
                if (!clk_pck1 || !clk_main) {
                        printk("ERROR getting PCK1 or Main clock sources\n");
                        return -EIO;
                } else {

                        clk_set_parent(clk_pck1, clk_main);
                        clk_set_rate(clk_pck1, CCM2200_QUAD_UART_CLOCK_RATE);
                        clk_enable(clk_pck1);
                }
        }


#else
        at91rm9200_config_prg_clk_generator(1,
                                            AT91RM9200_MAIN_CLOCK,
                                            //PCK_SELECTED_CLOCK
                                            // 2006-06-26 gc:
                                            // we must use a divider of 2 here
                                            // on account of EMI....
                                            PCK_SELECTED_CLOCK_DIV_2
                );
#endif /* CONFIG_AT91_PROGRAMMABLE_CLOCKS */


        /* switch gpio PIN to the PCK1 output */
        at91_set_B_periph(CCM2200_PIN_PCK1_UART_CLOCK, GPIO_NO_PULLUP);
        /* reset UART */
        gpio_direction_output(CCM2200_PIN_UART_RESET, GPIO_INIT_1);
        udelay(10);
        at91_set_gpio_value(CCM2200_PIN_UART_RESET, 0);

        /* initialize IRQ0 pin */
        gpio_direction_input(CCM2200_PIN_UART_IRQ);
        at91_set_deglitch(CCM2200_PIN_UART_IRQ, 1);

        /* testweise */
	at91_set_A_periph(CCM2200_PIN_UART_IRQ, 1);
        at91_set_gpio_input(CCM2200_PIN_UART_IRQ, 1);
        at91_set_deglitch(CCM2200_PIN_UART_IRQ, 1);

	set_irq_type(CCM2200_QUAD_UART_IRQ, IRQT_HIGH);
/* 	set_irq_type(CCM2200_QUAD_UART_IRQ, IRQT_RISING); */

	platform_device_register(&serial_device);
        printk( "CCM2200 SC16C754 quad UART successfully initialized\n" );

        return 0;
}


static void __exit ccm2200_quad_uart_exit(void)
{
}

module_init(ccm2200_quad_uart_init);
module_exit(ccm2200_quad_uart_exit);

MODULE_AUTHOR("Guido Classen");
MODULE_DESCRIPTION("CCM2200 external quad UART configuration");
MODULE_LICENSE("GPL");

/*
 *Local Variables:
 * mode: c
 * c-file-style: "linux"
 * End:
 */
