/*
 * linux/arch/arm/mach-at91/at91rm9200_smc.c
 *
 * Copyright (C) 2010 Guido Classen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <linux/module.h>
#include <linux/io.h>

#include <mach/at91-util.h>
#include <mach/at91rm9200_mc.h>
#include <mach/at91_pio.h>
#include <mach/at91_ramc.h>

#include "mach/at91rm9200_smc.h"

int __init at91rm9200_smc_configure(int chip_select,
				    const struct at91rm9200_smc_config *config)
{
	unsigned int mask;

	/* check parameters */
	if (chip_select < 0 || chip_select > 7
	    || config->wait_states > 128
	    || config->data_float_time > 15
	    || (config->byte_access_type != AT91RM9200_BAT_TWO_8_BIT
		&& config->byte_access_type != AT91RM9200_BAT_16_BIT)
	    || (config->data_read_protocol != AT91RM9200_DRP_STANDARD
		&& config->data_read_protocol != AT91RM9200_DRP_EARLY)
	    || config->address_to_cs_setup < AT91RM9200_ACSS_STANDARD
	    || config->address_to_cs_setup > AT91RM9200_ACSS_3_CYCLES
	    || config->rw_setup > 7
	    || config->rw_hold > 7)
		return -EINVAL;


	// configure gpios, if necessary
	if (chip_select > 3)
	{
#define AT91C_PC10_NCS4_CFCS  (1<<10)
#define AT91C_PC11_NCS5_CFCE1 (1<<11)
#define AT91C_PC12_NCS6_CFCE2 (1<<12)
#define AT91C_PC13_NCS7	      (1<<13)
		switch (chip_select)
		{
		case 4:	mask = AT91C_PC10_NCS4_CFCS;	break;
		case 5:	mask = AT91C_PC11_NCS5_CFCE1;	break;
		case 6:	mask = AT91C_PC12_NCS6_CFCE2;	break;
		case 7:	mask = AT91C_PC13_NCS7;		break;
		default: mask = 0;
		}
		/* select peripheral a function */
		__raw_writel(mask, AT91C_VA_BASE_PIOC + PIO_ASR);


		/* disable pio controller and enable peripheral */
		__raw_writel(mask, AT91C_VA_BASE_PIOC + PIO_PDR);
	}

	/* write the new configuration to SMC chip select register */
	at91_ramc_write(0, AT91_SMC_CSR(chip_select),
			(config->wait_states > 0
			 ? (AT91_SMC_NWS & (config->wait_states-1))
			 | AT91_SMC_WSEN
			 : 0)
			| AT91_SMC_TDF_(config->data_float_time)
			| (config->byte_access_type == AT91RM9200_BAT_16_BIT
			   ? AT91_SMC_BAT : 0)
			| ((int)config->data_bus_width << 13)
			| (config->data_read_protocol == AT91RM9200_DRP_EARLY
			   ? AT91_SMC_DPR : 0)
			| ((int)config->address_to_cs_setup << 16)
			| AT91_SMC_RWSETUP_(config->rw_setup)
			| AT91_SMC_RWHOLD_(config->rw_hold) );
	return 0;
}
