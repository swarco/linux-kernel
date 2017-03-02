/*
 * linux/arch/arm/mach-at91/at91rm9200_smc.h
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

struct at91rm9200_smc_config {
	/* wait states 0...128 (0 = no wait states) */
	u8 wait_states;
	/* 0...15 wait states after memory read cycle */
	u8 data_float_time;

	/* only valid for data_bus_width=AT91RM9200_DATA_BUS_WIDTH_16 */
	enum byte_access_type {
		/* dummy value for 8 bit device (ignored) */
		AT91RM9200_BAT_8_BIT = 0,
		/* chip select is connected to two/four 8-bit
		 * devices */
		AT91RM9200_BAT_TWO_8_BIT = 0,
		/* chip select is connected too a 16 bit device */
		AT91RM9200_BAT_16_BIT = 1
	} byte_access_type;

	enum data_bus_width {
		AT91RM9200_DATA_BUS_WIDTH_8 = 2,  /* 8 bit data bus */
		AT91RM9200_DATA_BUS_WIDTH_16 = 1, /* 16 bit data bus */
	} data_bus_width;
	enum data_read_protocol {
		AT91RM9200_DRP_STANDARD = 0, /* standard data read protocol */
		AT91RM9200_DRP_EARLY	= 1  /* early data read protocol */
	} data_read_protocol;

	enum address_to_cs_setup {
		/* standard: address asserted at the beginning of the
		 * access and deasserted at the end */
		AT91RM9200_ACSS_STANDARD   = 0,
		/* one cycle less at the beginning and end */
		AT91RM9200_ACSS_1_CYCLE	   = 1,
		/* two cycles less at the beginning and end */
		AT91RM9200_ACSS_2_CYCLES   = 2,
		/* three cycles less at the beginning and end */
		AT91RM9200_ACSS_3_CYCLES   = 3
	} address_to_cs_setup;
	/* 0...7 number of read/write setup cycles */
	u8 rw_setup;
	/* 0...7 number of read/write hold cycles */
	u8 rw_hold;
};

extern int __init at91rm9200_smc_configure(int chip_select,
					   const struct at91rm9200_smc_config *config);
