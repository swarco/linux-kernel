/*
 * linux/drivers/mtd/maps/ccm2200-sram.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      MTD mapping and access for non-volatile SRAM from CCM2200 board
 *
 * @references: [1] based on modarm9.c by Markus Pietrek
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
 *     2007-06-04 gc: initial version (based on modarm9.c by Markus Pietrek,
 *                    FS Forth-Systeme GmbH)
 */

#include <linux/autoconf.h>

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <linux/ioport.h>	/* request_mem_region */

#define DRIVER_NAME	"CCM2200"

/* 2006-04-24 gc: support for CCM2200 */
#if defined(CONFIG_MACH_CCM2200)

#include <asm/arch/hardware.h>

#define FLASH_MEM_BASE_P	AT91_CHIPSELECT_0
#define FLASH_SIZE		0x001000000
#define	FLASH_BANK_WIDTH	2

/* 2006-04-24 gc: SWARCO Traffic Systems CCM2200 mapping */
static struct mtd_partition ccm2200_partitions[] = {
	{
		.name       = "all-ccm2200",
		.offset     = 0x00000000,
	},
	{
		.name       = "u-boot",
		.offset     = 0x00000000,
		.size       = 0x00040000,	// 4 * 64kB for u-boot and boot.bin 
	},
	{
		.name       = "kernel",		// default kernel image (1.75MB)
		.offset     = 0x00040000,
		.size       = 0x1c0000,
	},
	{
		.name       = "dummy",		// dummy partion to get nand at MTD5
		.offset     = 0x00200000,
		.size       = 0x00000000,
	},
};

#endif /* defined(CONFIG_MACH_CCM2200) */


static struct map_info ccm2200_map = 
{
	.name      = "ccm2200",
	.phys 	   = FLASH_MEM_BASE_P, 
	.size 	   = FLASH_SIZE, 
	.bankwidth = FLASH_BANK_WIDTH,
};

static struct mtd_partition* parsed_parts;
static const char* part_probes[] = { "cmdlinepart", NULL };
static struct mtd_info* mtd_info;
static struct resource* mtd_res;

static int __init ccm2200_mtd_init( void )
{
	const char* part_type = NULL;
	int nr_parts = 0;
	int ret = 0;

	printk( KERN_INFO DRIVER_NAME
		": Using NOR Flash device: %lu kB @ 0x%x\n",
		ccm2200_map.size / 1024, (unsigned int)ccm2200_map.phys );

	mtd_res = request_mem_region( ccm2200_map.phys, ccm2200_map.size,
				      ccm2200_map.name );
	if( mtd_res == NULL ) {
		ret = -EBUSY;
		goto error;
	}

	ccm2200_map.virt = (void __iomem *) ioremap( ccm2200_map.phys,
						     ccm2200_map.size );
	if( !ccm2200_map.virt ) {
		ret = -ENOMEM;
		goto error_map;
	}
	
	simple_map_init( &ccm2200_map );

	/* probe if there is really flash */

#ifdef CONFIG_MACH_CCXP
	mtd_info = NULL;
#else
	mtd_info = do_map_probe( "amd_flash", &ccm2200_map );
#endif
	if( mtd_info == NULL ) {
		mtd_info = do_map_probe( "cfi_probe", &ccm2200_map );
		if( mtd_info == NULL ) {
			ret = -ENXIO;
			goto error_probe;
		}
	}
	
	mtd_info->owner = THIS_MODULE;

	/* setup partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nr_parts = parse_mtd_partitions( mtd_info, part_probes,
					 &parsed_parts, 0 );
	if( nr_parts > 0 )
		part_type = "dynamic";
	else {
		parsed_parts = ccm2200_partitions;
		nr_parts = ARRAY_SIZE( ccm2200_partitions );
		part_type = "static";
	}
#endif

	if( nr_parts == 0 ) {
		printk( KERN_INFO DRIVER_NAME ": no partition info "
			"available, registering whole flash\n" );
		add_mtd_device( mtd_info );
	} else {
		printk( KERN_INFO DRIVER_NAME ": using %s partition "
			"definition\n", part_type );
		add_mtd_partitions( mtd_info, parsed_parts, nr_parts );
	}

	return 0;

error_probe:
	iounmap( ccm2200_map.virt );

error_map:
	release_resource( mtd_res );

error:
	return ret;
}

static void __exit ccm2200_mtd_cleanup( void )
{

	del_mtd_partitions( mtd_info );

	if( parsed_parts )
		kfree( parsed_parts );

	iounmap( ccm2200_map.virt );

	release_resource( mtd_res );

	parsed_parts = NULL;
}

module_init( ccm2200_mtd_init );
module_exit( ccm2200_mtd_cleanup );

MODULE_AUTHOR( "Guido Classen <guido.classen@swarco.de>" );
MODULE_DESCRIPTION( "CCM2200 NOR flash mapping driver" );
MODULE_LICENSE( "GPL" );
