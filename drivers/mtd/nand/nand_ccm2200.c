/***********************************************************************
 *
 * linux/drivers/mtd/nand/nand_ccm2200.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * Copyright (C) 2010 by SWARCO Traffic Systems GmbH.
 * All rights reserved.
 *
 * @author:     Guido Classen <guido.classen@swarco.de>
 * @descr:      NAND flash driver for CCM2200
 * @references: [1] u-boot-1.1.0/include/ns9750_nand.h
 *              [2] spia.c
 *              [3] nand.c
 *              [4] nand_a9m9750.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 *  @par Modification History:
 *     2006-04-24 gc: initial version (taken from nand_a9m9750.c)
 ***********************************************************************/

#include <linux/init.h>		/* __init */
#include <linux/module.h>	/* module_init */
#include <linux/mtd/mtd.h>	/* add_mtd_device */
#include <linux/mtd/nand.h>	/* NAND_CMD_*  */
#include <linux/mtd/partitions.h> /* parse_mtd_partitions */

#include <asm/io.h>		/* ioremap */
#include <asm/sizes.h>		/* SZ_ */
#include <asm/delay.h>		/* udelay */
#include <asm/arch/hardware.h>  /* AT91_SYS */
#include <asm/arch/at91-util.h>
#include <asm/arch/board-ccm2200.h>

#include <asm/arch/at91_pio.h>
#include <asm/arch/at91rm9200_mc.h>

static struct mtd_info* ccm2200_mtd = NULL;


#define NAND_FLASH_DAT	(0x0000) /* Data Read/Write */
#define NAND_FLASH_ADR	(0x2000) /* Adr Write */
#define NAND_FLASH_CMD	(0x4000) /* Cmd Write */

#define NAND_IO_ADDR(x)  ((x)->IO_ADDR_W+NAND_FLASH_DAT)
#define NAND_CMD_ADDR(x) ((x)->IO_ADDR_W+NAND_FLASH_CMD)
#define NAND_ADR_ADDR(x) ((x)->IO_ADDR_W+NAND_FLASH_ADR)

#ifdef CONFIG_MTD_PARTITIONS
/*
 * Define static partitions for flash device
 */
static struct mtd_partition partition_info[] = {
        /*
         * {
         *         .name   = "nand-u-boot",
         *         .size   = 256*1024,
         *         .offset = 0,
         *         .mask_flags     = MTD_WRITEABLE,  /\* force read-only *\/
         * },
         */
/* 
 *         {
 *                 .name   = "nand-kernel",
 * //                .size   = (3*1024*1024)-(256*1024),
 * //                .offset = 256*1024,
 *                 .size   = (3*1024*1024),
 *                 .offset = 0,
 *         },
 *         {
 *                 .name   = "nand-rootfs",
 *                 .offset = 3*1024*1024,
 *                 .size   = MTDPART_SIZ_FULL,
 *         },
 */
        {
                .name   = "nand-all", /* for Memory Validation Test */
                .offset = 0,
                .size   = MTDPART_SIZ_FULL,
        }
};

static const char* probes[] = { "cmdlinepart", NULL };
# define NUM_PARTITIONS (sizeof(partition_info) / sizeof(partition_info[0]))
#endif /* CONFIG_MTD_PARTITIONS */

static void ccm2200_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)

{
	register struct nand_chip *this = mtd->priv;

	if (cmd != NAND_CMD_NONE) {

		writeb(cmd, (unsigned long)this->IO_ADDR_W
		       | (ctrl & NAND_CLE ? NAND_FLASH_CMD : 0)
		       | (ctrl & NAND_ALE ? NAND_FLASH_ADR : 0) );
	}
}


/*
 *	read device NAND Ready/_Busy signal using gpio pin
 */
static int ccm2200_nand_device_ready(struct mtd_info *minfo)
{
	//return ((AT91_SYS->PIOA_PDSR & AT91C_PIO_PA19) != 0) ? 1 : 0;
	return ((at91_sys_read(AT91_PIOA + PIO_PDSR) & (1<<19)) != 0) ? 1 : 0;
}

static int __init ccm2200_nand_init( void )
{
	struct nand_chip* this;
	int mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = 0;
	const char *part_type = 0;
	void __iomem *nand_virt_base;
        
        /*
         * Setup static memory controller chip select for NAND Flash
         *
         */
        static const struct at91_smc_cs_info __initdata nand_cs_config = {
                .chip_select          = CCM2200_NAND_FLASH_CS,
                .wait_states          = 5,
                .data_float_time      = 2,
                .byte_access_type     = AT91_BAT_8_BIT,
                .data_bus_width       = AT91_DATA_BUS_WIDTH_8,
                .data_read_protocol   = AT91_DRP_STANDARD,
                .address_to_cs_setup  = AT91_ACSS_STANDARD,
                .rw_setup             = 1,
                .rw_hold              = 1
        };
        printk ( KERN_INFO "CCM2200 NAND driver\n" );
        /* let u-boot do static memory controller initialization */
        if ( at91_config_smc_cs( &nand_cs_config ) != 0 )
	{
	    printk( KERN_ERR 
                    "Unable to configure NAND flash chip select signal\n" );
            return -EIO;
	}

	/* Allocate memory for MTD device structure and private data */
	ccm2200_mtd = kmalloc( sizeof(struct mtd_info)+sizeof(struct nand_chip),
			       GFP_KERNEL );
	if ( !ccm2200_mtd ) {
		printk( "Unable to allocate CCM2200 MTD device structure.\n" );
		return -ENOMEM;
	}

	nand_virt_base = ioremap_nocache( CCM2200_NAND_FLASH_PHYS, 
                                          CCM2200_NAND_FLASH_SIZE );
	if ( !nand_virt_base ) {
		printk( "ioremap CCM2200 NAND flash failed\n" );
		kfree( ccm2200_mtd );
		return -EIO;
	}
	printk("NAND Flash memory mapped to virtual %p\n", nand_virt_base);

	/* Get pointer to private data */
	this = (struct nand_chip *) ( &ccm2200_mtd[ 1 ] );

	/* Initialize structures */
	memset((char *) ccm2200_mtd, 0, sizeof( struct mtd_info ) );
	memset((char *) this, 0, sizeof( struct nand_chip ) );

	ccm2200_mtd->name = "CCM2200";

	/* Link the private data with the MTD structure */
	ccm2200_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = nand_virt_base;
	this->IO_ADDR_W = nand_virt_base;
	/* Set address of hardware control function */
	this->cmd_ctrl = ccm2200_nand_cmd_ctrl;
	this->dev_ready = ccm2200_nand_device_ready;

	/* 15 us command delay time */
	this->chip_delay = 15;
	this->ecc.mode = NAND_ECC_SOFT;

        /* 2006-09-27 gc: do we need this? */
	//this->options = NAND_SAMSUNG_LP_OPTIONS; 


	/* Scan to find existence of the device */
	if ( nand_scan( ccm2200_mtd, 1 ) ) {
                printk ( "CCM2200 NAND chip not found!\n" );
		kfree( ccm2200_mtd );
		return -ENXIO;
	}

	/* Allocate memory for internal data buffer */
/* 	this->data_buf = kmalloc(sizeof(u_char)*(ccm2200_mtd->oobblock+ */
/* 						 ccm2200_mtd->oobsize), */
/* 				 GFP_KERNEL); */
/* 	if ( !this->data_buf ) { */
/* 		printk ( "Unable to allocate NAND data buffer for CCM2200.\n" ); */
/* 		iounmap(  (void*) nand_virt_base ); */
/* 		kfree( ccm2200_mtd ); */
/* 		return -ENOMEM; */
/* 	} */

#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd_parts_nb = parse_mtd_partitions( ccm2200_mtd, probes, &mtd_parts, 0);
	if (mtd_parts_nb > 0)
		part_type = "command line";
	else
		mtd_parts_nb = 0;
#endif
	if (mtd_parts_nb == 0)
	{
		mtd_parts = partition_info;
		mtd_parts_nb = NUM_PARTITIONS;
		part_type = "static";
	}

	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions( ccm2200_mtd, mtd_parts, mtd_parts_nb);

	/* Return happy */
	return 0;
}
device_initcall( ccm2200_nand_init );

static void __exit ccm2200_cleanup( void )
{
	//struct nand_chip *this = (struct nand_chip *) &ccm2200_mtd[1];

	iounmap( (void*) ((struct nand_chip*)ccm2200_mtd[1].priv)->IO_ADDR_R );

	/* Unregister the device */
	del_mtd_device( ccm2200_mtd );

	/* Free internal data buffer */
/* 	kfree( this->data_buf ); */

	/* Free the MTD device structure */
	kfree( ccm2200_mtd );
}
module_exit( ccm2200_cleanup );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Guido Classen <guido.classen@swarco.de" );
MODULE_DESCRIPTION( "Board-specific glue layer for NAND flash on CCM2200 board");
