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
 * @references: [1] based on autcpu12-nvram.c by Thomas Gleixner
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
 *     2006-05-02 gc: initial version (based on autcpu12-nvram.c by Thomas
 *                    Gleixner)
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/hardware.h>

#include <asm/arch/at91-util.h>
#include <asm/arch/board-ccm2200.h>

#include <linux/delay.h>		/* udelay */
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


static struct mtd_info *sram_mtd;

struct map_info ccm2200_sram_map = {
	.name           = "sram-ccm2200",
	.size           = CCM2200_SRAM_SIZE,
	.bankwidth      = 4,    /* use 32bit access for more speed! */
	.phys           = CCM2200_SRAM_PHYS,
};

//#define TEST_SRAM       1       /* test if SRAM timing is okay */

#ifdef TEST_SRAM
static void print(const char *str, void *addr)
{
  u32 *u32ptr = (u32*) addr;
  u8 *u8ptr   = (u8*)  addr;
  u16 *u16ptr = (u16*) addr;
  printk("%s test read u32: %08x - ", str, *u32ptr);
  printk("u16: %04x %04x - ", u16ptr[0], u16ptr[1]);
  printk("u8: %02x %02x %02x %02x\n", 
         u8ptr[0], u8ptr[1], u8ptr[2], u8ptr[3]);
}

static void test32(const char *str, void *addr)
{
  *(u32*)addr = 0x0badaffe;
  printk("test32 ");
  print(str, addr);
}

static void test16(const char *str, void *addr)
{
  u16 *u16ptr = (u16*) addr;
  u16ptr[0] = 0xaffe;
  u16ptr[1] = 0x0bad;
  printk("test16 ");
  print(str, addr);
}


static void test8(const char *str, void *addr)
{
  u8 *u8ptr = (u8*) addr;
  u8ptr[0] = 0xfe;
  u8ptr[1] = 0xaf;
  u8ptr[2] = 0xad;
  u8ptr[3] = 0x0b;
  printk("test8  ");
  print(str, addr);
}
#endif

static int __init init_ccm2200_sram (void)
{
	int err;

        /*
         * Setup static memory controller chip select for SRAM
         *
         */
        static const struct at91_smc_cs_info __initdata sram_cs_config = {
                .chip_select          = CCM2200_SRAM_CS,
                .wait_states          = 15, /* SRAM needs at least 3 wait states! */
                .data_float_time      = 2,
                .byte_access_type     = AT91_BAT_16_BIT,
                .data_bus_width       = AT91_DATA_BUS_WIDTH_16,
                .data_read_protocol   = AT91_DRP_EARLY, //AT91_DRP_STANDARD,
                .address_to_cs_setup  = AT91_ACSS_STANDARD,
                .rw_setup             = 2, /* SRAM needs 1 cycle rw_setup! */
                .rw_hold              = 2  /* SRAM needs 1 cycle rw_hold! */
        };

        if ( at91_config_smc_cs( &sram_cs_config ) != 0 ) {
	    printk( KERN_ERR 
                    "Unable to configure SRAM chip select signal\n" );
            return -EIO;
	}

	ccm2200_sram_map.virt = ioremap_nocache(CCM2200_SRAM_PHYS, 
                                                CCM2200_SRAM_SIZE );
	if (!ccm2200_sram_map.virt) {
		printk("Failed to ioremap CCM2200 SRAM space\n");
		err = -EIO;
		goto out;
	}

#ifdef TEST_SRAM
        /* SRAM timing test
         * test output from correct running SRAM:
test8  x test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b
test16 x test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b
test32 x test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b
test8  sram std test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b
test16 sram std test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b
test32 sram std test read u32: 0badaffe - u16: affe 0bad - u8: fe af ad 0b

         */
        { 
          u32 x;
          test8("x", &x);
          test16("x", &x);
          test32("x", &x);
          test8("sram std", ccm2200_sram_map.virt);
          test16("sram std", ccm2200_sram_map.virt);
          test32("sram std", ccm2200_sram_map.virt);
        }
#endif

	simple_map_init(&ccm2200_sram_map);
	sram_mtd = do_map_probe("map_ram", &ccm2200_sram_map);
	if (!sram_mtd) {
		printk("CCM22200 SRAM probe failed\n");
		err = -ENXIO;
		goto out_ioremap;
	}

	sram_mtd->owner = THIS_MODULE;
	sram_mtd->erasesize = 16;
	
	if (add_mtd_device(sram_mtd)) {
		printk("SRAM device addition failed\n");
		err = -ENOMEM;
		goto out_probe;
	}

	printk("SRAM device size %ldKiB registered on CCM2200\n",
               ccm2200_sram_map.size/SZ_1K);
		
	return 0;

out_probe:
	map_destroy(sram_mtd);
	sram_mtd = 0;

out_ioremap:
	iounmap((void *)ccm2200_sram_map.virt);
out:
	return err;
}

static void __exit cleanup_ccm2200_maps(void)
{
	if (sram_mtd) {
		del_mtd_device(sram_mtd);
		map_destroy(sram_mtd);
		iounmap((void *)ccm2200_sram_map.virt);
	}
}

module_init(init_ccm2200_sram);
module_exit(cleanup_ccm2200_maps);

MODULE_AUTHOR("Guido Classen");
MODULE_DESCRIPTION("CCM2200 SRAM map driver");
MODULE_LICENSE("GPL");
