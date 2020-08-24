/*
 * Micrel KSZ8795 / KSZ8765 switch using SPI
 *
 * Copyright (C) 2011-2013 Jonas Gorski <jogo@openwrt.org>
 * Copyright (C) 2016 Karl Olsen <karl@micro-technic.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <asm/unaligned.h>

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>
#include <linux/phy.h>
#include <linux/gpio/consumer.h>
#include <net/dsa.h>
#include "ksz8795.h"

#define SPI_CMD_READ	0x60
#define SPI_CMD_WRITE	0x40

#define PORTS		4	/* Downstream ports, excluding the CPU port */


struct ksz_port {
	int fiber;	/* 1 = fiber, 0 = copper */
};

struct ksz_device {
	struct dsa_switch *ds;
	struct spi_device *priv;
	struct device *dev;
	struct mutex reg_mutex;
	struct gpio_desc *reset_gpio;
	struct ksz_port port[PORTS];
	struct bin_attribute    tailtag_attr;

	unsigned int tail_tagging_enabled;
	const struct dsa_device_ops * no_tail_tagging_ops;
};


static inline int ksz_spi_read_raw(struct ksz_device *kszd, u8 reg, u8 *data,
				   unsigned int len)
{
	struct spi_device *spi = kszd->priv;
	u8 txbuf[2];

	/* Command (RD=011, 4 dummy bits, 8 address bits, 1 turnaround bit */
	txbuf[0] = SPI_CMD_READ | (reg >> 7);
	txbuf[1] = reg << 1;

	return spi_write_then_read(spi, txbuf, 2, data, len);
}


static int ksz_spi_write8_raw(struct ksz_device *kszd, u8 reg, u8 value)
{
	struct spi_device *spi = kszd->priv;
	u8 txbuf[3];
	
	/* Command (WR=010, 4 dummy bits, 8 address bits, 1 turnaround bit */
	txbuf[0] = SPI_CMD_WRITE | (reg >> 7);
	txbuf[1] = reg << 1;
	txbuf[2] = value;
	return spi_write(spi, txbuf, sizeof(txbuf));
}


static int ksz_spi_write16_raw(struct ksz_device *kszd, u8 reg, u8 value0, u8 value1)
{
	struct spi_device *spi = kszd->priv;
	u8 txbuf[4];
	
	/* Command (WR=010, 4 dummy bits, 8 address bits, 1 turnaround bit */
	txbuf[0] = SPI_CMD_WRITE | (reg >> 7);
	txbuf[1] = reg << 1;
	txbuf[2] = value0;
	txbuf[3] = value1;
	return spi_write(spi, txbuf, sizeof(txbuf));
}


static int ksz_spi_read8(struct ksz_device *kszd, u8 reg, u8 *val)
{
	int ret;
	
	mutex_lock(&kszd->reg_mutex);
	ret = ksz_spi_read_raw(kszd, reg, val, 1);
	mutex_unlock(&kszd->reg_mutex);
	
	return ret;
}


static int ksz_spi_write8(struct ksz_device *kszd, u8 reg, u8 value)
{
	int ret;
	
	mutex_lock(&kszd->reg_mutex);
	ret = ksz_spi_write8_raw(kszd, reg, value);
	mutex_unlock(&kszd->reg_mutex);
	
	return ret;
}


static int ksz_spi_rmw8(struct ksz_device *kszd, u8 reg, u8 and, u8 or)
{
	struct spi_device *spi = kszd->priv;
	u8 txbuf[3];
	int ret;

	mutex_lock(&kszd->reg_mutex);
	txbuf[0] = SPI_CMD_WRITE | (reg >> 7);
	txbuf[1] = reg << 1;
	ret = ksz_spi_read_raw(kszd, reg, &txbuf[2], 1);
	if (!ret) {
		txbuf[2] &= and;
		txbuf[2] |= or;
		ret = spi_write(spi, txbuf, sizeof(txbuf));
	}
	mutex_unlock(&kszd->reg_mutex);

	return ret;
}


static int ksz_write_aclreg_raw(struct ksz_device *kszd, int port, u8 aclreg, u8 val)
{
	int ret;
	
	ret = ksz_spi_write16_raw(kszd, REG_IND_CTRL_0, TABLE_ACL | (port+1), aclreg);
	if (ret)
		return ret;
	ret = ksz_spi_write8_raw(kszd, REG_IND_DATA_PME_EEE_ACL, val);
	return ret;
}


static int ksz_read_aclreg_raw(struct ksz_device *kszd, int port, u8 aclreg, u8 *val)
{
	int ret;
	
	ret = ksz_spi_write16_raw(kszd, REG_IND_CTRL_0, TABLE_ACL | TABLE_READ | (port+1), aclreg);
	if (ret)
		return ret;
	ret = ksz_spi_read_raw(kszd, REG_IND_DATA_PME_EEE_ACL, val, 1);
	return ret;
}


static int ksz_write_acl(struct ksz_device *kszd, int port, int entry, u8 acl[14])
{
	int i;
	int ret;
	u8 val;
	
	mutex_lock(&kszd->reg_mutex);
	
	/* We want to write all 14 bytes. Bitmask = 0x3FFF. */
	ret = ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_BYTE_EN_MSB, 0x3F);
	if (ret)
		goto out;
	ret = ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_BYTE_EN_LSB, 0xFF);
	if (ret)
		goto out;
		
	/* Write the 14 bytes to the "temporal storage for 14 bytes ACL rules" */
	for (i=0; i < 14; i++) {
		ret = ksz_write_aclreg_raw(kszd, port, i, acl[i]);
		if (ret)
			goto out;
	}
	/* Write the "temporary storage" to the right ACL entry (0-15) */
	ret = ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_CTRL_0, PORT_ACL_WRITE | entry);
	if (ret)
		goto out;
		
	/* Wait until the write completes */
	i = 0;
	do {
		ret = ksz_read_aclreg_raw(kszd, port, REG_PORT_ACL_CTRL_0, &val);
		if (ret)
			goto out;
		if (++i >= 10) {
			dev_err(kszd->ds->dev, "ksz_write_acl: Write timeout, port %d entry %d\n",
				port, entry);
			ret = -EIO;
			goto out;
		}
	} while (!(val & PORT_ACL_WRITE_DONE));

out:
	mutex_unlock(&kszd->reg_mutex);
	
	return ret;
}


#if 0
static void ksz_dump_acl(struct ksz_device *kszd, int port, int entry)
{
	int i;
	u8 val;
	u8 acl[14];
	
	mutex_lock(&kszd->reg_mutex);

	/* We want to read all 14 bytes. Bitmask = 0x3FFF. */
	if (ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_BYTE_EN_MSB, 0x3F))
		goto out;
	if (ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_BYTE_EN_LSB, 0xFF))
		goto out;
	/* Read 14 bytes from the ACL rule to the "temporary storage" */
	if (ksz_write_aclreg_raw(kszd, port, REG_PORT_ACL_CTRL_0, entry))
		goto out;
	/* Wait until read operation complete */
	i = 0;
	do {
		if (ksz_read_aclreg_raw(kszd, port, REG_PORT_ACL_CTRL_0, &val))
			goto out;
		if (++i >= 10) {
			dev_err(kszd->ds->dev, "ksz_dump_acl: Read timeout, port %d entry %d\n",
				port, entry);
			goto out;
		}
	} while (!(val & PORT_ACL_READ_DONE));
	/* Read the 14 bytes out of the "temporary storage" */
	for (i=0; i < 14; i++) {
		if (ksz_read_aclreg_raw(kszd, port, i, &acl[i]))
			goto out;
	}
	printk(KERN_INFO "ACL port %d, entry %2d: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		port, entry,
		acl[0], acl[1], acl[2], acl[3],  acl[4],  acl[5],  acl[6],
		acl[7], acl[8], acl[9], acl[10], acl[11], acl[12], acl[13]);
out:
	mutex_unlock(&kszd->reg_mutex);
}
#endif


static inline u8 ksz_portreg(int port, int ofs)
{
	return port*0x10 + ofs;
}

static enum dsa_tag_protocol ksz_get_tag_protocol(struct dsa_switch *ds)
{
    struct ksz_device *kszd = ds->priv;

    if (kszd->tail_tagging_enabled)
        return DSA_TAG_PROTO_MICRELSW;
    else
        return DSA_TAG_PROTO_NONE;
}

/* tag_micrelsw.c */
extern const struct dsa_device_ops micrelsw_netdev_ops;

static int ksz_setup(struct dsa_switch *ds)
{
	struct ksz_device *kszd = ds->priv;
	u8 acl[14];
	int port;
	int rule;
	int ret;
	
	dev_info(ds->dev, "Configuring switch\n");
	
	/* Don't drop incoming packets larger than 1518 bytes (1522 with
	 * tag). Tail tagged packets from the CPU can be 1519/1523 bytes. */
	ret = ksz_spi_rmw8(kszd, REG_SW_CTRL_2, 0xFF, SW_LEGAL_PACKET_DISABLE);
	if (ret)
		return ret;

    /* disable tail tagging mode, use with DSA_TAG_PROTO_NONE */
    ret = ksz_spi_rmw8(kszd, REG_SW_CTRL_10, !SW_TAIL_TAG_ENABLE, 0);
    if (ret)
        return ret;

	/* For each port, define an ACL rule, rule 0, that restricts all
	 * forwarding to our own port and the CPU port. This rule should
	 * match all packets. */
	for (port=0; port < PORTS; port++)
	{
		dev_info(ds->dev, "Configuring port %d\n", port);
			
		/* Disable rule 0-15 */
		memset (acl, 0, sizeof(acl));
		for (rule=0; rule < 16; rule++) {
			ret = ksz_write_acl(kszd, port, rule, acl);
			if (ret)
				return ret;
		}

        /* Disable ACL for this port */
		ret = ksz_spi_rmw8(kszd, ksz_portreg(port, REG_PORT_1_CTRL_5), !PORT_ACL_ENABLE, 0);
		if (ret)
			return ret;
	}
	
	kszd->no_tail_tagging_ops = ds->dst->tag_ops;

	return 0;
}

static int ksz_disable_tail_tagging(struct dsa_switch *ds)
{
    struct ksz_device *kszd = ds->priv;
    u8 acl[14];
    int port;
    int rule;
    int ret;

    dev_info(ds->dev, "Disabling tail tagging, restoring normal switch operation\n");


    /* disable tail tagging mode, use with DSA_TAG_PROTO_NONE */
    ret = ksz_spi_rmw8(kszd, REG_SW_CTRL_10, !SW_TAIL_TAG_ENABLE, 0);
    if (ret)
        return ret;

    /* For each port, disabling ACL */
    for (port=0; port < PORTS; port++)
    {
        dev_info(ds->dev, "Disabling ACL forwarding rule for port %d\n", port);

        /* Disable rule 0-15 */
        memset (acl, 0, sizeof(acl));
        for (rule=0; rule < 16; rule++) {
            ret = ksz_write_acl(kszd, port, rule, acl);
            if (ret)
                return ret;
        }

        /* Disable ACL for this port */
        ret = ksz_spi_rmw8(kszd, ksz_portreg(port, REG_PORT_1_CTRL_5), !PORT_ACL_ENABLE, 0);
        if (ret)
            return ret;
    }

    return 0;
}

static int ksz_enable_tail_tagging(struct dsa_switch *ds)
{
    struct ksz_device *kszd = ds->priv;
    u8 acl[14];
    int port;
    int rule;
    int ret;

    dev_info(ds->dev, "Enabling tail tagging, suspending normal switch operation\n");

    /* Enable tail tagging mode, use with DSA_TAG_PROTO_MICRELSW */
    ret = ksz_spi_rmw8(kszd, REG_SW_CTRL_10, 0xFF, SW_TAIL_TAG_ENABLE);
    if (ret)
        return ret;

    /* For each port, define an ACL rule, rule 0, that restricts all
     * forwarding to our own port and the CPU port. This rule should
     * match all packets. */
    for (port=0; port < PORTS; port++)
    {
        dev_info(ds->dev, "Enabling ACL forwarding rule for port %d\n", port);

        /* FRN = 0 (this rule) */
        acl[0] = 0;

        /* Rule enabled. There does not seem to be a "match always"
         * condition. Match if destination MAC address not equal to
         * 58:e0:2c:00:2f:05. I own this MAC address, and I have
         * reserved it for this purpose.
         * Note: Don't use the ACL_ENABLE_2_MAC #define, it is wrong
         * in ksz8795.h. */
        acl[1] = 0x14;  /* MD=01, ENB=01, SD=0, EQ=0 */
        acl[2] = 0x58;
        acl[3] = 0xe0;
        acl[4] = 0x2c;
        acl[5] = 0x00;
        acl[6] = 0x2f;
        acl[7] = 0x05;
        acl[8] = 0x00;  /* EtherType MSB, not used in comparison */
        acl[9] = 0x00;  /* Ethertype LSB, not used in comparison */

        /* Action: Map mode = the map in FORWARD field replaces the
         * forwarding map from the look-up table. Regardless of learned
         * MAC addresses, send this packet to the CPU port and nowhere
         * else. */
        acl[10] = 0x00; /* No priority change, no VLAN priority remarking */
        acl[11] = (ACL_MAP_MODE_REPLACE << ACL_MAP_MODE_S) | (1 << 4);

        /* RULESET: Only this rule (rule 0) in the ruleset. */
        acl[12] = 0x00; /* RULESET MSB */
        acl[13] = 0x01; /* RULESET LSB */

        ret = ksz_write_acl(kszd, port, 0, acl);
        if (ret)
            return ret;

        /* Disable rule 1-15 */
        memset (acl, 0, sizeof(acl));
        for (rule=1; rule < 16; rule++) {
            ret = ksz_write_acl(kszd, port, rule, acl);
            if (ret)
                return ret;
        }

        /* Enable ACL for this port */
        ret = ksz_spi_rmw8(kszd, ksz_portreg(port, REG_PORT_1_CTRL_5), 0xFF, PORT_ACL_ENABLE);
        if (ret)
            return ret;
    }

    return 0;
}

static int ksz_set_addr(struct dsa_switch *ds, u8 *addr)
{
	return 0;
}


static int ksz_phy_read(struct dsa_switch *ds, int addr, int reg)
{
	struct ksz_device *kszd = ds->priv;
	int ret;
	u8 portctrl9;
	u8 portstatus2;
	u8 portctrl7;
	u8 portstatus0;
	
	switch (reg) {
	case MII_BMCR:
		ret = ksz_spi_read8(kszd, ksz_portreg(addr, REG_PORT_1_CTRL_9), &portctrl9);
		if (ret)
			break;
		ret = 0;
		if (portctrl9 & PORT_FORCE_100_MBIT)
			ret |= BMCR_SPEED100;
		if (!(portctrl9 & PORT_AUTO_NEG_DISABLE))
			ret |= BMCR_ANENABLE;
		if (portctrl9 & PORT_FORCE_FULL_DUPLEX)
			ret |= BMCR_FULLDPLX;
		dev_dbg(ds->dev, "ksz_phy_read BMCR[%d] = 0x%04x\n", addr, ret);
		break;
		
	case MII_BMSR:
		ret = ksz_spi_read8(kszd, ksz_portreg(addr, REG_PORT_1_STATUS_2), &portstatus2);
		if (ret)
			break;
		ret = BMSR_100FULL | BMSR_100HALF | BMSR_10FULL | BMSR_10HALF | BMSR_ANEGCAPABLE;
		if (portstatus2 & PORT_AUTO_NEG_COMPLETE)
			ret |= BMSR_ANEGCOMPLETE;
		if (portstatus2 & PORT_STAT_LINK_GOOD)
			ret |= BMSR_LSTATUS;
		dev_dbg(ds->dev, "ksz_phy_read BMSR[%d] = 0x%04x\n", addr, ret);
		break;
		
	case MII_PHYSID1:
		/* When using the MDIO interface the KSZ8795 returns
		 * MII_PHYSID1 = 0x0022 and MII_PHYSID2 = 0x1550. If we do that
		 * here, the kernel would detect us as a KSZ8051 which also has
		 * these IDs. Instead invent something so that we are detected
		 * as a "Generic PHY". */
		ret = 0xfede;	/* was 0x0022 */
		break;
		
	case MII_PHYSID2:
		ret = 0xebbe;	/* was 0x1550 */
		break;
		
	case MII_ADVERTISE:
		ret = ksz_spi_read8(kszd, ksz_portreg(addr, REG_PORT_1_CTRL_7), &portctrl7);
		if (ret)
			break;
		ret = ADVERTISE_CSMA;
		if (portctrl7 & PORT_AUTO_NEG_10BT)
			ret |= ADVERTISE_10HALF;
		if (portctrl7 & PORT_AUTO_NEG_10BT_FD)
			ret |= ADVERTISE_10FULL;
		if (portctrl7 & PORT_AUTO_NEG_100BTX)
			ret |= ADVERTISE_100HALF;
		if (portctrl7 & PORT_AUTO_NEG_100BTX_FD)
			ret |= ADVERTISE_100FULL;
		if (portctrl7 & PORT_AUTO_NEG_SYM_PAUSE)
			ret |= ADVERTISE_PAUSE_CAP;
		if (portctrl7 & PORT_AUTO_NEG_ASYM_PAUSE)
			ret |= ADVERTISE_PAUSE_ASYM;
		dev_dbg(ds->dev, "ksz_phy_read ADVERTISE[%d] = 0x%04x\n", addr, ret);
		break;
		
	case MII_LPA:
		ret = ksz_spi_read8(kszd, ksz_portreg(addr, REG_PORT_1_STATUS_0), &portstatus0);
		if (ret)
			break;
		ret = ADVERTISE_CSMA;
		if (portstatus0 & PORT_REMOTE_10BT)
			ret |= LPA_10HALF;
		if (portstatus0 & PORT_REMOTE_10BT_FD)
			ret |= LPA_10FULL;
		if (portstatus0 & PORT_REMOTE_100BTX)
			ret |= LPA_100HALF;
		if (portstatus0 & PORT_REMOTE_100BTX_FD)
			ret |= LPA_100FULL;
		if (portstatus0 & PORT_REMOTE_SYM_PAUSE)
			ret |= LPA_PAUSE_CAP;
		if (portstatus0 & PORT_REMOTE_ASYM_PAUSE)
			ret |= LPA_PAUSE_ASYM;
		dev_dbg(ds->dev, "ksz_phy_read LPA[%d] = 0x%04x\n", addr, ret);
		break;

	default:
		dev_info(ds->dev, "ksz_phy_read[%d] unimplemented reg 0x%x\n", addr, reg);
		ret = -EINVAL;
		break;
	}
	return ret;
}


static int ksz_phy_write(struct dsa_switch *ds, int addr, int reg, u16 val)
{
	struct ksz_device *kszd = ds->priv;
	int ret = 0;
	u8 portctrl7;
	u8 portctrl9;
	
	switch(reg) {
	case MII_BMCR:
		dev_dbg(ds->dev, "ksz_phy_write BMCR[%d] = 0x%04x\n", addr, val);
		portctrl9 = 0;
		if (val & BMCR_FULLDPLX)
			portctrl9 |= PORT_FORCE_FULL_DUPLEX;
		if (!(val & BMCR_ANENABLE))
			portctrl9 |= PORT_AUTO_NEG_DISABLE;
		if (val & BMCR_SPEED100)
			portctrl9 |= PORT_FORCE_100_MBIT;
		ret = ksz_spi_write8(kszd, ksz_portreg(addr, REG_PORT_1_CTRL_9), portctrl9);
		if (ret)
			break;
		if (val & BMCR_RESET) {
			/* Write 1 to the self-clearing bit, preserving the
			 * other bits in the register */
			dev_dbg(ds->dev, "- PHY soft reset\n");
			ret = ksz_spi_rmw8(kszd, ksz_portreg(addr, REG_PORT_1_STATUS_3), 0xFF, PORT_PHY_SOFT_RESET);
			if (ret)
				break;
		}
		if (val & BMCR_ANRESTART) {
			/* Write 1 to the self-clearing bit, preserving the
			 * other bits in the register */
			dev_dbg(ds->dev, " - PHY restart autonegotiation\n");
			ret = ksz_spi_rmw8(kszd, ksz_portreg(addr, REG_PORT_1_CTRL_10), 0xFF, PORT_AUTO_NEG_RESTART);
			if (ret)
				break;
		}
		break;
		
	case MII_ADVERTISE:
		dev_dbg(ds->dev, "ksz_phy_write ADVERTISE[%d] = 0x%04x\n", addr, val);
		portctrl7 = 0;
		if (val & ADVERTISE_10HALF)
			portctrl7 |= PORT_AUTO_NEG_10BT;
		if (val & ADVERTISE_10FULL)
			portctrl7 |= PORT_AUTO_NEG_10BT_FD;
		if (val & ADVERTISE_100HALF)
			portctrl7 |= PORT_AUTO_NEG_100BTX;
		if (val & ADVERTISE_100FULL)
			portctrl7 |= PORT_AUTO_NEG_100BTX_FD;
		if (val & ADVERTISE_PAUSE_CAP)
			portctrl7 |= PORT_AUTO_NEG_SYM_PAUSE;
		if (val & ADVERTISE_PAUSE_ASYM)
			portctrl7 |= PORT_AUTO_NEG_ASYM_PAUSE;
		ret = ksz_spi_write8(kszd, ksz_portreg(addr, REG_PORT_1_CTRL_7), portctrl7);
		break;
		
	default:
		dev_info(ds->dev, "ksz_phy_write[%d] unimplemented reg 0x%x = 0x%04x\n", addr, reg, val);
		break;
	}
	return ret;
}


static void ksz_fixed_link_update(struct dsa_switch *ds, int port, struct fixed_phy_status *status)
{
	struct ksz_device *kszd = ds->priv;
	u8 portstatus2;
	int ret;
	
	/* We are also called if link-gpios= is specified in devicetree. In
	 * that case, the GPIO result takes precedence. */
	ret = ksz_spi_read8(kszd, ksz_portreg(port, REG_PORT_1_STATUS_2), &portstatus2);
	if (!ret) {
		status->link = (portstatus2 & PORT_STAT_LINK_GOOD) != 0;
		dev_dbg(ds->dev, "ksz_fixed_link_update for port %d: link=%d\n", port, status->link);
	}
}


struct ksz_mib {
	const char *name;
};

static const struct ksz_mib ksz_port_mib[] = {
	{ "RxHiPriorityByte",	},
	{ "RxUndersizePkt",	},
	{ "RxFragments",	},
	{ "RxOversize",		},
	{ "RxJabbers",		},
	{ "RxSymbolErrors",	},
	{ "RxCRCerror",		},
	{ "RxAlignmentError",	},
	{ "RxControl8808Pkts",	},
	{ "RxPausePkts",	},
	{ "RxBroadcast",	},
	{ "RxMulticast",	},
	{ "RxUnicast",		},
	{ "Rx64Octets",		},
	{ "Rx65to127Octets",	},
	{ "Rx128to255Octets",	},
	{ "Rx256to511Octets",	},
	{ "Rx512to1023Octets",	},
	{ "Rx1024to1522Octets",	},
	{ "Rx1523to2000Octets",	},
	{ "Rx2001toMax-1Octets",},
	{ "TxHiPriorityByte",	},
	{ "TxLateCollision",	},
	{ "TxPausePkts",	},
	{ "TxBroadcastPkts",	},
	{ "TxMulticastPkts",	},
	{ "TxUnicastPkts",	},
	{ "TxDeferred",		},
	{ "TxTotalCollision",	},
	{ "TxExcessiveCollision",},
	{ "TxSingleCollision",	},
	{ "TxMultipleCollision",},
};

static const struct ksz_mib ksz_allport_mib[] = {
	{ "RxTotalBytes",	},
	{ "TxTotalBytes",	},
	{ "RxDropPackets",	},
	{ "TxDropPackets",	},
};


static void ksz_read_mib_raw(struct ksz_device *kszd, int mibreg, uint64_t *value)
{
	int ret;
	uint8_t indctrl0 = TABLE_MIB | TABLE_READ | (mibreg >> 8);
	uint8_t indctrl1 = mibreg & 0xFF; 
	uint8_t dat[5];
	
	ret = ksz_spi_write16_raw(kszd, REG_IND_CTRL_0, indctrl0, indctrl1);
	if (ret) {
		*value = 0;
		return;
	}
	/* Read data 4-0 (bit 39 - bit 0) */
	ret = ksz_spi_read_raw(kszd, REG_IND_DATA_4, dat, 5);
	if (ret) {
		*value = 0;
		return;
	}
	switch (dat[0] & 0x60) {
	case 0x00: /* No overflow, counter value not valid */
		dev_info(kszd->dev, "ksz_read_mib_raw: MIB 0x%03x invalid\n", mibreg);
		break;
		
	case 0x20: /* No overflow, counter value is valid */
		break;
		
	case 0x40: /* Overflow, counter value not valid */
		dev_info(kszd->dev, "ksz_read_mib_raw: MIB 0x%03x invalid+overflow\n", mibreg);
		break;
		
	case 0x60: /* Overflow, counter value is valid */
		dev_info(kszd->dev, "ksz_read_mib_raw: MIB 0x%03x overflow\n", mibreg);
		break; 
	}
	*value = ((uint64_t)(dat[0] & 0x1F) << 32) |
		 ((uint64_t)(dat[1]) << 24) |
		 ((uint64_t)(dat[2]) << 16) |
		 ((uint64_t)(dat[3]) << 8)  |
		 ((uint64_t)(dat[4]) << 0);
}


#if 0
static int ksz_read_dynmac_entry_raw(struct ksz_device *kszd, int dynmacreg, u8 data[9])
{
	int ret;
	uint8_t indctrl0 = TABLE_DYNAMIC_MAC | TABLE_READ | (dynmacreg >> 8);
	uint8_t indctrl1 = dynmacreg & 0xFF; 
	
	ret = ksz_spi_write16_raw(kszd, REG_IND_CTRL_0, indctrl0, indctrl1);
	if (ret)
		return ret;
	ret = ksz_spi_read_raw(kszd, REG_IND_DATA_8, data, 9);
	if (ret)
		return ret;
	if (data[2] & 0x80) {
		dev_warn(kszd->dev, "ksz_read_dynmac_entry_raw entry %d: Not ready, retrying\n", dynmacreg);
		ret = ksz_spi_read_raw(kszd, REG_IND_DATA_8, data, 9);
		if (ret)
			return ret;
		if (data[2] & 0x80) {
			dev_err(kszd->dev, "ksz_read_dynmac_entry_raw entry %d: Still not ready, giving up\n", dynmacreg);
			return -EIO;
		}
	}
	return 0;
}


static void ksz_dump_dynmac_raw(struct ksz_device *kszd)
{
	u8 data[9];
	unsigned lastentry;
	unsigned entry;
	
	entry = 0;
	while (1) {
		if (ksz_read_dynmac_entry_raw(kszd, entry, data))
			break;
		if (data[0] & 0x80) {
			printk(KERN_INFO "KSZ8795 dynamic MAC table empty\n");
			break;
		}
		lastentry = (data[1] >> 5) | (data[0] << 3);
		if (entry > lastentry)
			break;
		printk(KERN_INFO "KSZ8795 dynamic MAC entry%4u: ts=%u port=%u fid=0x%02x mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
			entry,
			(data[1] >> 3) & 0x03,		/* Timestamp 0-3 */
			(data[1] >> 0) & 0x07,		/* Port 0-4 */
			(data[2] >> 0) & 0x7F,		/* FID 00-7F */
			data[3], data[4], data[5], data[6], data[7], data[8]);	/* MAC */
		entry++;
	}
}
#endif


static void ksz_get_strings(struct dsa_switch *ds, int port, uint8_t *data)
{
	unsigned i, j;
	
	j = 0;
	for (i=0; i < ARRAY_SIZE(ksz_port_mib); i++, j++)
		memcpy(data + j * ETH_GSTRING_LEN, ksz_port_mib[i].name, ETH_GSTRING_LEN);
	for (i=0; i < ARRAY_SIZE(ksz_allport_mib); i++, j++)
		memcpy(data + j * ETH_GSTRING_LEN, ksz_allport_mib[i].name, ETH_GSTRING_LEN);
}


static void ksz_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data)
{
	struct ksz_device *kszd = ds->priv;
	unsigned i, j;
	
	mutex_lock(&kszd->reg_mutex);
	
	j = 0;
	for (i=0; i < ARRAY_SIZE(ksz_port_mib); i++)
		ksz_read_mib_raw(kszd, (port<<5) + i, &data[j++]);
	for (i=0; i < ARRAY_SIZE(ksz_allport_mib); i++)
		ksz_read_mib_raw(kszd, 0x100 + (port<<2) + i, &data[j++]);

	mutex_unlock(&kszd->reg_mutex);
}


static int ksz_get_sset_count(struct dsa_switch *ds)
{
	return ARRAY_SIZE(ksz_port_mib) + ARRAY_SIZE(ksz_allport_mib);
}


static ssize_t ksz_tailtagging_read(struct file *filp, struct kobject *kobj,
    struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
    struct device *dev;
    struct ksz_device *kszd;

    dev = container_of(kobj, struct device, kobj);
    kszd = dev_get_drvdata(dev);

    if (!kszd->tail_tagging_enabled)
        buf[0] = '0';
    else if (kszd->tail_tagging_enabled)
        buf[0] = '1';
    else
        // todo: error
        buf[0] = '0';

    buf[1] = '\n';
    buf[2] = '\0';
    count = 3;
    return count;
}

static ssize_t ksz_tailtagging_write(struct file *filp, struct kobject *kobj,
    struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
    struct device *dev;
    struct ksz_device *kszd;

    dev = container_of(kobj, struct device, kobj);
    kszd = dev_get_drvdata(dev);

    if ((buf[0] == '1') && (!kszd->tail_tagging_enabled))
    {
        if (ksz_enable_tail_tagging(kszd->ds) == 0)
        {
            kszd->tail_tagging_enabled = 1;
            dsa_update_tagging_protocol(kszd->ds);
        }
        else
        {
            // todo: error
            ksz_disable_tail_tagging(kszd->ds);
        }
    }
    else if ((buf[0] == '0') && (kszd->tail_tagging_enabled))
    {
        if (ksz_disable_tail_tagging(kszd->ds) == 0)
        {
            kszd->tail_tagging_enabled = 0;
            dsa_update_tagging_protocol(kszd->ds);
        }
        else
        {
            // todo: error
            ksz_enable_tail_tagging(kszd->ds);
        }
    }
    else
    {
        // todo: error
        // do nothing
    }

    return count;
}

static struct dsa_switch_ops ksz_switch_ops = {
	.get_tag_protocol   = ksz_get_tag_protocol,
	.setup			= ksz_setup,
	.set_addr		= ksz_set_addr,
	.phy_read		= ksz_phy_read,
	.phy_write		= ksz_phy_write,
	.fixed_link_update	= ksz_fixed_link_update,
	.get_strings		= ksz_get_strings,
	.get_ethtool_stats	= ksz_get_ethtool_stats,
	.get_sset_count		= ksz_get_sset_count,
};

static const struct bin_attribute ksz_tailtagging_attr = {
    .attr = {
        .name   = "tailtagging",
        .mode   = S_IRUSR | S_IWUSR,
    },
    .size   = 3,
    .read   = ksz_tailtagging_read,
    .write  = ksz_tailtagging_write,
};

static int ksz_spi_probe(struct spi_device *spi)
{
	struct dsa_switch *ds;
	struct ksz_device *kszd;
	int ret;
	int i;
	u8 val;

	dev_info(&spi->dev, "ksz_spi_probe\n");
	ds = devm_kzalloc(&spi->dev, sizeof(*ds) + sizeof(*kszd), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;
		
	kszd = (struct ksz_device *)(ds + 1);
	ds->priv = kszd;
	ds->dev = &spi->dev;
	ds->ops = &ksz_switch_ops;
	kszd->dev = &spi->dev;
	kszd->ds = ds;
	kszd->priv = spi;
	kszd->tail_tagging_enabled = 1; // default operation set to tailtagging
	mutex_init(&kszd->reg_mutex);
	
	/* If a reset GPIO ("reset-gpio" or "reset-gpios") is defined, set it
	 * active and then inactive. */
	kszd->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(kszd->reset_gpio))
		return PTR_ERR(kszd->reset_gpio);
	else if (kszd->reset_gpio) {
		msleep(10);
		gpiod_set_value(kszd->reset_gpio, 0);
		msleep(10);
	}
	
	ret = ksz_spi_read8(kszd, REG_CHIP_ID0, &val);
	if (ret)
		return ret;
	if (val != FAMILY_ID) {
		dev_err(&spi->dev, "Error: KSZ8795 not found. ID0=0x%02x, expected 0x%02x\n", val, FAMILY_ID);
		return -EIO;
	}
	/* Reset default of KSZ8795 RGMII port is RGMII-TXID (enable Internal
	 * Delay egress, disable on ingress. With this, transmit doesn't work
	 * with the SAMA5D3 GMAC RGMII. We need RGMII-ID (Internal Delay on
	 * both RX and TX). */
	ret = ksz_spi_rmw8(kszd, REG_PORT_5_CTRL_6, 0xFF,
			(PORT_RGMII_ID_IN_ENABLE | PORT_RGMII_ID_OUT_ENABLE));
	if (ret)
		return ret;
		
	for (i=0; i < PORTS; i++) {
		struct ksz_port *port = &kszd->port[i];
		
		ret = ksz_spi_read8(kszd, ksz_portreg(i, REG_PORT_1_STATUS_0), &val);
		if (ret)
			return ret;
		port->fiber = (val & 0x80) != 0;
		dev_info(&spi->dev, "Port %d is a %s port.\n", i, port->fiber ? "fiber" : "copper");
	}
	
	ret = dsa_register_switch(ds, spi->dev.of_node);
	if (ret)
		return ret;
	
	spi_set_drvdata(spi, kszd);

	kszd->tailtag_attr.size = 1;
    memcpy(&kszd->tailtag_attr, &ksz_tailtagging_attr, sizeof(kszd->tailtag_attr));

    ret = sysfs_create_bin_file(&spi->dev.kobj, &kszd->tailtag_attr);
    if (ret) {
        dev_err(&spi->dev, "unable to create sysfs file, err=%d\n", ret);
        return ret;
    }

	// activate to default tailtagging mode
	if (kszd->tail_tagging_enabled == 1)
    {
		ksz_enable_tail_tagging(kszd->ds);
		dsa_update_tagging_protocol(kszd->ds);

    }
    else
    {
        ksz_disable_tail_tagging(kszd->ds);
		dsa_update_tagging_protocol(kszd->ds);
    }

	return 0;
}


static int ksz_spi_remove(struct spi_device *spi)
{
	struct ksz_device *kszd = spi_get_drvdata(spi);

	dev_info(&spi->dev, "ksz_spi_remove\n");
	if (kszd) {
		dsa_unregister_switch(kszd->ds);
		if (kszd->reset_gpio) {
			/* Reset switch */
			gpiod_set_value(kszd->reset_gpio, 1);
		}
	}
	return 0;
}


static const struct of_device_id ksz8795_dt_ids[] = {
	{ .compatible = "micrel,ksz8795", },
	{ .compatible = "micrel,ksz8765", },
	{},
};
MODULE_DEVICE_TABLE(of, ksz8795_dt_ids);

static struct spi_driver ksz8795_spi_driver = {
	.driver = {
		.name	= "ksz8795-switch",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = ksz8795_dt_ids,
	},
	.probe	= ksz_spi_probe,
	.remove	= ksz_spi_remove,
};

module_spi_driver(ksz8795_spi_driver);

MODULE_AUTHOR("Karl Olsen <karl@micro-technic.com>");
MODULE_DESCRIPTION("KSZ8795/KSZ8765 switch DSA driver");
MODULE_LICENSE("GPL");
