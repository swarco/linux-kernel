/*
 * net/dsa/tag_micrelsw.c - Micrel switch tail tag format handling
 * Copyright (c) 2015 Micrel, Inc.
 * Copyright (c) 2008-2009 Marvell Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/slab.h>
#include "dsa_priv.h"


static struct sk_buff *micrelsw_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	struct sk_buff *nskb;
	int padlen;
	u8 *trailer;

	dev_dbg(p->parent->dev, "micrelsw_xmit to port %d\n", p->port);
	/*
	 * We have to make sure that the trailer ends up as the very last
	 * byte of the packet. This means that we have to pad the packet to
	 * the minimum ethernet frame size, if necessary, before adding the
	 * trailer.
	 */
	padlen = 0;
	if (skb->len < 60)
	padlen = 60 - skb->len;


	/* @@@kro:This is taken from tag_trailer.c. Is it really necessary to
	 * copy to a new skb? Can't we almost always just skb_pub(skb, 1) on
	 * the original packet? */
	nskb = alloc_skb(NET_IP_ALIGN + skb->len + padlen + 1, GFP_ATOMIC);
	if (nskb == NULL) {
		kfree_skb(skb);
		return NULL;
	}
	skb_reserve(nskb, NET_IP_ALIGN);

	skb_reset_mac_header(nskb);
	skb_set_network_header(nskb, skb_network_header(skb) - skb->head);
	skb_set_transport_header(nskb, skb_transport_header(skb) - skb->head);
	skb_copy_and_csum_dev(skb, skb_put(nskb, skb->len));
	kfree_skb(skb);

	if (padlen) {
		u8 *pad = skb_put(nskb, padlen);
		memset(pad, 0, padlen);
	}

	/* Tail tag byte: Direct forward to the specified port, unspecified
	 * destination port queue number. */
	trailer = skb_put(nskb, 1);
	trailer[0] = 0x40 | (1 << p->port);

	return nskb;
}


static int micrelsw_rcv(struct sk_buff *skb, struct net_device *dev,
		       struct packet_type *pt, struct net_device *orig_dev)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct dsa_switch *ds;
	u8 *trailer;
	int source_port;

	if (unlikely(dst == NULL))
		goto out_drop;
	ds = dst->ds[0];

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	if (skb_linearize(skb))
		goto out_drop;

        trailer = skb_tail_pointer(skb) - 1;
        source_port = trailer[0] & 0x03;
	if (source_port >= DSA_MAX_PORTS || !ds->ports[source_port].netdev)
		goto out_drop;

        dev_dbg(ds->dev, "micrelsw_recv from port %d\n", source_port);
	pskb_trim_rcsum(skb, skb->len - 1);

	skb->dev = ds->ports[source_port].netdev;
	skb_push(skb, ETH_HLEN);
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, skb->dev);

	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
out:
	return 0;
}

const struct dsa_device_ops micrelsw_netdev_ops = {
	.xmit	= micrelsw_xmit,
	.rcv	= micrelsw_rcv,
};
