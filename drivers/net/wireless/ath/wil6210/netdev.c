/*
<<<<<<< HEAD
 * Copyright (c) 2012-2014 Qualcomm Atheros, Inc.
=======
 * Copyright (c) 2012-2016 Qualcomm Atheros, Inc.
>>>>>>> 0e91d2a... Nougat
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

#include <linux/moduleparam.h>
#include <linux/etherdevice.h>

#include "wil6210.h"
#include "txrx.h"

static bool alt_ifname; /* = false; */
module_param(alt_ifname, bool, S_IRUGO);
MODULE_PARM_DESC(alt_ifname, " use an alternate interface name wigigN instead of wlanN");

static int wil_open(struct net_device *ndev)
{
	struct wil6210_priv *wil = ndev_to_wil(ndev);

	wil_dbg_misc(wil, "%s()\n", __func__);

	return wil_up(wil);
}

static int wil_stop(struct net_device *ndev)
{
	struct wil6210_priv *wil = ndev_to_wil(ndev);

	wil_dbg_misc(wil, "%s()\n", __func__);

	return wil_down(wil);
}

static int wil_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct wil6210_priv *wil = ndev_to_wil(ndev);

	if (new_mtu < 68 || new_mtu > mtu_max) {
		wil_err(wil, "invalid MTU %d\n", new_mtu);
		return -EINVAL;
	}

	wil_dbg_misc(wil, "change MTU %d -> %d\n", ndev->mtu, new_mtu);
	ndev->mtu = new_mtu;

	return 0;
}

static int wil_do_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct wil6210_priv *wil = ndev_to_wil(ndev);

	return wil_ioctl(wil, ifr->ifr_data, cmd);
}

static const struct net_device_ops wil_netdev_ops = {
	.ndo_open		= wil_open,
	.ndo_stop		= wil_stop,
	.ndo_start_xmit		= wil_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= wil_change_mtu,
	.ndo_do_ioctl		= wil_do_ioctl,
};

static int wil6210_netdev_poll_rx(struct napi_struct *napi, int budget)
{
	struct wil6210_priv *wil = container_of(napi, struct wil6210_priv,
						napi_rx);
	int quota = budget;
	int done;

	wil_rx_handle(wil, &quota);
	done = budget - quota;

	if (done <= 1) { /* burst ends - only one packet processed */
		napi_complete(napi);
		wil6210_unmask_irq_rx(wil);
		wil_dbg_txrx(wil, "NAPI RX complete\n");
	}

	wil_dbg_txrx(wil, "NAPI RX poll(%d) done %d\n", budget, done);

	return done;
}

static int wil6210_netdev_poll_tx(struct napi_struct *napi, int budget)
{
	struct wil6210_priv *wil = container_of(napi, struct wil6210_priv,
						napi_tx);
	int tx_done = 0;
	uint i;

	/* always process ALL Tx complete, regardless budget - it is fast */
	for (i = 0; i < WIL6210_MAX_TX_RINGS; i++) {
		struct vring *vring = &wil->vring_tx[i];
		struct vring_tx_data *txdata = &wil->vring_tx_data[i];

		if (!vring->va || !txdata->enabled)
			continue;

		tx_done += wil_tx_complete(wil, i);
	}

	if (tx_done <= 1) { /* burst ends - only one packet processed */
		napi_complete(napi);
		wil6210_unmask_irq_tx(wil);
		wil_dbg_txrx(wil, "NAPI TX complete\n");
	}

	wil_dbg_txrx(wil, "NAPI TX poll(%d) done %d\n", budget, tx_done);

	return min(tx_done, budget);
}

void *wil_if_alloc(struct device *dev, void __iomem *csr)
{
	struct net_device *ndev;
	struct wireless_dev *wdev;
	struct wil6210_priv *wil;
	struct ieee80211_channel *ch;
	int rc = 0;
	const char *ifname = alt_ifname ? "wigig%d" : "wlan%d";

	wdev = wil_cfg80211_init(dev);
	if (IS_ERR(wdev)) {
		dev_err(dev, "wil_cfg80211_init failed\n");
		return wdev;
	}

	wil = wdev_to_wil(wdev);
	wil->csr = csr;
	wil->wdev = wdev;
	wil->radio_wdev = wdev;

	wil_dbg_misc(wil, "%s()\n", __func__);

	rc = wil_priv_init(wil);
	if (rc) {
		dev_err(dev, "wil_priv_init failed\n");
		goto out_wdev;
	}

	wdev->iftype = NL80211_IFTYPE_STATION; /* TODO */
	/* default monitor channel */
	ch = wdev->wiphy->bands[IEEE80211_BAND_60GHZ]->channels;
	cfg80211_chandef_create(&wdev->preset_chandef, ch, NL80211_CHAN_NO_HT);

<<<<<<< HEAD
	ndev = alloc_netdev(0, "wlan%d", ether_setup);
=======
	ndev = alloc_netdev(0, ifname, NET_NAME_UNKNOWN, wil_dev_setup);
>>>>>>> 0e91d2a... Nougat
	if (!ndev) {
		dev_err(dev, "alloc_netdev_mqs failed\n");
		rc = -ENOMEM;
		goto out_priv;
	}

	ndev->netdev_ops = &wil_netdev_ops;
	wil_set_ethtoolops(ndev);
	ndev->ieee80211_ptr = wdev;
	ndev->hw_features = NETIF_F_HW_CSUM | NETIF_F_RXCSUM |
			    NETIF_F_SG | NETIF_F_GRO;
	ndev->features |= ndev->hw_features;
	SET_NETDEV_DEV(ndev, wiphy_dev(wdev->wiphy));
	wdev->netdev = ndev;

	netif_napi_add(ndev, &wil->napi_rx, wil6210_netdev_poll_rx,
		       WIL6210_NAPI_BUDGET);
	netif_napi_add(ndev, &wil->napi_tx, wil6210_netdev_poll_tx,
		       WIL6210_NAPI_BUDGET);

	wil_link_off(wil);

	return wil;

 out_priv:
	wil_priv_deinit(wil);

 out_wdev:
	wil_wdev_free(wil);

	return ERR_PTR(rc);
}

void wil_if_free(struct wil6210_priv *wil)
{
	struct net_device *ndev = wil_to_ndev(wil);

	wil_dbg_misc(wil, "%s()\n", __func__);

	if (!ndev)
		return;

	wil_priv_deinit(wil);

	wil_to_ndev(wil) = NULL;
	free_netdev(ndev);

	wil_wdev_free(wil);
}

int wil_if_add(struct wil6210_priv *wil)
{
	struct net_device *ndev = wil_to_ndev(wil);
	int rc;

	wil_dbg_misc(wil, "%s()\n", __func__);

	rc = register_netdev(ndev);
	if (rc < 0) {
		dev_err(&ndev->dev, "Failed to register netdev: %d\n", rc);
		return rc;
	}

	wil_link_off(wil);

	return 0;
}

void wil_if_remove(struct wil6210_priv *wil)
{
	struct net_device *ndev = wil_to_ndev(wil);

	wil_dbg_misc(wil, "%s()\n", __func__);

	unregister_netdev(ndev);
}
