/*
 * Linux cfg80211 Vendor Extension Code
 *
 * Copyright (C) 1999-2014, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: wl_cfgvendor.c 473890 2014-04-30 01:55:06Z $
*/

/*
 * New vendor interface additon to nl80211/cfg80211 to allow vendors
 * to implement proprietary features over the cfg80211 stack.
*/

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <linux/kernel.h>

#include <bcmutils.h>
#include <bcmwifi_channels.h>
#include <bcmendian.h>
#include <proto/ethernet.h>
#include <proto/802.11.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>


#include <dngl_stats.h>
#include <dhd.h>
#include <dhdioctl.h>
#include <wlioctl.h>
#include <dhd_cfg80211.h>
#ifdef PNO_SUPPORT
#include <dhd_pno.h>
#endif /* PNO_SUPPORT */
#ifdef RTT_SUPPORT
#include <dhd_rtt.h>
#endif /* RTT_SUPPORT */
#include <proto/ethernet.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include <wlioctl.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>
#include <wl_android.h>
#include <wl_cfgvendor.h>
#ifdef PROP_TXSTATUS
#include <dhd_wlfc.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 13, 0)) || defined(WL_VENDOR_EXT_SUPPORT)

static int wl_cfgvendor_send_cmd_reply(struct wiphy *wiphy,
        struct net_device *dev, const void  *data, int len)
{
	struct sk_buff *skb;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, len);
	if (unlikely(!skb)) {
		WL_ERR(("skb alloc failed"));
		return -ENOMEM;
	}

	/* Push the data to the skb */
	nla_put_nohdr(skb, len, data);

	return cfg80211_vendor_cmd_reply(skb);
}

static int wl_cfgvendor_gscan_get_channel_list(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int err = 0, type, band;
	uint16 *reply = NULL;
	uint32 reply_len = 0, num_channels, mem_needed;
	struct sk_buff *skb;
	struct wl_priv *wl = wiphy_priv(wiphy);

	type = nla_type(data);
	if (type == GSCAN_ATTRIBUTE_BAND) {
		band = nla_get_u32(data);
	} else {
		return -1;
	}
	reply = dhd_dev_pno_get_gscan(wl_to_prmry_ndev(wl),
	   DHD_PNO_GET_CHANNEL_LIST, &band, &reply_len);

	if (!reply) {
		WL_ERR(("Could not get channel list\n"));
		err = -EINVAL;
		return err;
	}
	num_channels =  reply_len/ sizeof(uint32);
	mem_needed = reply_len + VENDOR_REPLY_OVERHEAD + (ATTRIBUTE_U32_LEN * 2);

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, mem_needed);
	if (unlikely(!skb)) {
		WL_ERR(("skb alloc failed"));
		err = -ENOMEM;
		goto exit;
	}

	nla_put_u32(skb, GSCAN_ATTRIBUTE_NUM_CHANNELS, num_channels);
	nla_put(skb, GSCAN_ATTRIBUTE_CHANNEL_LIST, reply_len, reply);

	err =  cfg80211_vendor_cmd_reply(skb);

	if (unlikely(err))
		WL_ERR(("Vendor Command reply failed ret:%d \n", err));
exit:
	kfree(reply);
	return err;
}

static int wl_cfgvendor_priv_string_handler(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	int err = 0;
	int data_len = 0;

	bzero(wl->ioctl_buf, WLC_IOCTL_MAXLEN);

	if (strncmp((char *)data, BRCM_VENDOR_SCMD_CAPA, strlen(BRCM_VENDOR_SCMD_CAPA)) == 0) {
		err = wldev_iovar_getbuf(wl_to_prmry_ndev(wl), "cap", NULL, 0,
			wl->ioctl_buf, WLC_IOCTL_MAXLEN, &wl->ioctl_buf_sync);
		if (unlikely(err)) {
			WL_ERR(("error (%d)\n", err));
			return err;
		}
		data_len = strlen(wl->ioctl_buf);
		wl->ioctl_buf[data_len] = '\0';
	}

	err =  wl_cfgvendor_send_cmd_reply(wiphy, wl_to_prmry_ndev(wl),
		wl->ioctl_buf, data_len+1);
	if (unlikely(err))
		WL_ERR(("Vendor Command reply failed ret:%d \n", err));
	else
		WL_INFO(("Vendor Command reply sent successfully!\n"));

	return err;
}

static int wl_cfgvendor_set_country(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int err = BCME_ERROR, rem, type;
	char country_code[WLC_CNTRY_BUF_SZ] = {0};
	const struct nlattr *iter;
	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
			case ANDR_WIFI_ATTRIBUTE_COUNTRY:
				memcpy(country_code, nla_data(iter),
					MIN(nla_len(iter), WLC_CNTRY_BUF_SZ));
				break;
			default:
				WL_ERR(("Unknown type: %d\n", type));
				return err;
		}
	}

	err = wldev_set_country(wdev->netdev, country_code, true, true);
	if (err < 0) {
		WL_ERR(("Set country failed ret:%d\n", err));
	}

	return err;
}

#if defined(KEEP_ALIVE)
static int wl_cfgvendor_start_mkeep_alive(struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int len)
{
	/* max size of IP packet for keep alive */
	const int MKEEP_ALIVE_IP_PKT_MAX = 256;

	int ret = BCME_OK, rem, type;
	u8 mkeep_alive_id = 0;
	u8 *ip_pkt = NULL;
	u16 ip_pkt_len = 0;
	u8 src_mac[ETHER_ADDR_LEN];
	u8 dst_mac[ETHER_ADDR_LEN];
	u32 period_msec = 0;
	const struct nlattr *iter;
	struct wl_priv *wl = wiphy_priv(wiphy);
	dhd_pub_t *dhd_pub = wl->pub;
	gfp_t kflags = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
			case MKEEP_ALIVE_ATTRIBUTE_ID:
				mkeep_alive_id = nla_get_u8(iter);
				break;
			case MKEEP_ALIVE_ATTRIBUTE_IP_PKT_LEN:
				ip_pkt_len = nla_get_u16(iter);
				if ( ip_pkt_len > MKEEP_ALIVE_IP_PKT_MAX) {
					ret = BCME_BADARG;
					goto exit;
				}
				break;
			case MKEEP_ALIVE_ATTRIBUTE_IP_PKT:
				ip_pkt = (u8 *)kzalloc(ip_pkt_len, kflags);
				if (ip_pkt == NULL) {
					ret = BCME_NOMEM;
					WL_ERR(("Failed to allocate mem for ip packet\n"));
					goto exit;
				}
				memcpy(ip_pkt, (u8*)nla_data(iter), ip_pkt_len);
				break;
			case MKEEP_ALIVE_ATTRIBUTE_SRC_MAC_ADDR:
				memcpy(src_mac, nla_data(iter), ETHER_ADDR_LEN);
				break;
			case MKEEP_ALIVE_ATTRIBUTE_DST_MAC_ADDR:
				memcpy(dst_mac, nla_data(iter), ETHER_ADDR_LEN);
				break;
			case MKEEP_ALIVE_ATTRIBUTE_PERIOD_MSEC:
				period_msec = nla_get_u32(iter);
				break;
			default:
				WL_ERR(("Unknown type: %d\n", type));
				ret = BCME_BADARG;
				goto exit;
		}
	}

	ret = dhd_dev_start_mkeep_alive(dhd_pub, mkeep_alive_id, ip_pkt, ip_pkt_len, src_mac,
				        dst_mac, period_msec);
	if (ret < 0) {
		WL_ERR(("start_mkeep_alive is failed ret: %d\n", ret));
	}

exit:
	if (ip_pkt) {
		kfree(ip_pkt);
	}

	return ret;
}

static int wl_cfgvendor_stop_mkeep_alive(struct wiphy *wiphy, struct wireless_dev *wdev,
	const void *data, int len)
{
	int ret = BCME_OK, rem, type;
	u8 mkeep_alive_id = 0;
	const struct nlattr *iter;
	struct wl_priv *wl = wiphy_priv(wiphy);
	dhd_pub_t *dhd_pub = wl->pub;

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
			case MKEEP_ALIVE_ATTRIBUTE_ID:
				mkeep_alive_id = nla_get_u8(iter);
				break;
			default:
				WL_ERR(("Unknown type: %d\n", type));
				ret = BCME_BADARG;
				break;
		}
	}

	ret = dhd_dev_stop_mkeep_alive(dhd_pub, mkeep_alive_id);
	if (ret < 0) {
		WL_ERR(("stop_mkeep_alive is failed ret: %d\n", ret));
	}

	return ret;
}
#endif /* defined(KEEP_ALIVE) */

static const struct wiphy_vendor_command wl_vendor_cmds [] = {
	{
		{
			.vendor_id = OUI_BRCM,
			.subcmd = BRCM_VENDOR_SCMD_PRIV_STR
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wl_cfgvendor_priv_string_handler
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = GSCAN_SUBCMD_GET_CHANNEL_LIST
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wl_cfgvendor_gscan_get_channel_list
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = ANDR_WIFI_SET_COUNTRY
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wl_cfgvendor_set_country
	},
#ifdef KEEP_ALIVE
        {
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = WIFI_OFFLOAD_SUBCMD_START_MKEEP_ALIVE
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wl_cfgvendor_start_mkeep_alive
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = WIFI_OFFLOAD_SUBCMD_STOP_MKEEP_ALIVE
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wl_cfgvendor_stop_mkeep_alive
	},
#endif /* KEEP_ALIVE */
};

static const struct  nl80211_vendor_cmd_info wl_vendor_events [] = {
		{ OUI_BRCM, BRCM_VENDOR_EVENT_UNSPEC },
		{ OUI_BRCM, BRCM_VENDOR_EVENT_PRIV_STR },
		{ OUI_GOOGLE, GOOGLE_GSCAN_SIGNIFICANT_EVENT },
		{ OUI_GOOGLE, GOOGLE_GSCAN_GEOFENCE_FOUND_EVENT },
		{ OUI_GOOGLE, GOOGLE_GSCAN_BATCH_SCAN_EVENT },
		{ OUI_GOOGLE, GOOGLE_SCAN_FULL_RESULTS_EVENT },
		{ OUI_GOOGLE, GOOGLE_RTT_COMPLETE_EVENT },
		{ OUI_GOOGLE, GOOGLE_SCAN_COMPLETE_EVENT },
		{ OUI_GOOGLE, GOOGLE_GSCAN_GEOFENCE_LOST_EVENT },
		{ OUI_GOOGLE, GOOGLE_SCAN_EPNO_EVENT },
		{ OUI_GOOGLE, GOOGLE_DEBUG_RING_EVENT },
		{ OUI_GOOGLE, GOOGLE_FW_DUMP_EVENT },
		{ OUI_GOOGLE, GOOGLE_PNO_HOTSPOT_FOUND_EVENT },
		{ OUI_GOOGLE, GOOGLE_RSSI_MONITOR_EVENT },
		{ OUI_GOOGLE, GOOGLE_MKEEP_ALIVE_EVENT }
};

int wl_cfgvendor_attach(struct wiphy *wiphy, dhd_pub_t *dhd)
{

	WL_INFO(("Vendor: Register BRCM cfg80211 vendor cmd(0x%x) interface \n",
		NL80211_CMD_VENDOR));

	wiphy->vendor_commands	= wl_vendor_cmds;
	wiphy->n_vendor_commands = ARRAY_SIZE(wl_vendor_cmds);
	wiphy->vendor_events	= wl_vendor_events;
	wiphy->n_vendor_events	= ARRAY_SIZE(wl_vendor_events);

	return 0;
}

int wl_cfgvendor_detach(struct wiphy *wiphy)
{
	WL_INFO(("Vendor: Unregister BRCM cfg80211 vendor interface \n"));

	wiphy->vendor_commands  = NULL;
	wiphy->vendor_events    = NULL;
	wiphy->n_vendor_commands = 0;
	wiphy->n_vendor_events  = 0;

	return 0;
}
#endif /* (LINUX_VERSION_CODE > KERNEL_VERSION(3, 13, 0)) || defined(WL_VENDOR_EXT_SUPPORT) */
