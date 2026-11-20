/*
 * WiFi side of the bridge (milestone 2): the XR871 is the WiFi station, the
 * T30 is the host behind it with the SAME MAC (wlan0 takes the station MAC
 * from the SPI info frames), so frames are passed through unchanged:
 *
 *   air -> SDK ethernetif -> netif->input (hooked here) -> bridge_wifi_rx
 *   bridge wifi_tx -> netif->linkoutput -> air
 *
 * The XR871's own lwIP stays without an address (no DHCP here: the T30
 * runs udhcpc), so it never answers or sends anything itself.
 * Credentials come from the T30 (cmd 0xc0) and are kept in the SDK sysinfo
 * (flash), so the XR871 rejoins on its own after a cold start or while the
 * T30 sleeps. Never logged.
 *
 * NOT verified on hardware.
 */
#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "net/wlan/wlan.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/sysinfo.h"
#include "core/bridge_core.h"
#include "wifi.h"

static OS_Mutex_t *core_lock;
static netif_input_fn sdk_input;
static uint32_t n_rx, n_tx, n_tx_err, n_rx_busy;

/* ---- WiFi role worker (portal support) ---------------------------------
 * Mode switches (they reload the net core firmware: net.bin <-> net_ap.bin)
 * and scans block for seconds, so they run here and not under the core
 * lock. One request at a time. */
static OS_Thread_t op_thread;
static OS_Semaphore_t op_sem;
static volatile int op_req = -1;	/* BR_WIFI_OP_*, -1 = idle */
static uint8_t ap_ssid[33], ap_psk[65];
static int ap_ssid_len;
static volatile int is_ap;
static volatile int op_ready;	/* worker exists (wifi_start done) */
static int ps_off;		/* WLAN power save disabled for this link */
static int ps_standby;		/* T30 off: keep WLAN power save on */
/* signal of our AP in dBm for the status record / SPI info frame; refreshed
 * by wifi_link_info() (the 10 s diagnostics), never under the core lock */
static volatile int8_t link_dbm;

/* frames from the air: copy out and hand to the core, lwIP never sees them */
static err_t bridge_input(struct pbuf *p, struct netif *nif)
{
	static uint8_t f[1600];
	u16_t len;

	(void)nif;
	len = pbuf_copy_partial(p, f, sizeof(f), 0);
	pbuf_free(p);
	if (len) {
		n_rx++;
		/* Never stall this thread: it is the SDK's receive path from
		 * the net core. While the core lock is held for long (flash
		 * erase/write of a firmware update) the net core's queue filled
		 * up, its supplicant stalled and the WPA handshake failed in a
		 * loop. Wait briefly, then drop the frame (TCP resends). */
		if (OS_MutexLock(core_lock, 20) == OS_OK) {
			bridge_wifi_rx(f, len);
			OS_MutexUnlock(core_lock);
		} else {
			n_rx_busy++;
		}
	}
	return ERR_OK;
}

static void hook(void)
{
	struct netif *nif = g_wlan_netif;

	if (nif && nif->input != bridge_input) {
		sdk_input = nif->input;
		nif->input = bridge_input;
	}
}

static void op_done(int op, int ok)
{
	OS_MutexLock(core_lock, OS_WAIT_FOREVER);
	bridge_t30_send(XR_WIFI_DONE, (uint16_t)(op | (ok ? 0 : 0xff) << 8), NULL, 0);
	OS_MutexUnlock(core_lock);
}

/* 0 open, 1 WEP, 2 WPA, 3 WPA2, 4 WPA/WPA2 (see XR_SCAN_ENTRY) */
static int sec_type(const wlan_sta_ap_t *a)
{
	int wpa = a->wpa_key_mgmt != 0, rsn = a->wpa2_key_mgmt != 0;

	if (wpa && rsn)
		return 4;
	if (rsn)
		return 3;
	if (wpa)
		return 2;
	return a->wpa_flags ? 1 : 0;
}

static int do_scan(void)
{
	static wlan_sta_ap_t aps[24];
	wlan_sta_scan_results_t res;
	uint8_t e[6 + 4 + 32];
	int i, n = 0;

	if (is_ap || wlan_sta_scan_once() != 0)
		return -1;
	OS_MSleep(3500);	/* active scan of all 2.4 GHz channels */
	res.ap = aps;
	res.size = 24;
	res.num = 0;
	if (wlan_sta_scan_result(&res) != 0)
		return -1;
	for (i = 0; i < res.num; i++) {
		const wlan_sta_ap_t *a = &aps[i];
		int sl = a->ssid.ssid_len > 32 ? 32 : a->ssid.ssid_len;

		if (!sl)
			continue;	/* hidden networks are not offered */
		memcpy(e, a->bssid, 6);
		e[6] = (uint8_t)(int8_t)(a->level < -127 ? -127 : a->level > 0 ? 0 : a->level);	/* level = dBm; rssi is a positive SDK value */
		e[7] = a->channel;
		e[8] = (uint8_t)sec_type(a);
		e[9] = (uint8_t)sl;
		memcpy(e + 10, a->ssid.ssid, sl);
		OS_MutexLock(core_lock, OS_WAIT_FOREVER);
		bridge_t30_send(XR_SCAN_ENTRY, (uint16_t)n, e, (uint16_t)(10 + sl));
		OS_MutexUnlock(core_lock);
		n++;
	}
	return n;
}

static void join(const uint8_t *ssid, int ssid_len, const uint8_t *psk);

static int do_ap(void)
{
	struct sysinfo *si = sysinfo_get();

	if (si)	/* the XR871's own lwIP keeps no address in AP mode either */
		memset(&si->netif_ap_param, 0, sizeof(si->netif_ap_param));
	if (!is_ap && net_switch_mode(WLAN_MODE_HOSTAP) != 0)
		return -1;
	is_ap = 1;
	hook();
	wlan_ap_disable();
	if (wlan_ap_set(ap_ssid, (uint8_t)ap_ssid_len, ap_psk[0] ? ap_psk : NULL) != 0)
		return -1;
	return wlan_ap_enable();
}

static int do_sta(void)
{
	struct sysinfo *si = sysinfo_get();

	if (is_ap) {
		wlan_ap_disable();
		if (net_switch_mode(WLAN_MODE_STA) != 0)
			return -1;
		is_ap = 0;
	}
	hook();
	if (si && si->wlan_sta_param.ssid_len)
		join(si->wlan_sta_param.ssid, si->wlan_sta_param.ssid_len,
		     si->wlan_sta_param.psk);
	return 0;
}

static void op_task(void *arg)
{
	(void)arg;
	for (;;) {
		int op, r;

		OS_SemaphoreWait(&op_sem, OS_WAIT_FOREVER);
		op = op_req;
		r = op == BR_WIFI_OP_AP ? do_ap() :
		    op == BR_WIFI_OP_STA ? do_sta() :
		    op == BR_WIFI_OP_SCAN ? do_scan() : -1;
		printf("wifi: op %d -> %d (mode %s)\n", op, r, is_ap ? "ap" : "sta");
		op_req = -1;
		op_done(op, r >= 0);
	}
}

/* core hook (core lock held): queue the request for the worker */
int wifi_mode(int op, const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len)
{
	if (!op_ready || op_req >= 0 || ssid_len > 32 || psk_len > 64)
		return -1;	/* WLAN not up yet / busy / invalid: T30 retries */
	if (op == BR_WIFI_OP_AP) {
		memset(ap_ssid, 0, sizeof(ap_ssid));
		memset(ap_psk, 0, sizeof(ap_psk));
		memcpy(ap_ssid, ssid, ssid_len);
		ap_ssid_len = ssid_len;
		if (psk_len)
			memcpy(ap_psk, psk, psk_len);
	}
	op_req = op;
	OS_SemaphoreRelease(&op_sem);
	return 0;
}

void wifi_tx(const uint8_t *frame, int len)
{
	struct netif *nif = g_wlan_netif;
	struct pbuf *p;

	if (!nif || !nif->linkoutput || len <= 0)
		return;
	p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
	if (!p) {
		n_tx_err++;
		return;
	}
	pbuf_take(p, frame, len);
	if (nif->linkoutput(nif, p) == ERR_OK)
		n_tx++;
	else
		n_tx_err++;
	pbuf_free(p);
}

int wifi_status(uint8_t *mac, int8_t *rssi)
{
	struct netif *nif = g_wlan_netif;
	wlan_sta_states_t st = WLAN_STA_STATE_DISCONNECTED;

	*rssi = 0;
	if (!nif || op_req >= 0)	/* netif may be rebuilt by a mode switch */
		return 0;
	hook();
	memcpy(mac, nif->hwaddr, 6);
	if (is_ap)
		return 1;	/* the AP is "up" for the T30's wlan0 */
	wlan_sta_state(&st);
	/* The SDK turns WLAN power save off in netif_up_handler(), which never
	 * runs here (the XR871's lwIP gets no address, the T30 owns IP). With
	 * power save on, the AP buffers every frame for us until the next
	 * beacon: 50-150 ms latency, TCP crawls. Turn it off once per link. */
	if (st == WLAN_STA_STATE_CONNECTED && !ps_off && !ps_standby) {
		int r = wlan_set_ps_mode(nif, 0);

		ps_off = 1;	/* one try per link, no retry storm from the SPI loop */
		printf("wifi: power save off -> %d\n", r);
	} else if (st != WLAN_STA_STATE_CONNECTED) {
		ps_off = 0;
		link_dbm = 0;
	}
	if (st == WLAN_STA_STATE_CONNECTED)
		*rssi = link_dbm;	/* 0 = not measured yet */
	return st == WLAN_STA_STATE_CONNECTED;
}

/* bridge HAL low_power: while the T30 is off nothing needs low latency, so
 * let the AP buffer frames (power save on); on wake the next wifi_status()
 * call turns it off again. */
void wifi_low_power(int enter)
{
	struct netif *nif = g_wlan_netif;

	ps_standby = enter;
	ps_off = 0;
	if (enter && nif && !is_ap)
		printf("wifi: standby, power save on -> %d\n", wlan_set_ps_mode(nif, 1));
}

static void join(const uint8_t *ssid, int ssid_len, const uint8_t *psk)
{
	wlan_sta_disable();
	wlan_sta_set((uint8_t *)ssid, ssid_len, (uint8_t *)psk);
	wlan_sta_enable();
}

int wifi_config(const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len)
{
	struct sysinfo *si = sysinfo_get();
	uint8_t key[SYSINFO_PSK_LEN_MAX];
	int changed;

	if (!si || ssid_len <= 0 || ssid_len > SYSINFO_SSID_LEN_MAX ||
	    psk_len < 8 || psk_len >= SYSINFO_PSK_LEN_MAX)
		return -1;
	memset(key, 0, sizeof(key));
	memcpy(key, psk, psk_len);
	changed = si->wlan_sta_param.ssid_len != ssid_len ||
		  memcmp(si->wlan_sta_param.ssid, ssid, ssid_len) ||
		  memcmp(si->wlan_sta_param.psk, key, sizeof(key));
	if (changed) {
		memset(si->wlan_sta_param.ssid, 0, sizeof(si->wlan_sta_param.ssid));
		memcpy(si->wlan_sta_param.ssid, ssid, ssid_len);
		si->wlan_sta_param.ssid_len = ssid_len;
		memcpy(si->wlan_sta_param.psk, key, sizeof(key));
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
		sysinfo_save();	/* only on change: spare the flash */
#endif
		if (!is_ap)
			join(ssid, ssid_len, key);
	}
	/* credentials from the portal: leave AP mode and join */
	if (is_ap && op_req < 0) {
		op_req = BR_WIFI_OP_STA;
		OS_SemaphoreRelease(&op_sem);
	}
	memset(key, 0, sizeof(key));
	return changed;
}

int wifi_start(OS_Mutex_t *lock)
{
	struct sysinfo *si = sysinfo_get();

	core_lock = lock;
	if (!si)
		return -1;
	si->sta_use_dhcp = 0;	/* the T30 owns the IP layer */
	memset(&si->netif_sta_param, 0, sizeof(si->netif_sta_param));
	if (OS_SemaphoreCreate(&op_sem, 0, 1) != OS_OK ||
	    OS_ThreadCreate(&op_thread, "wifiop", op_task, NULL,
			    OS_THREAD_PRIO_APP, 3 * 1024) != OS_OK)
		printf("wifi: op worker failed\n");
	else
		op_ready = 1;
	hook();
	if (si->wlan_sta_param.ssid_len)
		join(si->wlan_sta_param.ssid, si->wlan_sta_param.ssid_len,
		     si->wlan_sta_param.psk);
	return 0;
}

/* signal of the AP we are associated with (diagnostics) */
int wifi_link_info(int *rssi, int *level, int *chan)
{
	wlan_sta_ap_t ap;

	if (is_ap || op_req >= 0 || wlan_sta_ap_info(&ap) != 0)
		return -1;
	*rssi = ap.rssi;
	*level = ap.level;	/* level = dBm; rssi is a positive SDK value */
	*chan = ap.channel;
	link_dbm = (int8_t)(ap.level < -127 ? -127 : ap.level > 0 ? 0 : ap.level);
	return 0;
}

/* frames from the air dropped because the core was busy (> 20 ms) */
uint32_t wifi_rx_busy(void)
{
	return n_rx_busy;
}

void wifi_stats(uint32_t *rx, uint32_t *tx, uint32_t *tx_err)
{
	*rx = n_rx;
	*tx = n_tx;
	*tx_err = n_tx_err;
}
