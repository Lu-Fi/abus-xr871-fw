/*
 * Unit test for the standby DHCP renewal in the core:
 *   - the lease is learnt from the T30's own DHCP traffic (request up,
 *     ACK down), the ACK still reaches the T30;
 *   - nothing is sent while the T30 runs (its own client renews);
 *   - T30 asleep: a RENEWING DHCPREQUEST at T1 (unicast to the server MAC/IP,
 *     ciaddr set, client id + hostname copied, no server id / requested IP),
 *     valid IPv4 header checksum; a retry every BR_DHCP_RETRY_MS;
 *   - an ACK with our xid renews (next request only after the new T1), one
 *     with a foreign xid is ignored; a NAK stops the renewal;
 *   - no answer until the lease ends: give up;
 *   - ARP for the T30's IP is still answered in standby.
 *
 * Build: cc -Wall -Wextra -I.. -o dhcp dhcp.c ../core/bridge_core.c ../core/fw_update.c
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "core/bridge_core.h"
#include "core/spi_frame.h"

static uint32_t ms = 1000;
static uint8_t last_tx[1600];
static int last_len, n_dhcp_tx, n_arp_tx;

static const uint8_t T30_MAC[6] = { 0x02, 0x00, 0x5e, 0x10, 0x20, 0x30 };
static const uint8_t SRV_MAC[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
static const uint8_t SRV_IP[4] = { 192, 168, 10, 1 };
static const uint8_t MY_IP[4] = { 192, 168, 10, 50 };
static const uint8_t CID[7] = { 1, 0x02, 0x00, 0x5e, 0x10, 0x20, 0x30 };
static const char HOST[] = "test-camera";

static void w(const uint8_t *b, int l) { (void)b; (void)l; }
static int mcu(uint8_t c, uint8_t v) { (void)c; (void)v; return 0; }
static void pw(int on) { (void)on; }
static uint32_t mil(void) { return ms; }
static void lg(const char *f, ...) { (void)f; }

static void wtx(const uint8_t *f, int len)
{
	if (len > (int)sizeof(last_tx))
		return;
	if (len > 37 && f[12] == 0x08 && f[13] == 0x00 && f[23] == 17 && f[37] == 67) {
		memcpy(last_tx, f, len);
		last_len = len;
		n_dhcp_tx++;
	}
	if (len >= 42 && f[12] == 0x08 && f[13] == 0x06)
		n_arp_tx++;
}

static int fails;

static void expect(const char *what, int ok)
{
	if (!ok)
		fails++;
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

static void t30_cmd(uint8_t cmd, uint16_t param)
{
	uint8_t f[12] = { 0xaa, 0x55, 0xaa, 0x55, 0, 0, cmd, 0x01, 0, 0,
			  param & 0xff, param >> 8 };

	bridge_t30_rx(f, sizeof(f));
}

/* a DHCP frame: op 1 = request from the T30, 2 = reply from the server */
static int dhcp_frame(uint8_t *f, int op, uint8_t type, uint32_t xid, uint32_t lease)
{
	int b = 42, o;

	memset(f, 0, 42 + 300);
	memcpy(f, op == 1 ? (const uint8_t *)"\xff\xff\xff\xff\xff\xff" : T30_MAC, 6);
	memcpy(f + 6, op == 1 ? T30_MAC : SRV_MAC, 6);
	f[12] = 0x08;
	f[14] = 0x45;
	f[23] = 17;
	if (op == 2) {
		memcpy(f + 26, SRV_IP, 4);
		memcpy(f + 30, MY_IP, 4);
	} else {
		memset(f + 30, 0xff, 4);
	}
	f[34] = 0; f[35] = op == 1 ? 68 : 67;
	f[36] = 0; f[37] = op == 1 ? 67 : 68;
	f[b] = op; f[b + 1] = 1; f[b + 2] = 6;
	f[b + 4] = xid >> 24; f[b + 5] = xid >> 16; f[b + 6] = xid >> 8; f[b + 7] = xid;
	if (op == 2)
		memcpy(f + b + 16, MY_IP, 4);
	memcpy(f + b + 28, T30_MAC, 6);
	f[b + 236] = 0x63; f[b + 237] = 0x82; f[b + 238] = 0x53; f[b + 239] = 0x63;
	o = b + 240;
	f[o++] = 53; f[o++] = 1; f[o++] = type;
	if (op == 1) {
		f[o++] = 61; f[o++] = sizeof(CID);
		memcpy(f + o, CID, sizeof(CID)); o += sizeof(CID);
		f[o++] = 12; f[o++] = strlen(HOST);
		memcpy(f + o, HOST, strlen(HOST)); o += strlen(HOST);
		f[o++] = 50; f[o++] = 4; memcpy(f + o, MY_IP, 4); o += 4;
	} else {
		f[o++] = 54; f[o++] = 4; memcpy(f + o, SRV_IP, 4); o += 4;
		if (type == 5) {
			f[o++] = 51; f[o++] = 4;
			f[o++] = lease >> 24; f[o++] = lease >> 16; f[o++] = lease >> 8; f[o++] = lease;
		}
	}
	f[o++] = 255;
	return 42 + 300 > o ? 42 + 300 : o;
}

static void t30_up(const uint8_t *eth, int len)
{
	uint8_t s[SPIF_SIZE];

	spif_build(s, SPIF_ETH, 0, 1, eth, len);
	bridge_spi_rx(s, SPIF_SIZE);
}

static void advance(uint32_t step_ms, uint32_t total_ms, int hb)
{
	uint32_t end = ms + total_ms;

	while ((int32_t)(ms - end) < 0) {
		ms += step_ms;
		if (hb)
			t30_cmd(T30_HEARTBEAT, 0);
		bridge_poll();
	}
}

static uint32_t tx_xid(void)
{
	return (uint32_t)last_tx[46] << 24 | last_tx[47] << 16 | last_tx[48] << 8 | last_tx[49];
}

static int tx_has_opt(uint8_t code, const uint8_t *val, int vlen)
{
	int i = 42 + 240;

	while (i < last_len && last_tx[i] != 255) {
		if (last_tx[i] == 0) {
			i++;
			continue;
		}
		if (last_tx[i] == code)
			return vlen < 0 || (last_tx[i + 1] == vlen && !memcmp(last_tx + i + 2, val, vlen));
		i += 2 + last_tx[i + 1];
	}
	return 0;
}

static int ip_csum_ok(const uint8_t *ip)
{
	uint32_t s = 0;
	int i;

	for (i = 0; i < 20; i += 2)
		s += ip[i] << 8 | ip[i + 1];
	while (s >> 16)
		s = (s & 0xffff) + (s >> 16);
	return s == 0xffff;
}

static void sleep_t30(void)
{
	t30_cmd(T30_POWER, 0);
	advance(100, BR_SLEEP_GRACE_MS + 200, 0);
}

int main(void)
{
	static struct bridge_hal h;
	uint8_t f[1600], s[SPIF_SIZE];
	const struct bridge_stats *S;
	uint32_t xid;
	int n;

	h.t30_write = w; h.mcu_send = mcu; h.t30_power = pw; h.millis = mil;
	h.log = lg; h.wifi_tx = wtx;
	bridge_init(&h, 0);
	S = bridge_stats();

	printf("standby DHCP renewal:\n");
	t30_cmd(T30_HEARTBEAT, 0);
	n = dhcp_frame(f, 1, 3, 0x1111, 0);
	t30_up(f, n);
	n = dhcp_frame(f, 2, 5, 0x1111, 1800);
	bridge_wifi_rx(f, n);
	expect("ACK for the T30 is still delivered to it", bridge_spi_next(s) == 1);
	n_dhcp_tx = 0;			/* the T30's own request went out */
	advance(1000, 20 * 60 * 1000, 1);
	expect("T30 running: the core sends no DHCP itself", n_dhcp_tx == 0);

	sleep_t30();
	n_dhcp_tx = 0;
	advance(1000, 60 * 1000, 0);	/* 21 min since the ACK, T1 = 15 min */
	expect("asleep past T1: one request", n_dhcp_tx == 1);
	expect("unicast to the server MAC from the T30 MAC",
	       !memcmp(last_tx, SRV_MAC, 6) && !memcmp(last_tx + 6, T30_MAC, 6));
	expect("IP T30 -> server, header checksum valid",
	       !memcmp(last_tx + 26, MY_IP, 4) && !memcmp(last_tx + 30, SRV_IP, 4) &&
	       ip_csum_ok(last_tx + 14));
	expect("UDP 68 -> 67, BOOTREQUEST, ciaddr + chaddr",
	       last_tx[35] == 68 && last_tx[37] == 67 && last_tx[42] == 1 &&
	       !memcmp(last_tx + 42 + 12, MY_IP, 4) && !memcmp(last_tx + 42 + 28, T30_MAC, 6));
	expect("DHCPREQUEST with the T30's client id and hostname",
	       tx_has_opt(53, (const uint8_t *)"\x03", 1) && tx_has_opt(61, CID, sizeof(CID)) &&
	       tx_has_opt(12, (const uint8_t *)HOST, strlen(HOST)));
	expect("RENEWING: no server id, no requested IP", !tx_has_opt(54, NULL, -1) &&
	       !tx_has_opt(50, NULL, -1));
	expect("UDP length matches the frame",
	       (last_tx[38] << 8 | last_tx[39]) == last_len - 34 &&
	       (last_tx[16] << 8 | last_tx[17]) == last_len - 14);
	xid = tx_xid();

	n_dhcp_tx = 0;
	advance(1000, 30 * 1000, 0);
	expect("no repeat before the retry time", n_dhcp_tx == 0);
	advance(1000, 31 * 1000, 0);
	expect("retry after BR_DHCP_RETRY_MS without an answer", n_dhcp_tx == 1);
	xid = tx_xid();

	n = dhcp_frame(f, 2, 5, xid ^ 1, 1800);
	bridge_wifi_rx(f, n);
	expect("ACK with a foreign xid ignored", S->dhcp_ack == 0);
	n = dhcp_frame(f, 2, 5, xid, 1800);
	bridge_wifi_rx(f, n);
	expect("ACK with our xid renews the lease", S->dhcp_ack == 1);
	n_dhcp_tx = 0;
	advance(1000, 14 * 60 * 1000, 0);
	expect("after the renewal: quiet until the new T1", n_dhcp_tx == 0);
	advance(1000, 90 * 1000, 0);
	expect("new T1: next request", n_dhcp_tx == 1);

	/* ARP for the T30 still answered while asleep */
	memset(f, 0, 42);
	memset(f, 0xff, 6);
	memcpy(f + 6, SRV_MAC, 6);
	f[12] = 0x08; f[13] = 0x06; f[15] = 1; f[16] = 0x08; f[18] = 6; f[19] = 4; f[21] = 1;
	memcpy(f + 22, SRV_MAC, 6);
	memcpy(f + 28, SRV_IP, 4);
	memcpy(f + 38, MY_IP, 4);
	n_arp_tx = 0;
	bridge_wifi_rx(f, 42);
	expect("standby ARP reply for the leased IP", n_arp_tx == 1);

	/* NAK stops it */
	xid = tx_xid();
	n = dhcp_frame(f, 2, 6, xid, 0);
	bridge_wifi_rx(f, n);
	n_dhcp_tx = 0;
	advance(1000, 40 * 60 * 1000, 0);
	expect("NAK: no further requests", S->dhcp_nak == 1 && n_dhcp_tx == 0);

	/* new lease while running, then no answers at all: give up at the end */
	bridge_wake(BR_WAKE_PIR);
	t30_cmd(T30_HEARTBEAT, 0);
	n = dhcp_frame(f, 2, 5, 0x2222, 1800);
	bridge_wifi_rx(f, n);
	sleep_t30();
	n_dhcp_tx = 0;
	advance(1000, 30 * 60 * 1000, 0);
	expect("no answers: requests from T1 to the lease end (about 15)",
	       n_dhcp_tx >= 13 && n_dhcp_tx <= 16);
	n_dhcp_tx = 0;
	advance(1000, 30 * 60 * 1000, 0);
	expect("lease ended: given up, quiet", n_dhcp_tx == 0);

	/* awake again: nothing from the core */
	bridge_wake(BR_WAKE_PIR);
	n = dhcp_frame(f, 2, 5, 0x3333, 1800);
	bridge_wifi_rx(f, n);
	n_dhcp_tx = 0;
	advance(1000, 40 * 60 * 1000, 1);
	expect("T30 awake: still no DHCP from the core", n_dhcp_tx == 0);

	printf(fails ? "DHCP FAIL\n" : "DHCP PASS\n");
	return fails ? 1 : 0;
}
