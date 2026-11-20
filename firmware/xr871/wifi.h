#ifndef XR871_WIFI_H
#define XR871_WIFI_H

#include <stdint.h>
#include "kernel/os/os.h"

int wifi_start(OS_Mutex_t *core_lock);
void wifi_tx(const uint8_t *frame, int len);
int wifi_status(uint8_t *mac, int8_t *rssi);
int wifi_config(const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len);
int wifi_mode(int op, const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len);
void wifi_stats(uint32_t *rx, uint32_t *tx, uint32_t *tx_err);

void wifi_low_power(int enter);

#endif
