#ifndef XR871_SPI_SLAVE_H
#define XR871_SPI_SLAVE_H

#include <stdint.h>
#include "kernel/os/os.h"

int spis_start(OS_Mutex_t *core_lock);	/* core calls happen under core_lock */
int spis_pin_allowed(int pin);
int spis_probe_pin(int pin, int level);
int spis_set_ready_pin(int pin);
int spis_ready_pin(void);	/* current ready pin (GPIO test refuses it) */
void spis_stats(uint32_t *xfer, uint32_t *aborted);
void spis_diag(void);

#endif
