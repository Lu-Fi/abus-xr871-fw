#ifndef XR871_HAL_EXT_H
#define XR871_HAL_EXT_H

#include "core/bridge_core.h"

/* build name, shown in the boot banner and in the status record (0xd0) */
#define HAL_EXT_BUILD "ap25"

/* fill in the optional bridge_hal extensions (version, battery, GPIO test,
 * log queue lock, flash access for the update, reboot). Call once before
 * bridge_init(). */
void hal_ext_fill(struct bridge_hal *hal);
/* HAL_Flash_Init once, shared by main.c (SDK init, service console) and
 * the update hooks; 0 = ok */
int hal_ext_flash_init(void);
/* last fuel gauge reading (main.c bat_report, every 30 s) for the status
 * record: the status command itself never touches the PMIC */
void hal_ext_battery_set(int percent, int mv);

#endif
