/*
 * ble_serialble.h
 *
 * BLE GATT server module for the "serialble" example.
 * Exposes a single custom service (UUID 0xFF00) with a READ | NOTIFY
 * characteristic (UUID 0xFF01) that forwards UART data to connected centrals.
 */

#ifndef _BLE_SERIALBLE_H
#define _BLE_SERIALBLE_H

#include <stdint.h>
#include "app_config.h"
#include "gatt_common/le_gatt_common.h"

#if CONFIG_APP_SERIALBLE

/* Called once before btstack_init() to register the gatt_ctrl_t block */
void serialble_ble_before_start_init(void);

/* Called once after btstack_init() to set profile and start advertising */
void serialble_server_init(void);

/* Disconnect the current BLE central (if any) */
void serialble_disconnect(void);

#endif /* CONFIG_APP_SERIALBLE */
#endif /* _BLE_SERIALBLE_H */
