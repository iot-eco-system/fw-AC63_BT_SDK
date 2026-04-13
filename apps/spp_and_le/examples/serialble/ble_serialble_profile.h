/*
 * ble_serialble_profile.h
 *
 * GATT profile for the "serialble" example.
 * One custom service (UUID 0xFF00) with one characteristic (UUID 0xFF01)
 * that supports READ, WRITE and NOTIFY.
 *
 * Service layout:
 *   0x0001  PRIMARY_SERVICE   0x1800  (GAP - Device Name)
 *   0x0002  CHARACTERISTIC    0x2A00  READ
 *   0x0003  VALUE             0x2A00  READ | DYNAMIC
 *   0x0004  PRIMARY_SERVICE   0x1801  (GATT - Service Changed)
 *   0x0005  CHARACTERISTIC    0x2A05  INDICATE
 *   0x0006  VALUE             0x2A05  INDICATE
 *   0x0007  CCCD              0x2902
 *   0x0008  PRIMARY_SERVICE   0xFF00  (Serial BLE custom service)
 *   0x0009  CHARACTERISTIC    0xFF01  READ | WRITE | NOTIFY
 *   0x000A  VALUE             0xFF01  READ | WRITE | NOTIFY | DYNAMIC
 *   0x000B  CCCD              0x2902
 */

#ifndef _BLE_SERIALBLE_PROFILE_H
#define _BLE_SERIALBLE_PROFILE_H

#include <stdint.h>
#include "app_config.h"

#if CONFIG_APP_SERIALBLE

/*
 * Flags encoding (little-endian 16-bit):
 *   ATT_PROPERTY_READ                = 0x0002
 *   ATT_PROPERTY_WRITE               = 0x0008
 *   ATT_PROPERTY_NOTIFY              = 0x0010
 *   ATT_PROPERTY_INDICATE            = 0x0020
 *   ATT_PROPERTY_DYNAMIC             = 0x0100
 */

static const uint8_t serialble_profile_data[] = {

    ////////////////////////////////////////////////////
    // 0x0001 PRIMARY_SERVICE 0x1800 (GAP)
    ////////////////////////////////////////////////////
    0x0a,
    0x00,
    0x02,
    0x00,
    0x01,
    0x00,
    0x00,
    0x28,
    0x00,
    0x18,

    /* CHARACTERISTIC, 2a00, READ | DYNAMIC */
    // 0x0002 CHARACTERISTIC 2a00 READ
    0x0d,
    0x00,
    0x02,
    0x00,
    0x02,
    0x00,
    0x03,
    0x28,
    0x02,
    0x03,
    0x00,
    0x00,
    0x2a,
    // 0x0003 VALUE 2a00 READ | DYNAMIC
    0x08,
    0x00,
    0x02,
    0x01,
    0x03,
    0x00,
    0x00,
    0x2a,

    ////////////////////////////////////////////////////
    // 0x0004 PRIMARY_SERVICE 0x1801 (GATT)
    ////////////////////////////////////////////////////
    0x0a,
    0x00,
    0x02,
    0x00,
    0x04,
    0x00,
    0x00,
    0x28,
    0x01,
    0x18,

    /* CHARACTERISTIC, 2a05, INDICATE */
    // 0x0005 CHARACTERISTIC 2a05 INDICATE
    0x0d,
    0x00,
    0x02,
    0x00,
    0x05,
    0x00,
    0x03,
    0x28,
    0x20,
    0x06,
    0x00,
    0x05,
    0x2a,
    // 0x0006 VALUE 2a05 INDICATE
    0x08,
    0x00,
    0x20,
    0x00,
    0x06,
    0x00,
    0x05,
    0x2a,
    // 0x0007 CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a,
    0x00,
    0x0a,
    0x01,
    0x07,
    0x00,
    0x02,
    0x29,
    0x00,
    0x00,

    ////////////////////////////////////////////////////
    // 0x0008 PRIMARY_SERVICE 0xFF00 (Serial BLE)
    ////////////////////////////////////////////////////
    0x0a,
    0x00,
    0x02,
    0x00,
    0x08,
    0x00,
    0x00,
    0x28,
    0x00,
    0xff,

    /* CHARACTERISTIC, ff01, READ | WRITE | NOTIFY */
    // 0x0009 CHARACTERISTIC ff01 READ | WRITE | NOTIFY
    0x0d,
    0x00,
    0x02,
    0x00,
    0x09,
    0x00,
    0x03,
    0x28,
    0x1a,
    0x0a,
    0x00,
    0x01,
    0xff,
    // 0x000a VALUE ff01 READ | WRITE | NOTIFY | DYNAMIC
    0x08,
    0x00,
    0x1a,
    0x01,
    0x0a,
    0x00,
    0x01,
    0xff,
    // 0x000b CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a,
    0x00,
    0x0a,
    0x01,
    0x0b,
    0x00,
    0x02,
    0x29,
    0x00,
    0x00,

    // END
    0x00,
    0x00,
};

/*
 * ATT handle definitions
 */
#define ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_2a05_01_VALUE_HANDLE 0x0006
#define ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE 0x0007
#define ATT_CHARACTERISTIC_ff01_01_VALUE_HANDLE 0x000a
#define ATT_CHARACTERISTIC_ff01_01_CLIENT_CONFIGURATION_HANDLE 0x000b

#endif /* CONFIG_APP_SERIALBLE */
#endif /* _BLE_SERIALBLE_PROFILE_H */
