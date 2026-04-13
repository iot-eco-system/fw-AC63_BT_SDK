/*
 * ble_serialble.c
 *
 * BLE GATT server for the "serialble" demo.
 *
 * Behaviour:
 *   - Advertises as a BLE peripheral.
 *   - Exposes one custom service (UUID 0xFF00) with one characteristic
 *     (UUID 0xFF01, READ | WRITE | NOTIFY).
 *   - When a BLE central connects and enables notifications, any data
 *     received on the AT serial UART is forwarded as a BLE notification.
 *   - The READ property lets the central read the most-recently received
 *     UART frame at any time.
 *
 * Hardware note:
 *   UART pins are configured via UART_DB_TX_PIN and UART_DB_RX_PIN in
 *   the board configuration file (board_xxx_cfg.h), e.g.:
 *     #define UART_DB_TX_PIN  IO_PORTC_02
 *     #define UART_DB_RX_PIN  IO_PORTC_03
 *   Baud rate is set by SERIALBLE_UART_BAUD (default 115200).
 */

#include "system/app_core.h"
#include "system/includes.h"
#include "app_config.h"
#include "app_action.h"
#include "btstack/btstack_task.h"
#include "btstack/bluetooth.h"
#include "user_cfg.h"
#include "bt_common.h"
#include "le_common.h"
#include "gatt_common/le_gatt_common.h"
#include "ble_serialble.h"
#include "ble_serialble_profile.h"

extern int le_controller_get_mac(void *addr);

#if CONFIG_APP_SERIALBLE

#define LOG_TAG_CONST SERIALBLE
#define LOG_TAG "[SERIALBLE]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
#define LOG_CLI_ENABLE
#include "debug.h"

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

/* AT serial UART (UART_DB_TX_PIN / UART_DB_RX_PIN) */
#define SERIALBLE_UART_BAUD 115200
#define SERIALBLE_UART_BUF_SIZE 0x100

/* ATT send buffer: 2 packets × (head + MTU) */
#define SERIALBLE_MTU_SIZE (247)
#define SERIALBLE_PACKET_NUMS (2)
#define SERIALBLE_CBUF_SIZE (SERIALBLE_PACKET_NUMS * (ATT_PACKET_HEAD_SIZE + SERIALBLE_MTU_SIZE))

/* BLE advertising interval (unit: 0.625 ms) */
#define SERIALBLE_ADV_INTERVAL_MIN (160 * 5) /* ~500 ms */

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */

/* Handle of the current BLE connection; 0 means no connection. */
static u16 serialble_con_handle = 0;

/* Last UART frame received – held so that a READ on ff01 returns it. */
#define SERIALBLE_LAST_RX_MAX 247
static u8 serialble_last_rx_buf[SERIALBLE_LAST_RX_MAX];
static u16 serialble_last_rx_len = 0;

static u8 serialble_adv_data[ADV_RSP_PACKET_MAX];
static u8 serialble_rsp_data[ADV_RSP_PACKET_MAX];
static adv_cfg_t serialble_adv_config;

/* serial UART state */
static u8 serialble_uart_dma_buf[SERIALBLE_UART_BUF_SIZE] __attribute__((aligned(4)));
static const uart_bus_t *serialble_uart_bus = NULL;

/* -------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------- */
static uint16_t serialble_att_read_callback(hci_con_handle_t connection_handle,
                                            uint16_t att_handle, uint16_t offset,
                                            uint8_t *buffer, uint16_t buffer_size);
static int serialble_att_write_callback(hci_con_handle_t connection_handle,
                                        uint16_t att_handle, uint16_t transaction_mode,
                                        uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int serialble_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param);

/* -------------------------------------------------------------------------
 * GATT server config
 * ---------------------------------------------------------------------- */

static const gatt_server_cfg_t serialble_server_cfg = {
    .att_read_cb = &serialble_att_read_callback,
    .att_write_cb = &serialble_att_write_callback,
    .event_packet_handler = &serialble_event_packet_handler,
};

static const sm_cfg_t serialble_sm_cfg = {
    .slave_security_auto_req = 0,
    .slave_set_wait_security = 0,
    .io_capabilities = IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
    .authentication_req_flags = SM_AUTHREQ_BONDING,
    .min_key_size = 7,
    .max_key_size = 16,
    .sm_cb_packet_handler = NULL,
};

static gatt_ctrl_t serialble_gatt_ctrl = {
    .mtu_size = SERIALBLE_MTU_SIZE,
    .cbuffer_size = SERIALBLE_CBUF_SIZE,
    .multi_dev_flag = 0,

    .server_config = &serialble_server_cfg,
    .client_config = NULL,

#if CONFIG_BT_SM_SUPPORT_ENABLE
    .sm_config = &serialble_sm_cfg,
#else
    .sm_config = NULL,
#endif

    .hci_cb_packet_handler = NULL,
};

/* -------------------------------------------------------------------------
 * UART → BLE bridge
 * ---------------------------------------------------------------------- */

/*
 * serialble_uart_rx_to_ble()
 *
 * Called from the UART IRQ context whenever a frame is received.
 * Stores the data so READ requests return fresh data, then sends a
 * BLE notification to the connected central (if notifications are enabled).
 * Data written by central to ff01 is forwarded out through UART.
 */
static void serialble_uart_rx_to_ble(u8 *packet, u32 size)
{
  if (size == 0 || packet == NULL)
  {
    return;
  }

  /* Cache the latest UART frame for subsequent READ operations */
  u16 copy_len = (size > SERIALBLE_LAST_RX_MAX) ? SERIALBLE_LAST_RX_MAX : (u16)size;
  memcpy(serialble_last_rx_buf, packet, copy_len);
  serialble_last_rx_len = copy_len;

  /* Forward via BLE notification if central has subscribed */
  if (serialble_con_handle == 0)
  {
    log_info("no ble connection, drop uart data\n");
    return;
  }

  if (!ble_gatt_server_characteristic_ccc_get(
          serialble_con_handle,
          ATT_CHARACTERISTIC_ff01_01_CLIENT_CONFIGURATION_HANDLE))
  {
    log_info("notify not enabled, drop uart data\n");
    return;
  }

  if (!ble_comm_att_check_send(serialble_con_handle, copy_len))
  {
    log_info("ble tx busy, drop uart data\n");
    return;
  }

  ble_comm_att_send_data(serialble_con_handle,
                         ATT_CHARACTERISTIC_ff01_01_VALUE_HANDLE,
                         serialble_last_rx_buf, copy_len,
                         ATT_OP_AUTO_READ_CCC);
}

/* -------------------------------------------------------------------------
 * serial UART open / close / rx-handler
 * ---------------------------------------------------------------------- */

static void serialble_uart_isr_cb(void *ut_bus, u32 status)
{
  if (status == UT_RX || status == UT_RX_OT)
  {
    static u8 rx_buf[SERIALBLE_LAST_RX_MAX];
    uart_bus_t *bus = (uart_bus_t *)ut_bus;
    u32 len = bus->read(rx_buf, SERIALBLE_LAST_RX_MAX, 0);
    if (len > 0)
    {
      serialble_uart_rx_to_ble(rx_buf, len);
    }
  }
}

static int serialble_uart_open(void)
{
  struct uart_platform_data_t u_arg = {0};
  u_arg.tx_pin = UART_DB_TX_PIN;
  u_arg.rx_pin = UART_DB_RX_PIN;
  u_arg.rx_cbuf = serialble_uart_dma_buf;
  u_arg.rx_cbuf_size = SERIALBLE_UART_BUF_SIZE;
  u_arg.frame_length = SERIALBLE_UART_BUF_SIZE;
  u_arg.rx_timeout = 6;
  u_arg.isr_cbfun = serialble_uart_isr_cb;
  u_arg.baud = SERIALBLE_UART_BAUD;
  u_arg.is_9bit = 0;
  serialble_uart_bus = uart_dev_open(&u_arg);
  if (serialble_uart_bus)
  {
    log_info("uart_dev_open() success\n");
    return 0;
  }
  log_info("uart_dev_open() failed\n");
  return -1;
}

static void serialble_uart_close(void)
{
  if (serialble_uart_bus)
  {
    uart_dev_close(serialble_uart_bus);
    serialble_uart_bus = NULL;
    log_info("uart_dev_close()\n");
  }
}

/* -------------------------------------------------------------------------
 * ATT read callback
 * ---------------------------------------------------------------------- */
static uint16_t serialble_att_read_callback(hci_con_handle_t connection_handle,
                                            uint16_t att_handle, uint16_t offset,
                                            uint8_t *buffer, uint16_t buffer_size)
{
  uint16_t att_value_len = 0;

  log_info("read_callback handle=%04x\n", att_handle);

  switch (att_handle)
  {

  case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE:
  {
    /* Return the GAP device name */
    char *gap_name = ble_comm_get_gap_name();
    att_value_len = (uint16_t)strlen(gap_name);
    if (offset >= att_value_len || (offset + buffer_size) > att_value_len)
    {
      break;
    }
    if (buffer)
    {
      memcpy(buffer, &gap_name[offset], buffer_size);
      att_value_len = buffer_size;
    }
    break;
  }

  case ATT_CHARACTERISTIC_ff01_01_VALUE_HANDLE:
  {
    /* Return the last UART frame received */
    att_value_len = serialble_last_rx_len;
    if (att_value_len == 0)
    {
      break;
    }
    if (offset >= att_value_len || (offset + buffer_size) > att_value_len)
    {
      break;
    }
    if (buffer)
    {
      memcpy(buffer, &serialble_last_rx_buf[offset], buffer_size);
      att_value_len = buffer_size;
    }
    break;
  }

  case ATT_CHARACTERISTIC_ff01_01_CLIENT_CONFIGURATION_HANDLE:
  case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
    if (buffer)
    {
      buffer[0] = ble_gatt_server_characteristic_ccc_get(connection_handle, att_handle);
      buffer[1] = 0;
    }
    att_value_len = 2;
    break;

  default:
    break;
  }

  return att_value_len;
}

/* -------------------------------------------------------------------------
 * ATT write callback
 * ---------------------------------------------------------------------- */
static int serialble_att_write_callback(hci_con_handle_t connection_handle,
                                        uint16_t att_handle, uint16_t transaction_mode,
                                        uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
  log_info("write_callback handle=%04x size=%d\n", att_handle, buffer_size);

  if (buffer == NULL || buffer_size == 0)
  {
    return 0;
  }

  switch (att_handle)
  {

  case ATT_CHARACTERISTIC_ff01_01_VALUE_HANDLE:
    /* Forward all BLE payload bytes to UART */
    if (serialble_uart_bus && serialble_uart_bus->write)
    {
      serialble_uart_bus->write(buffer, buffer_size);
    }
    break;

  case ATT_CHARACTERISTIC_ff01_01_CLIENT_CONFIGURATION_HANDLE:
  case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
    /* Central enabling/disabling notifications */
    ble_gatt_server_characteristic_ccc_set(connection_handle, att_handle, buffer[0]);
    log_info("ccc handle=%04x value=%02x\n", att_handle, buffer[0]);
    break;

  default:
    break;
  }

  return 0;
}

/* -------------------------------------------------------------------------
 * GATT / HCI event handler
 * ---------------------------------------------------------------------- */
static int serialble_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param)
{
  switch (event)
  {

  case GATT_COMM_EVENT_CONNECTION_COMPLETE:
    serialble_con_handle = little_endian_read_16(packet, 0);
    log_info("connected, handle=%04x\n", serialble_con_handle);
    serialble_uart_open();
    break;

  case GATT_COMM_EVENT_DISCONNECT_COMPLETE:
    log_info("disconnected, handle=%04x reason=%02x\n",
             little_endian_read_16(packet, 0), packet[2]);
    if (serialble_con_handle == little_endian_read_16(packet, 0))
    {
      serialble_uart_close();
      serialble_con_handle = 0;
    }
    break;

  case GATT_COMM_EVENT_CAN_SEND_NOW:
    break;

  case GATT_COMM_EVENT_MTU_EXCHANGE_COMPLETE:
    log_info("MTU exchanged: handle=%04x mtu=%u\n",
             little_endian_read_16(packet, 0),
             little_endian_read_16(packet, 2));
    break;

  case GATT_COMM_EVENT_ENCRYPTION_CHANGE:
    log_info("encryption_change: handle=%04x state=%d\n",
             little_endian_read_16(packet, 0), packet[2]);
    if (packet[3] == LINK_ENCRYPTION_RECONNECT)
    {
      /* Re-enable notify CCC after reconnect with bonding */
      ble_gatt_server_characteristic_ccc_set(
          little_endian_read_16(packet, 0),
          ATT_CHARACTERISTIC_ff01_01_CLIENT_CONFIGURATION_HANDLE,
          ATT_OP_NOTIFY);
    }
    break;

  case GATT_COMM_EVENT_SERVER_STATE:
    log_info("server_state: handle=%04x state=%02x\n",
             little_endian_read_16(packet, 1), packet[0]);
    break;

  default:
    break;
  }
  return 0;
}

/* -------------------------------------------------------------------------
 * Advertising helpers
 * ---------------------------------------------------------------------- */
static u8 adv_name_ok = 0;

static int serialble_make_adv_data(void)
{
  u8 offset = 0;
  u8 *buf = serialble_adv_data;

  /* Flags: LE General Discoverable, BR/EDR not supported */
  offset += make_eir_packet_val(&buf[offset], offset,
                                HCI_EIR_DATATYPE_FLAGS,
                                FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_EDR_NOT_SUPPORTED, 1);

  /* Advertise the custom service UUID 0xFF00 */
  offset += make_eir_packet_val(&buf[offset], offset,
                                HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS,
                                0xFF00, 2);

  /* Device name (if it fits) */
  char *gap_name = ble_comm_get_gap_name();
  u8 name_len = (u8)strlen(gap_name);
  u8 avail = ADV_RSP_PACKET_MAX - (offset + 2);
  if (name_len <= avail)
  {
    offset += make_eir_packet_data(&buf[offset], offset,
                                   HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME,
                                   (void *)gap_name, name_len);
    adv_name_ok = 1;
  }
  else
  {
    adv_name_ok = 0;
  }

  if (offset > ADV_RSP_PACKET_MAX)
  {
    puts("serialble_adv_data overflow!\n");
    return -1;
  }

  serialble_adv_config.adv_data_len = offset;
  serialble_adv_config.adv_data = serialble_adv_data;
  return 0;
}

static int serialble_make_rsp_data(void)
{
  u8 offset = 0;
  u8 *buf = serialble_rsp_data;

  /* If name didn't fit in the ADV packet, include it in the scan response */
  if (!adv_name_ok)
  {
    char *gap_name = ble_comm_get_gap_name();
    u8 name_len = (u8)strlen(gap_name);
    u8 avail = ADV_RSP_PACKET_MAX - (offset + 2);
    if (name_len > avail)
    {
      name_len = avail;
    }
    offset += make_eir_packet_data(&buf[offset], offset,
                                   HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME,
                                   (void *)gap_name, name_len);
  }

  if (offset > ADV_RSP_PACKET_MAX)
  {
    puts("serialble_rsp_data overflow!\n");
    return -1;
  }

  serialble_adv_config.rsp_data_len = offset;
  serialble_adv_config.rsp_data = serialble_rsp_data;
  return 0;
}

static void serialble_adv_config_set(void)
{
  int ret = 0;
  ret |= serialble_make_adv_data();
  ret |= serialble_make_rsp_data();

  serialble_adv_config.adv_interval = SERIALBLE_ADV_INTERVAL_MIN;
  serialble_adv_config.adv_auto_do = 1;
  serialble_adv_config.adv_type = ADV_IND;
  serialble_adv_config.adv_channel = ADV_CHANNEL_ALL;
  memset(serialble_adv_config.direct_address_info, 0, 7);

  if (ret == 0)
  {
    ble_gatt_server_set_adv_config(&serialble_adv_config);
  }
  else
  {
    log_info("adv config error\n");
  }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

static void serialble_set_name(void)
{
  static const char hex_table[] = "0123456789ABCDEF";
  u8 ble_addr_tmp[6];
  u8 ble_addr[6];
  u8 i;
  char ble_name[15] = "serialble_0000";

  if (le_controller_get_mac(ble_addr_tmp) != 0)
  {
    ble_comm_set_config_name("serialble", 0);
    return;
  }

  /* Convert to normal MAC byte order before taking the tail. */
  for (i = 0; i < 6; i++)
  {
    ble_addr[i] = ble_addr_tmp[5 - i];
  }

  ble_name[10] = hex_table[(ble_addr[4] >> 4) & 0x0F];
  ble_name[11] = hex_table[ble_addr[4] & 0x0F];
  ble_name[12] = hex_table[(ble_addr[5] >> 4) & 0x0F];
  ble_name[13] = hex_table[ble_addr[5] & 0x0F];

  ble_comm_set_config_name(ble_name, 0);
}

/*
 * serialble_ble_before_start_init()
 *
 * Must be called before btstack_init() to register the gatt_ctrl block.
 */
void serialble_ble_before_start_init(void)
{
  log_info("%s\n", __FUNCTION__);
  ble_comm_init(&serialble_gatt_ctrl);
}

/*
 * serialble_server_init()
 *
 * Must be called once after btstack_init() (e.g. from bt_ble_before_start_init
 * handler or equivalent).  Sets the GATT profile and starts advertising.
 */
void serialble_server_init(void)
{
  log_info("%s\n", __FUNCTION__);
  serialble_con_handle = 0;
  serialble_set_name();
  ble_gatt_server_set_profile(serialble_profile_data, sizeof(serialble_profile_data));
  serialble_adv_config_set();
}

/*
 * serialble_disconnect()
 *
 * Cleanly disconnects the current BLE central.
 */
void serialble_disconnect(void)
{
  log_info("%s\n", __FUNCTION__);
  if (serialble_con_handle)
  {
    ble_comm_disconnect(serialble_con_handle);
  }
}

#endif /* CONFIG_APP_SERIALBLE */
