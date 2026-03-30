/*
 * app_serialble.c
 *
 * Application state machine for the "serialble" demo.
 * Listens to UART and forwards received bytes to a BLE central via a
 * custom READ | NOTIFY characteristic.
 *
 * To activate this example, set in apps/spp_and_le/include/app_config.h:
 *   #define CONFIG_APP_SERIALBLE  1
 * (and set all other CONFIG_APP_* to 0)
 */

#include "system/app_core.h"
#include "system/includes.h"
#include "server/server_core.h"
#include "app_config.h"
#include "app_action.h"
#include "os/os_api.h"
#include "btcontroller_config.h"
#include "btctrler/btctrler_task.h"
#include "config/config_transport.h"
#include "btstack/avctp_user.h"
#include "btstack/btstack_task.h"
#include "bt_common.h"
#include "app_charge.h"
#include "app_chargestore.h"
#include "app_power_manage.h"
#include "app_comm_bt.h"
#include "ble_serialble.h"

#define LOG_TAG_CONST SERIALBLE_APP
#define LOG_TAG "[SERIALBLE_APP]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
#define LOG_CLI_ENABLE
#include "debug.h"

#if CONFIG_APP_SERIALBLE

static u8 is_app_serialble_active = 0;
static u8 enter_btstack_num = 0;

/* -------------------------------------------------------------------------
 * Power management helpers
 * ---------------------------------------------------------------------- */

static void serialble_power_event_to_user(u8 event)
{
  struct sys_event e;
  e.type = SYS_DEVICE_EVENT;
  e.arg = (void *)DEVICE_EVENT_FROM_POWER;
  e.u.dev.event = event;
  e.u.dev.value = 0;
  sys_event_notify(&e);
}

static void serialble_set_soft_poweroff(void)
{
  log_info("set_soft_poweroff\n");
  is_app_serialble_active = 1;

#if TCFG_USER_BLE_ENABLE
  btstack_ble_exit(0);
#endif
#if TCFG_USER_EDR_ENABLE
  btstack_edr_exit(0);
#endif

#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
  sys_timeout_add(NULL, power_set_soft_poweroff, WAIT_DISCONN_TIME_MS);
#else
  power_set_soft_poweroff();
#endif
}

/* -------------------------------------------------------------------------
 * BLE init config
 * ---------------------------------------------------------------------- */

static const ble_init_cfg_t serialble_ble_config = {
    .same_address = 0,
    .appearance = 0,
};

/* -------------------------------------------------------------------------
 * App start
 * ---------------------------------------------------------------------- */

static void serialble_app_start(void)
{
  log_info("=======================================\n");
  log_info("---------  serialble demo  ------------\n");
  log_info("=======================================\n");
  log_info("app_file: %s\n", __FILE__);

  if (enter_btstack_num == 0)
  {
    enter_btstack_num = 1;
    clk_set("sys", BT_NORMAL_HZ);

#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
    u32 sys_clk = clk_get("sys");
    bt_pll_para(TCFG_CLOCK_OSC_HZ, sys_clk, 0, 0);

#if TCFG_USER_BLE_ENABLE
    btstack_ble_start_before_init(&serialble_ble_config, 0);
#endif
    btstack_init();
#endif
  }

  sys_key_event_enable();
}

/* -------------------------------------------------------------------------
 * App state machine
 * ---------------------------------------------------------------------- */

static int serialble_state_machine(struct application *app, enum app_state state,
                                   struct intent *it)
{
  switch (state)
  {
  case APP_STA_CREATE:
    break;
  case APP_STA_START:
    if (!it)
    {
      break;
    }
    switch (it->action)
    {
    case ACTION_SERIALBLE_MAIN:
      serialble_app_start();
      break;
    }
    break;
  case APP_STA_PAUSE:
    break;
  case APP_STA_RESUME:
    break;
  case APP_STA_STOP:
    break;
  case APP_STA_DESTROY:
    log_info("APP_STA_DESTROY\n");
    break;
  }
  return 0;
}

/* -------------------------------------------------------------------------
 * HCI / connection event handlers (delegated to common helpers)
 * ---------------------------------------------------------------------- */

static int serialble_bt_hci_event_handler(struct bt_event *bt)
{
  log_info("%s event=%x value=%x\n", __FUNCTION__, bt->event, bt->value);
#if TCFG_USER_BLE_ENABLE
  bt_comm_ble_hci_event_handler(bt);
#endif
  return 0;
}

static int serialble_bt_connction_status_event_handler(struct bt_event *bt)
{
  log_info("%s event=%d\n", __FUNCTION__, bt->event);
#if TCFG_USER_BLE_ENABLE
  bt_comm_ble_status_event_handler(bt);
#endif
  return 0;
}

/* -------------------------------------------------------------------------
 * Key event handler
 * ---------------------------------------------------------------------- */

static void serialble_key_event_handler(struct sys_event *event)
{
  if (event->arg != (void *)DEVICE_EVENT_FROM_KEY)
  {
    return;
  }
  u8 event_type = event->u.key.event;
  u8 key_value = event->u.key.value;
  log_info("key event: type=%d value=%d\n", event_type, key_value);

  /* Triple-click: soft power-off */
  if (event_type == KEY_EVENT_TRIPLE_CLICK)
  {
    serialble_power_event_to_user(POWER_EVENT_POWER_SOFTOFF);
    return;
  }

  /* Single click on key0: force disconnect */
  if (event_type == KEY_EVENT_CLICK && key_value == TCFG_ADKEY_VALUE0)
  {
    log_info("key0 click: disconnect BLE\n");
    serialble_disconnect();
  }
}

/* -------------------------------------------------------------------------
 * App event handler
 * ---------------------------------------------------------------------- */

static int serialble_event_handler(struct application *app, struct sys_event *event)
{
  switch (event->type)
  {
  case SYS_KEY_EVENT:
    serialble_key_event_handler(event);
    return 0;

  case SYS_BT_EVENT:
#if (TCFG_USER_EDR_ENABLE || TCFG_USER_BLE_ENABLE)
    if ((u32)event->arg == SYS_BT_EVENT_TYPE_CON_STATUS)
    {
      serialble_bt_connction_status_event_handler(&event->u.bt);
    }
    else if ((u32)event->arg == SYS_BT_EVENT_TYPE_HCI_STATUS)
    {
      serialble_bt_hci_event_handler(&event->u.bt);
    }
#endif
    return 0;

  case SYS_DEVICE_EVENT:
    if ((u32)event->arg == DEVICE_EVENT_FROM_POWER)
    {
      return app_power_event_handler(&event->u.dev, serialble_set_soft_poweroff);
    }
#if TCFG_CHARGE_ENABLE
    else if ((u32)event->arg == DEVICE_EVENT_FROM_CHARGE)
    {
      app_charge_event_handler(&event->u.dev);
    }
#endif
    return 0;

  default:
    return FALSE;
  }
  return FALSE;
}

/* -------------------------------------------------------------------------
 * Application registration
 * ---------------------------------------------------------------------- */

static const struct application_operation app_serialble_ops = {
    .state_machine = serialble_state_machine,
    .event_handler = serialble_event_handler,
};

REGISTER_APPLICATION(app_serialble) = {
    .name = "serialble",
    .action = ACTION_SERIALBLE_MAIN,
    .ops = &app_serialble_ops,
    .state = APP_STA_DESTROY,
};

/* -------------------------------------------------------------------------
 * Sleep gate: allow sleep when not actively powering down
 * ---------------------------------------------------------------------- */
static u8 serialble_state_idle_query(void)
{
  return !is_app_serialble_active;
}

REGISTER_LP_TARGET(serialble_lp_target) = {
    .name = "serialble_lp",
    .is_idle = serialble_state_idle_query,
};

/* -------------------------------------------------------------------------
 * BLE stack hooks (called by btstack during initialisation)
 * ---------------------------------------------------------------------- */

/*
 * bt_ble_before_start_init()
 *
 * Called by the BLE stack before btstack_init().
 * Register the gatt_ctrl block here.
 *
 * NOTE: only ONE function named bt_ble_before_start_init may be linked.
 * When CONFIG_APP_SERIALBLE is active, the trans_data version is excluded
 * by its own CONFIG_APP_SPP_LE guard, so there is no conflict.
 */
void bt_ble_before_start_init(void)
{
  log_info("%s\n", __FUNCTION__);
  serialble_ble_before_start_init();
}

/*
 * bt_ble_init()
 *
 * Called by the BLE stack (via btstack_ble_start_after_init -> bt_ble_init)
 * after btstack_init() completes.  Sets up the GATT profile, starts
 * advertising, and enables the BLE module.
 */
void bt_ble_init(void)
{
  log_info("%s\n", __FUNCTION__);
  serialble_server_init();
  ble_module_enable(1);
}

/*
 * bt_ble_exit()
 *
 * Called when the BLE stack is being shut down.
 */
void bt_ble_exit(void)
{
  log_info("%s\n", __FUNCTION__);
  ble_module_enable(0);
  ble_comm_exit();
}

/*
 * ble_module_enable()
 *
 * Called by testbox_update.c and app_comm_edr.c to enable/disable the
 * BLE radio module.  Must be provided by whichever example is active.
 */
void ble_module_enable(u8 en)
{
  ble_comm_module_enable(en);
}

#endif /* CONFIG_APP_SERIALBLE */
