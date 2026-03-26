/*******************************************************************************
 *  ble_trans.c  —  ESC_BLE_App firmware
 *  AC6328 / Jieli JLAI SDK
 *
 *  BLE Service:  ae00
 *
 *  Characteristics:
 *    ae03  WRITE_WITHOUT_RESPONSE   Android → MCU  motor command (5 bytes)
 *    ae02  NOTIFY                   MCU → Android  command echo  (5 bytes)
 *    ae10  READ | WRITE             Android reads status / writes mode change
 *
 *  ── ae03 / ae02 Packet Format (5 bytes) ────────────────────────────────────
 *
 *    Byte 0  CMD
 *              CMD_ESC_PWM  (0x01)  ESC pulse-width command
 *              CMD_BLDC_DUTY(0x02)  BLDC duty-cycle command
 *              CMD_STOP     (0xFF)  Stop both motors immediately
 *
 *    Bytes 1-2  port_value      little-endian uint16
 *    Bytes 3-4  starboard_value little-endian uint16
 *
 *    For CMD_ESC_PWM:   values are µs  (1000 = stop, 2000 = full throttle)
 *    For CMD_BLDC_DUTY: values are 0–10000 (0 = stop, 10000 = 100%, default = 500)
 *    For CMD_STOP:      values are ignored
 *
 *  ── ae10 WRITE (1 byte) — runtime mode switch ───────────────────────────────
 *    0x01  → switch to ESC  mode
 *    0x02  → switch to BLDC mode
 *
 *  ── ae10 READ response (ASCII) ──────────────────────────────────────────────
 *    "M<mode>A<vbat_mv>T<uptime_min>"
 *    e.g. "M1A3712T5"   M1 = ESC mode, M2 = BLDC mode
 *
 *  ── Hardware ────────────────────────────────────────────────────────────────
 *    PORT      IO_PORT_DM  JL_TIMER3
 *    STARBOARD IO_PORT_DP  JL_TIMER2
 *
 *  ── Safety ──────────────────────────────────────────────────────────────────
 *    Watchdog: motors stop if no command received within WATCHDOG_TIMEOUT_MS
 ******************************************************************************/

#include "system/app_core.h"
#include "system/includes.h"
#include "system/power_manage.h"
#include "app_config.h"
#include "app_action.h"
#include "btstack/btstack_task.h"
#include "btstack/bluetooth.h"
#include "user_cfg.h"
#include "vm.h"
#include "btcontroller_modules.h"
#include "bt_common.h"
#include "3th_profile_api.h"
#include "le_common.h"
#include "rcsp_bluetooth.h"
#include "JL_rcsp_api.h"
#include "custom_cfg.h"
#include "btstack/btstack_event.h"
#include "gatt_common/le_gatt_common.h"
#include "ble_trans.h"
#include "ble_trans_profile.h"

#if CONFIG_APP_SPP_LE

/* ── Debug logging ──────────────────────────────────────────────────────── */
#if LE_DEBUG_PRINT_EN
#define log_info(x, ...)    printf("[BLE_TRANS] " x, ## __VA_ARGS__)
#define log_info_hexdump    put_buf
#else
#define log_info(...)
#define log_info_hexdump(...)
#endif

/* ── BLE stack config ───────────────────────────────────────────────────── */
#define ATT_MTU_REQUEST_ENABLE    0
#define ATT_LOCAL_MTU_SIZE        512
#define ATT_PACKET_NUMS_MAX       2
#define ATT_SEND_CBUF_SIZE        (ATT_PACKET_NUMS_MAX * (ATT_PACKET_HEAD_SIZE + ATT_LOCAL_MTU_SIZE))
#define ADV_INTERVAL_MIN          (160 * 5)   /* 0.625 ms units → 500 ms */

/* ── Motor / PWM config ─────────────────────────────────────────────────── */
#define PWM_FREQ_ESC              50           /* Hz — standard RC ESC     */
#define PWM_FREQ_BLDC             1000         /* Hz — BLDC direct drive   */

//#define ESC_US_MIN                1000         /* µs — ESC stop / arm      */
//#define ESC_US_MAX                2000         /* µs — ESC full throttle   */
#define ESC_US_MIN                500
#define ESC_US_MAX                1000

#define BLDC_DUTY_MIN             0            /* 0%   duty                */
#define BLDC_DUTY_MAX             10000        /* 100% duty (0–10000)      */
#define BLDC_DUTY_DEFAULT         500          /* 5%   safe start          */

#define WATCHDOG_TIMEOUT_MS       2000         /* ms — stop if no command  */

/* ── Command bytes (Byte 0 of ae03 packet) ──────────────────────────────── */
#define CMD_ESC_PWM               0x01
#define CMD_BLDC_DUTY             0x02
#define CMD_STOP                  0xFF

/* ── ae03 packet size ───────────────────────────────────────────────────── */
#define CMD_PACKET_LEN            5

/* ── ae10 mode-switch bytes ─────────────────────────────────────────────── */
#define MODE_ESC                  0x01
#define MODE_BLDC                 0x02

/* ── ae10 READ response format ──────────────────────────────────────────── */
#define AE10_RESP_FMT             "M%uA%uT%u"
#define AE10_RESP_MAX_LEN         32

/*******************************************************************************
 * Private types
 ******************************************************************************/

/* Parsed ae03 command packet */
typedef struct {
    u8  cmd;            /* CMD_ESC_PWM | CMD_BLDC_DUTY | CMD_STOP */
    u16 port_value;     /* µs (ESC) or duty×10 (BLDC) */
    u16 stbd_value;
} motor_cmd_t;

/* Motor mode */
typedef enum {
    MOTOR_MODE_ESC  = MODE_ESC,
    MOTOR_MODE_BLDC = MODE_BLDC
} motor_mode_t;

/*******************************************************************************
 * Static state
 ******************************************************************************/

static u16           trans_con_handle       = 0;
static adv_cfg_t     trans_server_adv_config;
static uint8_t       trans_connection_update_enable = 1;
static u32           last_cmd_time          = 0;
static motor_mode_t  motor_mode             = MOTOR_MODE_ESC;  /* default ESC */

static u8  trans_adv_data[ADV_RSP_PACKET_MAX];
static u8  trans_scan_rsp_data[ADV_RSP_PACKET_MAX];

static const char user_tag_string[] = {
    0xd6, 0x05, 0x08, 0x00, 'J', 'L', 'A', 'I', 'S', 'D', 'K'
};

static const struct conn_update_param_t trans_connection_param_table[] = {
    {16, 24, 10, 600},
    {12, 28, 10, 600},
    {8,  20, 10, 600},
};
#define CONN_PARAM_TABLE_CNT \
    (sizeof(trans_connection_param_table) / sizeof(struct conn_update_param_t))

/*******************************************************************************
 * Forward declarations
 ******************************************************************************/

static uint16_t trans_att_read_callback(
    hci_con_handle_t connection_handle,
    uint16_t att_handle, uint16_t offset,
    uint8_t *buffer, uint16_t buffer_size);

static int trans_att_write_callback(
    hci_con_handle_t connection_handle,
    uint16_t att_handle, uint16_t transaction_mode,
    uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

static int trans_event_packet_handler(
    int event, u8 *packet, u16 size, u8 *ext_param);

extern void uart_db_regiest_recieve_callback(void *rx_cb);

/*******************************************************************************
 * BLE stack config structs
 ******************************************************************************/

#define PASSKEY_ENABLE 0

static const sm_cfg_t trans_sm_init_config = {
    .slave_security_auto_req   = 0,
    .slave_set_wait_security   = 0,
#if PASSKEY_ENABLE
    .io_capabilities           = IO_CAPABILITY_DISPLAY_ONLY,
#else
    .io_capabilities           = IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
#endif
    .authentication_req_flags  = SM_AUTHREQ_BONDING | SM_AUTHREQ_MITM_PROTECTION,
    .min_key_size              = 7,
    .max_key_size              = 16,
    .sm_cb_packet_handler      = NULL,
};

const gatt_server_cfg_t trans_server_init_cfg = {
    .att_read_cb        = &trans_att_read_callback,
    .att_write_cb       = &trans_att_write_callback,
    .event_packet_handler = &trans_event_packet_handler,
};

static gatt_ctrl_t trans_gatt_control_block = {
    .mtu_size        = ATT_LOCAL_MTU_SIZE,
    .cbuffer_size    = ATT_SEND_CBUF_SIZE,
    .multi_dev_flag  = 0,
#if CONFIG_BT_GATT_SERVER_NUM
    .server_config   = &trans_server_init_cfg,
#else
    .server_config   = NULL,
#endif
    .client_config   = NULL,
#if CONFIG_BT_SM_SUPPORT_ENABLE
    .sm_config       = &trans_sm_init_config,
#else
    .sm_config       = NULL,
#endif
    .hci_cb_packet_handler = NULL,
};

/*******************************************************************************
 * Packet helpers
 ******************************************************************************/

/* Read a little-endian uint16 from a byte buffer */
static inline u16 read_le16(const u8 *buf)
{
    return (u16)buf[0] | ((u16)buf[1] << 8);
}

/* Write a little-endian uint16 into a byte buffer */
static inline void write_le16(u8 *buf, u16 value)
{
    buf[0] = (u8)(value & 0xFF);
    buf[1] = (u8)((value >> 8) & 0xFF);
}

/* Parse a 5-byte ae03 command buffer into motor_cmd_t */
static int parse_motor_cmd(const u8 *buf, u16 buf_size, motor_cmd_t *out)
{
    if (buf_size < CMD_PACKET_LEN) {
        log_info("cmd too short: %u bytes (need %u)\n", buf_size, CMD_PACKET_LEN);
        return -1;
    }
    out->cmd        = buf[0];
    out->port_value = read_le16(&buf[1]);
    out->stbd_value = read_le16(&buf[3]);
    return 0;
}

/* Build a 5-byte ae02 echo buffer from a motor_cmd_t */
static void build_echo_packet(u8 *buf, const motor_cmd_t *cmd)
{
    buf[0] = cmd->cmd;
    write_le16(&buf[1], cmd->port_value);
    write_le16(&buf[3], cmd->stbd_value);
}

/*******************************************************************************
 * Motor control
 ******************************************************************************/

/*
 * Apply ESC pulse-width to both motors.
 * port_us / stbd_us: 1000–2000 µs
 */
static void motor_apply_esc(u16 port_us, u16 stbd_us)
{
    /* Clamp to safe range */
    if (port_us < ESC_US_MIN) port_us = ESC_US_MIN;
    if (port_us > ESC_US_MAX) port_us = ESC_US_MAX;
    if (stbd_us < ESC_US_MIN) stbd_us = ESC_US_MIN;
    if (stbd_us > ESC_US_MAX) stbd_us = ESC_US_MAX;

    log_info("ESC: port=%u stbd=%u µs\n", port_us, stbd_us);

    timer_pwm_init(JL_TIMER3, IO_PORT_DM, PWM_FREQ_ESC, port_us);
    timer_pwm_init(JL_TIMER2, IO_PORT_DP, PWM_FREQ_ESC, stbd_us);
}

/*
 * Apply BLDC duty cycle to both motors.
 * port_duty / stbd_duty: 0–10000  (0 = stop, 10000 = 100%, default = 500)
 * Resolution ×10 allows 0.1% steps without floats.
 */
static void motor_apply_bldc(u16 port_duty, u16 stbd_duty)
{
    /* Clamp */
    if (port_duty > BLDC_DUTY_MAX) port_duty = BLDC_DUTY_MAX;
    if (stbd_duty > BLDC_DUTY_MAX) stbd_duty = BLDC_DUTY_MAX;

    /* Convert 0–10000 to 0–100 integer percent for timer API */
    u16 port_pct = port_duty / 100;
    u16 stbd_pct = stbd_duty / 100;

    log_info("BLDC: port=%u%% stbd=%u%%\n", port_pct, stbd_pct);

    timer_pwm_init(JL_TIMER3, IO_PORT_DM, PWM_FREQ_BLDC, port_pct);
    timer_pwm_init(JL_TIMER2, IO_PORT_DP, PWM_FREQ_BLDC, stbd_pct);
}

/* Stop both motors — always safe regardless of mode */
void motor_stop(void)
{
    log_info("motor_stop\n");
    timer_pwm_init(JL_TIMER3, IO_PORT_DM, PWM_FREQ_ESC,  ESC_US_MIN);
    timer_pwm_init(JL_TIMER2, IO_PORT_DP, PWM_FREQ_ESC,  ESC_US_MIN);
}

/*
 * Dispatch a parsed motor_cmd_t to the appropriate apply function.
 * Returns 0 on success, -1 if cmd byte is unknown.
 */
static int motor_dispatch(const motor_cmd_t *cmd)
{
    switch (cmd->cmd) {

    case CMD_ESC_PWM:
        if (motor_mode != MOTOR_MODE_ESC) {
            log_info("warning: CMD_ESC_PWM received but mode is BLDC\n");
        }
        motor_apply_esc(cmd->port_value, cmd->stbd_value);
        return 0;

    case CMD_BLDC_DUTY:
        if (motor_mode != MOTOR_MODE_BLDC) {
            log_info("warning: CMD_BLDC_DUTY received but mode is ESC\n");
        }
        motor_apply_bldc(cmd->port_value, cmd->stbd_value);
        return 0;

    case CMD_STOP:
        motor_stop();
        return 0;

    default:
        log_info("unknown cmd byte: 0x%02X\n", cmd->cmd);
        return -1;
    }
}

/*******************************************************************************
 * Watchdog
 ******************************************************************************/
/*
static void pwm_watchdog(void *p)
{
    if (trans_con_handle && (timer_get_ms() - last_cmd_time > WATCHDOG_TIMEOUT_MS)) {
        log_info("watchdog: no command for %ums, stopping motors\n", WATCHDOG_TIMEOUT_MS);
        motor_stop();
    }
}
*/
/*******************************************************************************
 * ae02 echo — notify Android with confirmed command values
 ******************************************************************************/

static void send_command_echo(const motor_cmd_t *cmd)
{
    u8 echo[CMD_PACKET_LEN];
    build_echo_packet(echo, cmd);

    if (ble_comm_att_check_send(trans_con_handle, CMD_PACKET_LEN) &&
        ble_gatt_server_characteristic_ccc_get(
            trans_con_handle,
            ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE))
    {
        ble_comm_att_send_data(
            trans_con_handle,
            ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
            echo,
            CMD_PACKET_LEN,
            ATT_OP_AUTO_READ_CCC);
    }
}

/*******************************************************************************
 * ATT read callback  (ae10 READ — status response)
 ******************************************************************************/

static uint16_t trans_att_read_callback(hci_con_handle_t connection_handle,uint16_t att_handle, uint16_t offset,uint8_t *buffer, uint16_t buffer_size){
    uint16_t att_value_len = 0;

    switch (att_handle) {

    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE: {
        /* GAP device name */
        char *gap_name = ble_comm_get_gap_name();
        att_value_len = strlen(gap_name);
        if (buffer && offset < att_value_len) {
            u16 copy_len = att_value_len - offset;
            if (copy_len > buffer_size) copy_len = buffer_size;
            memcpy(buffer, gap_name + offset, copy_len);
            att_value_len = copy_len;
        }
        break;
    }

    case ATT_CHARACTERISTIC_ae10_01_VALUE_HANDLE: {
        char tmp[AE10_RESP_MAX_LEN];

        u16 vbat_mv    = adc_get_voltage(AD_CH_VBAT) * 4;
        u32 uptime_min = (timer_get_ms() / 1000) / 60;

        u16 total_len = (u16)snprintf(tmp, sizeof(tmp),
                                     AE10_RESP_FMT,
                                     (u32)motor_mode,
                                     (u32)vbat_mv,
                                     uptime_min);

        if (offset >= total_len) {
            att_value_len = 0;
            break;
        }

        if (buffer) {
            u16 copy_len = total_len - offset;
            if (copy_len > buffer_size) copy_len = buffer_size;
            memcpy(buffer, tmp + offset, copy_len);
            att_value_len = copy_len;
        } else {
            att_value_len = total_len;
        }
        break;
    }

    case ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE:
        if (buffer) {
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

static void trans_send_connetion_updata_deal(u16 conn_handle)
{
    if (trans_connection_update_enable) {
        if (0 == ble_gatt_server_connetion_update_request(
                conn_handle,
                trans_connection_param_table,
                CONN_PARAM_TABLE_CNT)) {
            trans_connection_update_enable = 0;
        }
    }
}

/*******************************************************************************
 * ATT write callback  (ae03 motor command, ae10 mode switch, CCC)
 ******************************************************************************/

static int trans_att_write_callback(
    hci_con_handle_t connection_handle,
    uint16_t att_handle, uint16_t transaction_mode,
    uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    switch (att_handle) {

    /* ── ae03: motor command ─────────────────────────────────────────────── */
    case ATT_CHARACTERISTIC_ae03_01_VALUE_HANDLE: {
        motor_cmd_t cmd;

        if (parse_motor_cmd(buffer, buffer_size, &cmd) != 0) {
            break;  /* malformed packet — ignore */
        }

        last_cmd_time = timer_get_ms();

        if (motor_dispatch(&cmd) == 0) {
            send_command_echo(&cmd);
        }
        break;
    }

    /* ── ae10: runtime mode switch ───────────────────────────────────────── */
    case ATT_CHARACTERISTIC_ae10_01_VALUE_HANDLE: {
        if (buffer_size < 1) break;

        u8 requested_mode = buffer[0];

        if (requested_mode == MODE_ESC) {
            motor_mode = MOTOR_MODE_ESC;
            motor_stop();
            log_info("mode → ESC\n");

        } else if (requested_mode == MODE_BLDC) {
            motor_mode = MOTOR_MODE_BLDC;
            motor_stop();
            log_info("mode → BLDC\n");

        } else {
            log_info("unknown mode byte: 0x%02X\n", requested_mode);
        }
        break;
    }

    /* ── CCC writes (notification enable/disable) ────────────────────────── */
    case ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE:
        trans_send_connetion_updata_deal(connection_handle);
        ble_gatt_server_characteristic_ccc_set(connection_handle, att_handle, buffer[0]);
        log_info("ae02 CCC: 0x%02X\n", buffer[0]);
        break;

    default:
        break;
    }

    return 0;
}

/*******************************************************************************
 * Connection parameter update
 ******************************************************************************/

static void trans_resume_all_ccc_enable(u16 conn_handle, u8 update_request)
{
    ble_gatt_server_characteristic_ccc_set(
        conn_handle,
        ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE,
        ATT_OP_NOTIFY);

    if (update_request) {
        trans_send_connetion_updata_deal(conn_handle);
    }
}

/*******************************************************************************
 * GATT event handler
 ******************************************************************************/

static int trans_event_packet_handler(
    int event, u8 *packet, u16 size, u8 *ext_param)
{
    switch (event) {

    case GATT_COMM_EVENT_CONNECTION_COMPLETE:
        trans_con_handle = little_endian_read_16(packet, 0);
        trans_connection_update_enable = 1;
        last_cmd_time = timer_get_ms();
        log_info("connected: handle=0x%04X\n", trans_con_handle);

#if ATT_MTU_REQUEST_ENABLE
        att_server_set_exchange_mtu(trans_con_handle);
#endif
#if TCFG_UART0_RX_PORT != NO_CONFIG_PORT
        uart_db_regiest_recieve_callback(NULL);
#endif
        break;

    case GATT_COMM_EVENT_DISCONNECT_COMPLETE:
        log_info("disconnected: handle=0x%04X reason=0x%02X\n",
            little_endian_read_16(packet, 0), packet[2]);
        motor_stop();
        if (trans_con_handle == little_endian_read_16(packet, 0)) {
            trans_con_handle = 0;
        }
        break;

    case GATT_COMM_EVENT_ENCRYPTION_CHANGE:
        log_info("encryption change: handle=0x%04X state=%d\n",
            little_endian_read_16(packet, 0), packet[2]);
        if (packet[3] == LINK_ENCRYPTION_RECONNECT) {
            trans_resume_all_ccc_enable(little_endian_read_16(packet, 0), 1);
        }
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_COMPLETE:
        log_info("conn params updated: interval=%d latency=%d timeout=%d\n",
            little_endian_read_16(ext_param, 6),
            little_endian_read_16(ext_param, 8),
            little_endian_read_16(ext_param, 10));
        break;

    case GATT_COMM_EVENT_CAN_SEND_NOW:
    case GATT_COMM_EVENT_CONNECTION_UPDATE_REQUEST_RESULT:
    case GATT_COMM_EVENT_MTU_EXCHANGE_COMPLETE:
    case GATT_COMM_EVENT_SERVER_INDICATION_COMPLETE:
        break;

    default:
        break;
    }
    return 0;
}

/*******************************************************************************
 * Advertising data
 ******************************************************************************/

static u8 adv_name_ok = 0;

static int trans_make_set_adv_data(void)
{
    u8  offset = 0;
    u8 *buf    = trans_adv_data;

#if DOUBLE_BT_SAME_MAC
    offset += make_eir_packet_val(&buf[offset], offset,
        HCI_EIR_DATATYPE_FLAGS,
        FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_LE_AND_EDR_SAME_CONTROLLER, 1);
#else
    offset += make_eir_packet_val(&buf[offset], offset,
        HCI_EIR_DATATYPE_FLAGS,
        FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_EDR_NOT_SUPPORTED, 1);
#endif

    /* Advertise the actual GATT service UUID ae00 */
    offset += make_eir_packet_val(&buf[offset], offset,
        HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, 0xAE00, 2);

    char *gap_name  = ble_comm_get_gap_name();
    u8    name_len  = strlen(gap_name);
    u8    valid_len = ADV_RSP_PACKET_MAX - (offset + 2);

    if (name_len <= valid_len) {
        offset += make_eir_packet_data(&buf[offset], offset,
            HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME,
            (void *)gap_name, name_len);
        adv_name_ok = 1;
    } else {
        adv_name_ok = 0;
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        log_info("ERROR: adv_data overflow\n");
        return -1;
    }

    trans_server_adv_config.adv_data_len = offset;
    trans_server_adv_config.adv_data     = trans_adv_data;
    return 0;
}

static int trans_make_set_rsp_data(void)
{
    u8  offset = 0;
    u8 *buf    = trans_scan_rsp_data;

#if RCSP_BTMATE_EN
    u8 tag_len = sizeof(user_tag_string);
    offset += make_eir_packet_data(&buf[offset], offset,
        HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA,
        (void *)user_tag_string, tag_len);
#endif

    if (!adv_name_ok) {
        char *gap_name  = ble_comm_get_gap_name();
        u8    name_len  = strlen(gap_name);
        u8    valid_len = ADV_RSP_PACKET_MAX - (offset + 2);
        if (name_len > valid_len) name_len = valid_len;
        offset += make_eir_packet_data(&buf[offset], offset,
            HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME,
            (void *)gap_name, name_len);
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        log_info("ERROR: rsp_data overflow\n");
        return -1;
    }

    trans_server_adv_config.rsp_data_len = offset;
    trans_server_adv_config.rsp_data     = trans_scan_rsp_data;
    return 0;
}

static void trans_adv_config_set(void)
{
    trans_make_set_adv_data();
    trans_make_set_rsp_data();

    trans_server_adv_config.adv_interval = ADV_INTERVAL_MIN;
    trans_server_adv_config.adv_auto_do  = 1;
    trans_server_adv_config.adv_type     = ADV_IND;
    trans_server_adv_config.adv_channel  = ADV_CHANNEL_ALL;
    memset(trans_server_adv_config.direct_address_info, 0, 7);

    ble_gatt_server_set_adv_config(&trans_server_adv_config);
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

void trans_server_init(void)
{
    ble_gatt_server_set_profile(trans_profile_data, sizeof(trans_profile_data));
    trans_adv_config_set();
}

void trans_disconnect(void)
{
    if (trans_con_handle) {
        ble_comm_disconnect(trans_con_handle);
    }
}

void bt_ble_before_start_init(void)
{
    ble_comm_init(&trans_gatt_control_block);
}

/*************************************************************************************************/
/*!
 *  \brief       Send MMC5603 sensor data via notifications (if enabled)
 *
 *  \note       Called periodically by a timer
 */
/*************************************************************************************************/
static void trans_send_sensor_data(u8 *data, u8 len)
{
    if (!trans_con_handle)
        return;

    if (ble_comm_att_check_send(trans_con_handle, len) &&
        (ble_gatt_server_characteristic_ccc_get(trans_con_handle,
         ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE) == ATT_OP_NOTIFY))
    {
        int ret = ble_comm_att_send_data(trans_con_handle,
                                         ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
                                         data,
                                         len,
                                         ATT_OP_AUTO_READ_CCC);
        if (ret != 0) {
            log_info("BLE send busy: %d\n", ret);
        }
    }
}

void bt_ble_init(void)
{
    motor_mode    = MOTOR_MODE_ESC;
    trans_con_handle = 0;

    /* Device name reflects default mode */
    //ble_comm_set_config_name("ESC_PWM", 0);
    ble_comm_set_config_name("GPS_PWM", 0);

    trans_server_init();

    /* RF power */
    //bt_max_pwr_set(6, 6, 6, 10);
    //ble_set_fix_pwr(10);

    ble_module_enable(1);

    /* Watchdog timer: check every 200 ms */
    //sys_timer_add(NULL, pwm_watchdog, 200);
}

void bt_ble_exit(void)
{
    ble_module_enable(0);
    ble_comm_exit();
}

void ble_module_enable(u8 en)
{
    ble_comm_module_enable(en);
}

#endif /* CONFIG_APP_SPP_LE */
