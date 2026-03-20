/*********************************************************************************************
    *   Filename        : ble_ftms.c

    *   Description     :

    *   Author          : JM

    *   Email           : zh-jieli.com

    *   Last modifiled  : 2024-08-29 11:14

    *   Copyright:(c)JIELI  2011-2016  @ , All Rights Reserved.
*********************************************************************************************/
#include "system/app_core.h"
#include "system/includes.h"

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
#include "ble_ftms_profile.h"

#if CONFIG_APP_FTMS

#if LE_DEBUG_PRINT_EN
#define log_info(x, ...)  printf("[BLE_FTMS]" x " ", ## __VA_ARGS__)
#define log_info_hexdump  put_buf

#else
#define log_info(...)
#define log_info_hexdump(...)
#endif

//demo 实现了协议的广播包和特征，数据订阅需要给根据不同设备进行添加，参看文档
//https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/

/*
 打开流控使能后,确定使能接口 att_server_flow_enable 被调用
 然后使用过程 通过接口 att_server_flow_hold 来控制流控开关
 注意:流控只能控制对方使用带响应READ/WRITE等命令方式
 例如:ATT_WRITE_REQUEST = 0x12
 */
#define ATT_DATA_RECIEVT_FLOW      0//流控功能使能


//测试NRF连接,工具不会主动发起交换流程,需要手动操作; 但设备可配置主动发起MTU长度交换请求
#define ATT_MTU_REQUEST_ENALBE     0    /*配置1,就是设备端主动发起交换*/

//检测对方的系统类型，ios or 非ios
#define ATT_CHECK_REMOTE_REQUEST_ENALBE     0    /*配置1,就是设备端主动检查*/

//ATT发送的包长,    note: 23 <=need >= MTU
#define ATT_LOCAL_MTU_SIZE        (512) /*一般是主机发起交换,如果主机没有发起,设备端也可以主动发起(ATT_MTU_REQUEST_ENALBE set 1)*/

//ATT缓存的buffer支持缓存数据包个数
#define ATT_PACKET_NUMS_MAX       (2)

//ATT缓存的buffer大小,  note: need >= 23,可修改
#define ATT_SEND_CBUF_SIZE        (ATT_PACKET_NUMS_MAX * (ATT_PACKET_HEAD_SIZE + ATT_LOCAL_MTU_SIZE))

// 广播周期 (unit:0.625ms)
#define ADV_INTERVAL_MIN          (160 * 5)//


//---------------
//连接参数更新请求设置
//是否使能参数请求更新,0--disable, 1--enable
static uint8_t ftms_connection_update_enable = 1; ///0--disable, 1--enable

//请求的参数数组表,排队方式请求;哪组对方接受就用那组
static const struct conn_update_param_t ftms_connection_param_table[] = {
    {16, 24, 10, 600},//11
    {12, 28, 10, 600},//3.7
    {8,  20, 10, 600},
};

//共可用的参数组数
#define CONN_PARAM_TABLE_CNT      (sizeof(ftms_connection_param_table)/sizeof(struct conn_update_param_t))

#define EIR_TAG_STRING   0xd6, 0x05, 0x08, 0x00, 'J', 'L', 'A', 'I', 'S', 'D','K'
static const char user_tag_string[] = {EIR_TAG_STRING};

static u8  ftms_adv_data[ADV_RSP_PACKET_MAX];//max is 31
static u8  ftms_scan_rsp_data[ADV_RSP_PACKET_MAX];//max is 31
static u8  ftms_test_read_write_buf[8];
static u16 ftms_con_handle;
static adv_cfg_t ftms_server_adv_config;
//-------------------------------------------------------------------------------------
static uint16_t ftms_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int ftms_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int ftms_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param);
extern void uart_db_regiest_recieve_callback(void *rx_cb);
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
//输入passkey 加密
#define PASSKEY_ENABLE                     0

static const sm_cfg_t ftms_sm_init_config = {
    .slave_security_auto_req = 0,
    .slave_set_wait_security = 0,

#if PASSKEY_ENABLE
    .io_capabilities = IO_CAPABILITY_DISPLAY_ONLY,
#else
    .io_capabilities = IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
#endif

    .authentication_req_flags = SM_AUTHREQ_BONDING | SM_AUTHREQ_MITM_PROTECTION,
    .min_key_size = 7,
    .max_key_size = 16,
    .sm_cb_packet_handler = NULL,
};

const gatt_server_cfg_t ftms_server_init_cfg = {
    .att_read_cb = &ftms_att_read_callback,
    .att_write_cb = &ftms_att_write_callback,
    .event_packet_handler = &ftms_event_packet_handler,
};

static gatt_ctrl_t ftms_gatt_control_block = {
    //public
    .mtu_size = ATT_LOCAL_MTU_SIZE,
    .cbuffer_size = ATT_SEND_CBUF_SIZE,
    .multi_dev_flag	= 0,

    //config
#if CONFIG_BT_GATT_SERVER_NUM
    .server_config = &ftms_server_init_cfg,
#else
    .server_config = NULL,
#endif

    .client_config = NULL,

#if CONFIG_BT_SM_SUPPORT_ENABLE
    .sm_config = &ftms_sm_init_config,
#else
    .sm_config = NULL,
#endif
    //cbk,event handle
    .hci_cb_packet_handler = NULL,
};

/*************************************************************************************************/
/*!
 *  \brief      发送请求连接参数表
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void ftms_send_connetion_updata_deal(u16 conn_handle)
{
    if (ftms_connection_update_enable) {
        if (0 == ble_gatt_server_connetion_update_request(conn_handle, ftms_connection_param_table, CONN_PARAM_TABLE_CNT)) {
            ftms_connection_update_enable = 0;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief      回连状态，使能所有profile
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      配对绑定的方式，主机回连不是在使能server的通知开关，需要自己打开
 */
/*************************************************************************************************/
static void ftms_resume_all_ccc_enable(u16 conn_handle, u8 update_request)
{
    log_info("resume_all_ccc_enable\n");

#if RCSP_BTMATE_EN
    ble_gatt_server_characteristic_ccc_set(conn_handle, ATT_CHARACTERISTIC_ae02_02_CLIENT_CONFIGURATION_HANDLE, ATT_OP_NOTIFY);
#endif

    if (update_request) {
        ftms_send_connetion_updata_deal(conn_handle);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      反馈检查对方的操作系统
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note 参考识别手机系统
 */
/*************************************************************************************************/
static void ftms_check_remote_result(u16 con_handle, remote_type_e remote_type)
{
    char *str;
    if (REMOTE_TYPE_IOS == remote_type) {
        str = "is";
    } else {
        str = "not";
    }

    log_info("ftms_check %02x:remote_type= %02x, %s ios", con_handle, remote_type, str);
}

/*************************************************************************************************/
/*!
 *  \brief      处理gatt 返回的事件（hci && gatt）
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int ftms_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param)
{
    log_info("event: %02x,size= %d\n", event, size);

    switch (event) {

    case GATT_COMM_EVENT_CAN_SEND_NOW:
#if TEST_AUDIO_DATA_UPLOAD
        ftms_test_send_audio_data(0);
#endif
        break;

    case GATT_COMM_EVENT_SERVER_INDICATION_COMPLETE:
        log_info("INDICATION_COMPLETE:con_handle= %04x,att_handle= %04x\n", \
                 little_endian_read_16(packet, 0), little_endian_read_16(packet, 2));
        break;


    case GATT_COMM_EVENT_CONNECTION_COMPLETE:
        ftms_con_handle = little_endian_read_16(packet, 0);
        ftms_connection_update_enable = 1;

        log_info("connection_handle:%04x\n", little_endian_read_16(packet, 0));
        log_info("connection_handle:%04x, rssi= %d\n", ftms_con_handle, ble_vendor_get_peer_rssi(ftms_con_handle));
        log_info("peer_address_info:");
        put_buf(&ext_param[7], 7);

        log_info("con_interval = %d\n", little_endian_read_16(ext_param, 14 + 0));
        log_info("con_latency = %d\n", little_endian_read_16(ext_param, 14 + 2));
        log_info("cnn_timeout = %d\n", little_endian_read_16(ext_param, 14 + 4));

#if ATT_MTU_REQUEST_ENALBE
        att_server_set_exchange_mtu(ftms_con_handle);/*主动请求MTU长度交换*/
#endif


#if ATT_CHECK_REMOTE_REQUEST_ENALBE
        att_server_set_check_remote(ftms_con_handle, ftms_check_remote_result);
#endif
        break;

    case GATT_COMM_EVENT_DISCONNECT_COMPLETE:
        log_info("disconnect_handle:%04x,reason= %02x\n", little_endian_read_16(packet, 0), packet[2]);
        if (ftms_con_handle == little_endian_read_16(packet, 0)) {
            ftms_con_handle = 0;
        }
        break;

    case GATT_COMM_EVENT_ENCRYPTION_CHANGE:
        log_info("ENCRYPTION_CHANGE:handle=%04x,state=%d,process =%d", little_endian_read_16(packet, 0), packet[2], packet[3]);
        if (packet[3] == LINK_ENCRYPTION_RECONNECT) {
            ftms_resume_all_ccc_enable(little_endian_read_16(packet, 0), 1);
        }
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_COMPLETE:
        log_info("conn_param update_complete:%04x\n", little_endian_read_16(packet, 0));
        log_info("update_interval = %d\n", little_endian_read_16(ext_param, 6 + 0));
        log_info("update_latency = %d\n", little_endian_read_16(ext_param, 6 + 2));
        log_info("update_timeout = %d\n", little_endian_read_16(ext_param, 6 + 4));
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_REQUEST_RESULT:
        break;

    case GATT_COMM_EVENT_MTU_EXCHANGE_COMPLETE:
        log_info("con_handle= %02x, ATT MTU = %u\n", little_endian_read_16(packet, 0), little_endian_read_16(packet, 2));
        break;

    case GATT_COMM_EVENT_SERVER_STATE:
        log_info("server_state: handle=%02x,%02x\n", little_endian_read_16(packet, 1), packet[0]);
        break;

    case GATT_COMM_EVENT_SM_PASSKEY_INPUT: {
        u32 *key = little_endian_read_32(packet, 2);
        *key = 888888;
        r_printf("input_key:%6u\n", *key);
    }
    break;

    default:
        break;
    }
    return 0;
}

static int app_send_treadmill_data(u8 flags)
{
    u8 tx_data[10] = {0x02, 0x00};
    u8 len ;
    switch (flags) {
    case 0: //Receive Complete Data Record
        little_endian_store_16(tx_data, 0, BIT(7)); //flags
        little_endian_store_16(tx_data, 2, 2); //instantaneous speed
        little_endian_store_16(tx_data, 4, 2); //total energy
        little_endian_store_16(tx_data, 6, 2); //energy per hour
        tx_data[8] = 0;//energy per minute
        len = 9;
        break;
    case 1: //Average Speed Present
        little_endian_store_16(tx_data, 0, BIT(1)); //flags
        little_endian_store_16(tx_data, 2, 2); //instantaneous speed
        little_endian_store_16(tx_data, 2, 2); //Average Speed
        len = 6;
        break;
    }

    return ble_comm_att_send_data(ftms_con_handle, ATT_CHARACTERISTIC_2acd_01_VALUE_HANDLE, tx_data, len, ATT_OP_AUTO_READ_CCC);
}

/*************************************************************************************************/
/*!
 *  \brief      处理client 读操作
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      profile的读属性uuid 有配置 DYNAMIC 关键字，就有read_callback 回调
 */
/*************************************************************************************************/
// ATT Client Read Callback for Dynamic Data
// - if buffer == NULL, don't copy data, just return size of value
// - if buffer != NULL, copy data and return number bytes copied
// @param con_handle of hci le connection
// @param attribute_handle to be read
// @param offset defines start of attribute value
// @param buffer
// @param buffer_size
static uint16_t ftms_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    uint16_t  att_value_len = 0;
    uint16_t handle = att_handle;

    log_info("read_callback,conn_handle =%04x, handle=%04x,buffer=%08x\n", connection_handle, handle, (u32)buffer);

    memset(ftms_test_read_write_buf, 0, 8);
    switch (handle) {
    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE: {
        char *gap_name = ble_comm_get_gap_name();
        att_value_len = strlen(gap_name);

        if ((offset >= att_value_len) || (offset + buffer_size) > att_value_len) {
            break;
        }

        if (buffer) {
            memcpy(buffer, &gap_name[offset], buffer_size);
            att_value_len = buffer_size;
            log_info("\n------read gap_name: %s\n", gap_name);
        }
    }
    break;

    case ATT_CHARACTERISTIC_2ad7_01_VALUE_HANDLE:   //Supported Heart Rate Range  数据结构参看文档4.15
        att_value_len = 3;
        ftms_test_read_write_buf[0] = 50;
        ftms_test_read_write_buf[1] = 96;
        ftms_test_read_write_buf[2] = 0x02;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;

    case ATT_CHARACTERISTIC_2ad8_01_VALUE_HANDLE:   //Supported Power Range 数据结构参看文档4.14
        att_value_len = 6;
        ftms_test_read_write_buf[0] = 0x01;
        ftms_test_read_write_buf[1] = 0x00;
        ftms_test_read_write_buf[2] = 0x02;
        ftms_test_read_write_buf[3] = 0x01;
        ftms_test_read_write_buf[4] = 0x02;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;

    case ATT_CHARACTERISTIC_2ad6_01_VALUE_HANDLE:   //Supported Resistance Level Range
        att_value_len = 6;
        ftms_test_read_write_buf[0] = 0x01;
        ftms_test_read_write_buf[1] = 0x09;
        ftms_test_read_write_buf[2] = 0x02;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;
    case ATT_CHARACTERISTIC_2ad5_01_VALUE_HANDLE:   //Supported Inclination Range
        att_value_len = 6;
        ftms_test_read_write_buf[0] = 0x01;
        ftms_test_read_write_buf[1] = 0x00;
        ftms_test_read_write_buf[2] = 0x02;
        ftms_test_read_write_buf[3] = 0x01;
        ftms_test_read_write_buf[4] = 0x02;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;
    case ATT_CHARACTERISTIC_2ad4_01_VALUE_HANDLE:   //Supported Speed Range
        att_value_len = 6;
        ftms_test_read_write_buf[0] = 0x01;
        ftms_test_read_write_buf[1] = 0x00;
        ftms_test_read_write_buf[2] = 0x02;
        ftms_test_read_write_buf[3] = 0x01;
        ftms_test_read_write_buf[4] = 0x02;
        ftms_test_read_write_buf[5] = 0x00;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;
    case ATT_CHARACTERISTIC_2ad3_01_VALUE_HANDLE: //Training Status
        att_value_len = 4;
        ftms_test_read_write_buf[0] = 0x03;
        ftms_test_read_write_buf[1] = 0x03;
        ftms_test_read_write_buf[2] = '2';
        ftms_test_read_write_buf[3] = '5';
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;
    case ATT_CHARACTERISTIC_2acc_01_VALUE_HANDLE:
        att_value_len = 8;

        ftms_test_read_write_buf[1] = 0x01;
        ftms_test_read_write_buf[0] = 0x03;
        ftms_test_read_write_buf[5] = 0x01;
        ftms_test_read_write_buf[4] = 0x03;
        if (buffer) {
            memcpy(buffer, ftms_test_read_write_buf, buffer_size);
            att_value_len = buffer_size;
        }
        break;

    case ATT_CHARACTERISTIC_2acd_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ace_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2acf_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad0_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad1_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad2_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad3_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad9_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ada_01_CLIENT_CONFIGURATION_HANDLE:
        if (buffer) {
            buffer[0] = ble_gatt_server_characteristic_ccc_get(connection_handle, handle);
            buffer[1] = 0;
        }
        att_value_len = 2;
        break;

    default:
        break;
    }

    log_info("att_value_len= %d\n", att_value_len);
    return att_value_len;
}


/*************************************************************************************************/
/*!
 *  \brief      处理client write操作
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      profile的写属性uuid 有配置 DYNAMIC 关键字，就有write_callback 回调
 */
/*************************************************************************************************/
// ATT Client Write Callback for Dynamic Data
// @param con_handle of hci le connection
// @param attribute_handle to be written
// @param transaction - ATT_TRANSACTION_MODE_NONE for regular writes, ATT_TRANSACTION_MODE_ACTIVE for prepared writes and ATT_TRANSACTION_MODE_EXECUTE
// @param offset into the value - used for queued writes and long attributes
// @param buffer
// @param buffer_size
// @param signature used for signed write commmands
// @returns 0 if write was ok, ATT_ERROR_PREPARE_QUEUE_FULL if no space in queue, ATT_ERROR_INVALID_OFFSET if offset is larger than max buffer

static int ftms_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    int result = 0;
    u16 tmp16;

    u16 handle = att_handle;

    log_info("write_callback,conn_handle =%04x, handle =%04x,size =%d\n", connection_handle, handle, buffer_size);

    switch (handle) {

    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE:
        break;

    case ATT_CHARACTERISTIC_2acd_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ace_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2acf_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad0_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad1_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad2_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad3_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ad9_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2ada_01_CLIENT_CONFIGURATION_HANDLE:
        ftms_send_connetion_updata_deal(connection_handle);
        log_info("\n------write ccc:%04x,%02x\n", handle, buffer[0]);
        ble_gatt_server_characteristic_ccc_set(connection_handle, handle, buffer[0]);
        /* int app_send_treadmill_data(u8 flags); */
        /* if (buffer[0]) { */
        /*     app_send_treadmill_data(1); */
        /* } */
        break;

    case ATT_CHARACTERISTIC_2ad9_01_VALUE_HANDLE:
        log_info("\n-2ad9_rx(%d):", buffer_size);
        put_buf(buffer, buffer_size);

        u8 tx_data[2] = {0x80, 0x07};
        if (ble_gatt_server_characteristic_ccc_get(ftms_con_handle, ATT_CHARACTERISTIC_2ad9_01_CLIENT_CONFIGURATION_HANDLE)) {
            log_info("-loop send1\n");
            ble_comm_att_send_data(connection_handle, ATT_CHARACTERISTIC_2ad9_01_VALUE_HANDLE, tx_data, 2, ATT_OP_AUTO_READ_CCC);
        }
        break;

    default:
        break;
    }
    return 0;
}


/*************************************************************************************************/
/*!
 *  \brief      组织adv包数据，放入buff
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static u8  adv_name_ok = 0;//name 优先存放在ADV包
static int ftms_make_set_adv_data(void)
{
    u8 offset = 0;
    u8 *buf = ftms_adv_data;

#if DOUBLE_BT_SAME_MAC
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_LE_AND_EDR_SAME_CONTROLLER, 1);
#else
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_EDR_NOT_SUPPORTED, 1);
#endif

    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, 0x1826, 2);
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x01, 1);
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, 0x0001, 2);


    if (offset > ADV_RSP_PACKET_MAX) {
        puts("***ftms_adv_data overflow!!!!!!\n");
        return -1;
    }
    log_info("ftms_adv_data(%d):", offset);
    log_info_hexdump(buf, offset);
    ftms_server_adv_config.adv_data_len = offset;
    ftms_server_adv_config.adv_data = ftms_adv_data;
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      组织rsp包数据，放入buff
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int ftms_make_set_rsp_data(void)
{
    u8 offset = 0;
    u8 *buf = ftms_scan_rsp_data;

#if RCSP_BTMATE_EN
    u8  tag_len = sizeof(user_tag_string);
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA, (void *)user_tag_string, tag_len);
#endif

    if (!adv_name_ok) {
        char *gap_name = ble_comm_get_gap_name();
        u8 name_len = strlen(gap_name);
        u8 vaild_len = ADV_RSP_PACKET_MAX - (offset + 2);
        if (name_len > vaild_len) {
            name_len = vaild_len;
        }
        offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)gap_name, name_len);
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        puts("***rsp_data overflow!!!!!!\n");
        return -1;
    }

    log_info("rsp_data(%d):", offset);
    log_info_hexdump(buf, offset);
    ftms_server_adv_config.rsp_data_len = offset;
    ftms_server_adv_config.rsp_data = ftms_scan_rsp_data;
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      配置广播参数
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      开广播前配置都有效
 */
/*************************************************************************************************/
static void ftms_adv_config_set(void)
{
    int ret = 0;
    ret |= ftms_make_set_adv_data();
    ret |= ftms_make_set_rsp_data();

    ftms_server_adv_config.adv_interval = ADV_INTERVAL_MIN;
    ftms_server_adv_config.adv_auto_do = 1;
    ftms_server_adv_config.adv_type = ADV_IND;
    ftms_server_adv_config.adv_channel = ADV_CHANNEL_ALL;
    memset(ftms_server_adv_config.direct_address_info, 0, 7);

    if (ret) {
        log_info("adv_setup_init fail!!!\n");
        return;
    }
    ble_gatt_server_set_adv_config(&ftms_server_adv_config);
}

/*************************************************************************************************/
/*!
 *  \brief      server init初始化
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ftms_server_init(void)
{
    log_info("%s", __FUNCTION__);
    ble_gatt_server_set_profile(ftms_profile_data, sizeof(ftms_profile_data));
    ftms_adv_config_set();
}

/*************************************************************************************************/
/*!
 *  \brief      断开连接
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ftms_disconnect(void)
{
    log_info("%s", __FUNCTION__);
    if (ftms_con_handle) {
        ble_comm_disconnect(ftms_con_handle);
    }
}


/*************************************************************************************************/
/*!
 *  \brief      协议栈初始化前调用
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/

void bt_ble_before_start_init(void)
{
    log_info("%s", __FUNCTION__);
    ble_comm_init(&ftms_gatt_control_block);
}


/*************************************************************************************************/
/*!
 *  \brief      控制应答对方READ/WRITE行为的响应包RESPONSE的回复
 *
 *  \param      [in]流控使能 en: 1-停止收数 or 0-继续收数
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
int ble_ftms_flow_enable(u8 en)
{
    int ret = -1;

#if ATT_DATA_RECIEVT_FLOW
    if (ftms_con_handle) {
        att_server_flow_hold(ftms_con_handle, en);
        ret = 0;
        log_info("ble_ftms_flow_enable:%d\n", en);
    }
#endif

    return ret;
}

//for test
static void timer_ftms_flow_test(void)
{
    static u8 sw = 0;
    if (ftms_con_handle) {
        sw = !sw;
        ble_ftms_flow_enable(sw);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      模块初始化
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void bt_ble_init(void)
{
    log_info("%s\n", __FUNCTION__);
    log_info("ble_file: %s", __FILE__);

#if DOUBLE_BT_SAME_NAME
    ble_comm_set_config_name(bt_get_local_name(), 0);
#else
    ble_comm_set_config_name(bt_get_local_name(), 1);
#endif
    ftms_con_handle = 0;
    ftms_server_init();


#if ATT_DATA_RECIEVT_FLOW
    log_info("att_server_flow_enable\n");
    att_server_flow_enable(1);
//    sys_timer_add(0, timer_ftms_flow_test, 5000);
#endif

    ble_module_enable(1);

}

/*************************************************************************************************/
/*!
 *  \brief      模块退出
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void bt_ble_exit(void)
{
    log_info("%s\n", __FUNCTION__);

    ble_module_enable(0);
    ble_comm_exit();
}

/*************************************************************************************************/
/*!
 *  \brief      模块开发使能
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ble_module_enable(u8 en)
{
    ble_comm_module_enable(en);
}

/*************************************************************************************************/
/*!
 *  \brief      testbox 按键测试
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ble_server_send_test_key_num(u8 key_num)
{
    if (ftms_con_handle) {
        if (get_remote_test_flag()) {
            ble_op_test_key_num(ftms_con_handle, key_num);
        } else {
            log_info("-not conn testbox\n");
        }
    }
}


#endif


