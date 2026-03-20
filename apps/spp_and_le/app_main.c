/*********************************************************************************************
    *   Filename        : app_main.c

    *   Description     :

    *   Copyright:(c)JIELI  2011-2019  @ , All Rights Reserved.
*********************************************************************************************/
#include "system/includes.h"
#include "app_config.h"
#include "app_action.h"
#include "app_main.h"
#include "update.h"
#include "update_loader_download.h"
#include "app_charge.h"
#include "app_power_manage.h"
#include "asm/charge.h"

#include "examples/trans_data/ble_trans.h"

#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
#include "jl_kws/jl_kws_api.h"
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

#define LOG_TAG_CONST       APP
#define LOG_TAG             "[APP]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
/* #define LOG_DUMP_ENABLE */
#define LOG_CLI_ENABLE
#include "debug.h"

#include "asm/iic_hw.h"
#define MMC5603_ADDR        0x30
#define MMC5603_REG_XOUT0   0x00
#define MMC5603_REG_ODR    0x1A
#define MMC5603_REG_CTRL0  0x1B
#define MMC5603_REG_CTRL1  0x1C
#define MMC5603_REG_CTRL2  0x1D // Required to enable CMM
#define MMC5603_CTRL0_TM    0x01   // Trigger one-shot measurement
//#define MMC5603_CTRL0_TM    0x03   // Read temp as well, not working for continues mode
//#define RAW_DATA_LEN         9      // X, Y, Z as 24-bit each
#define RAW_DATA_LEN         6      // Only read 16 bits to fit inside BLE 20 bytes payload
//Update MMC5603_RAW_SIZE in ble_trans.c as well

// Initialize I2C as master
hw_iic_dev i2c_dev = 0; // Use I2C hardware instance 0


/*任务列表   */
const struct task_info task_info_table[] = {
#if CONFIG_APP_FINDMY
    {"app_core",            1,     0,   640 * 2,   128  },
#else
    {"app_core",            1,     0,   640,   128  },
#endif

    {"sys_event",           7,     0,   256,   0    },
    {"btctrler",            4,     0,   512,   256  },
    {"btencry",             1,     0,   512,   128  },
    {"btstack",             3,     0,   768,   256   },
    {"systimer",		    7,	   0,   128,   0	},
    {"update",				1,	   0,   512,   0    },
    {"dw_update",		 	2,	   0,   256,   128  },
#if (RCSP_BTMATE_EN)
    {"rcsp_task",		    2,	   0,   640,	0},
#endif
#if(USER_UART_UPDATE_ENABLE)
    {"uart_update",	        1,	   0,   256,   128	},
#endif
#if (XM_MMA_EN)
    {"xm_mma",   		    2,	   0,   640,   256	},
#endif
    {"usb_msd",           	1,     0,   512,   128  },
#if TCFG_AUDIO_ENABLE
    {"audio_dec",           3,     0,   768,   128  },
    {"audio_enc",           4,     0,   512,   128  },
#endif/*TCFG_AUDIO_ENABLE*/
#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    {"kws",                 2,     0,   256,   64   },
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */
#if (TUYA_DEMO_EN)
    {"user_deal",           2,     0,   512,   512  },//定义线程 tuya任务调度
#endif
#if (CONFIG_APP_HILINK)
    {"hilink_task",         2,     0,   1024,   0},//定义线程 hilink任务调度
#endif
    {0, 0},
};

APP_VAR app_var;

void app_var_init(void)
{
    app_var.play_poweron_tone = 1;

    app_var.auto_off_time =  TCFG_AUTO_SHUT_DOWN_TIME;
    app_var.warning_tone_v = 340;
    app_var.poweroff_tone_v = 330;
}

__attribute__((weak))
u8 get_charge_online_flag(void)
{
    return 0;
}

void clr_wdt(void);
void check_power_on_key(void)
{
#if TCFG_POWER_ON_NEED_KEY

    u32 delay_10ms_cnt = 0;
    while (1) {
        clr_wdt();
        os_time_dly(1);

        extern u8 get_power_on_status(void);
        if (get_power_on_status()) {
            log_info("+");
            delay_10ms_cnt++;
            if (delay_10ms_cnt > 70) {
                /* extern void set_key_poweron_flag(u8 flag); */
                /* set_key_poweron_flag(1); */
                return;
            }
        } else {
            log_info("-");
            delay_10ms_cnt = 0;
            log_info("enter softpoweroff\n");
            power_set_soft_poweroff();
        }
    }
#endif
}

int retval = -1;

uint16_t readMS = 100;
// Definition (storage allocated here)
//volatile uint8_t mmc5603_raw[RAW_DATA_LEN];
volatile uint8_t sensor_valid = 0;

/* Global packed BLE packet */
static u8  ble_pkt[20];
static u8  ble_seq = 0;

/* MMC5603 decoded values shared between read and pack */
static int16_t mx_raw, my_raw, mz_raw;

/* Update process_mmc5603_full to save raw values */
void process_mmc5603_full(uint8_t *raw) {
    uint16_t ux = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t uy = ((uint16_t)raw[2] << 8) | raw[3];
    uint16_t uz = ((uint16_t)raw[4] << 8) | raw[5];
    mx_raw = (int16_t)(ux - 32768);
    my_raw = (int16_t)(uy - 32768);
    mz_raw = (int16_t)(uz - 32768);
}

/* Pack all sensor data into BLE packet */
static void pack_ble_packet(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
    ble_pkt[0]  = 0xA1;
    ble_pkt[1]  = ble_seq++;
    /* Accel */
    ble_pkt[2]  = ax & 0xFF;  ble_pkt[3]  = ax >> 8;
    ble_pkt[4]  = ay & 0xFF;  ble_pkt[5]  = ay >> 8;
    ble_pkt[6]  = az & 0xFF;  ble_pkt[7]  = az >> 8;
    /* Gyro */
    ble_pkt[8]  = gx & 0xFF;  ble_pkt[9]  = gx >> 8;
    ble_pkt[10] = gy & 0xFF;  ble_pkt[11] = gy >> 8;
    ble_pkt[12] = gz & 0xFF;  ble_pkt[13] = gz >> 8;
    /* Mag */
    ble_pkt[14] = mx_raw & 0xFF;  ble_pkt[15] = mx_raw >> 8;
    ble_pkt[16] = my_raw & 0xFF;  ble_pkt[17] = my_raw >> 8;
    ble_pkt[18] = mz_raw & 0xFF;  ble_pkt[19] = mz_raw >> 8;
}

// Write one byte to a register
static int mmc5603_write_reg(u8 reg, u8 data) {
    hw_iic_start(i2c_dev);
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 0)) { // Write bit
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, reg)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, data)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    hw_iic_stop(i2c_dev);
    return 0;
}

// Read multiple bytes starting from a register
static int mmc5603_read_regs(u8 reg, u8 *buf, int len) {
    hw_iic_start(i2c_dev);
    // Write register address
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 0)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    if (!hw_iic_tx_byte(i2c_dev, reg)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    // Repeated start (use stop/start if restart not supported)
    hw_iic_stop(i2c_dev);
    hw_iic_start(i2c_dev);
    // Send read address
    if (!hw_iic_tx_byte(i2c_dev, (MMC5603_ADDR << 1) | 1)) {
        hw_iic_stop(i2c_dev);
        return -1;
    }
    // Read data
    int read_len = hw_iic_read_buf(i2c_dev, buf, len);
    hw_iic_stop(i2c_dev);
    return read_len;
}

static int mmc5603_read_raw(u8 *raw) {
    if (mmc5603_write_reg(MMC5603_REG_CTRL0, MMC5603_CTRL0_TM) != 0)
        return -1;
    os_time_dly(1); //delay_2ms(2);  // Wait for conversion
    //os_time_dly(1);
    return mmc5603_read_regs(MMC5603_REG_XOUT0, raw, RAW_DATA_LEN);
}

/*
void process_mmc5603_full(uint8_t *raw) {
    clr_wdt();

    uint16_t ux = ((uint16_t)raw[0] << 8) | raw[1];
    uint32_t uy = ((uint16_t)raw[2] << 8) | raw[3];
    uint32_t uz = ((uint16_t)raw[4] << 8) | raw[5];

    // Math: If you shifted 20-bit down to 16-bit (dropped 4 bits),
    // the sensitivity becomes 1.0 mG per LSB.
    int x_mg = (ux - 32768);
    int y_mg = (uy - 32768);
    int z_mg = (uz - 32768);

    clr_wdt();

    printf("X:%d Y:%d Z:%d mG \r\n", x_mg, y_mg, z_mg);
}
*/

void init_mmc5603(){
    // 1. Set Bandwidth to BW=01 (~4ms measurement time)
    mmc5603_write_reg(MMC5603_REG_CTRL1, 0x01);
    os_time_dly(1);

    // 2. Disable CMM in CTRL2 (Set bit 4 to 0)
    mmc5603_write_reg(MMC5603_REG_CTRL2, 0x00);
    os_time_dly(1);

    // 3. Configure CTRL0: Enable Auto Set/Reset (0x20) but keep CMM bits (0x10, 0x80) OFF
    mmc5603_write_reg(MMC5603_REG_CTRL0, 0x20);
    os_time_dly(1);
}


u8 read_mmc5603_to_buffer()
{
    u8 raw[RAW_DATA_LEN];
    int ret = mmc5603_read_raw(raw);
    clr_wdt();
    if (ret == RAW_DATA_LEN) {
        sensor_valid = 1;
        process_mmc5603_full(raw);  /* updates mx_raw/my_raw/mz_raw */
    } else {
        sensor_valid = 0;
        printf("mmc5603 read failed, ret=%d\n", ret);
    }
    return sensor_valid;
}


#define QMI8658_ADDR    0x6A
#define QMI8658_REG_CTRL2      0x03
#define QMI8658_REG_CTRL3      0x04
#define QMI8658_REG_CTRL5      0x06
#define QMI8658_REG_CTRL7      0x08

/* QMI8658 Register Definitions */
#define QMI8658_REG_WHO_AM_I   0x00
#define QMI8658_REG_CTRL1      0x02
#define QMI8658_REG_CTRL2      0x03
#define QMI8658_REG_CTRL3      0x04
#define QMI8658_REG_CTRL5      0x06
#define QMI8658_REG_CTRL7      0x08
#define QMI8658_REG_AX_L       0x35  /* Accel X low byte */
#define QMI8658_REG_GX_L       0x3B  /* Gyro  X low byte */

#define QMI8658_WHO_AM_I_VAL   0x05
#define QMI8658_ADDR           0x6B  /* SDO low  => 0x6A, SDO high => 0x6B */

void qmi8658_write_reg(hw_iic_dev iic, u8 reg, u8 data)
{
    hw_iic_start(iic);                                    // start_signal = 0x81
    hw_iic_tx_byte(iic, (QMI8658_ADDR << 1) | 0);        // kick-starts, sends addr
    hw_iic_tx_byte(iic, reg);                             // sends reg
    hw_iic_tx_byte(iic, data);                            // sends data
    hw_iic_stop(iic);                                     // stop, reset start_signal
}

static int qmi8658_read_reg(hw_iic_dev iic, u8 reg, u8 *data, int len)
{
    hw_iic_stop(iic);   // ensure clean state before every transaction
    delay_us_by_nop(200);

    /* Write phase */
    hw_iic_start(iic);                                    // start_signal = 0x81
    if (!hw_iic_tx_byte(iic, (QMI8658_ADDR << 1) | 0)) {  // ! added
        printf("[QMI8658] read_reg: addr_w NAK\n");
        hw_iic_stop(iic);
        return -1;
    }
    hw_iic_tx_byte(iic, reg);

    /* Read phase — second START triggers restart inside tx_byte */
    hw_iic_start(iic);                                    // start_signal = 0x82
    hw_iic_tx_byte(iic, (QMI8658_ADDR << 1) | 1);        // triggers iic_restart()

    for (int i = 0; i < len; i++) {
        u8 ack = (i < len - 1) ? 1 : 0;
        data[i] = hw_iic_rx_byte(iic, ack);
    }

    hw_iic_stop(iic);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */
int qmi8658_init(hw_iic_dev iic)
{
    /*
    u8 who_am_i = 0;
    // --- Who Am I check ---
    if (qmi8658_read_reg(iic, QMI8658_REG_WHO_AM_I, &who_am_i, 1) != 0) {
        printf("[QMI8658] init: failed to read WHO_AM_I\n");
        return -1;
    }
    printf("[QMI8658] WHO_AM_I = 0x%02X (expect 0x%02X)\n",
           who_am_i, QMI8658_WHO_AM_I_VAL);

    if (who_am_i != QMI8658_WHO_AM_I_VAL) {
        printf("[QMI8658] init: WHO_AM_I mismatch, abort\n");
        return -1;
    }
    */

    /* --- CTRL1: address auto-increment ON, big-endian OFF --- */
    qmi8658_write_reg(iic, QMI8658_REG_CTRL1, 0x40);
    printf("[QMI8658] CTRL1 = 0x40 (addr auto-inc ON)\n");

    /* --- CTRL2: Accel ±8 g, 500 Hz ODR --- */
    /*   [6:4] aODR = 011 (500 Hz)  [3:1] aFS = 010 (±8 g)           */
    qmi8658_write_reg(iic, QMI8658_REG_CTRL2, 0x34);
    printf("[QMI8658] CTRL2 = 0x34 (Accel 500 Hz, ±8 g)\n");

    /* --- CTRL3: Gyro ±512 dps, 500 Hz ODR --- */
    /*   [6:4] gODR = 011 (500 Hz)  [3:1] gFS = 011 (±512 dps)       */
    qmi8658_write_reg(iic, QMI8658_REG_CTRL3, 0x36);
    printf("[QMI8658] CTRL3 = 0x36 (Gyro 500 Hz, ±512 dps)\n");

    /* --- CTRL5: Accel LPF BW/10, Gyro LPF BW/10 --- */
    qmi8658_write_reg(iic, QMI8658_REG_CTRL5, 0x11);
    printf("[QMI8658] CTRL5 = 0x11 (LPF enabled)\n");

    /* --- CTRL7: enable Accel + Gyro --- */
    qmi8658_write_reg(iic, QMI8658_REG_CTRL7, 0x03);
    printf("[QMI8658] CTRL7 = 0x03 (Accel + Gyro enabled)\n");

    printf("[QMI8658] init OK\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/* Read raw accel + gyro (6 + 6 = 12 bytes burst)                      */
/* ------------------------------------------------------------------ */
typedef struct {
    float ax, ay, az;   /* m/s²  (or g – see scale below) */
    float gx, gy, gz;   /* dps                              */
} qmi8658_data_t;

int qmi8658_read(hw_iic_dev iic, qmi8658_data_t *out)
{
    u8 raw[12];

    /* Burst-read 12 bytes starting from AX_L */
    if (qmi8658_read_reg(iic, QMI8658_REG_AX_L, raw, 12) != 0) {
        printf("[QMI8658] read: burst read failed\n");
        return -1;
    }

    /* Reconstruct signed 16-bit values (little-endian) */
    int16_t ax_raw = (int16_t)((raw[1]  << 8) | raw[0]);
    int16_t ay_raw = (int16_t)((raw[3]  << 8) | raw[2]);
    int16_t az_raw = (int16_t)((raw[5]  << 8) | raw[4]);
    int16_t gx_raw = (int16_t)((raw[7]  << 8) | raw[6]);
    int16_t gy_raw = (int16_t)((raw[9]  << 8) | raw[8]);
    int16_t gz_raw = (int16_t)((raw[11] << 8) | raw[10]);

    /* Scale factors matching CTRL2/CTRL3 settings above:
       Accel ±8 g  → 4096 LSB/g
       Gyro ±512 dps → 64   LSB/dps                       */
    const float ACCEL_SCALE = 1.0f / 4096.0f;   /* g per LSB  */
    const float GYRO_SCALE  = 1.0f / 64.0f;     /* dps per LSB */

    out->ax = ax_raw * ACCEL_SCALE;
    out->ay = ay_raw * ACCEL_SCALE;
    out->az = az_raw * ACCEL_SCALE;
    out->gx = gx_raw * GYRO_SCALE;
    out->gy = gy_raw * GYRO_SCALE;
    out->gz = gz_raw * GYRO_SCALE;

    printf("[QMI8658] raw: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
       (int)ax_raw, (int)ay_raw, (int)az_raw,
       (int)gx_raw, (int)gy_raw, (int)gz_raw);

    return 0;
}

/*
static void sensor_timer_cb(void *priv)
{
    qmi8658_data_t imu;
    qmi8658_read(i2c_dev, &imu);
    u8 ret=read_mmc5603_to_buffer();
    //if(!ret){ printf("failed to read MMC5603"); }
}
*/

static void sensor_timer_cb(void *priv)
{
    clr_wdt();

    /* 1. MMC5603 first — slow, triggers measurement */
    read_mmc5603_to_buffer();  /* updates mx_raw/my_raw/mz_raw */

    /* 2. QMI8658C — fast burst read */
    u8 raw[12];
    if (qmi8658_read_reg(i2c_dev, QMI8658_REG_AX_L, raw, 12) != 0)
        return;

    int16_t ax = (int16_t)((raw[1]  << 8) | raw[0]);
    int16_t ay = (int16_t)((raw[3]  << 8) | raw[2]);
    int16_t az = (int16_t)((raw[5]  << 8) | raw[4]);
    int16_t gx = (int16_t)((raw[7]  << 8) | raw[6]);
    int16_t gy = (int16_t)((raw[9]  << 8) | raw[8]);
    int16_t gz = (int16_t)((raw[11] << 8) | raw[10]);

    /* 3. Pack all 9 axes into 20-byte BLE packet */
    pack_ble_packet(ax, ay, az, gx, gy, gz);

    /* 4. Send only if MMC5603 also succeeded */
    if (sensor_valid)
        trans_send_sensor_data(ble_pkt, sizeof(ble_pkt));
}

/*
void hw_iic_scan(hw_iic_dev iic)
{
    printf("[SCAN] Scanning I2C bus...\n");
    for (u8 addr = 0x08; addr < 0x78; addr++) {
        hw_iic_start(iic);
        u8 ack = hw_iic_tx_byte(iic, addr << 1); // write direction
        hw_iic_stop(iic);   // ALWAYS stop, whether ACK or NAK

        if (ack) {
            printf("[SCAN] Found device at 0x%02X\n", addr);
        }
        delay_us_by_nop(200); // brief gap between probes
    }
    printf("[SCAN] Scan complete\n");
}
*/

void app_main()
{
    void *timer_handle = NULL;

    //hw_iic_bus_recover();
    retval = hw_iic_init(i2c_dev);
    if (retval != 0) {
        printf("hw_iic_init failed with code %d\n", retval);
        // Optionally, blink an LED or halt
    } else {
        printf("hw_iic_init OK\n");
        //printf("[IIC] CON0=0x%04X BAUD=0x%02X\n", JL_IIC->CON0, JL_IIC->BAUD);
        init_mmc5603();
        os_time_dly(1);
        //hw_iic_scan(i2c_dev);
        //hw_iic_stop(i2c_dev);   // force-clean start_count and start_pending
        //delay_us_by_nop(1000);  // 1ms settling
        qmi8658_init(i2c_dev);
        os_time_dly(1);

        timer_handle = sys_timer_add(NULL, sensor_timer_cb, readMS); // 200 ms period
        //timer_handle = sys_timer_add(NULL, sensor_timer_cb, 100); // 20 Hz, 50 ms period
        if (timer_handle == NULL) {
            retval = 11;
            printf("Failed to create sensor timer");
        }
    }

    struct intent it;

    if (!UPDATE_SUPPORT_DEV_IS_NULL()) {
        int update = 0;
        update = update_result_deal();
    }

    //printf(">>>>>>>>>>>>>>>>>app_main...\n");
    //printf(">>> v220,2022-11-23 >>>\n");

    if (get_charge_online_flag()) {
#if(TCFG_SYS_LVD_EN == 1)
        vbat_check_init();
#endif
    } else {
        check_power_on_voltage();
    }

#if TCFG_POWER_ON_NEED_KEY
    check_power_on_key();
#endif

#if TCFG_AUDIO_ENABLE
    extern int audio_dec_init();
    extern int audio_enc_init();
    audio_dec_init();
    audio_enc_init();
#endif/*TCFG_AUDIO_ENABLE*/

#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
    jl_kws_main_user_demo();
#endif /* #if TCFG_KWS_VOICE_RECOGNITION_ENABLE */

    init_intent(&it);

#if CONFIG_APP_SPP_LE
    it.name = "spp_le";
    it.action = ACTION_SPPLE_MAIN;

#elif CONFIG_APP_AT_COM || CONFIG_APP_AT_CHAR_COM
    it.name = "at_com";
    it.action = ACTION_AT_COM;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_MULTI
    it.name = "multi_conn";
    it.action = ACTION_MULTI_MAIN;

#elif CONFIG_APP_NONCONN_24G
    it.name = "nonconn_24g";
    it.action = ACTION_NOCONN_24G_MAIN;

#elif CONFIG_APP_HILINK
    it.name = "hilink";
    it.action = ACTION_HILINK_MAIN;

#elif CONFIG_APP_LL_SYNC
    it.name = "ll_sync";
    it.action = ACTION_LL_SYNC;

#elif CONFIG_APP_TUYA
    it.name = "tuya";
    it.action = ACTION_TUYA;

#elif CONFIG_APP_CENTRAL
    it.name = "central";
    it.action = ACTION_CENTRAL_MAIN;

#elif CONFIG_APP_DONGLE
    it.name = "dongle";
    it.action = ACTION_DONGLE_MAIN;

#elif CONFIG_APP_BEACON
    it.name = "beacon";
    it.action = ACTION_BEACON_MAIN;

#elif CONFIG_APP_IDLE
    it.name = "idle";
    it.action = ACTION_IDLE_MAIN;

#elif CONFIG_APP_CONN_24G
    it.name = "conn_24g";
    it.action = ACTION_CONN_24G_MAIN;

#elif CONFIG_APP_FINDMY
    it.name = "findmy";
    it.action = ACTION_FINDMY;

#elif CONFIG_APP_FTMS
    it.name = "ftms";
    it.action = ACTION_FTMS;

#else
    while (1) {
        printf("no app!!!");
    }
#endif


    log_info("run app>>> %s", it.name);
    log_info("%s,%s", __DATE__, __TIME__);

    start_app(&it);

#if TCFG_CHARGE_ENABLE
    set_charge_event_flag(1);
#endif
}

/*
 * app模式切换
 */
void app_switch(const char *name, int action)
{
    struct intent it;
    struct application *app;

    log_info("app_exit\n");

    init_intent(&it);
    app = get_current_app();
    if (app) {
        /*
         * 退出当前app, 会执行state_machine()函数中APP_STA_STOP 和 APP_STA_DESTORY
         */
        it.name = app->name;
        it.action = ACTION_BACK;
        start_app(&it);
    }

    /*
     * 切换到app (name)并执行action分支
     */
    it.name = name;
    it.action = action;
    start_app(&it);
}

int eSystemConfirmStopStatus(void)
{
    /* 系统进入在未来时间里，无任务超时唤醒，可根据用户选择系统停止，或者系统定时唤醒(100ms) */
    //1:Endless Sleep
    //0:100 ms wakeup
    /* log_info("100ms wakeup"); */
    return 1;
}

__attribute__((used)) int *__errno()
{
    static int err;
    return &err;
}


