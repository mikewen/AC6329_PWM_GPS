#include "asm/iic_hw.h"
#include "system/generic/gpio.h"
#include "system/generic/log.h"
#include "asm/clock.h"
#include "asm/cpu.h"

// #if 0
/*
    [[  注意!!!  ]]
    * 适用于带cfg_done的硬件IIC，另一种硬件IIC另作说明
    * 硬件IIC的START / ACK(NACK)必须在发送或接收字节cfg_done前设置，且不能
      接cfg_done单独发送；而STOP则应在发送或接收字节cfg_done后设置，必须接
      cfg_done单独发送
*/

/* const struct hw_iic_config hw_iic_cfg_test[] = { */
/*     //iic0 data */
/*     { */
/*         //         SCL          SDA */
/*         .port = {IO_PORTA_06, IO_PORTA_07}, */
/*         .baudrate = 100000,      //IIC通讯波特率 */
/*         .hdrive = 0,             //是否打开IO口强驱 */
/*         .io_filter = 1,          //是否打开滤波器（去纹波） */
/*         .io_pu = 1,              //是否打开上拉电阻，如果外部电路没有焊接上拉电阻需要置1 */
/*         .role = IIC_MASTER, */
/*     }, */
/* }; */

const struct hw_iic_config hw_iic_cfg[IIC_HW_NUM] = {
    {
        // I2C0 (the only one on AC6328)
        .port     = {IO_PORTA_07, IO_PORTA_08}, // SCL, SDA pins – CHANGE THESE TO MATCH YOUR BOARD
        .baudrate = 100000,                     // 100 kHz (or 400000 for fast mode)
        //.baudrate = 200000,
        .hdrive   = 0,                           // Disable high drive (unless needed)
        //.io_filter = 1,                          // Enable input filter
        .io_filter = 0,
        .io_pu    = 1,                            // Enable internal pull-ups (or 0 if external)
        //.io_pu    = 0,
        .role     = IIC_MASTER,                   // We are master
    },
};


static JL_IIC_TypeDef *const iic_regs[IIC_HW_NUM] = {
    JL_IIC,
};

#define iic_get_id(iic)         (iic)

#define iic_info_port(iic, x)   (hw_iic_cfg[iic_get_id(iic)].port[x])
#define iic_info_baud(iic)      (hw_iic_cfg[iic_get_id(iic)].baudrate)
#define iic_info_hdrive(iic)    (hw_iic_cfg[iic_get_id(iic)].hdrive)
#define iic_info_io_filt(iic)   (hw_iic_cfg[iic_get_id(iic)].io_filter)
#define iic_info_io_pu(iic)     (hw_iic_cfg[iic_get_id(iic)].io_pu)
#define iic_info_role(iic)      (hw_iic_cfg[iic_get_id(iic)].role)

static inline u32 iic_get_scl(hw_iic_dev iic)
{
    u8 port = iic_info_port(iic, 0);
    return port;
}

static inline u32 iic_get_sda(hw_iic_dev iic)
{
    u8 port = iic_info_port(iic, 1);
    return port;
}

static int iic_port_init(hw_iic_dev iic)
{
    u32 reg;
    int ret = 0;
    u8 id = iic_get_id(iic);
    u32 scl, sda;
    scl = iic_get_scl(iic);
    sda = iic_get_sda(iic);
    if (id == 0) {
        gpio_set_fun_output_port(scl, FO_IIC_SCL, 1, 1);
        gpio_set_fun_output_port(sda, FO_IIC_SDA, 1, 1);
        gpio_set_fun_input_port(scl, PFI_IIC_SCL);
        gpio_set_fun_input_port(sda, PFI_IIC_SDA);
        if (scl >= IO_PORT_DP || sda >= IO_PORT_DP) {
            usb_iomode(1);
        }
        if (iic_info_hdrive(iic)) {
            gpio_set_hd(scl, 1);
            gpio_set_hd(sda, 1);
        } else {
            gpio_set_hd(scl, 0);
            gpio_set_hd(sda, 0);
        }
        if (iic_info_io_pu(iic)) {
            gpio_set_pull_up(scl, 1);
            gpio_set_pull_down(scl, 0);
            gpio_set_pull_up(sda, 1);
            gpio_set_pull_down(sda, 0);
        } else {
            gpio_set_pull_up(scl, 0);
            gpio_set_pull_down(scl, 0);
            gpio_set_pull_up(sda, 0);
            gpio_set_pull_down(sda, 0);
        }
    } else {
        ret = -EINVAL;
    }
    return ret;
}

int hw_iic_set_baud(hw_iic_dev iic, u32 baud)
{
    //f_iic = f_sys / (IIC_BAUD * 2)
    //=> IIC_BAUD = f_sys / (2 * f_iic)
    u32 sysclk;
    u8 id = iic_get_id(iic);

    sysclk = clk_get("lsb");
    /* printf("lsb clk:%d",sysclk); */
    if (sysclk < 2 * baud) {
        return -EINVAL;
    }
    iic_baud_reg(iic_regs[id]) = sysclk / (2 * baud);
    return 0;
}

static void hw_iic_set_die(hw_iic_dev iic, u8 en)
{
    u8 id = iic_get_id(iic);
    u32 scl, sda;
    scl = iic_get_scl(iic);
    sda = iic_get_sda(iic);
    if (id == 0) {
        gpio_set_die(scl, en);
        gpio_set_die(sda, en);
    } else {
        //undefined
    }
}

void hw_iic_suspend(hw_iic_dev iic)
{
    hw_iic_set_die(iic, 0);
}

void hw_iic_resume(hw_iic_dev iic)
{
    hw_iic_set_die(iic, 1);
}

int hw_iic_init(hw_iic_dev iic)
{
    int ret;
    u8 id = iic_get_id(iic);

    if ((ret = iic_port_init(iic))) {
        printf("invalid hardware iic port\n");
        return ret;
    }
    hw_iic_set_die(iic, 1);
    if (iic_info_role(iic) == IIC_MASTER) {
        iic_host_mode(iic_regs[id]);
        if ((ret = hw_iic_set_baud(iic, iic_info_baud(iic)))) {
            printf("iic baudrate is invalid\n");
            return ret ;
        }
    } else {
        iic_slave_mode(iic_regs[id]);
        iic_slave_scl_pull_down_enble(iic_regs[id]); //在收到/在发送数据时把SCL拉低
        iic_dir_in(iic_regs[id]);
    }
    if (iic_info_io_filt(iic)) {
        iic_isel_filter(iic_regs[id]);
    } else {
        iic_isel_direct(iic_regs[id]);
    }

    iic_auto_ack(iic_regs[id]);
    iic_int_disable(iic_regs[id]);
    iic_pnding_clr(iic_regs[id]);
    iic_enable(iic_regs[id]);
    /* iic_disable(iic_regs[id]); */
#if 0
    printf("info->scl = %d\n", iic_get_scl(iic));
    printf("info->sda = %d\n", iic_get_sda(iic));
    printf("info->baudrate = %d\n", iic_info_baud(iic));
    printf("info->hdrive = %d\n", iic_info_hdrive(iic));
    printf("info->io_filter = %d\n", iic_info_io_filt(iic));
    printf("info->io_pu = %d\n", iic_info_io_pu(iic));
    printf("info->role = %d\n", iic_info_role(iic));
    printf("IIC_CON0 0x%04x\n", iic_regs[id]->CON0);
    printf("IIC_CON1 0x%04x\n", iic_regs[id]->CON1);
    printf("IIC_BAUD 0x%02x\n", iic_regs[id]->BAUD);
    //printf("IIC_BUF %02x\n", iic_regs[id]->BUF);
    printf("IOMC1 0x%08x\n", JL_IOMAP->CON1);
#endif
    return 0;
}

void hw_iic_uninit(hw_iic_dev iic)
{
    u8 id = iic_get_id(iic);
    u32 scl, sda;

    scl = iic_get_scl(iic);
    sda = iic_get_sda(iic);
    hw_iic_set_die(iic, 0);
    if (id == 0) {
        gpio_set_hd(scl, 0);
        gpio_set_hd(sda, 0);
        gpio_set_pull_up(scl, 0);
        gpio_set_pull_up(sda, 0);
        if (scl >= IO_PORT_DP || sda >= IO_PORT_DP) {
            usb_iomode(0);
        }
    }
    iic_disable(iic_regs[id]);
}

/*
 * FIX (Bug 3): Replaced the fragile single-byte start_signal state machine
 * (which packed a flag and a counter into one u8, risking counter overflow
 * into the flag bit after 127 calls) with two explicit, clearly-named
 * variables:
 *   - start_pending : set when hw_iic_start() is called, consumed by the
 *                     first hw_iic_tx_byte() to issue the hardware kick-start.
 *   - start_count   : counts how many consecutive hw_iic_start() calls have
 *                     been made without an intervening hw_iic_stop(), used to
 *                     detect a repeated-START (restart) condition.
 * Both are reset to 0 in hw_iic_stop() to guarantee a clean state for the
 * next transaction regardless of any error path taken.
 */
static u8 start_pending = 0; /* 1 = kick-start must be issued on next tx */
static u8 start_count   = 0; /* number of consecutive STARTs (for restart detection) */

void hw_iic_stop(hw_iic_dev iic)
{
    u8 id = iic_get_id(iic);
    iic_host_send_stop(iic_regs[id]);
    start_pending = 0;
    start_count   = 0;
    u32 timeout = 100000;
    while (!iic_host_is_stop_pending(iic_regs[id]) && --timeout);

    /* Inter-transaction gap — original code comment said 20us minimum
     * between communications. Stop must fully complete before next start */
    delay_us_by_nop(20);
}

void hw_iic_start(hw_iic_dev iic)
{
    if (start_count == 0) {
        start_pending = 1;
    }
    start_count++;
}

u8 hw_iic_tx_byte(hw_iic_dev iic, u8 byte)
{
    u8 ack = 0;
    u8 id  = iic_get_id(iic);

    iic_dir_out(iic_regs[id]);

    if (start_count == 2) {
        iic_dir_in(iic_regs[id]);
        iic_host_receive_continue_byte(iic_regs[id]);
        iic_restart(iic_regs[id]);
    }

    iic_buf_reg(iic_regs[id]) = byte;

    if (start_pending) {
        iic_kick_start(iic_regs[id]);
        start_pending = 0;
    }

    u32 timeout = 100000;
    while (!iic_is_pnding(iic_regs[id]) && --timeout);
    if (!timeout) {
        iic_host_send_stop(iic_regs[id]);
        start_pending = 0;
        start_count   = 0;
        return 0;
    }

    if (!iic_host_send_is_ack(iic_regs[id])) {
        ack = 1;
    }
    iic_regs[id]->CON0 = (iic_regs[id]->CON0 & ~BIT(1)) | BIT(7);

    return ack;
}

u8 hw_iic_rx_byte(hw_iic_dev iic, u8 ack)
{
    u8 data = 0;
    u8 id   = iic_get_id(iic);

    iic_dir_in(iic_regs[id]);
    iic_host_receive_continue_byte(iic_regs[id]);

    /* FIX (Bug 2): Configure NACK/stop bits BEFORE issuing the kick-start.
     * Previously the kick-start was issued first, meaning the hardware could
     * begin clocking the byte before the NACK-auto-stop and
     * receive_continue_byte_stop bits were set, causing the last byte of a
     * read sequence to be clocked without the correct termination config. */
    if (!ack) {
        iic_host_nack_auto_stop(iic_regs[id]);             //硬件检测到nack(这个nack是我们硬件发出的), 自动产生stop信号
        iic_host_receive_continue_byte_stop(iic_regs[id]); //最后1byte, 完成后自动nack
    }

    iic_host_read_kick_start(iic_regs[id]); /* kick AFTER config (Bug 2 fix) */

    while (!iic_is_pnding(iic_regs[id]));

    data = iic_buf_reg(iic_regs[id]);

    /* FIX (Bug 5): Safe pending clear – mask out BIT(1) to avoid
     * inadvertently re-triggering the kick-start via RMW on CON0. */
    iic_regs[id]->CON0 = (iic_regs[id]->CON0 & ~BIT(1)) | BIT(7);

    return data;
}

/*
 * NOTE (Bug 4): hw_iic_write_buf() relies on the caller having already
 * invoked hw_iic_start() followed by hw_iic_tx_byte() (to send the device
 * address) before calling this function. The kick-start for the overall
 * transaction is therefore issued inside hw_iic_tx_byte(); this function
 * only needs to clock out the subsequent data bytes.
 *
 * If you want to use hw_iic_write_buf() as a fully self-contained call
 * (without a preceding hw_iic_tx_byte()), uncomment the kick-start block
 * inside the loop below and ensure hw_iic_start() has been called first.
 */
int hw_iic_write_buf(hw_iic_dev iic, const void *buf, int len)
{
    u8  id = iic_get_id(iic);
    int i  = 0;

    if (!buf || !len) {
        return -1;
    }
    iic_dir_out(iic_regs[id]);

    for (i = 0; i < len; i++) {
        iic_buf_reg(iic_regs[id]) = ((u8 *)buf)[i];

        /* Uncomment below if using hw_iic_write_buf standalone (after
         * hw_iic_start() but without a preceding hw_iic_tx_byte() call):
         *
         * if (i == 0 && start_pending) {
         *     iic_kick_start(iic_regs[id]);
         *     start_pending = 0;
         * }
         */

        while (!iic_is_pnding(iic_regs[id]));

        if (iic_host_send_is_ack(iic_regs[id])) {
            /* FIX (Bug 5): Safe pending clear on early-exit (NACK) path. */
            iic_regs[id]->CON0 = (iic_regs[id]->CON0 & ~BIT(1)) | BIT(7);
            break;
        }

        /* FIX (Bug 5): Safe pending clear – mask BIT(1) before setting BIT(7). */
        iic_regs[id]->CON0 = (iic_regs[id]->CON0 & ~BIT(1)) | BIT(7);
    }

    return i;
}

int hw_iic_read_buf(hw_iic_dev iic, void *buf, int len)
{
    u8  id = iic_get_id(iic);
    int i;

    if (!buf || !len) {
        return -1;
    }
    iic_dir_in(iic_regs[id]);
    iic_host_receive_continue_byte(iic_regs[id]);

    for (i = 0; i < len; i++) {
        /* FIX (Bug 2 pattern): Configure last-byte NACK/stop BEFORE the
         * kick-start so the hardware has the correct termination settings
         * before it begins clocking the byte. */
        if (i == len - 1) {
            iic_host_nack_auto_stop(iic_regs[id]);             //硬件检测到nack(这个nack是我们硬件发出的), 自动产生stop信号
            iic_host_receive_continue_byte_stop(iic_regs[id]); //最后1byte, 完成后自动nack
        }

        iic_host_read_kick_start(iic_regs[id]); /* kick AFTER config */

        while (!iic_is_pnding(iic_regs[id]));

        /* FIX (Bug 1): Read BUF BEFORE clearing the pending flag.
         * Previously iic_pnding_clr() was called first; clearing the flag
         * can trigger the next hardware cycle and overwrite BUF before the
         * software has a chance to read it, causing stale or corrupted data.
         * Reading BUF first guarantees we capture the correct byte. */
        ((u8 *)buf)[i] = iic_buf_reg(iic_regs[id]);

        /* FIX (Bug 5): Safe pending clear – mask BIT(1) before setting BIT(7)
         * to avoid re-asserting the kick-start bit via a naive |= on CON0. */
        iic_regs[id]->CON0 = (iic_regs[id]->CON0 & ~BIT(1)) | BIT(7);
    }

    return len;
}


void iic_disable_for_ota()
{
    JL_IIC->CON0 = 0;
}

/*
 * hw_iic_bus_recover() — Software I2C bus recovery (RFC 4.3 / I2C spec §3.1.16)
 *
 * Call this BEFORE hw_iic_init() on every boot.
 *
 * Problem: If the MCU was reset mid-transaction, a slave (e.g. QMI8658C) may
 * be stuck holding SDA low waiting to clock out the rest of a byte. The master
 * cannot issue a START while SDA is low, so the bus is permanently locked until
 * the slave is freed.
 *
 * Solution: Temporarily take the SCL pin as a plain GPIO and toggle it up to 9
 * times. This allows the stuck slave to finish clocking out its byte and release
 * SDA. We then issue a manual STOP condition (SDA low→high while SCL high) to
 * cleanly terminate whatever transaction the slave thought it was in.
 * After recovery the pins are returned to open-drain/high so hw_iic_init() can
 * configure them normally.
 *
 * Timing targets 100 kHz (each half-period ~5 µs). Safe to call at any clock
 * speed because delay_us() is used.
 */

/* Adjust these to match your board — same as hw_iic_cfg[0].port */
#define RECOVERY_SCL_PIN    IO_PORTA_07
#define RECOVERY_SDA_PIN    IO_PORTA_08

/* Half-period for ~100 kHz: 5 µs */
#define RECOVERY_HALF_PERIOD_US   5

void delay_us_by_nop(u32 usec)
{
    u32 sys = clk_get("sys");
    u32 cnt, div;
    if(usec == 1)     {  div = 30;}
    else if(usec == 2){  div = 12;}
    else if(usec == 3){  div = 8; }
    else if(usec < 10){  div = 6; }
    else              {  div = 5; }
    cnt = usec * (sys / 1000000L / div);
    while(cnt--){ asm volatile("nop"); }
}

static void recovery_delay(void)
{
    delay_us_by_nop(RECOVERY_HALF_PERIOD_US);
}

static void scl_high(void) { gpio_direction_input(RECOVERY_SCL_PIN);  recovery_delay(); }
static void scl_low(void)  { gpio_direction_output(RECOVERY_SCL_PIN, 0); recovery_delay(); }
static void sda_high(void) { gpio_direction_input(RECOVERY_SDA_PIN);  recovery_delay(); }
static void sda_low(void)  { gpio_direction_output(RECOVERY_SDA_PIN, 0); recovery_delay(); }
static int  sda_read(void) { return gpio_read(RECOVERY_SDA_PIN); }

void hw_iic_bus_recover(void)
{
    int i;
    int sda_free = 0;

    printf("[IIC] starting bus recovery\n");

    /*
     * Step 1: Configure both pins as open-drain GPIO (no peripheral function).
     * Drive them high (input mode = pulled high via external/internal resistor)
     * to start from a known idle state.
     */
    gpio_set_pull_up(RECOVERY_SCL_PIN, 1);
    gpio_set_pull_up(RECOVERY_SDA_PIN, 1);
    gpio_set_pull_down(RECOVERY_SCL_PIN, 0);
    gpio_set_pull_down(RECOVERY_SDA_PIN, 0);
    gpio_set_die(RECOVERY_SCL_PIN, 1);
    gpio_set_die(RECOVERY_SDA_PIN, 1);

    sda_high();
    scl_high();

    /*
     * Step 2: Check if SDA is already free. If it is, we still issue a clean
     * STOP below just in case the slave is in a confused state.
     */
    if (sda_read()) {
        printf("[IIC] SDA already high — issuing precautionary STOP\n");
        sda_free = 1;
    } else {
        printf("[IIC] SDA stuck LOW — toggling SCL to free slave\n");
    }

    /*
     * Step 3: Clock SCL up to 9 times.
     * A stuck slave is mid-byte (max 8 bits) or waiting for an ACK bit (1 bit)
     * = 9 clocks worst case. After each rising edge we check whether SDA has
     * been released by the slave.
     */
    for (i = 0; i < 9; i++) {
        scl_low();
        sda_high();   /* allow slave to drive SDA if it needs to */
        scl_high();

        if (sda_read()) {
            sda_free = 1;
            printf("[IIC] SDA released after %d clock(s)\n", i + 1);
            break;
        }
    }

    if (!sda_free) {
        /*
         * Still stuck after 9 clocks. The slave hardware may need a power
         * cycle. Nothing more we can do in software.
         */
        printf("[IIC] WARNING: SDA still LOW after 9 clocks — slave may need power cycle\n");
    }

    /*
     * Step 4: Issue a manual STOP condition regardless of SDA state.
     * STOP = SDA low→high while SCL is high.
     * This terminates any transaction the slave believed was in progress.
     */
    scl_low();
    sda_low();
    scl_high();
    sda_high();   /* STOP condition */

    recovery_delay();
    recovery_delay();  /* extra settling time before hw_iic_init() */

    printf("[IIC] bus recovery complete\n");
}

//#endif
