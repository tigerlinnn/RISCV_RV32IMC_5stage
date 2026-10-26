// ============================================================================
// yolo_main.c — Phase A: one convolution layer on the BRV32P + conv accelerator
// ----------------------------------------------------------------------------
// Drives accel_mmio at 0x2000_0300 through the same stream sequence the unit
// testbench uses, then reads the 90 OFM beats back and compares a 32-bit
// checksum against the bit-exact Python golden model (tools/gen_vectors.py).
//
// Result: UART "RESULT: PASS" / "RESULT: FAIL", and LED = 0x5A on pass.
// Vectors/config are compiled in from yolo_vectors.h (generated per case).
// ============================================================================
#include <stdint.h>
#include "uart.h"
#include "yolo_vectors.h"

#define ACCEL_BASE 0x20000300u

#define A_REG0       (ACCEL_BASE + 0x00u)
#define A_REG1       (ACCEL_BASE + 0x04u)
#define A_REG2       (ACCEL_BASE + 0x08u)
#define A_REG3       (ACCEL_BASE + 0x0Cu)
#define A_TXLEN      (ACCEL_BASE + 0x10u)
#define A_TX_LO      (ACCEL_BASE + 0x14u)
#define A_TX_HI      (ACCEL_BASE + 0x18u)
#define A_TXPUSH     (ACCEL_BASE + 0x1Cu)
#define A_RXADDR     (ACCEL_BASE + 0x24u)
#define A_RX_LO      (ACCEL_BASE + 0x28u)
#define A_RX_HI      (ACCEL_BASE + 0x2Cu)
#define A_RXSTAT     (ACCEL_BASE + 0x30u)
#define A_STATUS     (ACCEL_BASE + 0x34u)
#define A_STATUS_CLR (ACCEL_BASE + 0x38u)
#define A_ID         (ACCEL_BASE + 0x3Cu)
#define A_RESET      (ACCEL_BASE + 0x40u)

#define ST_CONV_DONE 0x02u
#define ST_RECV_DONE 0x04u
#define ST_RX_DONE   0x10u

#define GPIO_DIR 0x20000008u
#define GPIO_OUT 0x20000000u

#define A_STATIC ((uint32_t)((YOLO_ROW << 24) | (YOLO_SEL << 10) | \
                             (YOLO_POOL << 5) | (YOLO_STRIDE << 15) | \
                             (YOLO_RELU << 16) | (YOLO_FIRST << 6) | \
                             (YOLO_LAST << 7) | (YOLO_IFMSEL << 8)))

#define WAIT_LIMIT 2000000u

static inline void wr(uint32_t addr, uint32_t v)
{
    *(volatile uint32_t *)addr = v;
}

static inline uint32_t rd(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

static void led(uint32_t v)
{
    wr(GPIO_DIR, 0xFFu);
    wr(GPIO_OUT, v & 0xFFu);
}

static void puthex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[9];
    int i;
    for (i = 7; i >= 0; i--) {
        buf[i] = hex[v & 0xFu];
        v >>= 4;
    }
    buf[8] = '\0';
    uart_puts(buf);
}

static void putdec(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    uart_puts(&buf[i]);
}

static void halt(void)
{
    for (;;) {
    }
}

static void fail(uint32_t code)
{
    uart_puts("RESULT: FAIL code=");
    puthex32(code);
    uart_puts("\r\n");
    led(0x80u | (code & 0x7Fu));
    halt();
}

static int wait_status(uint32_t mask)
{
    uint32_t i;
    for (i = 0; i < WAIT_LIMIT; i++) {
        if ((rd(A_STATUS) & mask) == mask) {
            return 0;
        }
    }
    return 1;
}

static void push64(uint64_t v)
{
    wr(A_TX_LO, (uint32_t)v);
    wr(A_TX_HI, (uint32_t)(v >> 32));
    wr(A_TXPUSH, 0u);
}

static int stream(const uint64_t *data, uint32_t len, uint32_t bufsel)
{
    uint32_t i;

    wr(A_TXLEN, len);
    wr(A_STATUS_CLR, ST_RECV_DONE);
    for (i = 0; i < len; i++) {
        push64(data[i]);
    }
    // Assert recv+task, then drop task_valid: the MMIO starts the stream on
    // that falling edge so the core's write counters are no longer cleared.
    wr(A_REG0, 0x201u | (bufsel << 2));
    wr(A_REG0, (bufsel << 2));
    return wait_status(ST_RECV_DONE);
}

int main(void)
{
    uint32_t id, n, i, chk, lo, hi;

    uart_init();
    led(0x00u);

    uart_puts("\r\nBRV32P YOLO PhaseA case " YOLO_CASE_NAME "\r\n");

    id = rd(A_ID);
    uart_puts("ID=");
    puthex32(id);
    uart_puts("\r\n");
    if (id != 0xACCE0001u) {
        fail(0x01u);
    }

    wr(A_RESET, 1u);
    for (i = 0; i < 200u; i++) {
        rd(A_STATUS);
    }

    wr(A_REG1, (uint32_t)YOLO_SCALE | ((uint32_t)YOLO_SHIFT << 16));
    wr(A_REG2, (uint32_t)YOLO_ZP_IN | ((uint32_t)YOLO_ZP_OUT << 8) |
               ((uint32_t)YOLO_ZP_ACT << 16));
    wr(A_REG3, 0u);

    uart_puts("stream LUT\r\n");
    if (stream(yolo_lut, YOLO_LUT_BEATS, 2u)) {
        fail(0x12u);
    }
    uart_puts("stream bias\r\n");
    if (stream(yolo_bias, YOLO_BIA_BEATS, 3u)) {
        fail(0x13u);
    }
    uart_puts("stream wgt\r\n");
    if (stream(yolo_wgt, YOLO_WGT_BEATS, 1u)) {
        fail(0x14u);
    }
    uart_puts("stream ifm\r\n");
    if (stream(yolo_ifm, YOLO_IFM_BEATS, 0u)) {
        fail(0x15u);
    }

    uart_puts("conv\r\n");
    wr(A_STATUS_CLR, ST_CONV_DONE);
    wr(A_REG0, A_STATIC | (1u << 9) | (1u << 4));
    wr(A_REG0, A_STATIC);
    if (wait_status(ST_CONV_DONE)) {
        fail(0x21u);
    }

    uart_puts("send\r\n");
    wr(A_STATUS_CLR, 0x08u | ST_RX_DONE);
    wr(A_REG0, A_STATIC | (1u << 9) | (1u << 1));
    wr(A_REG0, A_STATIC);
    if (wait_status(ST_RX_DONE)) {
        fail(0x22u);
    }

    n = rd(A_RXSTAT) & 0x1FFFu;
    chk = 5381u;
    for (i = 0; i < n; i++) {
        wr(A_RXADDR, i);
        (void)rd(A_STATUS);           // let the registered BRAM read settle
        lo = rd(A_RX_LO);
        hi = rd(A_RX_HI);
        chk = (chk * 33u) ^ lo ^ hi;
    }

    uart_puts("RX=");
    putdec(n);
    uart_puts(" CHK=");
    puthex32(chk);
    uart_puts(" EXP=");
    puthex32(YOLO_GOLD_CHK);
    uart_puts("\r\n");

    if (n == YOLO_GOLD_BEATS && chk == YOLO_GOLD_CHK) {
        uart_puts("RESULT: PASS\r\n");
        led(0x5Au);
    } else {
        uart_puts("RESULT: FAIL\r\n");
        led(0x80u | (n & 0x7Fu));
    }

    halt();
    return 0;
}