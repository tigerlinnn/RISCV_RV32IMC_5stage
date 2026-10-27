// ============================================================================
// yolo_real.c — real YOLOv3-tiny layer 0 on the BRV32P + conv accelerator
// ----------------------------------------------------------------------------
// Runs the upstream 416x256x3 -> 416x256x16 3x3 conv (+ 2x2/s2 maxpool)
// layer with the trained int8 weights, chunked into 43 row blocks exactly like
// the upstream PS driver.  The input feature map is generated procedurally
// (mirrored in tools/gen_real_layer.py), so no 26.75 KB pixel array is needed.
//
// Per chunk: stream the IFM once, then run both 8-channel output groups over
// the same IFM bank, sending and checksumming each group's OFM against the
// Python golden model.
// ============================================================================
#include <stdint.h>
#include "uart.h"
#include "yolo_real_vectors.h"

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

#define RL_STATIC(send_sel) \
    ((uint32_t)((RL_ROW << 24) | (RL_SEL << 10) | (RL_POOL << 5) | \
                (RL_RELU_TYPE << 16) | (1u << 6) | (1u << 7) | (1u << 8) | \
                (((uint32_t)(send_sel)) << 13)))

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
    uart_puts("\r\nRESULT: FAIL code=");
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

static int stream_array(const uint64_t *data, uint32_t len, uint32_t bufsel)
{
    uint32_t i;

    wr(A_TXLEN, len);
    wr(A_STATUS_CLR, ST_RECV_DONE);
    for (i = 0; i < len; i++) {
        push64(data[i]);
    }
    wr(A_REG0, 0x201u | (bufsel << 2));
    wr(A_REG0, (bufsel << 2));
    return wait_status(ST_RECV_DONE);
}

static int stream_ifm_chunk(uint32_t chunk)
{
    uint32_t r0 = rl_row_off[chunk];
    uint32_t r, c, ch0, ch1, ch2;

    wr(A_TXLEN, RL_IFM_BEATS);
    wr(A_STATUS_CLR, ST_RECV_DONE);
    for (r = 0; r < RL_ROW; r++) {
        for (c = 0; c < RL_COLS; c++) {
            uint32_t gr = r0 + r;
            ch0 = (5u * gr + 3u * c + 7u) & 0x7Fu;
            ch1 = (2u * gr + 11u * c + 1u) & 0x7Fu;
            ch2 = (13u * gr + 7u * c + 9u) & 0x7Fu;
            push64((uint64_t)ch0 | ((uint64_t)ch1 << 8) |
                   ((uint64_t)ch2 << 16));
        }
    }
    wr(A_REG0, 0x201u);            // IFM buffer 0, recv+task
    wr(A_REG0, 0x000u);
    return wait_status(ST_RECV_DONE);
}

int main(void)
{
    uint32_t id, nchunks, c, og, i, lo, hi, chk, errors = 0, n;
    uint32_t stat;

    uart_init();
    led(0x00u);

    uart_puts("\r\nBRV32P YOLO layer0 ");
    uart_puts(RL_POOL ? "pooled" : "conv-only");
    uart_puts("\r\n");

    id = rd(A_ID);
    if (id != 0xACCE0001u) {
        fail(0x01u);
    }

    wr(A_RESET, 1u);
    for (i = 0; i < 200u; i++) {
        (void)rd(A_STATUS);
    }

    wr(A_REG1, (uint32_t)RL_SCALE | ((uint32_t)RL_SHIFT << 16));
    wr(A_REG2, (uint32_t)RL_ZP_IN | ((uint32_t)RL_ZP_OUT << 8) |
               ((uint32_t)RL_ZP_ACT << 16));
    wr(A_REG3, 0u);

    uart_puts("load LUT/bias/wgt\r\n");
    if (stream_array(rl_lut, RL_LUT_BEATS, 2u)) {
        fail(0x12u);
    }
    if (stream_array(rl_bia, RL_BIA_BEATS, 3u)) {
        fail(0x13u);
    }
    if (stream_array(rl_wgt, RL_WGT_BEATS, 1u)) {
        fail(0x14u);
    }

    nchunks = RL_CHUNKS;
#ifdef RL_LIMIT
    if (nchunks > RL_LIMIT) {
        nchunks = RL_LIMIT;
    }
#endif

    for (c = 0; c < nchunks; c++) {
        if (stream_ifm_chunk(c)) {
            fail(0x15u);
        }
        for (og = 0; og < 2u; og++) {
            stat = RL_STATIC(rl_send_sel[c]);
            wr(A_REG3, og | (og << 16));

            wr(A_STATUS_CLR, ST_CONV_DONE);
            wr(A_REG0, stat | (1u << 9) | (1u << 4));
            wr(A_REG0, stat);
            if (wait_status(ST_CONV_DONE)) {
                fail(0x21u);
            }

            wr(A_STATUS_CLR, 0x08u | ST_RX_DONE);
            wr(A_REG0, stat | (1u << 9) | (1u << 1));
            wr(A_REG0, stat);
            if (wait_status(ST_RX_DONE)) {
                fail(0x22u);
            }

            n = rd(A_RXSTAT) & 0x1FFFu;
            if (n != rl_beats[c]) {
                fail(0x41u);
            }
            chk = 5381u;
            for (i = 0; i < n; i++) {
                wr(A_RXADDR, i);
                (void)rd(A_STATUS);
                lo = rd(A_RX_LO);
                hi = rd(A_RX_HI);
                chk = (chk * 33u) ^ lo ^ hi;
            }
            if (chk != rl_gold[c][og]) {
                errors++;
                if (errors <= 8u) {
                    uart_puts("BAD c=");
                    putdec(c);
                    uart_puts(" g=");
                    putdec(og);
                    uart_puts(" chk=");
                    puthex32(chk);
                    uart_puts(" exp=");
                    puthex32(rl_gold[c][og]);
                    uart_puts("\r\n");
                }
            }
        }
        if ((c & 7u) == 7u) {
            uart_putc('.');
        }
    }

    if (errors == 0u) {
        uart_puts("\r\nRESULT: PASS\r\n");
        led(0x5Au);
    } else {
        uart_puts("\r\nRESULT: FAIL\r\n");
        led(0x80u | (errors & 0x7Fu));
    }

    halt();
    return 0;
}