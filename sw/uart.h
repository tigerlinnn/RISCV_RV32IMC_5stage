// ============================================================================
// uart.h — RISCV_1 SoC 的 UART 驱动（给 CoreMark 的 ee_printf 用）
// ----------------------------------------------------------------------------
// 寄存器映射来自 rtl/periph/uart.v，地址窗口由 axi_periph_bridge.v 译码：
//      0x20000000 + [11:8]=0  -> GPIO
//      0x20000100 + [11:8]=1  -> UART   <- 我们这里
//      0x20000200 + [11:8]=2  -> TIMER
// UART 内部用 addr[3:2] 选寄存器。
// ============================================================================
#ifndef UART_H
#define UART_H

#define UART_BASE    0x20000100u
#define UART_TX      (UART_BASE + 0x00u)   // 写：把 1 个字节发出去
#define UART_RX      (UART_BASE + 0x04u)   // 读：收到的字节
#define UART_STATUS  (UART_BASE + 0x08u)   // bit0=tx_busy  bit1=rx_valid  bit2=rx_overrun
#define UART_DIV     (UART_BASE + 0x0Cu)   // 波特率分频 = 主频/波特率 - 1

// 主频 50 MHz、目标 115200 bps 时的分频值：50e6/115200 - 1 ≈ 433
// （这个值在之前那份点灯固件里上板验证过，串口确实收到了数据）
/* RISCV_1: 允许在"只做性能测量"的仿真里用 -DUART_DIV_50MHZ_115200=4u 把串口加速,
 * 这样统计周期都花在 CoreMark 迭代本身, 不会被串口打印的等待时间淹掉。
 * 板上构建不传这个宏, 分频仍然是 433 (115200bps)。 */
#ifndef UART_DIV_50MHZ_115200
#define UART_DIV_50MHZ_115200   433u
#endif

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);

// CoreMark 的 ee_printf.c 会调用这个函数
void uart_send_char(char c);

#endif  // UART_H
