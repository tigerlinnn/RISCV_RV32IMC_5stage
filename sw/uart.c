// ============================================================================
// uart.c — UART 驱动实现
// ============================================================================
#include "uart.h"

static inline void mmio_write(unsigned int addr, unsigned int val)
{
    *(volatile unsigned int *)addr = val;
}

static inline unsigned int mmio_read(unsigned int addr)
{
    return *(volatile unsigned int *)addr;
}

void uart_init(void)
{
    mmio_write(UART_DIV, UART_DIV_50MHZ_115200);
}

// 发一个字符。
//   关键：必须先等上一个字节发完（tx_busy == 0）再写下一个，
//   否则 UART 状态机还没发完就被新数据覆盖，字符会丢、输出会乱码。
void uart_putc(char c)
{
    while (mmio_read(UART_STATUS) & 0x1u) {
        // 等 tx_busy 清零
    }
    mmio_write(UART_TX, (unsigned int)(unsigned char)c);
    /* RISCV_1 修复(串口丢字符): D-Cache 对**外设写**是"缓存后放行"(posted write),
     *   写完那一刻字符还没真正进 UART 的发送移位器。若这里直接返回, 下一次 putc
     *   的"等 tx_busy 清零"会立刻通过(上一个字节还没开始发), 第二个字符就把第一个
     *   覆盖掉了 —— 实测表现为串口随机丢字符。
     *   标准做法: 写完等到"这一字节真的开始发"(tx_busy 变 1)。 */
    while (!(mmio_read(UART_STATUS) & 0x1u)) {
        // 等 tx_busy 置 1
    }
    /* Pango 移植新增（可选，默认关闭）:
     *   远程实验平台的串口日志每次只取 64 字节，整份 CoreMark 报告
     *   (~570 字节) 在 50 ms 内打印完会被截掉约 1/3。定义
     *   UART_SLOW_PRINT=N 后每个字符忙等约 N 个循环（27 MHz 下
     *   20000 ≈ 3 ms/字符），报告就能完整收下来。
     *   不计分影响：CoreMark 在打印前已经调用过 stop_time()。
     *   不定义该宏时，生成结果与原始驱动完全一致。 */
#ifdef UART_SLOW_PRINT
    for (volatile int slow_i = 0; slow_i < UART_SLOW_PRINT; slow_i++) {
    }
#endif
}

void uart_puts(const char *s)
{
    while (*s != '\0') {
        uart_putc(*s++);
    }
}
