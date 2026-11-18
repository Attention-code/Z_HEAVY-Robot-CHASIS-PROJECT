/**
 * @file bsp_log.c
 * @brief BSP 日志模块实现（输出到 SEGGER RTT Channel 0）
 * @note  使用 SEGGER RTT：主机通过 SWD 读写目标 RAM 中的环形缓冲区，
 *        不占用 UART，也不需要 SWO 引脚，OpenOCD + DAPLink 即可查看。
 */
#include "bsp_log.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "SEGGER_RTT.h"

#define BSP_LOG_BUFFER_SIZE (128U)
#define BSP_LOG_RTT_CHANNEL (0U)

/* 绘图通道：通道 1/2 独立于日志通道，供 cortex-debug 的 graph 解码器绘制波形 */
#define BSP_LOG_PLOT_CHANNEL_BASE (1U)
#define BSP_LOG_PLOT_CURVE_NUM    (2U)
#define BSP_LOG_PLOT_BUFFER_SIZE  (512U)

static uint8_t plot_buffer[BSP_LOG_PLOT_CURVE_NUM][BSP_LOG_PLOT_BUFFER_SIZE];

static void bsp_log_plot_init(void)
{
    static const char *const curve_names[] = {"Plot0", "Plot1"};
    unsigned int index;

    for (index = 0U; index < BSP_LOG_PLOT_CURVE_NUM; index++) {
        (void)SEGGER_RTT_ConfigUpBuffer(BSP_LOG_PLOT_CHANNEL_BASE + index,
                                        curve_names[index], plot_buffer[index],
                                        BSP_LOG_PLOT_BUFFER_SIZE,
                                        SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    }
}

static void bsp_log_send_string(const char *text)
{
    if (text == NULL) {
        return;
    }
    (void)SEGGER_RTT_Write(BSP_LOG_RTT_CHANNEL, text, (unsigned int)strlen(text));
}

void bsp_log_init(void)
{
    SEGGER_RTT_Init();
    bsp_log_plot_init();
}

/**
 * @brief 启动阶段提前初始化 RTT
 * @note  cortex-debug 连接后会立即执行 `rtt start`，而默认配置停在 main 入口，
 *        此时 MCU_Init() 里的 bsp_log_init() 还没执行，控制块中还没有 "SEGGER RTT" 标识，
 *        OpenOCD 找不到控制块就会报
 *        "SWO/RTT Initialization failed: Protocol error with Rcmd: 01"。
 *        构造函数在 __libc_init_array 阶段执行（早于 main），可保证调试器连接时控制块已就绪。
 */
__attribute__((constructor)) static void bsp_log_early_init(void)
{
    SEGGER_RTT_Init();
    bsp_log_plot_init();
}

int bsp_log_printf(const char *format, ...)
{
    char buffer[BSP_LOG_BUFFER_SIZE];
    va_list arguments;
    int length;

    if (format == NULL) {
        return -1;
    }

    va_start(arguments, format);
    length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    if (length > 0) {
        bsp_log_send_string(buffer);
    }
    return length;
}

int bsp_log_write(bsp_log_level_t level, const char *format, ...)
{
    static const char *const tags[] = {"[I] ", "[W] ", "[E] "};
    char buffer[BSP_LOG_BUFFER_SIZE];
    va_list arguments;
    int length;

    if ((level > BSP_LOG_LEVEL_ERROR) || (format == NULL)) {
        return -1;
    }

    va_start(arguments, format);
    length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    bsp_log_send_string(tags[level]);
    if (length > 0) {
        bsp_log_send_string(buffer);
    }
    bsp_log_send_string("\r\n");
    return length;
}

void bsp_log_clear(void)
{
    bsp_log_send_string("\033[2J\033[H");
}

void bsp_log_plot(uint32_t curve, float value)
{
    if (curve >= BSP_LOG_PLOT_CURVE_NUM) {
        return;
    }
    (void)SEGGER_RTT_Write(BSP_LOG_PLOT_CHANNEL_BASE + curve, &value, sizeof(float));
}
