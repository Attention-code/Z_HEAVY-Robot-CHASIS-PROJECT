//
// Created by LEGION on 26-4-29.
//

#ifndef IX6_REMOTE_H
#define IX6_REMOTE_H

#include <stdint.h>
#define I6X_FRAME_LENGTH 25u

#define I6X_SW_UP ((int8_t)1)
#define I6X_SW_MID  ((int8_t)0)
#define I6X_SW_DOWN  ((int8_t)-1)



#define i6x_switch_is_down(s) (s == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)  (s == I6X_SW_MID)
#define i6x_switch_is_up(s)  (s == I6X_SW_UP)

/* ---------------- SBUS 通道定义 ----------------
 * CH1 右摇杆 左右
 * CH2 右摇杆 前后
 * CH3 左摇杆 前后
 * CH4 左摇杆 左右
 * CH5 SWA（两档：上/下）
 * CH6 SWB（两档：上/下）
 * CH7 SWC（三档：上/中/下）
 * CH8 SWD（两档：上/下）
 */
#define I6X_CH_RIGHT_X   0U  /* CH1 右摇杆 左右 */
#define I6X_CH_RIGHT_Y   1U  /* CH2 右摇杆 前后 */
#define I6X_CH_LEFT_Y    2U  /* CH3 左摇杆 前后 */
#define I6X_CH_LEFT_X    3U  /* CH4 左摇杆 左右 */
#define I6X_CH_NUM       4U  /* 摇杆通道数量 */

#define I6X_SW_A         0U  /* CH5 两档 */
#define I6X_SW_B         1U  /* CH6 两档 */
#define I6X_SW_C         2U  /* CH7 三档 */
#define I6X_SW_D         3U  /* CH8 两档 */

/**
 * @brief SBUS 帧尾字节合法性判断
 * @note  Futaba 标准为 0x00；部分接收机（FrSky、部分 FlySky/兼容机）会输出
 *        0x04 / 0x14 / 0x24 / 0x34。只认 0x00 会导致"一个字节都解析不出、frame_cnt 恒为 0"。
 *        接收机实际帧尾可从 i6x_ctrl.last_end_byte 读出。
 */
#define I6X_SBUS_END_BYTE_VALID(b)  ((b) == 0x00u || (b) == 0x04u || (b) == 0x14u || \
                                     (b) == 0x24u || (b) == 0x34u)

typedef struct {
    int16_t ch[I6X_CH_NUM]; // CH1~CH4 摇杆原始值减中值后的整数
    int8_t s[4]; // SWA/SWB/SWC/SWD 档位（来自 CH5~CH8），1=上 0=中 -1=下
    uint8_t failsafe;   // 失控标志位
    uint8_t frame_lost;  // 丢帧标志位
    uint32_t frame_cnt;  // 合法帧计数器（每收到一帧合法 SBUS 自增，供失联检测）

    /* ---------------- 链路诊断（SBUS 收不到时用来看卡在哪一层） ---------------- */
    uint32_t rx_byte_cnt;   // UART5 收到的原始字节数：恒为 0 说明信号根本没进 MCU
    uint32_t rx_error_cnt;  // UART5 接收错误(FE/NE/ORE/PE)次数：猛涨说明波特率/极性/协议不对
    uint32_t frame_bad_cnt; // 起始字节 0x0F 正确、但帧尾非法的帧数：有字节但帧尾不匹配
    uint8_t  last_end_byte; // 最近一帧的帧尾字节（诊断用）
}__attribute__((packed)) i6x_ctrl_t;

void sbus_to_i6x(volatile i6x_ctrl_t* i6x_ctrl,const uint8_t *sbus_data);
// i6x_ctrl_t* get_i6x_point(void);
extern volatile i6x_ctrl_t i6x_ctrl;

volatile i6x_ctrl_t *get_i6x_point(void);

#endif //IX6_REMOTE_H
