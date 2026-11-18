//
// Created by LEGION on 26-4-29.
//

#include "Ix6_Remote.h"
#include "main.h"
#include "math.h"

#define MAPPING_ENABLE 1

volatile i6x_ctrl_t i6x_ctrl;

static int16_t map_to_660(const int16_t val) {
    if (val >= 0)
        return (int16_t) floorf((660.0f / 783.0f) * (float) val + 0.5f);
    else
        return (int16_t) floorf((660.0f / 784.0f) * (float) val + 0.5f);
}

/* 三档拨杆：上 / 中 / 下 */
static int8_t decode_switch_3pos(const int16_t value) {
    if (value <= -300) {
        return I6X_SW_DOWN;
    }
    if (value >= 300) {
        return I6X_SW_UP;
    }
    return I6X_SW_MID;
}

/* 两档拨杆：只有上 / 下，不做中点判定 */
static int8_t decode_switch_2pos(const int16_t value) {
    return (value >= 0) ? I6X_SW_UP : I6X_SW_DOWN;
}

void sbus_to_i6x(volatile i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data) {

    /* 帧尾校验：Futaba 标准是 0x00，但部分接收机用 0x04/0x14/0x24/0x34。
       这里把实际帧尾记到 last_end_byte，配合 rx_byte_cnt/frame_bad_cnt 就能判断
       是"没信号"还是"帧尾不匹配"。 */
    i6x_ctrl->last_end_byte = sbus_data[24];

    if (sbus_data[0] != 0x0F || !I6X_SBUS_END_BYTE_VALID(sbus_data[24])) {
        i6x_ctrl->frame_bad_cnt++;
        return;
    }

    /* CH1~CH4：左右摇杆（减中值后约 ±1024） */
    i6x_ctrl->ch[I6X_CH_RIGHT_X] = (int16_t) (((sbus_data[1] | (sbus_data[2] << 8)) &
 0x07FF) - 1024);
    i6x_ctrl->ch[I6X_CH_RIGHT_Y] = (int16_t) ((((sbus_data[2] >> 3) | (sbus_data[3] << 5))
 & 0x07FF) - 1024);
    i6x_ctrl->ch[I6X_CH_LEFT_Y] = (int16_t) ((((sbus_data[3] >> 6) | (sbus_data[4] << 2) |
(sbus_data[5] << 10)) & 0x07FF) - 1024);
    i6x_ctrl->ch[I6X_CH_LEFT_X] = (int16_t) ((((sbus_data[5] >> 1) | (sbus_data[6] << 7))
& 0x07FF) - 1024);

    /* CH5~CH8：SWA/SWB/SWC/SWD，其中仅 SWC 为三档 */
    i6x_ctrl->s[I6X_SW_A] = decode_switch_2pos((int16_t) ((((sbus_data[6] >> 4) |
        (sbus_data[7] << 4)) & 0x07FF) - 1024));
    i6x_ctrl->s[I6X_SW_B] = decode_switch_2pos((int16_t) ((((sbus_data[7] >> 7) |
        (sbus_data[8] << 1) | (sbus_data[9] << 9)) & 0x07FF) - 1024));
    i6x_ctrl->s[I6X_SW_C] = decode_switch_3pos((int16_t) ((((sbus_data[9] >> 2) |
        (sbus_data[10] << 6)) & 0x07FF) - 1024));
    i6x_ctrl->s[I6X_SW_D] = decode_switch_2pos((int16_t) ((((sbus_data[10] >> 5) |
        (sbus_data[11] << 3)) & 0x07FF) - 1024));

#if MAPPING_ENABLE
    for (int i = 0; i < (int) I6X_CH_NUM; i++) {
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);
    }

    const uint8_t flag = sbus_data[23];
    i6x_ctrl->frame_lost = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe = (flag >> 3) & 0x01;
#endif

    /* 合法帧计数自增（供上层 Arm_Remote_Lost 失联检测使用）*/
    i6x_ctrl->frame_cnt++;
}

volatile i6x_ctrl_t *get_i6x_point(void) {
    return &i6x_ctrl;
}

