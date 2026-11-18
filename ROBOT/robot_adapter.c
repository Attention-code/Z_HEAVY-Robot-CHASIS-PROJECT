/**
 * @file robot_adapter.c
 * @brief 遥控输入 → App 命令的适配层实现
 */
#include "robot_adapter.h"

#include "fdcan.h"
#include "Ix6_Remote.h"
#include "robot_config.h"

/* ======================== 标定参数 ======================== */
#define ROBOT_RC_DEADBAND       (30)
#define ROBOT_RC_CHANNEL_SCALE  (660.0f)
#define ROBOT_MAX_LINEAR_MPS    (2.5f)
#define ROBOT_MAX_ANGULAR_RPS   (6.0f)
#define ROBOT_RC_TIMEOUT_MS     (100U)

/* ======================== 内部状态 ======================== */
static uint32_t s_last_frame_count;
static uint32_t s_last_update_tick;
static bool s_last_fire_switch;

/* 死区内归零，便于观察摇杆实际有效输出 */
static int16_t apply_deadband(int16_t value)
{
    if ((value > -ROBOT_RC_DEADBAND) && (value < ROBOT_RC_DEADBAND)) {
        return 0;
    }
    return value;
}

static float normalize_channel(int16_t value)
{
    float normalized;

    value = apply_deadband(value);
    if (value == 0) {
        return 0.0f;
    }

    normalized = (float)value / ROBOT_RC_CHANNEL_SCALE;
    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }
    return normalized;
}

/* I6X 拨杆：1=上、0=中、-1=下 */
static app_switch_t convert_switch(int8_t value)
{
    if (value > 0) {
        return APP_SWITCH_UP;
    }
    if (value < 0) {
        return APP_SWITCH_DOWN;
    }
    return APP_SWITCH_MIDDLE;
}

void robot_adapter_update(void)
{
    volatile i6x_ctrl_t *rc = get_i6x_point();
    uint32_t now = HAL_GetTick();
    bool online;

    if (rc == NULL) {
        robot.remote.online = false;
        robot.chassis_cmd.enabled = false;
        robot.chassis_cmd.mode = APP_CHASSIS_MODE_NO_FORCE;
        return;
    }

    /* 以 frame_cnt 是否变化判断链路存活 */
    if (rc->frame_cnt != s_last_frame_count) {
        s_last_frame_count = rc->frame_cnt;
        s_last_update_tick = now;
    }
    online = ((now - s_last_update_tick) < ROBOT_RC_TIMEOUT_MS) &&
             (rc->failsafe == 0U) && (rc->frame_lost == 0U);

    robot.remote.online = online;
    if (online) {
        (void)app_safety_notify_online(&robot.rc_monitor, now);
    }
    robot.remote.channel[0] = apply_deadband(rc->ch[I6X_CH_RIGHT_X]);  /* CH1 右摇杆 左右 */
    robot.remote.channel[1] = apply_deadband(rc->ch[I6X_CH_RIGHT_Y]);  /* CH2 右摇杆 前后 */
    robot.remote.channel[2] = apply_deadband(rc->ch[I6X_CH_LEFT_Y]);   /* CH3 左摇杆 前后 */
    robot.remote.channel[3] = apply_deadband(rc->ch[I6X_CH_LEFT_X]);   /* CH4 左摇杆 左右 */
    robot.remote.left_switch = convert_switch(rc->s[I6X_SW_A]);   /* CH5 */
    robot.remote.right_switch = convert_switch(rc->s[I6X_SW_B]);  /* CH6 */
    robot.remote.sequence++;

    if (!online) {
        robot.chassis_cmd.velocity_x_m_per_s = 0.0f;
        robot.chassis_cmd.velocity_y_m_per_s = 0.0f;
        robot.chassis_cmd.angular_velocity_rad_per_s = 0.0f;
        robot.chassis_cmd.mode = APP_CHASSIS_MODE_NO_FORCE;
        robot.chassis_cmd.enabled = false;
        s_last_fire_switch = false;
        return;
    }

    /* 通道映射（如需换向改符号即可）
       CH3 左摇杆前后 -> 前后平移；CH1 右摇杆左右 -> 左右平移；CH4 左摇杆左右 -> 旋转 */
    robot.chassis_cmd.velocity_x_m_per_s = -normalize_channel(rc->ch[I6X_CH_LEFT_Y]) * ROBOT_MAX_LINEAR_MPS;
    robot.chassis_cmd.velocity_y_m_per_s = -normalize_channel(rc->ch[I6X_CH_RIGHT_X]) * ROBOT_MAX_LINEAR_MPS;
    robot.chassis_cmd.angular_velocity_rad_per_s = -normalize_channel(rc->ch[I6X_CH_LEFT_X]) * ROBOT_MAX_ANGULAR_RPS;
    robot.chassis_cmd.mode = APP_CHASSIS_MODE_NORMAL;
    robot.chassis_cmd.enabled = true;

    /* 右拨杆上位：上升沿触发单发（拨弹用） */
    {
        bool fire_switch = (robot.remote.right_switch == APP_SWITCH_UP);
        if (fire_switch && !s_last_fire_switch) {
            (void)module_shooter_request_shots(&robot.shooter, 1U);
        }
        s_last_fire_switch = fire_switch;
    }
}
