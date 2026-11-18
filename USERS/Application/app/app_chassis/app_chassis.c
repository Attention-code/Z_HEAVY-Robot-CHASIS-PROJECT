#include "app_chassis.h"

#include "alg_chassis_motion.h"
#include "alg_math.h"

#include <math.h>

/* ======================== 内部辅助 ======================== */

/* 台架测试开关：置 1 时允许只有部分轮电机在线（离线电机自动跳过并禁用），
 * 用于单电机/少电机台架验证；整机联调务必恢复为 0（任一轮离线即禁用全部）。 */
#ifndef APP_CHASSIS_ALLOW_PARTIAL_WHEELS
#define APP_CHASSIS_ALLOW_PARTIAL_WHEELS (0)
#endif

/** @brief 禁用全部轮电机。 */
static void app_chassis_disable_all(app_chassis_t *me)
{
    size_t index;
    for (index = 0U; index < ALG_MECANUM_WHEEL_COUNT; ++index)
    {
        (void)module_motor_disable(me->config.motors[index]);
    }
}

/** @brief 校验底盘配置：增益有限且非负，运动学与电机指针齐全。 */
static bool app_chassis_config_is_valid(const app_chassis_config_t *config)
{
    size_t index;
    if ((config == NULL) || !isfinite(config->follow_gain) ||
        !isfinite(config->stop_deadband) || (config->follow_gain < 0.0F) ||
        (config->stop_deadband < 0.0F) || (config->kinematics == NULL))
    {
        return false;
    }
    for (index = 0U; index < ALG_MECANUM_WHEEL_COUNT; ++index)
    {
        if (config->motors[index] == NULL)
        {
            return false;
        }
    }
    return true;
}

/* ======================== 公共 API ======================== */

/**
 * @brief  初始化底盘模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config)
{
    if ((me == NULL) || !app_chassis_config_is_valid(config))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_chassis_t){.config = *config, .initialized = true};
    return BSP_STATUS_OK;
}

/** @brief 使能四个轮电机并设置目标，全部成功才返回 true。 */
static bool app_chassis_apply_targets(app_chassis_t *me, const float *targets)
{
    bool online = true;
    size_t index;
    for (index = 0U; index < ALG_MECANUM_WHEEL_COUNT; ++index)
    {
        module_motor_t *motor = me->config.motors[index];
#if APP_CHASSIS_ALLOW_PARTIAL_WHEELS
        /* 台架模式：跳过反馈离线的电机（顺带禁用），只驱动在线的 */
        const module_motor_feedback_t *motor_fb = module_motor_get_feedback(motor);
        if ((motor_fb == NULL) || !motor_fb->is_online)
        {
            (void)module_motor_disable(motor);
            continue;
        }
#endif
        if ((module_motor_enable(motor) != MODULE_MOTOR_STATUS_OK) ||
            (module_motor_set_target(motor, targets[index]) != MODULE_MOTOR_STATUS_OK))
        {
            online = false;
        }
    }
    return online;
}

/** @brief 麦轮逆解算并下发四轮速度目标，同时输出逆解结果供调试观察。 */
static bool app_chassis_update_fixed(app_chassis_t *me,
                                     const alg_chassis_velocity_t *velocity,
                                     float targets_out[ALG_MECANUM_WHEEL_COUNT])
{
    bool available[ALG_MECANUM_WHEEL_COUNT] = {true, true, true, true};
    float targets[ALG_MECANUM_WHEEL_COUNT] = {0};
    float scale;
    size_t index;

    if (alg_mecanum_inverse(me->config.kinematics, velocity,
                            available, targets, &scale) != ALG_CHASSIS_STATUS_OK)
    {
        return false;
    }
    for (index = 0U; index < ALG_MECANUM_WHEEL_COUNT; ++index)
    {
        targets_out[index] = targets[index];
    }
    return app_chassis_apply_targets(me, targets);
}

/**
 * @brief  执行一个底盘控制周期。
 * @param  me            已初始化的底盘实例。
 * @param  input         命令层发布的底盘运动指令。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化/电机离线返回对应错误码。
 */
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *input,
                                float delta_time_s)
{
    app_chassis_feedback_t feedback = {0};
    alg_chassis_velocity_t reference_velocity;
    alg_chassis_velocity_t body_velocity;
    bool stopped;
    bool online;

    if ((me == NULL) || (input == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!me->initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    /* 指令无效或无动力模式：禁用全部执行器并发布无力反馈。 */
    if (!input->enabled || (input->mode == APP_CHASSIS_MODE_NO_FORCE))
    {
        app_chassis_disable_all(me);
        feedback.mode = APP_CHASSIS_MODE_NO_FORCE;
        me->feedback = feedback;
        return BSP_STATUS_OK;
    }

    reference_velocity = (alg_chassis_velocity_t){input->velocity_x_m_per_s,
                                                   input->velocity_y_m_per_s,
                                                   input->angular_velocity_rad_per_s};
    /* 跟随云台模式：用云台偏航角按比例增益生成偏航角速率。 */
    if (input->mode == APP_CHASSIS_MODE_FOLLOW_GIMBAL)
    {
        float wrapped_yaw_rad = 0.0F;
        if (alg_math_wrap_angle_pi(input->gimbal_yaw_rad, &wrapped_yaw_rad) ==
            ALG_MATH_STATUS_OK)
        {
            reference_velocity.angular_velocity_rad_per_s =
                me->config.follow_gain * wrapped_yaw_rad;
        }
    }
    /* 三轴速度均低于死区时判定为停车。 */
    stopped = (fabsf(reference_velocity.velocity_x_m_per_s) < me->config.stop_deadband) &&
              (fabsf(reference_velocity.velocity_y_m_per_s) < me->config.stop_deadband) &&
              (fabsf(reference_velocity.angular_velocity_rad_per_s) < me->config.stop_deadband);
    if (alg_chassis_transform_reference_to_body(&reference_velocity, input->gimbal_yaw_rad,
                                                &body_velocity) != ALG_CHASSIS_STATUS_OK)
    {
        app_chassis_disable_all(me);
        return BSP_STATUS_IO_ERROR;
    }

    feedback.self_lock_active = stopped && input->self_lock_when_stopped;
    online = app_chassis_update_fixed(me, feedback.self_lock_active
                                              ? &(alg_chassis_velocity_t){0}
                                              : &body_velocity,
                                      feedback.wheel_target_velocity_rad_per_s);
    if (!online)
    {
        app_chassis_disable_all(me);
    }
    feedback.velocity_x_m_per_s = reference_velocity.velocity_x_m_per_s;
    feedback.velocity_y_m_per_s = reference_velocity.velocity_y_m_per_s;
    feedback.angular_velocity_rad_per_s = reference_velocity.angular_velocity_rad_per_s;
    feedback.mode = input->mode;
    feedback.motors_online = online;
    me->feedback = feedback;
    return online ? BSP_STATUS_OK : BSP_STATUS_IO_ERROR;
}

/**
 * @brief  读取最近一次底盘反馈。
 * @param  me  已初始化的底盘实例。
 * @return 只读反馈指针，实例无效时返回 NULL。
 */
const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me)
{
    return ((me != NULL) && me->initialized) ? &me->feedback : NULL;
}
