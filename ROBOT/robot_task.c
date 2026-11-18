/**
 * @file robot_task.c
 * @brief 机器人周期调度实现
 */
#include "robot_task.h"

#include "bsp_log.h"
#include "cod-fdcan.h"
#include "robot_adapter.h"
#include "robot_config.h"

#define ROBOT_MECANUM_LOG_INTERVAL_MS (100U)

/* RTT 波形：左前轮（FL）目标 vs 反馈轮速，100Hz（cortex-debug "RTT 波形" 面板） */
#define ROBOT_PLOT_INTERVAL_MS        (10U)
#define ROBOT_PLOT_WHEEL_INDEX        (0U)

void robot_task_can_receive(void)
{
    if (!robot.initialized) {
        return;
    }

    (void)bsp_can_dispatcher_process(&robot.dji_dispatcher, NULL);
    (void)bsp_can_dispatcher_process(&robot.dm_dispatcher, NULL);

    FDCAN1_RxPending = 0U;
    FDCAN2_RxPending = 0U;
}

void robot_task_control(float delta_time_s)
{
    static uint32_t last_mecanum_log_tick;
    static uint32_t last_plot_tick;
    static bool mecanum_config_logged;
    uint32_t now_ms;

    if (!robot.initialized) {
        return;
    }

    now_ms = HAL_GetTick();

    /* 波形输出：曲线0=左前轮目标轮速 曲线1=左前轮反馈轮速（单位 rad/s） */
    if ((uint32_t)(now_ms - last_plot_tick) >= ROBOT_PLOT_INTERVAL_MS) {
        const module_motor_feedback_t *fb =
            module_motor_get_feedback(&robot.wheel[ROBOT_PLOT_WHEEL_INDEX].super);

        last_plot_tick = now_ms;
        bsp_log_plot(0, robot.wheel[ROBOT_PLOT_WHEEL_INDEX].target_velocity_rad_per_s);
        bsp_log_plot(1, (fb != NULL) ? fb->velocity_rad_per_s : 0.0F);
    }

    robot_adapter_update();

    /* 安全门控：必需监控器全部在线才允许输出 */
    (void)app_safety_process(&robot.safety, now_ms);
    app_safety_set_output_enabled(&robot.safety, robot.remote.online);

    if (!app_safety_output_allowed(&robot.safety)) {
        robot.chassis_cmd.enabled = false;
        robot.chassis_cmd.mode = APP_CHASSIS_MODE_NO_FORCE;
    }

    (void)app_chassis_update(&robot.chassis, &robot.chassis_cmd, delta_time_s);

    if (!mecanum_config_logged) {
        const alg_mecanum_config_t *config = &robot.mecanum.config;
        const module_motor_pid_config_t *pid = &robot.wheel[0].velocity_pid.config;

        BSP_LOG_INFO("MW cfg r=%.3f a=%.3f b=%.3f vmax=%.1f roller=%u sign=%.0f,%.0f,%.0f,%.0f",
                     (double)config->wheel_radius_m, (double)config->half_wheelbase_m,
                     (double)config->half_track_width_m,
                     (double)config->maximum_wheel_angular_velocity_rad_per_s,
                     (unsigned int)config->roller_arrangement,
                     (double)config->direction_sign[0], (double)config->direction_sign[1],
                     (double)config->direction_sign[2], (double)config->direction_sign[3]);
        BSP_LOG_INFO("MW PID vel Kp=%.2f Ki=%.2f Kd=%.2f out=%.0f..%.0f",
                     (double)pid->proportional_gain, (double)pid->integral_gain,
                     (double)pid->derivative_gain, (double)pid->output_min,
                     (double)pid->output_max);
        mecanum_config_logged = true;
    }

    if ((uint32_t)(now_ms - last_mecanum_log_tick) >= ROBOT_MECANUM_LOG_INTERVAL_MS) {
        const module_motor_feedback_t *fb[ROBOT_WHEEL_COUNT];
        uint32_t i;

        last_mecanum_log_tick = now_ms;

        for (i = 0U; i < ROBOT_WHEEL_COUNT; ++i) {
            fb[i] = module_motor_get_feedback(&robot.wheel[i].super);
        }

        /* 第 1 行：遥控通道值（已扣死区，CH1=右X CH2=右Y CH3=左Y CH4=左X）与归一化车体速度 */
        BSP_LOG_INFO("MW rc=%d,%d,%d,%d on=%u | vx=%.2f vy=%.2f wz=%.2f en=%u",
                     (int)robot.remote.channel[0], (int)robot.remote.channel[1],
                     (int)robot.remote.channel[2], (int)robot.remote.channel[3],
                     robot.remote.online ? 1U : 0U,
                     (double)robot.chassis_cmd.velocity_x_m_per_s,
                     (double)robot.chassis_cmd.velocity_y_m_per_s,
                     (double)robot.chassis_cmd.angular_velocity_rad_per_s,
                     robot.chassis_cmd.enabled ? 1U : 0U);

        /* 第 2 行：四轮期望轮速 vs 实际轮速（顺序 FL,FR,RL,RR），两者符号应与方向一致 */
        BSP_LOG_INFO("MW tgt=%.1f,%.1f,%.1f,%.1f fb=%.1f,%.1f,%.1f,%.1f",
                     (double)robot.wheel[0].target_velocity_rad_per_s,
                     (double)robot.wheel[1].target_velocity_rad_per_s,
                     (double)robot.wheel[2].target_velocity_rad_per_s,
                     (double)robot.wheel[3].target_velocity_rad_per_s,
                     (fb[0] != NULL) ? (double)fb[0]->velocity_rad_per_s : 0.0,
                     (fb[1] != NULL) ? (double)fb[1]->velocity_rad_per_s : 0.0,
                     (fb[2] != NULL) ? (double)fb[2]->velocity_rad_per_s : 0.0,
                     (fb[3] != NULL) ? (double)fb[3]->velocity_rad_per_s : 0.0);

        /* 第 3 行：逆解算目标轮速。不依赖电机使能，无电机时可单独验证运动学 */
        {
            const app_chassis_feedback_t *chassis_fb = app_chassis_get_feedback(&robot.chassis);

            BSP_LOG_INFO("MW ik=%.1f,%.1f,%.1f,%.1f selflock=%u on=%u",
                         (chassis_fb != NULL)
                             ? (double)chassis_fb->wheel_target_velocity_rad_per_s[0] : 0.0,
                         (chassis_fb != NULL)
                             ? (double)chassis_fb->wheel_target_velocity_rad_per_s[1] : 0.0,
                         (chassis_fb != NULL)
                             ? (double)chassis_fb->wheel_target_velocity_rad_per_s[2] : 0.0,
                         (chassis_fb != NULL)
                             ? (double)chassis_fb->wheel_target_velocity_rad_per_s[3] : 0.0,
                         (chassis_fb != NULL && chassis_fb->self_lock_active) ? 1U : 0U,
                         (chassis_fb != NULL && chassis_fb->motors_online) ? 1U : 0U);
        }

        /* 第 4 行：四轮电流命令、电机状态、反馈在线标志 */
        BSP_LOG_INFO("MW cmd=%d,%d,%d,%d st=%u,%u,%u,%u on=%u,%u,%u,%u",
                     (int)module_dji_motor_get_command(&robot.wheel[0]),
                     (int)module_dji_motor_get_command(&robot.wheel[1]),
                     (int)module_dji_motor_get_command(&robot.wheel[2]),
                     (int)module_dji_motor_get_command(&robot.wheel[3]),
                     (unsigned int)robot.wheel[0].super.state, (unsigned int)robot.wheel[1].super.state,
                     (unsigned int)robot.wheel[2].super.state, (unsigned int)robot.wheel[3].super.state,
                     (fb[0] != NULL && fb[0]->is_online) ? 1U : 0U,
                     (fb[1] != NULL && fb[1]->is_online) ? 1U : 0U,
                     (fb[2] != NULL && fb[2]->is_online) ? 1U : 0U,
                     (fb[3] != NULL && fb[3]->is_online) ? 1U : 0U);
    }

    robot_lift_update(delta_time_s);
    (void)module_shooter_update(&robot.shooter, delta_time_s);

    robot_try_enable_motors();

    /* 每条电机总线每周期只刷新一次 */
    (void)module_dji_motor_bus_update(&robot.dji_bus, delta_time_s);
    (void)module_dm_motor_bus_update(&robot.dm_bus, delta_time_s);
}
