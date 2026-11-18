/**
 * @file robot_config.c
 * @brief 机器人实例与配置总表实现
 */
#include "robot_config.h"

#include <stddef.h>

#include "fdcan.h"
#include "robot_port.h"

/* ======================== 整车实例 ======================== */
robot_t robot;

/* ======================== PID 参数（需实车标定） ======================== */

/* M3508 速度环：输出为 CAN 电流命令（±16384） */
static const module_motor_pid_config_t k_wheel_velocity_pid = {
    .proportional_gain = 150.0f,
    .integral_gain = 5.0f,
    .derivative_gain = 0.0f,
    .derivative_filter_cutoff_hz = 0.0f,
    .integral_min = -16000.0f,
    .integral_max = 16000.0f,
    .output_min = -16384.0f,
    .output_max = 16384.0f,
    .derivative_on_measurement = true,
};

/* M3508 角度环：输出为速度目标（rad/s） */
static const module_motor_pid_config_t k_lift_angle_pid = {
    .proportional_gain = 20.0f,
    .integral_gain = 0.0f,
    .derivative_gain = 0.0f,
    .derivative_filter_cutoff_hz = 0.0f,
    .integral_min = -10.0f,
    .integral_max = 10.0f,
    .output_min = -30.0f,
    .output_max = 30.0f,
    .derivative_on_measurement = true,
};

/* M3508 电流环（角度/速度串级的内环） */
static const module_motor_pid_config_t k_motor_current_pid = {
    .proportional_gain = 1.0f,
    .integral_gain = 0.0f,
    .derivative_gain = 0.0f,
    .derivative_filter_cutoff_hz = 0.0f,
    .integral_min = -5000.0f,
    .integral_max = 5000.0f,
    .output_min = -16384.0f,
    .output_max = 16384.0f,
    .derivative_on_measurement = true,
};

/* DM J4310 限制参数：必须与本机固件一致（见 DM-J4310 手册） */
static const module_dm_limits_t k_dm4310_limits = {
    .position_min_rad = -12.5f,
    .position_max_rad = 12.5f,
    .velocity_min_rad_per_s = -30.0f,
    .velocity_max_rad_per_s = 30.0f,
    .torque_min_nm = -10.0f,
    .torque_max_nm = 10.0f,
    .proportional_gain_min = 0.0f,
    .proportional_gain_max = 500.0f,
    .derivative_gain_min = 0.0f,
    .derivative_gain_max = 5.0f,
};

/* ======================== 抬升 ======================== */
static float s_lift_target_rad;
static bool s_lift_target_valid;

/* 各电机安装方向，需按实物标定 */
static const float k_lift_dji_direction[ROBOT_LIFT_DJI_COUNT] = {1.0f, -1.0f};
static const float k_lift_dm_direction[ROBOT_LIFT_DM_COUNT] = {1.0f, -1.0f};

/* ======================== 分发器回调 ======================== */

static void dji_feedback_callback(const bsp_can_frame_t *frame, void *user_context)
{
    (void)module_dji_motor_bus_handle_feedback((module_dji_motor_bus_t *)user_context, frame);
}

static void dm_feedback_callback(const bsp_can_frame_t *frame, void *user_context)
{
    (void)module_dm_motor_bus_handle_feedback((module_dm_motor_bus_t *)user_context, frame);
}

/* 必需监控器失联：直接进入安全态 */
static void robot_rc_offline_callback(void *user_context)
{
    (void)user_context;
    robot_enter_safe_state();
}

/* ======================== 配置构造辅助 ======================== */

static module_dji_motor_config_t make_dji_config(const char *name,
                                                 uint8_t identifier,
                                                 module_dji_control_mode_t mode)
{
    module_dji_motor_config_t config = {
        .name = name,
        .motor_bus = &robot.dji_bus,
        .motor_model = MODULE_DJI_MOTOR_M3508,
        .control_mode = mode,
        .motor_identifier = identifier,
        .direction_sign = 1.0f,
        .maximum_temperature_c = 80.0f,
        .current_scale_a_per_count = 20.0f / 16384.0f,
        .position_reference = MODULE_DJI_POSITION_BOOT_RELATIVE,
        .encoder_zero_count = 0U,
        .position_offset_rad = 0.0f,
        .current_pid_config = k_motor_current_pid,
        .velocity_pid_config = k_wheel_velocity_pid,
        .angle_pid_config = k_lift_angle_pid,
    };
    return config;
}

static module_dm_motor_config_t make_dm_config(const char *name,
                                               uint32_t master_id,
                                               uint32_t feedback_id)
{
    module_dm_motor_config_t config = {
        .name = name,
        .motor_bus = &robot.dm_bus,
        .control_mode = MODULE_DM_MODE_POSITION_VELOCITY,
        .master_identifier = master_id,
        .feedback_identifier = feedback_id,
        .transmit_timeout_ms = 2U,
        .limits = k_dm4310_limits,
    };
    return config;
}

/* ======================== 初始化 ======================== */

bool robot_init(void)
{
    bool ok = true;
    uint32_t i;

    /* ---- 1. CAN 总线 ---- */
    bsp_can_config_t can_dji_config = {
        .device_handle = &hfdcan1,
        .driver_ops = robot_port_fdcan_ops(),
        .callback = NULL,
        .user_context = NULL,
    };
    bsp_can_config_t can_dm_config = {
        .device_handle = &hfdcan2,
        .driver_ops = robot_port_fdcan_ops(),
        .callback = NULL,
        .user_context = NULL,
    };
    ok = ok && (bsp_can_init(&robot.can_dji, &can_dji_config) == BSP_STATUS_OK);
    ok = ok && (bsp_can_init(&robot.can_dm, &can_dm_config) == BSP_STATUS_OK);

    /* ---- 2. 电机总线 ---- */
    ok = ok && (module_dji_motor_bus_init(&robot.dji_bus, &robot.can_dji, 2U) ==
                MODULE_MOTOR_STATUS_OK);
    ok = ok && (module_dm_motor_bus_init(&robot.dm_bus, &robot.can_dm, robot.dm_storage,
                                         ROBOT_DM_MOTOR_COUNT, ROBOT_DM_MOTOR_COUNT) ==
                MODULE_MOTOR_STATUS_OK);

    /* ---- 3. 底盘 4×M3508（CAN ID 1~4，发送组 0x200） ---- */
    for (i = 0U; i < ROBOT_WHEEL_COUNT; ++i) {
        module_dji_motor_config_t config = make_dji_config("chassis_wheel", (uint8_t)(i + 1U),
                                                           MODULE_DJI_CONTROL_VELOCITY);
        ok = ok && (module_dji_motor_init(&robot.wheel[i], &config) == MODULE_MOTOR_STATUS_OK);
        ok = ok && (module_dji_motor_register(&robot.wheel[i]) == MODULE_MOTOR_STATUS_OK);
    }

    /* ---- 4. 抬升 2×M3508（CAN ID 5~6，发送组 0x1FF，角度模式） ---- */
    for (i = 0U; i < ROBOT_LIFT_DJI_COUNT; ++i) {
        module_dji_motor_config_t config = make_dji_config("lift_3508", (uint8_t)(i + 5U),
                                                           MODULE_DJI_CONTROL_ANGLE);
        ok = ok && (module_dji_motor_init(&robot.lift_dji[i], &config) == MODULE_MOTOR_STATUS_OK);
        ok = ok && (module_dji_motor_register(&robot.lift_dji[i]) == MODULE_MOTOR_STATUS_OK);
    }

    /* ---- 5. 抬升 2×J4310（主 ID 0x01/0x02，反馈 0x11/0x12） ---- */
    for (i = 0U; i < ROBOT_LIFT_DM_COUNT; ++i) {
        module_dm_motor_config_t config = make_dm_config("lift_4310", 0x01U + i, 0x11U + i);
        ok = ok && (module_dm_motor_init(&robot.lift_dm[i], &config) == MODULE_MOTOR_STATUS_OK);
        ok = ok && (module_dm_motor_register(&robot.lift_dm[i]) == MODULE_MOTOR_STATUS_OK);
    }

    /* ---- 6. 拨弹 1×J4310（主 ID 0x03，反馈 0x13） ---- */
    {
        module_dm_motor_config_t config = make_dm_config("shooter_feeder", 0x03U, 0x13U);
        ok = ok && (module_dm_motor_init(&robot.feeder, &config) == MODULE_MOTOR_STATUS_OK);
        ok = ok && (module_dm_motor_register(&robot.feeder) == MODULE_MOTOR_STATUS_OK);
    }

    /* ---- 7. 麦轮运动学（尺寸需实车标定） ---- */
    {
        alg_mecanum_config_t mecanum_config = {
            .wheel_radius_m = 0.076f,
            .half_wheelbase_m = 0.19f,
            .half_track_width_m = 0.21f,
            .direction_sign = {1.0f, -1.0f, 1.0f, -1.0f},
            .odometry_weight = {1.0f, 1.0f, 1.0f, 1.0f},
            .maximum_wheel_angular_velocity_rad_per_s = 60.0f,
            .roller_arrangement = ALG_MECANUM_ROLLER_X,
        };
        ok = ok && (alg_mecanum_init(&robot.mecanum, &mecanum_config) ==
                    ALG_CHASSIS_STATUS_OK);
    }

    /* ---- 8. 底盘 App ---- */
    {
        app_chassis_config_t chassis_config = {
            .follow_gain = 1.0f,
            .stop_deadband = 0.01f,
            .kinematics = &robot.mecanum,
            .motors = {
                module_dji_motor_as_base(&robot.wheel[0]),
                module_dji_motor_as_base(&robot.wheel[1]),
                module_dji_motor_as_base(&robot.wheel[2]),
                module_dji_motor_as_base(&robot.wheel[3]),
            },
        };
        ok = ok && (app_chassis_init(&robot.chassis, &chassis_config) == BSP_STATUS_OK);
    }

    /* ---- 9. 拨弹状态机（仅拨弹电机） ---- */
    {
        module_shooter_config_t shooter_config = {
            .feeder_motor = module_dm_motor_as_base(&robot.feeder),
            .feeder_direction_sign = 1.0f,
            .feeder_step_rad = 0.6283f,
            .feeder_position_tolerance_rad = 0.02f,
            .jam_velocity_threshold_rad_per_s = 1.0f,
            .jam_current_threshold_a = 3.0f,
            .jam_current_threshold_raw = 0,
            .jam_confirmation_time_s = 0.3f,
            .rollback_angle_rad = 0.35f,
            .rollback_position_tolerance_rad = 0.02f,
            .rollback_timeout_s = 0.5f,
            .fire_stable_time_s = 0.05f,
            .automatic_shot_interval_s = 0.15f,
            .maximum_jam_retries = 3U,
            .maximum_pending_shots = 8U,
        };
        ok = ok && (module_shooter_init(&robot.shooter, &shooter_config) ==
                    MODULE_SHOOTER_STATUS_OK);
    }

    /* ---- 10. 帧分发器 ---- */
    {
        bsp_can_dispatcher_config_t dji_dispatch_config = {
            .can = &robot.can_dji,
            .receive_fifo = BSP_CAN_RX_FIFO_0,
            .route_storage = robot.dji_routes,
            .route_capacity = ROBOT_DJI_ROUTE_COUNT,
            .maximum_frames_per_process = 16U,
        };
        bsp_can_dispatcher_config_t dm_dispatch_config = {
            .can = &robot.can_dm,
            .receive_fifo = BSP_CAN_RX_FIFO_1,
            .route_storage = robot.dm_routes,
            .route_capacity = ROBOT_DM_ROUTE_COUNT,
            .maximum_frames_per_process = 16U,
        };
        ok = ok && (bsp_can_dispatcher_init(&robot.dji_dispatcher, &dji_dispatch_config) ==
                    BSP_STATUS_OK);
        ok = ok && (bsp_can_dispatcher_init(&robot.dm_dispatcher, &dm_dispatch_config) ==
                    BSP_STATUS_OK);

        /* DJI 反馈 ID：0x201 ~ 0x206 */
        for (i = 0U; i < ROBOT_DJI_ROUTE_COUNT; ++i) {
            ok = ok && (bsp_can_dispatcher_add_route(&robot.dji_dispatcher, 0x201U + i, 0x7FFU,
                                                     BSP_CAN_ID_STANDARD, dji_feedback_callback,
                                                     &robot.dji_bus, NULL) == BSP_STATUS_OK);
        }
        /* DM 反馈 ID：0x11 / 0x12 / 0x13 */
        for (i = 0U; i < ROBOT_DM_ROUTE_COUNT; ++i) {
            ok = ok && (bsp_can_dispatcher_add_route(&robot.dm_dispatcher, 0x11U + i, 0x7FFU,
                                                     BSP_CAN_ID_STANDARD, dm_feedback_callback,
                                                     &robot.dm_bus, NULL) == BSP_STATUS_OK);
        }
    }

    /* ---- 11. 安全门控（遥控失联 → 关闭整机输出） ---- */
    {
        app_safety_monitor_config_t rc_monitor_config = {
            .name = "i6x",
            .timeout_ms = 100U,
            .required = true,
            .offline_callback = robot_rc_offline_callback,
            .online_callback = NULL,
            .user_context = NULL,
        };
        ok = ok && (app_safety_init(&robot.safety, NULL) == BSP_STATUS_OK);
        ok = ok && (app_safety_monitor_init(&robot.rc_monitor, &rc_monitor_config) ==
                    BSP_STATUS_OK);
        ok = ok && (app_safety_register(&robot.safety, &robot.rc_monitor) == BSP_STATUS_OK);
    }

    /* ---- 12. 命令默认值 ---- */
    robot.chassis_cmd.mode = APP_CHASSIS_MODE_NO_FORCE;
    robot.chassis_cmd.enabled = false;
    robot.shooter_cmd.fire_requested = false;
    robot.shooter_cmd.automatic_fire_enabled = false;

    robot.initialized = ok;
    return ok;
}

/* ======================== 运行期 ======================== */

void robot_try_enable_motors(void)
{
    uint32_t i;

    for (i = 0U; i < ROBOT_WHEEL_COUNT; ++i) {
        module_motor_t *base = module_dji_motor_as_base(&robot.wheel[i]);
        const module_motor_feedback_t *feedback = module_motor_get_feedback(base);
        if ((feedback != NULL) && feedback->is_online && (base->state == MODULE_MOTOR_STATE_DISABLED)) {
            (void)module_motor_enable(base);
        }
    }
    for (i = 0U; i < ROBOT_LIFT_DJI_COUNT; ++i) {
        module_motor_t *base = module_dji_motor_as_base(&robot.lift_dji[i]);
        const module_motor_feedback_t *feedback = module_motor_get_feedback(base);
        if ((feedback != NULL) && feedback->is_online && (base->state == MODULE_MOTOR_STATE_DISABLED)) {
            (void)module_motor_enable(base);
        }
    }
    for (i = 0U; i < ROBOT_LIFT_DM_COUNT; ++i) {
        module_motor_t *base = module_dm_motor_as_base(&robot.lift_dm[i]);
        const module_motor_feedback_t *feedback = module_motor_get_feedback(base);
        if ((feedback != NULL) && feedback->is_online && (base->state == MODULE_MOTOR_STATE_DISABLED)) {
            (void)module_motor_enable(base);
        }
    }

    /* 拨弹：由状态机统一使能（会检查反馈在线） */
    if (module_shooter_get_state(&robot.shooter) == MODULE_SHOOTER_STATE_DISABLED) {
        (void)module_shooter_enable(&robot.shooter);
    }
}

void robot_lift_set_target_rad(float target_rad)
{
    s_lift_target_rad = target_rad;
    s_lift_target_valid = true;
}

void robot_lift_update(float delta_time_s)
{
    uint32_t i;

    (void)delta_time_s;

    if (!s_lift_target_valid) {
        return;
    }

    for (i = 0U; i < ROBOT_LIFT_DJI_COUNT; ++i) {
        (void)module_motor_set_target(module_dji_motor_as_base(&robot.lift_dji[i]),
                                      s_lift_target_rad * k_lift_dji_direction[i]);
    }
    for (i = 0U; i < ROBOT_LIFT_DM_COUNT; ++i) {
        (void)module_dm_motor_set_position_velocity_target(
            &robot.lift_dm[i], s_lift_target_rad * k_lift_dm_direction[i], 1.0f);
    }
}

void robot_enter_safe_state(void)
{
    uint32_t i;

    for (i = 0U; i < ROBOT_WHEEL_COUNT; ++i) {
        (void)module_motor_disable(module_dji_motor_as_base(&robot.wheel[i]));
    }
    for (i = 0U; i < ROBOT_LIFT_DJI_COUNT; ++i) {
        (void)module_motor_disable(module_dji_motor_as_base(&robot.lift_dji[i]));
    }
    for (i = 0U; i < ROBOT_LIFT_DM_COUNT; ++i) {
        (void)module_motor_disable(module_dm_motor_as_base(&robot.lift_dm[i]));
    }
    (void)module_shooter_disable(&robot.shooter);

    robot.chassis_cmd.mode = APP_CHASSIS_MODE_NO_FORCE;
    robot.chassis_cmd.enabled = false;
}
