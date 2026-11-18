/**
 * @file robot_config.h
 * @brief 机器人实例与配置总表（麦轮底盘 + 抬升 + 拨弹）
 * @note  所有 CAN ID、机械尺寸、PID、电机实例集中在此，便于整机标定。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>

#include "app_chassis.h"
#include "app_safety.h"
#include "app_types.h"
#include "alg_mecanum.h"
#include "bsp_can.h"
#include "bsp_can_dispatcher.h"
#include "module_dji_motor.h"
#include "module_dm_motor.h"
#include "module_dm_motor_bus.h"
#include "module_shooter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 规模宏 ======================== */
#define ROBOT_WHEEL_COUNT        (4U)  /* 麦轮 4×M3508 */
#define ROBOT_LIFT_DJI_COUNT     (2U)  /* 抬升 2×M3508 */
#define ROBOT_LIFT_DM_COUNT      (2U)  /* 抬升 2×DM J4310 */
#define ROBOT_DM_MOTOR_COUNT     (3U)  /* 2 抬升 + 1 拨弹 */
#define ROBOT_DJI_ROUTE_COUNT    (6U)  /* 0x201 ~ 0x206 */
#define ROBOT_DM_ROUTE_COUNT     (3U)  /* 0x11 / 0x12 / 0x13 */

/* ======================== 整车对象 ======================== */
typedef struct
{
    /* ---- CAN 总线 ---- */
    bsp_can_t can_dji;  /* FDCAN1：DJI M3508 */
    bsp_can_t can_dm;   /* FDCAN2：DM J4310 */

    /* ---- 帧分发器 ---- */
    bsp_can_dispatcher_t dji_dispatcher;
    bsp_can_dispatcher_t dm_dispatcher;
    bsp_can_route_t dji_routes[ROBOT_DJI_ROUTE_COUNT];
    bsp_can_route_t dm_routes[ROBOT_DM_ROUTE_COUNT];

    /* ---- 电机总线 ---- */
    module_dji_motor_bus_t dji_bus;
    module_dm_motor_bus_t dm_bus;
    module_dm_motor_t *dm_storage[ROBOT_DM_MOTOR_COUNT];

    /* ---- 电机实例 ---- */
    module_dji_motor_t wheel[ROBOT_WHEEL_COUNT];       /* 底盘 4×M3508 */
    module_dji_motor_t lift_dji[ROBOT_LIFT_DJI_COUNT]; /* 抬升 2×M3508 */
    module_dm_motor_t lift_dm[ROBOT_LIFT_DM_COUNT];    /* 抬升 2×J4310 */
    module_dm_motor_t feeder;                          /* 拨弹 1×J4310 */

    /* ---- 运动学与 App ---- */
    alg_mecanum_t mecanum;
    app_chassis_t chassis;
    module_shooter_t shooter;

    /* ---- 安全门控 ---- */
    app_safety_t safety;
    app_safety_monitor_t rc_monitor;

    /* ---- 命令与输入 ---- */
    app_remote_input_t remote;
    app_chassis_command_t chassis_cmd;
    app_shooter_command_t shooter_cmd;

    bool initialized;
} robot_t;

extern robot_t robot;

/* ======================== 生命周期 ======================== */
bool robot_init(void);
void robot_enter_safe_state(void);

/* 在线后自动使能（DM 需要先收到一帧反馈才能使能） */
void robot_try_enable_motors(void);

/* ======================== 抬升控制（ECF 无专用 App，项目层实现） ======================== */
void robot_lift_set_target_rad(float target_rad);
void robot_lift_update(float delta_time_s);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_CONFIG_H */
