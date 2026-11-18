/**
 * @file robot_port.h
 * @brief HAL → ECF BSP 的端口适配层
 * @note  只负责把 STM32 HAL 的 FDCAN 句柄包装成 ECF 的 bsp_can_driver_ops_t，
 *        不包含任何机器人业务逻辑。
 */
#ifndef ROBOT_PORT_H
#define ROBOT_PORT_H

#include "bsp_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 FDCAN 的驱动操作集
 * @return 指向静态常量操作集的指针（全工程唯一，可复用于 FDCAN1/2/3）
 */
const bsp_can_driver_ops_t *robot_port_fdcan_ops(void);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_PORT_H */
