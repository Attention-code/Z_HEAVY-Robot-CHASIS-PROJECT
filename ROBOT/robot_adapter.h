/**
 * @file robot_adapter.h
 * @brief 遥控输入 → App 命令的适配层
 */
#ifndef ROBOT_ADAPTER_H
#define ROBOT_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 采集遥控器并把输入转换为底盘/发射命令
 * @note  每个控制周期调用一次；遥控失联时自动进入 NO_FORCE。
 */
void robot_adapter_update(void);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_ADAPTER_H */
