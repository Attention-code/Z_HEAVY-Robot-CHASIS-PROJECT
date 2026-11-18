/**
 * @file robot_task.h
 * @brief 机器人周期调度入口
 */
#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAN 接收周期（任务上下文）
 * @note  在 CAN_Task 中调用，负责把 FIFO 中的反馈帧分发到各电机。
 */
void robot_task_can_receive(void);

/**
 * @brief 控制周期
 * @param delta_time_s 距上次调用的时间（秒，必须 > 0）
 * @note  在 Control_Task 中调用：读取遥控 -> App 计算 -> 总线统一刷出。
 */
void robot_task_control(float delta_time_s);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_TASK_H */
