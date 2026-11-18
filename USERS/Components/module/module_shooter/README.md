# module_shooter — 拨弹机构

单拨弹电机的状态机控制（本板无摩擦轮）。`DISABLED → READY → FEEDING → ROLLBACK → FAULT`。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_shooter_t` | 拨弹机构对象 |
| `module_shooter_config_t` | 配置：拨弹电机、步进角度、堵转/回退参数 |
| `module_shooter_state_t` | 状态枚举：`DISABLED/READY/FEEDING/ROLLBACK/FAULT` |

## 状态机

| 状态 | 行为 |
|------|------|
| `DISABLED` | 拨弹电机禁用，清空待发队列 |
| `READY` | 电机使能并保持当前位置，等待射击请求 |
| `FEEDING` | 目标位置前进一步，等待到位或检测堵转 |
| `ROLLBACK` | 堵转后退回一段角度，再重新尝试送弹 |
| `FAULT` | 超过最大重试或回退超时，锁存故障 |

## 读取状态

```c
module_shooter_state_t state = module_shooter_get_state(&shooter);
uint16_t pending = module_shooter_get_pending_shots(&shooter);   // 待发弹量
uint8_t jams     = module_shooter_get_jam_retry_count(&shooter); // 卡弹次数
bool can_fire    = module_shooter_get_fire_permission(&shooter);
```

## 用法

```c
module_shooter_t shooter;
module_shooter_config_t cfg = {
    .feeder_motor = module_dm_motor_as_base(&feeder),
    .feeder_direction_sign = 1.0f,
    .feeder_step_rad = 0.6283f,        // 每发步进角度
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
module_shooter_init(&shooter, &cfg);
module_shooter_enable(&shooter);

// 单发/连发请求入队（返回后由 update 推进）
module_shooter_request_shots(&shooter, 1U);

// 火控更新（自动连发场景，由 App 周期性调用）
module_shooter_update_fire_control(&shooter, &fire_control_input, dt);

// 周期更新状态机
module_shooter_update(&shooter, dt);
```

`module_shooter_update()` 只推进状态机并设置目标。所有 App/Module 更新结束后，由 `module_dji_motor_bus_update()` 或 `module_dm_motor_bus_update()` 唯一推进电机控制并发送报文。

## API 速查

| 函数 | 功能 |
|------|------|
| `module_shooter_init(me, cfg)` | 初始化 |
| `module_shooter_enable(me)` | 使能拨弹电机，进入 `READY` |
| `module_shooter_disable(me)` | 禁用拨弹电机，清空待发队列 |
| `module_shooter_request_shots(me, count)` | 请求发射并累加到队列 |
| `module_shooter_cancel_shots(me)` | 取消所有待发请求 |
| `module_shooter_reset_fault(me)` | 清除故障锁存 |
| `module_shooter_update_fire_control(me, input, dt)` | 火控决策（跟踪/裁判/自动开火） |
| `module_shooter_update(me, dt)` | 周期更新状态机 |
| `module_shooter_get_state(me)` | 读状态 |
| `module_shooter_get_pending_shots(me)` | 待发弹量 |
| `module_shooter_get_jam_retry_count(me)` | 卡弹次数 |
| `module_shooter_get_fire_permission(me)` | 火控许可？ |
