# 麦轮底盘改进方案（基于当前代码）

> 状态：评审稿 v2（2026-11-17，按当前代码重新核对）
> 范围：麦轮底盘控制（app_chassis / robot_task / robot_adapter / INS）
> 关联文档：[ARCHITECTURE.md](ARCHITECTURE.md)、[STEP1_MECANUM_DEBUG_GUIDE.md](STEP1_MECANUM_DEBUG_GUIDE.md)

---

## 1. 当前代码状态（已核实）

### 1.1 已达成

| 事项 | 证据位置 | 状态 |
|---|---|---|
| 4×M3508 轮速闭环（级联 PID）跑通 | `module_dji_motor`，速度环 Kp≈100 / Ki≈5 | ✅ 整机联调通过 |
| 麦轮逆解（含绕任意旋转中心） | `USERS/Components/Algorithm/alg_mecanum/` | ✅ 已用 |
| 参考系/车体坐标变换 | `alg_chassis_motion.c` | ✅ 已用（无云台时 `gimbal_yaw_rad=0`，变换为恒等） |
| 方向符号标定 | `robot_config.c` `direction_sign = {1,-1,1,-1}` | ✅ 已标定 |
| 4 电机全在线保护 | `app_chassis.c` `APP_CHASSIS_ALLOW_PARTIAL_WHEELS=0` | ✅ 已恢复整机模式 |
| 安全门控（遥控离线→NO_FORCE） | `app_safety` + `robot_task.c` | ✅ 已用 |
| 调试基础设施 | RTT 100Hz 波形（FL 目标/反馈轮速）+ 100ms 四行状态日志 | ✅ 已用 |
| 摇杆死区 | `robot_adapter.c` `apply_deadband()`（±30） | ✅ 已用 |

### 1.2 当前控制数据流

```
遥控器(I6X) ──► robot_adapter_update() ──► chassis_cmd (vx, vy, wz)
                 模式固定 NORMAL，开环，无加速度限幅，无指数曲线
                        │
                        ▼
        robot_task_control() ──► app_chassis_update() ──► alg_mecanum_inverse() ──► 四轮目标速度
                        │
                        ▼
        module_dji_motor 速度环 PID ──► module_dji_motor_bus_update()
```

IMU 链路（**与底盘控制完全断开**）：

```
BMI088 ──► INS_Task.c (1kHz, QuaternionEKF) ──► INS_Info 全局变量 ──► (无控制消费者)
app_types.h 中 app_imu_snapshot_t 已定义，全工程无使用者
```

### 1.3 缺口清单（按影响排序）

| # | 缺口 | 当前代码证据 | 影响 |
|---|---|---|---|
| 1 | 偏航无陀螺仪闭环，wz 纯开环 | `app_chassis_update()` 直接把 wz 送逆解 | 旋转速率受负载/摩擦影响，打滑失控 |
| 2 | `APP_CHASSIS_MODE_SPIN` 无处理分支 | `app_chassis.c` 只处理 FOLLOW_GIMBAL | 小陀螺不可用 |
| 3 | IMU 数据未桥接至控制层 | `robot_t` 无 imu 成员，INS_Info 无消费者 | 姿态类功能全部无法开展 |
| 4 | FOLLOW_GIMBAL 单级 P、未串速度环 | `follow_gain × wrap(yaw)` 直接作 wz | 回正慢、易过冲；且无云台阶段该模式无意义 |
| 5 | 无加速度限幅 | `robot_adapter.c` 摇杆值直接乘满速 | 起步打滑、电流冲击 |
| 6 | 无摇杆指数曲线 | `normalize_channel()` 线性归一 | 低速平移不精细 |
| 7 | 无底盘功率限制 | 无裁判系统接入 | 比赛超功率风险 |
| 8 | 无里程计 | `alg_mecanum_forward()` 未被调用 | 功率控制与定位前提缺失 |
| 9 | 自锁未接线 | `chassis_cmd.self_lock_when_stopped` 恒 false（零初始化） | 停车无保持力矩 |

---

## 2. 目标控制架构

```
                            ┌──────────────────────────────────┐
 遥控指令(斜坡+指数曲线) ─► │ 偏航角速度环 (wz 闭环, P0 新增)   │ ◄── gyro z ── INS_Task
                            └──────────────┬───────────────────┘
                                           ▼
            麦轮逆解 (已有) ─► 四轮速度环 (已有, M3508 PID) ─► 轮电机 ×4
```

- 偏航角速度环串联在指令与逆解之间：`wz_out = wz_cmd + PID(wz_cmd − gyro_z)`（前馈+反馈叠加）。
- 内环（轮速）不动；新增环只修正 wz，vx/vy 不受影响。
- IMU 通过 `app_imu_snapshot_t` 单向流入 `app_chassis_update()` 新增入参，不产生反向依赖。

---

## 3. 分阶段实施方案

### P0 — 陀螺仪闭环跑通（核心）

#### P0-1 IMU 数据桥接

- **改动文件**：`ROBOT/robot_config.h`（`robot_t` 加成员）、`ROBOT/robot_task.c`
- **方案**：
  1. `robot_t` 新增 `app_imu_snapshot_t imu;`。
  2. `robot_task_control()` 每周期从全局 `INS_Info`（`USERS/Application/Task/Src/INS_Task.c`）拷贝。
- **单位换算（当前代码核实结果，必须处理）**：
  - `INS_Info.Angle[]` / `Yaw_Angle` 是**度**（INS_Task.c 中已乘 57.2958），桥接时需 ×π/180 转 rad；
  - `INS_Info.Gyro[]` 是 rad/s（BMI088 原始值），可直接用，z 轴取 `IMU_GYRO_INDEX_YAW` 对应索引，需确认该索引映射到 BMI088 的哪个物理轴及符号；
  - `valid` 可由 BMI088 在线状态推得。
- **注意**：`gimbal_yaw_rad` 在无云台阶段保持 0（保证参考系变换为恒等），**不要**填底盘自身 yaw，否则 vx/vy 会被坐标系旋转改变语义。IMU yaw 单独走快照成员。
- **验收**：日志打印 `imu.yaw_rad` 随手转底盘连续变化、无跳变；gyro z 符号与"车体左转为正"约定一致。

#### P0-2 偏航角速度闭环

- **改动文件**：`USERS/Application/app/app_chassis/app_chassis.{c,h}`、`ROBOT/robot_config.c`
- **接口设计**：
  ```c
  /* app_chassis_config_t 新增 */
  alg_pid_config_t yaw_rate_pid_config;   /* wz 闭环 PID 参数（复用 alg_pid 模块） */
  /* app_chassis_update() 增加入参：const app_imu_snapshot_t *imu */
  ```
- **控制逻辑**（`app_chassis_update()` 内，死区判断与逆解之前）：
  1. `wz_feedback = imu->angular_velocity_rad_per_s[2];`
  2. `wz_out = wz_cmd + PID(wz_cmd − wz_feedback, dt);`（输出叠加前馈，保留响应速度）
  3. 用 `wz_out` 参与死区判断与逆解。
- **退化路径**：`imu->valid == false` 时直接使用 `wz_cmd`（开环），不引入新故障点。
- **验收**：给定恒定 wz 指令，手按压底盘制造负载变化，RTT 波形观察实测 gyro z 仍稳定在给定值。

#### P0-3 补全 SPIN 模式

- **改动文件**：`app_chassis.c`、`ROBOT/robot_adapter.c`（拨杆映射 SPIN，建议 SWC 三位置拨杆：上=NORMAL、中=SPIN、下=NO_FORCE 之类，按手感定）
- **方案**：`app_chassis_config_t` 新增 `spin_rad_per_s`（含符号），`mode == APP_CHASSIS_MODE_SPIN` 时覆盖 `reference_velocity.angular_velocity_rad_per_s`，其余与 NORMAL 一致——自动获得 P0-2 的闭环稳速。
- **验收**：小陀螺实测转速波动 < ±5%，平移叠加正常。

### P1 — 操控品质

#### P1-1 航向角环（FOLLOW_GIMBAL 重定义）

- **现状**：`follow_gain × wrap(gimbal_yaw)` 单级 P，且当前无云台、`gimbal_yaw_rad` 恒 0，该模式实际无效。
- **方案**：改为基于 IMU 航向的**串级角度环**（无云台阶段语义为"航向锁定/定点"）：
  ```
  yaw_target_rad = 进入模式时锁定的 IMU yaw（或云台 yaw，有云台后）
  wz_cmd = PID_angle(wrap(yaw_target − imu_yaw))
  wz_cmd 进入 P0-2 偏航角速度环
  ```
- **改动文件**：`app_chassis.{c,h}`（新增 `yaw_angle_pid_config`、模式进入时刻记录 target）、`robot_config.c`
- **验收**：手扳动底盘后松手回正，回正时间与超调可调，无稳态振荡。

#### P1-2 加速度限幅

- **方案**：对 vx/vy/wz 指令各加一阶斜坡 `cmd += clamp(target − cmd, ±a_max·dt)`，`app_chassis_config_t` 新增三个 `*_accel_limit`。
- **改动文件**：`app_chassis.c`（NO_FORCE/失能/模式切换时斜坡状态清零，避免恢复瞬间跳变）
- **验收**：满杆起停，四轮电流无尖峰（看 `MW cmd=` 日志），轮子无尖叫打滑。

#### P1-3 摇杆指数曲线

- **方案**：`robot_adapter.c` 的 `normalize_channel()` 输出后做 `y = x·|x|^k`（k≈0.5~1 可配宏），小位移更精细。
- **验收**：低速平移可控到 0.1 m/s 档位。

#### P1-4 自锁接线（顺手项）

- **方案**：`robot_adapter.c` 中用空闲拨杆位映射 `self_lock_when_stopped = true`；闭环后死区判定建议改用实测值（P2-3）。

### P2 — 比赛级功能

#### P2-1 里程计（正解接入）

- **方案**：控制周期内用 `alg_mecanum_forward()` 将四轮实测速度正解为车体 vx/vy/wz，写入 `app_chassis_feedback_t`（新增实测字段）。
- **依赖**：P2-2 功率控制、上层定位导航均以此为前提。
- **验收**：直线推行 2 m，积分位移误差 < 5%（地面良好时）。

#### P2-2 底盘功率限制

- **方案**：
  1. 接入裁判系统功率/缓冲能量数据。
  2. 缓冲能量低于阈值时按比例缩放四轮**目标速度**（保持速度环结构，不直接限电流）。
  3. 限幅系数作为 `app_chassis_update()` 入口处的缩放因子。
- **验收**：满负载冲刺，裁判系统不判超功率，缓冲能量不触底。

#### P2-3 停车判定改进

- **现状**：死区判断用指令值（`app_chassis_update()` 中 `stopped` 由 `reference_velocity` 算出）。
- **方案**：闭环后改用实测值（IMU wz + 里程计 vx/vy），自锁更可靠。

---

## 4. 参数调试建议

| 参数 | 建议起点 | 调法 |
|---|---|---|
| yaw_rate_pid Kp | 2~5（输出叠加前馈） | 从小到大，直到实测 wz 跟踪指令无明显滞后；**首次必须空载小 Kp 验证方向** |
| yaw_rate_pid Ki | 0 起步 | 出现稳态偏差再加，防积分饱和 |
| 航向角环 Kp | 3~8 (rad/s)/rad | 观察回正时间，振荡即减 |
| accel limit | 线性 5 m/s²，角 20 rad/s² | 打滑就降 |
| 摇杆指数 k | 0.6 | 手感为准 |
| spin_rad_per_s | ±4~6 rad/s | 与 `ROBOT_MAX_ANGULAR_RPS`(6.0) 上限匹配 |

---

## 5. 风险与注意事项

1. **IMU 轴向与符号**：BMI088 装配方向决定 gyro z 的正负，接错会正反馈急速旋转失控。首次闭环必须小 Kp 空载验证；`IMU_GYRO_INDEX_YAW` 到物理轴的映射需在 P0-1 确认。
2. **单位不一致**：INS 角度是度、陀螺仪是 rad/s，桥接时只转角度；漏转会导致角度环增益错 57.3 倍。
3. **INS_Info 跨任务读取**：1kHz 写 / 1kHz 读，float 单字段撕裂风险低，yaw 建议单点拷贝；如需严格一致再加临界区。
4. **闭环与死区交互**：wz 死区判断在闭环修正后进行（用 `wz_out`），否则静止时 PID 噪声可能造成微抖。
5. **失能状态复位**：NO_FORCE/离线时必须复位 PID 积分与斜坡状态，避免重新使能瞬间跳变。
6. **gimbal_yaw_rad 语义**：无云台阶段保持 0；填底盘自身 yaw 会改变 vx/vy 坐标系语义，禁止。
7. **正运动学误差**：麦轮正解对轮速精度敏感，打滑时漂移，仅用于功率限制够用，勿直接当定位。
8. **速度上限余量**：当前轮速上限 60 rad/s × 0.076 m ≈ 4.5 m/s，指令上限 2.5 m/s，闭环修正叠加不会顶限幅，余量充足。

---

## 6. 改动点索引

| 阶段 | 文件 | 内容 |
|---|---|---|
| P0-1 | `ROBOT/robot_config.h` + `robot_task.c` | `robot_t` 加 imu 快照、INS_Info 桥接（度→rad）、gyro 轴/符号确认 |
| P0-2 | `app_chassis.{c,h}` + `robot_config.c` | 偏航角速度环、imu 入参、退化路径、PID 配置 |
| P0-3 | `app_chassis.c` + `robot_adapter.c` | SPIN 分支、`spin_rad_per_s`、拨杆模式映射 |
| P1-1 | `app_chassis.{c,h}` + `robot_config.c` | 航向角环串级、模式进入锁存 target |
| P1-2 | `app_chassis.{c,h}` | vx/vy/wz 斜坡限幅、失能复位 |
| P1-3 | `robot_adapter.c` | 摇杆指数曲线 |
| P1-4 | `robot_adapter.c` | 自锁开关接线 |
| P2-1 | `robot_task.c` | 正解接入、反馈实测字段 |
| P2-2 | `robot_task.c` + 裁判系统模块 | 功率限制 |
| P2-3 | `app_chassis.c` | 停车判定改实测值 |
