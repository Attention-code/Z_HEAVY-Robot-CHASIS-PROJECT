# SMC（滑模控制器）使用指南

> 对应实现：`slidingmodec.c / slidingmodec.h`（纯 C，无动态内存，单实例单控制轴）
>
> 适用场景：云台 Yaw/Pitch 等需要强抗扰、快速收敛的角度控制回路。
> 底盘速度环等平稳性要求高的场合建议仍使用 `alg_pid` 串级 PID。

---

## 1. 算法原理（结合本实现）

控制器分两个误差区间工作：

### 1.1 小误差区（`|error| < 1.0`，线性滑模面）

```
s  = C·e + ė                        （滑模面）
u  = J·(target_ddot − C·ė − ε·sat(s) − K·s)
```

- `sat(s)` 为饱和函数（边界层法），**用平滑切换代替符号函数，抑制抖振**。
- `K·s` 项提供指数收敛。

### 1.2 大误差区（`|error| ≥ 1.0`，幂次趋近律）

```
e_qp = sign(e)·|error|^(q/p)        （非线性终端滑模项）
s    = ė + C·e_qp
u    = J·(target_ddot + ṡ − C·(q/p)·ė·e_qp/e)
```

- `q/p` 通常取 1 ~ 2 之间，误差大时提供更强的纠偏能力。
- 误差跨过 1.0 边界时两段公式自动切换。

### 1.3 死区

`|error| < error_eps` 时直接输出 `u = 0`，避免稳态抖动和来回微调。

---

## 2. API 一览

```c
typedef struct SMC { ... } SMC;

void SMC_Init(SMC *self, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon);

void SMC_Reset(SMC *self);

void SMC_Tick(SMC *self, float angle_now, float angle_vel);
```

| 函数 | 说明 |
|------|------|
| `SMC_Init` | 写入全部控制参数并复位状态。`target_last = ref`，**启动时不会产生目标突变的微分冲击** |
| `SMC_Reset` | 只清除运行时历史量（error/s/u/target_dot 等），**保留控制参数**。重新使能控制器时调用 |
| `SMC_Tick` | 每个控制周期调用一次。传入当前角度和角速度，结果写入 `self->u` |

修改目标值直接写 `smc.target = xxx;`，无需额外 API。

---

## 3. 参数说明与参考量级

以云台 Yaw（DM 电机、输出为扭矩 Nm）为例，参考 `readme_c.md` 示例：

```c
SMC_Init(&yaw_smc, 20.0f, 120.0f, 0.0f, 0.001f, 21.0f, 27.0f, 25000.0f, 0.8f, 0.5f);
//                 C    K     ref  eps     q     p    u_max     J    epsilon
```

| 参数 | 含义 | 作用 | 调参方向 |
|------|------|------|----------|
| `C` | 滑模面斜率 | 越大 → 角度误差权重越高，响应越快但超调风险大 | 从小往大调，先保证不振荡 |
| `K` | 滑模面收敛增益 | 越大 → 滑模面上收敛越快，**过大引起抖振** | 第二个调，配合 `epsilon` |
| `ref` | 初始目标值 | 一般给 0，运行中直接改 `smc.target` | — |
| `error_eps` | 输出死区 | 稳态抗抖动。太小→稳态抖，太大→定位精度差 | 云台建议 0.0005~0.005 rad |
| `q`, `p` | 幂次趋近律指数 | `q/p` ∈ (1, 2] 可加快大误差收敛；示例 21/27 ≈ 0.78（偏保守） | 先不动，或让 `q/p ≈ 1.2` |
| `u_max` | 输出限幅 | 对齐电机协议的最大扭矩（注意协议单位换算） | 必须设置，防飞车 |
| `J` | 等效转动惯量 | 把加速度需求映射为扭矩的关键标度，**量纲核心** | 按负载估算，输出偏小则调大 |
| `epsilon` | 边界层切换增益 | 抵抗建模误差/扰动的项。越大抗扰越强、抖振越大 | 最后微调 |

### 单位约定（重要）

- `SMC_Tick` 输入：角度 `rad`，角速度 `rad/s`（来自 IMU 或电机反馈，**不要混用编码器多圈值**）。
- `u` 的单位由 `J` 决定：`u = J × 角加速度`，即若 `J` 取等效惯量（kg·m²），`u` 为扭矩 `Nm`。
- MIT 模式 DM 电机可直接把 `u` 作为 torque 前馈/目标发送。

---

## 4. 标准使用流程

```c
#include "slidingmodec.h"

static SMC yaw_smc;

/* 1. 初始化（上电一次） */
void Gimbal_Control_Init(void)
{
    SMC_Init(&yaw_smc, 20.0f, 120.0f, 0.0f, 0.001f,
             21.0f, 27.0f, 25000.0f, 0.8f, 0.5f);
}

/* 2. 重新使能时复位历史（参数保留） */
void Gimbal_On_Enable(void)
{
    SMC_Reset(&yaw_smc);
}

/* 3. 控制周期（如 1kHz） */
float Gimbal_Yaw_Update(float target_angle)
{
    const float angle_now = INS_GetYawAngle();        // rad，连续角
    const float angle_vel = INS_GetYawGyroZ();        // rad/s

    yaw_smc.target = target_angle;
    SMC_Tick(&yaw_smc, angle_now, angle_vel);

    return yaw_smc.u;   // → 电机扭矩指令
}
```

---

## 5. 调参步骤（推荐顺序）

1. **先设 `K = 0`，调 `C`**：只保留滑模面比例项，逐步增大直到响应够快且不超调。
2. **调 `J`**：观察实际输出的“力度”。同样误差下输出偏软 → `J` 调大；输出顶到 `u_max` 频繁 → 调小或放宽 `u_max`。
3. **调 `K`**：提高滑模面收敛速度。出现高频抖动（“嗡嗡”声 / 电机发烫）说明过大。
4. **调 `epsilon`**：有外部扰动（打弹后坐力、底盘加减速）导致稳态被推开时增大；抖振明显时减小。
5. **调 `error_eps`**：最后定稳态精度与抖动的平衡。
6. `q / p` 保持默认即可，除非大误差段收敛明显偏慢。

## 6. 注意事项（踩坑点）

1. **控制频率必须固定**：`target_dot` / `target_ddot` 用相邻两拍差分近似（未除以 dt），增益吸收了周期长度。改控制频率后所有参数需重调。
2. **角速度来源要一致**：`angle_now` 与 `angle_vel` 必须同源同轴（建议都用 IMU），否则 `ė` 计算错误会直接发散。
3. **角度必须是连续值**：IMU 连续角（不取模），或自行做最短路径解卷绕后再喂入。SMC 内部没有处理 ±π 跳变的逻辑。
4. **`u_max` 必须设置**：无内置保护输出为 0 之外的边界，漏设等于飞车风险。
5. **抖振对策**（按优先级）：调小 `K` → 调大 `error_eps` → 增大 `epsilon` 使切换更平滑 → 检查反馈噪声，必要时对 `angle_vel` 加低通。
6. **死区期间 `target_last` 不更新**：`error_eps` 内直接 `return`，目标连续缓变时无影响；目标大幅跳变后马上进死区可能残留旧的 `target_dot`，跳变目标时建议先 `SMC_Reset`。
7. **不支持前馈接入**：本实现已含 `target_ddot` 项（等效目标加速度前馈），不要再叠加额外前馈，除非改代码。

---

## 7. SMC vs 串级 PID 选型参考

| 维度 | SMC（本实现） | `alg_pid` 串级 |
|------|---------------|----------------|
| 抗突变扰动 | 强（滑模面设计目标） | 弱，靠积分慢慢顶 |
| 参数数量 | 8 个，但相互耦合 | 4 个（级间解耦，好调） |
| 稳态精度 | 依赖 `error_eps` 死区，精度换抖动 | 好，有积分消除静差 |
| 稳态抖振 | 需专门抑制 | 天然无 |
| 调参经验成本 | 高，需要理解滑模面 | 低，工程通用 |
| 推荐场合 | 云台 Yaw/Pitch | 底盘轮组、发射机构、随动 |

**本项目建议**：云台用 SMC（抗后坐力/耦合扰动），其余子系统继续用 `alg_pid`（框架已原生集成 `module_motor_pid_t`）。
