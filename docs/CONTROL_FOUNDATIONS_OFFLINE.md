# 自动控制算法基础库（离线）

本板块只冻结平台无关数学与状态机，验证等级为 `PCValidated`。所有增益、速度、加速度、jerk、settle、timeout 和 stall 数值均仍需本机数据；测试中的系数只用于验证公式，不是生产参数。

## 接口与约束

- `EngineeringPid`：显式实测 `dt`，微分先行、一阶 D 低通、积分分离、条件积分、积分钳位、误差死区、输出饱和和完整 reset。非法配置、NaN/Inf 或越界 `dt` 不更新内部历史。
- `calculateFeedforward`：计算 `V = kS(direction) + kV v + kA a`；支持左右侧及正反方向独立系数。函数不读取电池，也不做 `12 / battery_V` 放大。
- `TrapezoidProfile`：支持三角退化、梯形巡航、正反位移和可行的非零初速；不可行边界明确拒绝。
- `QuinticSCurveProfile`：以五次 smoothstep 提供零端点速度/加速度的 S 曲线，并按速度、加速度和 jerk 上限选择最短候选时长。
- `MotionTerminationMonitor`：同时检查误差带与速度带；settle、timeout、stall、状态无效和时间回退均产生确定状态；reset 清除全部跨命令记忆。

这些类型不包含 PROS 头、任务循环、动态分配、`DriveIO` 或硬件写入。后续命令只能使用它们计算 `DriveRequest`，仍须经过 Arbiter、SafetyGate 和 OutputService。

## PC 验证矩阵

- PID：可变 `dt`、目标阶跃无微分踢、D 滤波、积分区/钳位/抗饱和、死区、reset、非法边界和 NaN/Inf；
- 前馈：左右/正反/静止起步符号、非法系数；
- 规划：短/长、三角/梯形、正/反、非零初速、不可行边界、S 曲线端点及全程约束；
- 退出：误差+速度 settle、timeout、stall、状态失效、时间倒退和 reset。

真机必须按 `docs/HARDWARE_COMMISSIONING.md` 用独立训练/验证数据确定所有参数。没有这些证据时不得提升 profile，也不得开启 `autonomous_chassis_velocity`。
