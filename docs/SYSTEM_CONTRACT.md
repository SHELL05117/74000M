# 74000M 离线系统契约

本文件冻结可以在无真机阶段确认的系统边界。硬件参数的权威预留表位于
`config/hardware_profile.yaml`；真机到位后的填写、调试和能力解锁步骤位于
`docs/HARDWARE_COMMISSIONING.md`。

## 平台与范围

- 平台：VEX V5 + PROS 4.2.2，项目代码固定为 C++17。
- 当前可离线实现：HAL 接口、Fake IO、运行时、状态估计、手动控制、保护、
  HMI、自动算法与路线框架，以及 PC/仿真/重放测试。
- 当前不得解锁：真实电机输出、驾驶、Good 位姿、自动底盘速度、自动运动、
  比赛路线。
- 自动选择固定回退到稳定 `RouteId` 的 `DoNothing`；其他路线即使注册也必须
  保持 `implemented=false`、`competition_approved=false`。

## 坐标和单位

- 机体 `+X` 指向前方，`+Y` 指向左方，`+Z` 向上。
- 从上方观察，逆时针为 `+theta` 和 `+omega`。
- 左右侧正轮速都表示机器人向前，正电压必须对应正编码器增量。
- 核心只使用 m、rad、s、V、A、°C；degrees、centidegrees、RPM、mV 只允许
  出现在平台适配和人机显示边界。

## 所有权和安全

1. 驱动电机只能由 `OutputService` 经平台 HAL 写入。
2. 手动、测试和未来自动功能只能提交带 source、epoch、timestamp、sequence、
   owner lease 和 TTL 的 `DriveRequest`。
3. 每个控制拍只产生一份硬件输入快照；消费者只读同拍状态。
4. 模式切换采用 break-before-make，并递增 `mode_epoch`。
5. 请求和执行器帧分别设时效门；禁用、陈旧、未来时间戳、错误 epoch/owner、
   NaN/Inf 和越界值都必须停车。
6. 物理电压请求不得乘 `12 / battery_V`；有意停车通过 brake mode + `brake()`。

## 无硬件时的证据边界

离线通过只允许升级 `Implemented`、`PCValidated` 或 `SimValidated`。端口、反向、
机械几何、校准、制动行为、任务时序和保护阈值在真机确认前保持
`Unverified`，对应 capability 必须为 `false`。预设数据只能用于验证接口、数值
稳定性、状态机和故障路径，不能成为比赛参数。
