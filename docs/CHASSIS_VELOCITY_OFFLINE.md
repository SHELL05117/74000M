# 底盘速度闭环离线实现

状态：`PCValidated` 算法能力，真实 `autonomous_chassis_velocity` capability 仍为 `false`。

## 数据流

```text
已仲裁 FutureAutonomy ChassisVelocityPayload
  -> 模式/epoch/owner/TTL/capability 检查
  -> StateSnapshot 年龄与 translation/heading quality 门
  -> 差速逆解得到左右轮 m/s
  -> 分侧、分方向 kS/kV/kA 前馈 + 分侧 EngineeringPid
  -> 有界 FutureAutonomy WheelVoltagePayload
  -> SafetyGate -> OutputService
```

直接把 `ChassisVelocityPayload` 交给 SafetyGate 仍会停止；只有闭环转换器发布的、来源仍为 `FutureAutonomy` 的 `WheelVoltagePayload` 才可在本地测试 capability 打开时通过。`Test` 电压和自动电压按来源分别授权，互不借权。

## 离线门

- 差速逆解、正反目标、左右独立 FF/PID；
- 输出饱和与条件积分；
- 目标加速度只用于前馈且受配置上限限制；
- Degraded 可按显式策略缩放，Invalid/不允许的 Degraded/未来/陈旧/错误 epoch 一律不发布；
- 端到端测试覆盖 Arbiter → 闭环 → SafetyGate → OutputService → FakeDriveIO；
- 控制器 reset 清除 PID、目标差分和 epoch 历史，不会复活旧请求。

测试系数不写入硬件配置。机器完成后必须分别采集左右、正反的 quasistatic/dynamic 数据，验证轮速阶跃/斜坡、低电、热态、降额、状态年龄和质量恢复，再按 `docs/HARDWARE_COMMISSIONING.md` 决定是否提升 capability；当前不得开启真机自动速度请求。
