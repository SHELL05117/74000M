# 自动安全与失败策略（离线）

状态：策略解析、制动优先级、重试屏障和故障记录 `PCValidated`；真实碰撞/滑移/机构阈值未测试。

## 明确结果与策略

每个命令结果归一为 `Success/Timeout/Stalled/StateInvalid/Interrupted/Collision/Deviation/MechanismConflict`。固定策略表为每种失败指定 `Retry/Skip/EndRoute/EmergencyStop`、最大重试次数和耗尽后的动作；缺失策略默认 EmergencyStop。

安全监督器额外检查 mode/epoch/owner、状态年龄与质量、slip、偏离量、碰撞证据和机构冲突。任何失败都会锁存带 route ID、command ID、lease、attempt、结果和动作的记录，并发布 `RequestSource::Safety` 的 Brake；它在 Arbiter 中优先于自动请求。

## 不复活旧请求

- Retry 不直接重发命令或请求；
- 必须由 Scheduler 产生同命令的新 lease；
- 必须确认 Brake 已穿过输出屏障，且失败条件已消失；
- 同 lease、旧 lease、错误 epoch 或未确认屏障一律拒绝；
- Skip 同样要求新命令 lease 和制动屏障；
- 安全决策只允许过滤当前 owner 的 `ChassisVelocityPayload`，Degraded 比例在进入板块 16 前强制应用；旧 owner 不能通过过滤器。

PC 故障注入覆盖降级持续、Invalid、两次 timeout、stall skip、碰撞、偏离、机构冲突、模式中断、成功换命令和 Safety 仲裁优先。真机阈值、重试是否有意义、碰撞边界和机构恢复条件必须逐项 HIL/场地批准；当前默认路线仍应对失败 End/EmergencyStop。
