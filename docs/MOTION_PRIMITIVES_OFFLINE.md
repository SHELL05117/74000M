# 非阻塞自动运动原语（离线）

状态：五类命令接口与失败语义 `PCValidated`；真机自动运动 capability 保持关闭。

## 统一生命周期

`INIT -> RUNNING -> SETTLING -> SUCCESS`，并可从运行阶段确定进入 `TIMEOUT`、`STALLED`、`STATE_INVALID` 或 `INTERRUPTED`。每条命令：

- 只在 Scheduler 提供的单次 `initialize/execute/end` 调用中工作，没有内部等待循环；
- 只发布 `FutureAutonomy` 的 `ChassisVelocityPayload` 或 `BrakePayload`；
- 每个请求携带 owner lease、mode epoch、时间戳和 TTL；
- 状态年龄、模式、epoch、位置/航向质量或数值非法时立即发布 Brake 并失败；
- 完成、timeout、stall、取消和模式边界均发布 Brake；同一帧不重复发布制动；
- reset 由每次 initialize 重建 profile、起点、termination 和 owner，不复用上一命令历史。

## 原语

- `DriveDistanceCommand`：正反距离共用梯形/三角 profile，以起始航向投影距离并在线修正航向；
- `TurnToHeadingCommand`：绝对航向使用 `wrapPi` 最短角，平移质量可不参与；
- `DriveArcCommand`：前进/后退与 CW/CCW 共用曲率几何参考，输出耦合 `vx/omega`；
- `DriveToPoseCommand`：世界坐标目标，允许显式选择后退，最终位置与航向共同进入 settle；
- `BrakeCommand`：持续发布 Brake，直到实际线/角速度在 settle 带内。

PC 测试覆盖长短架构、正反、跨 ±π 转向、前后弧线、后退点到点、settle、timeout、stall、Invalid、取消和旧 epoch。所有测试参数仅为合成参数；必须先完成底盘方向、单位、速度闭环和定位质量真机门，才可从 Brake 开始逐项低速 HIL 验收。
