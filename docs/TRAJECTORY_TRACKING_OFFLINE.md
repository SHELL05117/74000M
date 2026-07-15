# 路径/轨迹跟踪器（离线）

状态：跟踪数学、质量门与失败语义 `PCValidated`；仿真/Fake/replay 可用，HIL/真机未测试。

## 跟踪边界

- 按运行时间从固定轨迹采样目标，并报告归一化距离进度；
- 将世界坐标位置误差旋转到当前机体系，以纵向、横向和航向误差修正参考 `vx/omega`；
- 输出仅为 `FutureAutonomy ChassisVelocityPayload`，继续交给板块 16 的分侧速度闭环；
- 线速度、曲率和角速度分别限幅，限制原因进入结果位；
- translation/heading Invalid、状态未来/陈旧、mode/epoch/owner/capability 错误立即 Brake；
- Degraded 状态和 Degraded slip 分别显式降速；slip 持续超时或 Invalid 中止；
- 位置/航向偏离边界中止；轨迹结束后同时满足最终位置/航向误差与实际速度 settle，另有 timeout/stall；
- cancel、成功或任意失败只产生 Brake，旧 tracker 失活，不能继续发布请求。

PC 测试覆盖直线、曲线误差符号、正反、扰动恢复指令、曲率限制、质量/年龄、滑移降速与中止、偏离、终点 settle、timeout 和确定性重放。真实直线/圆弧/S 弯/8 字、滑移阈值、扰动和全速跟踪仍为 `NOT TESTED`，不得据此开启真机自动能力。
