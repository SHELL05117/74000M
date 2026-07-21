# 底层离线冻结门

状态：接口可离线冻结，真机硬门未通过。禁止据此开始真机自动运动调参。

## 冻结范围

以下边界在后续自动算法板块中按稳定接口使用；如需破坏性修改，必须重新执行本板块全部 PC 测试和静态审计：

- `platform/io.hpp`：硬件读写抽象；
- `state/raw_inputs.hpp`、`state/robot_state.hpp`：输入与状态快照；
- `drive/drive_request.hpp`：命令只发布请求；
- `commands/drive_request_arbiter.hpp`：来源、owner、epoch、TTL 和 capability 仲裁；
- `drive/safety_gate.hpp`：唯一的请求到执行帧安全转换；
- `drive/output_service.hpp`：唯一业务电机写入路径与独立输出 TTL；
- `runtime/mode.hpp`、`runtime/mode_manager.hpp`：break-before-make 与 epoch 生命周期；
- `telemetry/log_frame.hpp`：重放所需固定日志 schema。

## 已通过的离线门

| 门 | 证据 | 结论 |
|---|---|---|
| 唯一写者 | `tools/audit_offline_freeze.ps1` 扫描源树 | 仅 PROS adapter 含物理 motor writer；业务调用只经 OutputService |
| Disabled 与 epoch | `test_runtime.cpp`、`test_freeze_gate.cpp` | 旧 epoch 满压帧、Disabled 请求均变为 stop |
| 请求 TTL | arbiter 与端到端 FakeIO 故障注入 | 陈旧请求不能进入 SafetyGate |
| 输出 TTL | OutputService 独立故障注入 | 控制帧停止更新后独立 brake |
| 非法帧 | future、乱序、NaN/Inf、越界和错误 owner 测试 | 拒绝输出并停止 |
| 手动/安全矩阵 | shaping、QuickTurn、heading assist、仲裁、SafetyGate 测试 | 数学与状态机 PC 通过 |
| 定位与质量 | 多布局 odometry、IMU-first、wheel fallback、slip quality 测试 | 离线数值行为通过 |
| 日志重放 | schema、CRC、ring 压力、replay 测试 | 离线完整性与确定性通过 |
| 语言与平台隔离 | PC MSVC C++17、PROS GNU++17 构建 | 通过 |

全套 PC 测试和 PROS 构建必须与本报告同一提交重新执行。`config/robots/492X.yaml` 与 `config/robots/492Z.yaml` 只把 PC 状态提升为 `PCValidated`；所有硬件/驾驶/定位/自动/路线 capability 仍为 `false`，路线仍为 `DoNothing`。

## 未通过的真机硬门

以下全部为 `NOT TESTED`，不能由 FakeIO 或预设数据替代：

1. 每个端口的设备身份、正反向、齿比、单位和受限点动；
2. Disabled/断控/epoch/双 TTL 在真实任务调度和真实电机上的停止延迟；
3. HAL 的 Coast/Brake/Hold 可区分性和单设备故障定位；
4. 传感器 freshness、冻结、跳变、同侧一致性与恢复阈值；
5. 直线、旋转、圆弧、多圈及闭合路径的定位误差与质量降级；
6. 手动中立、满幅、换向、QuickTurn、断联、热/低电/推撞矩阵；
7. SD 缺失/慢写/写满和 HMI 最坏负载下的控制 p50/p95/p99/max 时序；
8. Field Control 实际连接状态及比赛模式切换。

完成方法与记录字段见 `docs/HARDWARE_COMMISSIONING.md`。在这些证据完成前，验证等级不得写为 `HILValidated` 或 `FieldValidated`，也不得解锁任何真机运动 capability。后续板块只允许继续开发平台无关算法、FakeIO、仿真和离线编排。
