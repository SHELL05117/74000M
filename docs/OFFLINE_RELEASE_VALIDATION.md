# 离线交付验收与上机发布门

发布状态：`OfflineDevelopmentCandidate / PCValidated`。不是 HIL、Field 或 CompetitionApproved 发布。

## 已完成的离线链路

```text
固定轨迹生成
  -> 轨迹跟踪 ChassisVelocityPayload
  -> 自动安全监督与降级过滤
  -> DriveRequestArbiter
  -> 分侧轮速 FF + PID
  -> SafetyGate
  -> OutputService
  -> FakeDriveIO
  -> 合成编码器反馈与 SE(2) 位姿
  -> 下一控制周期
```

端到端测试使用合成、明确标注的底盘常数，验证接口、方向、TTL、owner、epoch、饱和和闭环收敛；这些常数不写入 `hardware_profile.yaml`，也不能替代真机参数。

## 发布矩阵

| 阶段 | 状态 | 证据/缺口 |
|---|---|---|
| 仿真与数值测试 | PASS | 运动学、SE(2)、profile、trajectory、tracker、故障矩阵 |
| PC 单元/属性测试 | PASS | `robot_tests` 全量通过 |
| 静态安全审计 | PASS | 唯一写者、平台隔离、无电池放大、capability 锁 |
| PROS GNU++17 构建 | PASS | hot package 编译/链接 |
| Fake 闭环 | PASS | 全自动链路到 FakeDriveIO 并反馈收敛 |
| 旧真机日志重放 | NOT TESTED | 未提供历史硬件日志；仅重放框架与合成帧通过 |
| HIL | NOT TESTED | 机器人仍在搭建 |
| 低速直线/转向/正反长短/CW/CCW | NOT TESTED | 需机器、场地和急停 |
| 点到点/圆弧/S 弯/8 字 | NOT TESTED | 需先通过速度闭环与定位质量门 |
| 整条路线与故障注入 | NOT TESTED | 当前没有 approved 路线 |
| 低电/热态/负载 | NOT TESTED | 需实测电池、温度、电流和机构载荷 |
| CompetitionApproved | NOT APPROVED | 全部 capability=false，selected route=DoNothing |

## 可追溯验证

运行：

```powershell
.\tools\validate_offline_release.ps1
```

脚本依次执行 CMake/CTest、完整测试程序、离线冻结审计和 PROS 构建，并在 `build/offline_release_evidence.json` 记录：commit、`hardware_profile.yaml` SHA-256、robot ID、calibration revision、测试摘要、未测试项和回滚 commit。`build/` 是本地证据目录，不作为批准材料提交；上机后应把原始日志、manifest、分析结果和签字批准复制到受版本控制的发布包。

当前回滚基线为自动安全板块之前的 `a9b97ef`，各中间板块也有独立 Git commit。机器完成后的参数、调试顺序和逐级 capability 解锁严格按 `docs/HARDWARE_COMMISSIONING.md`；HMI/SD 另见 `docs/HMI_OFFLINE_VALIDATION.md`。

## 当前硬锁

`hardware_output=false`、`driver_control=false`、`pose_good=false`、`autonomous_chassis_velocity=false`、`autonomous_motion=false`、`competition_routes=false`。在缺少相应 HIL/Field 证据时，任何人不得只改布尔值绕过门禁。
