# VEX V5 PC 状态监控、记录、分析与调参系统完整策划案

> 文档版本：1.0  
> 创作：OpenAI GPT-5.6（Codex）  
> 创作日期：2026-07-20  
> 适用项目：74000M / 1690X VEX V5 PROS C++17 工程  
> 对应计划板块：板块 5（日志、Fake IO、重放和测试底座）  
> 当前文档状态：Implementation Plan / 待实施  
> 当前验证等级：文档已形成；软件、链路、HIL 与真机效果均为 `NOT TESTED`

---

## 1. 执行摘要

本系统的目标是在机器人调试期间，通过 V5 Controller 到 PC 的连接持续接收机器人遥测，在 PC 上完成会话身份管理、实时监控、原始数据归档、完整性检查、自动分析、绘图、对比和报告生成。

推荐采用“完整快日志 + 精简实时遥测”的双层架构：

1. 机器人控制环每个名义 10 ms 控制拍生成一条定长快日志，写入预分配 SPSC 环形缓冲；
2. 后台 `TelemetryTask` 独立消费日志，绝不反压控制环；
3. 完整快日志优先保存到机器人 SD 或有界本地缓存；
4. Controller → PC 链路只发送精简实时遥测、事件和会话状态，默认 20 Hz，可选 50 Hz；
5. PC 保存收到的原始包，同时把完整日志转换为可分析数据；
6. 任何串口、Controller、PC、SD、CSV、绘图或报告故障都不得改变控制输出和控制时序；
7. 第一版 PC→机器人严格只读，不允许远程写 PID、驱动电机或改变 capability。

系统交付后，操作者可以在 PC 程序中选择：

- 新建正式测试会话；
- 新建快速临时会话；
- 复用或编辑队伍/机器人/操作者资料模板；
- 只监控、不记录；
- 打开已有日志做离线分析；
- 对比多个历史试次；
- 自动生成带身份、工况、完整性结论、图表和参数候选的报告。

---

## 2. 已核实的平台事实与频率定义

### 2.1 V5 Brain 官方硬件规格

VEX 官方 V5 Robot Brain 产品规格列出：

| 项目 | 官方规格 |
|---|---:|
| VEXos 主处理器 | 1 × ARM Cortex-A9，667 MHz |
| 辅助处理器 | 2 × Cortex-M0，各 32 MHz |
| 可编程逻辑 | 1 × FPGA |
| 用户处理器 | 1 × Cortex-A9 |
| 用户处理能力标称 | 1333 MIPS |
| RAM | 128 MB |
| Flash | 32 MB |
| USB | USB 2.0 High Speed，标称 480 Mbit/s |
| 可扩展存储 | FAT32，最高 16 GB |
| 无线 | VEXnet 3.0、Bluetooth 4.2 |

官方来源：

- VEX V5 Robot Brain 产品页：<https://www.vexrobotics.com/276-4810.html>
- VEX Library 的 V5 Brain 规格页：<https://kb.vex.com/hc/en-us/articles/360060662352-Understanding-the-V5-Robot-Brain>

重要边界：

- `667 MHz` 是处理器时钟，不是控制频率、传感器刷新率、串口帧率或 PC 图形刷新率；
- `480 Mbit/s` 是 Brain USB 2.0 物理接口规格，不代表 Brain → Controller → PC 无线桥可以达到该吞吐；
- Controller 中转链路必须在当前 VEXos、Controller、Radio、PROS CLI 和现场无线环境下实测；
- 不允许根据 CPU 主频推断“1000 Hz 控制一定可行”或“无线一定能传完整 100 Hz 日志”。

### 2.2 本项目需要区分的五类频率

| 频率 | 当前或推荐值 | 含义 | 是否可直接视为保证 |
|---|---:|---|---|
| CPU 主频 | 667 MHz | Cortex-A9 时钟 | 官方硬件事实 |
| PROS 调度检查 | 约 1 ms | PROS 抢占式优先级调度器的任务切换时间尺度 | 只说明调度时间尺度 |
| 底盘控制环 | 名义 100 Hz | 当前配置 `nominal_period_s=0.010` | 必须记录真实 `dt`、jitter、超期 |
| 完整快日志 | 候选 100 Hz | 每控制拍一帧，供重放与诊断 | 需压力测试 |
| PC 实时遥测 | 默认 20 Hz，可选 50 Hz | 人眼监控和实时曲线 | 需链路吞吐测试 |
| PC UI 重绘 | 建议 10–20 FPS | 图形刷新 | 与采集频率解耦 |
| 热/电池摘要 | 建议 5–10 Hz | 慢变量显示 | 不用于瞬态辨识 |

PROS 官方 RTOS 文档说明 `task_delay_until()` 使用毫秒周期，适用于稳定周期任务；PROS 多任务教程说明调度器为抢占式、优先级、轮转调度，并约每毫秒检查任务切换：

- <https://pros.cs.purdue.edu/v5/pros-4/group__c-rtos.html>
- <https://pros.cs.purdue.edu/v5/pros-4/multitasking.html>

### 2.3 为什么默认仍采用 100 Hz 控制与快日志

100 Hz 是本项目现有的名义控制节拍，不是由 667 MHz 简单相除得到。它兼顾：

- 电机和传感器实际数据更新；
- Smart Port 多设备读取时间；
- PROS 任务调度；
- 控制、估计、安全门和输出服务预算；
- 可变 `dt` 下 PID、滤波和里程计稳定性；
- 日志体积与无线/SD 吞吐；
- 调试可解释性。

实施时必须测量：

- `raw_dt_us`；
- `control_dt_used_us`；
- `exec_us`；
- `jitter_us`；
- 单次和连续超期；
- 每类传感器有效新样本率；
- 每帧采集跨度；
- 串口/Controller 链路丢包和接收延迟。

只有真机数据证明更高频率带来收益且不破坏时序，才允许建立新的候选配置；本策划案不建议在第一版提高控制频率。

---

## 3. 当前仓库基础与缺口

### 3.1 已有能力

当前工程已经具备：

- `LogFrame` 定长日志结构；
- 逐电机位置、速度、电流、温度、实际电压和故障字段；
- IMU、电池、位姿和机体速度字段；
- 请求、执行器、时序和故障字段；
- 固定容量 `SpscRing`；
- `TelemetryDrain` 与抽象 `TelemetrySink`；
- 日志 magic/schema/frame size/sequence/time/epoch/run ID 完整性检查；
- Fake IO、Fake Clock 和原始输入回放；
- 平台无关里程计、PID、前馈和自动控制基础库；
- PC CMake/CTest 测试入口。

### 3.2 当前缺口

当前可执行固件尚未：

- 在 `src/main.cpp` 创建日志环和 `TelemetryTask`；
- 在控制拍组装完整 `LogFrame`；
- 将 `LogFrame` 推入环；
- 实现 PROS SD sink；
- 实现 Controller/USB 串流 sink；
- 实现通信握手、CRC、断线恢复和分频；
- 实现 PC 接收、会话管理、存储、分析和报告；
- 记录 PID 的目标、误差、P/I/D/FF 分项；
- 记录轨迹参考、横向误差、饱和前后和过滤器状态。

### 3.3 当前机器人能力限制

当前 1690X 配置：

- `hardware.imu = false`；
- tracking wheel 未配置；
- 里程计校准为空；
- `pose_good = false`；
- 所有正式 `RobotCapabilities` 保持关闭；
- 自动路线仍为 `DoNothing`。

因此第一版可以先记录和显示：

- 六电机位置、速度、电流、温度和电压；
- 电池；
- Controller 输入；
- 请求和最终输出；
- 控制周期、执行时间和日志健康；
- 当前模式和故障。

二维位姿页面必须显示 `Invalid / NOT AVAILABLE`，直到 IMU、几何、方向和传感器组合完成校准并通过对应能力门。

---

## 4. 产品目标、非目标与用户角色

### 4.1 产品目标

1. 在 PC 上可靠创建、管理和关闭一次机器人测试会话；
2. 自动记录机器人真值身份和人工工况身份；
3. 实时展示关键状态，但不依赖实时链路保存完整证据；
4. 保存原始数据，允许未来版本重新分析；
5. 自动检查 schema、sequence、时间戳、丢帧和文件完整性；
6. 自动生成运动轨迹、速度、加速度、jerk、PID、频谱和健康图；
7. 支持同一日志对多个滤波器进行离线 A/B；
8. 支持多个运行的叠加对比和统计；
9. 输出可追溯的 HTML 报告和机器可读 JSON；
10. 对不完整或不可信试次明确输出 `REPEAT`，不静默修补。

### 4.2 第一版明确非目标

- 不从 PC 遥控机器人；
- 不从 PC 绕过 Controller、Scheduler、Arbiter、SafetyGate 或 OutputService；
- 不实时写 PID；
- 不自动把拟合参数写入比赛配置；
- 不把 Controller 无线链路当作硬实时总线；
- 不承诺未知机器人的定位精度、停止距离、温度阈值或无线带宽；
- 不用 PC 接收时间替代机器人单调时间；
- 不通过漂亮曲线掩盖丢帧、坏质量或不合法工况。

### 4.3 用户角色

| 角色 | 权限 |
|---|---|
| 操作员 Operator | 新建会话、连接、开始/停止记录、填写现场信息、添加备注 |
| 观察员 Observer | 记录安全观察、异常、碰撞、打滑和中止原因 |
| 分析员 Analyst | 离线分析、滤波器 A/B、运行对比、生成报告 |
| 审批人 Approver | 将报告结论升级为 FieldValidated/CompetitionApproved；第一版软件不自动批准 |

---

## 5. PC 端会话与身份信息设计

### 5.1 启动入口

PC 程序启动后显示五个入口：

1. **新建正式测试会话**
2. **新建快速会话**
3. **继续未完成会话**
4. **只监控，不记录**
5. **打开已有日志分析**

身份信息的创建和填写完全在 PC 端进行，但机器人自身身份必须来自握手，只读显示，不能被 PC 表单覆盖。

### 5.2 新建正式测试会话向导

向导分六步：

#### 第 1 步：团队与人员

| 字段 | 要求 | 来源 |
|---|---|---|
| 队号 `team_number` | 必填，可保存模板 | 人工或本地模板 |
| 操作员 `operator` | 必填 | 人工或最近值 |
| 观察员 `observer` | 真机运动时必填 | 人工 |
| 分析员 `analyst` | 可选 | 人工 |
| 审批人 `approver` | 创建时可空 | 报告批准时填写 |

支持：

- “新建身份资料”；
- “从历史资料选择”；
- “保存为默认队伍资料”；
- “本次使用但不保存”；
- 导出报告时匿名化操作员姓名。

#### 第 2 步：机器人连接与自动识别

PC 从机器人握手自动读取：

- `robot_id`；
- `robot_name`；
- hardware/config schema；
- calibration revision；
- firmware/software version；
- PROS kernel version；
- log schema；
- config hash；
- software commit；
- dirty flag；
- capability 位；
- 当前模式和 mode epoch；
- 已配置传感器与电机端口映射。

若机器人未连接，可先建立 `Draft` 会话；连接后必须完成身份合并才能进入 `Armed`。

禁止：

- PC 手工改写机器人上报的 `robot_id`；
- 用默认值替代缺失的 schema/config hash；
- 正式会话在机器人身份不匹配时继续记录并标为有效。

#### 第 3 步：测试定义

必填或条件必填字段：

- `test_case_id`；
- 测试类型；
- 前进/后退/CW/CCW/双向；
- repetition index；
- 主要变化变量；
- 目标距离、角度、速度或测试时间；
- 请求最大电压/速度候选；
- 超时和中止条件；
- 本次属于训练集、验证集或只观测；
- 预期结论：基线、A/B、校准、故障复现或验收。

测试类型枚举建议：

```text
IdleBaseline
ManualDrive
StraightDistance
Spin
SquareCW
SquareCCW
FigureEight
QuickTurn
Reversal
SysIdQuasistatic
SysIdDynamic
PidStep
PidProfile
FilterComparison
ThermalRun
LowBatteryRun
FaultInjection
Custom
```

#### 第 4 步：工况

建议字段：

- 场地/地面；
- 地点；
- 环境温度；
- 电池 ID；
- 电池起始状态；
- 机器人质量；
- 载荷；
- 重心备注；
- 轮胎型号、磨损、清洁状态；
- 传动/轮组 revision；
- 机械检查结果；
- IMU/定位状态；
- 备注。

字段可从上一次运行复制，但每次开始前要求人工确认。

#### 第 5 步：记录策略

用户可选择：

- 完整快日志：启用/禁用；
- SD 保存：启用/禁用；
- PC 实时包：20 Hz / 50 Hz / 自动降级；
- 是否保存 PC 接收原始包；
- 是否记录预触发窗口；
- 是否自动生成报告；
- 是否在结束后立即做完整分析；
- 是否保留匿名化导出副本。

正式验收会话不允许禁用完整性检查。

#### 第 6 步：审阅与创建

创建前显示：

- PC 人工身份；
- 机器人只读身份；
- 工况；
- 测试定义；
- 记录策略；
- capability 状态；
- 风险与未满足条件；
- 最终 `run_id` 和输出路径。

只有通过字段验证才能创建。

### 5.3 快速会话

快速会话用于现场观察：

- 队号、操作者可从默认资料自动填入；
- 测试类型默认为 `Custom`；
- 自动生成 run ID；
- 允许工况字段为空；
- 报告必须标记 `Informal / NOT FOR RELEASE`；
- 不得用于冻结参数或比赛批准。

### 5.4 会话状态机

```text
Draft
  -> Connecting
  -> IdentityVerified
  -> Armed
  -> Recording
  -> Finalizing
  -> Complete

异常分支：
Recording -> Aborted
Finalizing -> Incomplete
任意状态 -> InvalidIdentity
Complete -> Analyzed -> Reported -> Approved/Rejected
```

规则：

- `run_id` 创建后不可复用；
- `Recording` 后身份字段只能以审计事件追加，不能覆盖原值；
- 机器人断线时会话保持 `RecordingDisconnected`，继续记录 PC 事件；
- 重连必须重新握手并验证 robot/run/schema；
- run ID 不匹配时开启新分段，不把两台机器人数据拼接；
- 文件关闭或 CRC 失败时进入 `Incomplete`；
- 原始文件永不就地修改。

### 5.5 Run ID 规则

推荐：

```text
YYYYMMDD-HHMMSS_<team>_<robot>_<test>_R<repeat>_<random4>
```

示例：

```text
20260720-153042_1690X_1690X_PidStep_R03_A7F2
```

文件系统名称使用安全字符；原始人工显示名保存在 metadata 中。

---

## 6. 端到端系统架构

```text
V5 Brain
┌──────────────────────────────────────────────────────────┐
│ ControlLoop，名义 10 ms                                  │
│  ├─ 同拍读取一次 RawDriveInputs                          │
│  ├─ 计算 validated/state/request/actuator/fault          │
│  └─ 组装定长 LogFrame -> SPSC tryPush                    │
│                                                          │
│ OutputTask -> OutputService -> DriveIO（唯一电机写入者） │
│                                                          │
│ TelemetryTask，低优先级后台                              │
│  ├─ full binary sink -> SD                               │
│  ├─ compact live sink -> Controller/USB                  │
│  └─ event/health packets                                 │
└──────────────────────────────────────────────────────────┘
                           |
                    VEXnet / Controller
                           |
                         USB
                           |
PC
┌──────────────────────────────────────────────────────────┐
│ Transport Receiver                                       │
│  -> Deframer/CRC/Schema                                   │
│  -> Session Manager                                      │
│  -> Raw Packet Writer                                    │
│  -> Live State Store                                     │
│  -> Dashboard                                            │
│  -> Integrity + Decode + Parquet                         │
│  -> Analysis + Plots + Report                            │
└──────────────────────────────────────────────────────────┘
```

### 6.1 控制路径硬边界

必须保持：

- 只有 `OutputService` 写驱动电机；
- 控制环不做串口、文件、CSV、JSON、字符串格式化或动态分配；
- `tryPush` 失败只增加丢帧计数；
- TelemetryTask 不修改状态估计、不仲裁请求、不写电机；
- PC 断开不得触发运动控制逻辑变化；
- 日志故障属于观测域，不自动成为驱动输出命令；
- 所有真实时间由机器人单调时钟提供；
- PC 到达时间只用于链路健康和近似延迟。

---

## 7. 通信协议

### 7.1 传输原则

第一版推荐：

- 机器人内部和 SD 使用原生定长二进制快帧；
- Controller→PC 使用紧凑二进制包；
- 包边界使用 COBS；
- 包尾 CRC32；
- 禁止与任意 `printf` 文本混发；
- 调试文本改为独立事件包；
- 每个包有 sequence 和 robot monotonic time；
- 接收器遇到损坏包后能在下一 COBS 边界恢复。

如果当前 Controller 无线终端无法可靠传透明二进制，可降级为 Base64 包或短 NDJSON；这属于 HIL 选择，不能提前假定。

### 7.2 固定包头

建议网络字节序或明确的小端布局，schema 中固定：

```text
magic_u32
protocol_major_u16
protocol_minor_u16
packet_type_u16
flags_u16
payload_length_u32
packet_sequence_u32
robot_time_us_u64
run_id_hash_u32
mode_epoch_u32
payload[]
crc32_u32
```

### 7.3 包类型

```text
HELLO
HELLO_ACK
RUN_BIND_REQUEST
RUN_BIND_ACK
LIVE_CORE
LIVE_PID
LIVE_MOTOR
EVENT
HEALTH
LOG_CHUNK
RUN_END
ERROR
PING
PONG
```

第一版允许 PC 发送的包只限：

- 握手；
- run ID 绑定请求；
- ping；
- 请求当前只读元数据；
- 请求切换实时遥测分频。

机器人必须拒绝所有未定义写操作。

### 7.4 握手

握手成功条件：

1. magic 正确；
2. protocol major 兼容；
3. robot ID 非空；
4. log schema 可解析；
5. config hash 存在；
6. run ID 绑定成功；
7. PC 确认机器人身份；
8. capability 和模式已显示；
9. 时间戳单调；
10. 当前 packet sequence 合法。

### 7.5 时间处理

- 所有运动分析以 `robot_time_us` 为横轴；
- PC 保存 `pc_receive_monotonic_ns`，但不替换机器人时间；
- 通过多次 PING/PONG 估计往返时间和时钟偏移；
- 只报告延迟区间或估计值，不把单向延迟伪装成精确真值；
- 重连后重新估计；
- FFT、导数、积分和重放使用机器人真实时间戳；
- 需要频谱时先检查缺帧并按明确方法重采样。

### 7.6 带宽预算

当前 `LogFrame` 按现有 C++ 字段和常见 ABI 估算约 728 B：

```text
728 B × 100 Hz = 72.8 kB/s
```

这还不含 COBS、CRC、链路包头和重传，因此不得直接通过 Controller 全量发送。

建议 `LIVE_CORE` 控制在约 80–120 B：

```text
100 B × 20 Hz = 2.0 kB/s
100 B × 50 Hz = 5.0 kB/s
```

实际包长由协议实现后的 `static_assert` 和 golden packet 测试确定。最终频率由链路压力测试冻结。

### 7.7 降级策略

按以下顺序自动降级：

1. 50 Hz → 20 Hz；
2. 关闭逐电机实时页，只保留 `LIVE_CORE + EVENT`；
3. 20 Hz → 10 Hz；
4. 停止实时曲线，仅保留 2 Hz HEALTH；
5. 完全断开 PC sink，继续 SD/内存记录；
6. 任何情况下不要求控制环等待。

---

## 8. 数据模型

### 8.1 PC 身份元数据与机器人真值分域

`metadata.json` 分为：

```json
{
  "session_identity": {},
  "robot_identity": {},
  "software_identity": {},
  "test_definition": {},
  "environment": {},
  "recording_policy": {},
  "integrity_summary": {},
  "audit": []
}
```

- `session_identity`：队号、操作员、观察员等 PC 人工信息；
- `robot_identity`：机器人握手真值；
- `software_identity`：commit、dirty、PROS、schema、config hash；
- `test_definition`：测试类型、方向、重复、目标和限制；
- `environment`：地面、电池、轮胎、载荷；
- `recording_policy`：采样、分频和 sink；
- `audit`：创建、修改、开始、停止、中止、重连和报告事件。

### 8.2 快日志最低字段

#### 帧与身份

- `time_us`
- `sequence`
- `mode_epoch`
- `run_id_hash`
- `schema_major/minor`
- `test_case_id`
- `test_phase`

#### 原始输入

- 六电机逐端口位置、速度、电流、温度、实际电压、fault、API 状态；
- IMU rotation/rate/calibrating/API 状态；
- tracking wheel 原始累计值；
- 电池；
- Controller 原始轴、按钮和连接状态；
- competition mode/field connection。

#### 校验与状态

- 逐电机 quality/reject bits；
- 左右侧聚合位置、速度、spread、valid mask；
- 位姿 `x/y/theta`；
- `vx/vy/omega`；
- translation/heading quality；
- slip state；
- sensor age 和 fault。

#### 目标与控制

为了生成真正有用的 PID 图，需在新 schema 中增加：

- `target_position`
- `target_velocity`
- `target_acceleration`
- `target_jerk`
- `measured_position`
- `measured_velocity`
- `error`
- `p_term`
- `i_term`
- `d_term_raw`
- `d_term_filtered`
- `ff_static`
- `ff_velocity`
- `ff_acceleration`
- `controller_unclamped`
- `controller_clamped`
- `integrator_state`
- `integrator_enabled`
- `anti_windup_active`
- `saturated`
- `settle_state`

#### 请求、输出与安全

- request source/owner/lease/TTL/epoch；
- 仲裁拒绝原因；
- 分配前左右电压；
- 分配后左右电压；
- final 电压；
- stop mode；
- derate 各子因子；
- HAL write attempted/write ok；
- last written sequence；
- actuator age。

#### 时序与日志健康

- raw dt；
- math dt；
- exec；
- jitter；
- overrun；
- ring depth/high watermark；
- producer drops；
- sink failures；
- output heartbeat age。

### 8.3 不允许只记录派生值

速度、加速度、jerk、滤波后信号和 PID 输出都必须能追到原始累计位置、真实时间戳和原始命令。否则未来无法：

- 更换滤波器；
- 重新估计导数；
- 检查相位延迟；
- 排除坏样本；
- 重放新版本；
- 证明参数不是由错误预处理得到。

---

## 9. PC 软件技术栈与目录

### 9.1 推荐技术栈

| 功能 | 技术 |
|---|---|
| 语言 | Python 3.10+ |
| 包管理 | `pyproject.toml` + uv 或 pip |
| 串口 | pyserial；需要时 asyncio |
| 二进制协议 | `struct` + 生成的 schema/dataclass |
| 校验 | crc32、COBS |
| 数值 | NumPy、SciPy |
| 表格 | Polars 优先，Pandas 兼容 |
| 列式存储 | PyArrow / Parquet |
| 静态图 | Matplotlib |
| 交互图 | Plotly |
| 桌面/本地 UI | 第一版 Streamlit；稳定后可迁移 PySide6 |
| 报告 | Jinja2 HTML；可选 WeasyPrint PDF |
| 配置校验 | Pydantic |
| 测试 | pytest、Hypothesis |
| 打包 | PyInstaller 生成 Windows 可执行程序 |

### 9.2 推荐目录

```text
statusmonitor/
├── documents/
│   └── VEX_V5_PC状态监控与调参系统完整策划案_GPT5.6.md
├── protocol/
│   ├── telemetry_schema.yaml
│   ├── packet_ids.json
│   └── golden/
├── pc/
│   ├── pyproject.toml
│   ├── README.md
│   ├── src/statusmonitor/
│   │   ├── app.py
│   │   ├── cli.py
│   │   ├── transport/
│   │   ├── protocol/
│   │   ├── sessions/
│   │   ├── storage/
│   │   ├── live/
│   │   ├── integrity/
│   │   ├── analysis/
│   │   ├── plotting/
│   │   ├── replay/
│   │   └── reports/
│   └── tests/
├── shared/
│   └── generated_schema/
└── artifacts/                 # 默认 gitignore
```

### 9.3 CLI 入口

```powershell
statusmonitor connect
statusmonitor new-session
statusmonitor monitor
statusmonitor record --session <id>
statusmonitor finalize <run>
statusmonitor analyze <run>
statusmonitor report <run>
statusmonitor compare <run1> <run2> [...]
statusmonitor replay <run>
statusmonitor doctor
```

`doctor` 检查：

- Python 和依赖；
- 串口权限；
- 可用 COM 端口；
- schema；
- artifact 写权限；
- 磁盘空间；
- 报告渲染；
- golden packet 编解码。

---

## 10. PC 用户界面

### 10.1 页面

1. **Connect**
   - COM 端口；
   - Controller/Brain 识别；
   - 握手状态；
   - 协议和机器人身份；
   - 链路吞吐、丢包、延迟。

2. **Session**
   - 新建/复用身份；
   - 测试定义；
   - 工况；
   - run ID；
   - 开始/停止/中止。

3. **Live Dashboard**
   - 模式、capability、fault；
   - 电池、最高温度；
   - x/y/theta 和 quality；
   - vx/omega；
   - 请求与 final voltage；
   - PID 误差和分项；
   - 控制 dt、jitter、丢帧；
   - 状态明显标注 Good/Degraded/Invalid。

4. **Integrity**
   - schema；
   - sequence gap；
   - time regression；
   - CRC；
   - producer/sink/transport drops；
   - 可用分析窗口。

5. **Analysis**
   - 选择图表和分析模块；
   - 滤波器 A/B；
   - 频谱；
   - SysId；
   - PID 响应；
   - 机械、电流和热分析。

6. **Compare Runs**
   - 叠加曲线；
   - 统一时间/进度对齐；
   - 参数差异；
   - 工况差异；
   - 效应量和统计。

7. **Report**
   - 预览；
   - PASS/FAIL/REPEAT/NOT TESTED；
   - 人工备注；
   - 导出 HTML/PDF/JSON；
   - 匿名化。

### 10.2 UI 规则

- 接收与保存不运行在 UI 主线程；
- UI 卡死不得影响原始包写入；
- 绘图只取有界窗口或降采样视图；
- 完整数据保存在磁盘，不把全部长运行加载进内存；
- quality 无效时曲线仍可显示原始值，但背景和标签必须醒目标红；
- 数据有缺口时图上画断点，不插值伪装连续；
- 每张图显示 run ID、robot ID、schema、单位和滤波版本。

---

## 11. 存储与产物

### 11.1 运行目录

```text
statusmonitor/artifacts/<date>/<team>/<robot_id>/<run_id>/
├── metadata.json
├── audit.jsonl
├── raw/
│   ├── live_packets.bin
│   ├── fast_log.bin
│   └── events.bin
├── integrity/
│   ├── packet_integrity.json
│   ├── log_integrity.json
│   └── usable_windows.json
├── derived/
│   ├── samples.parquet
│   ├── events.parquet
│   └── analysis_manifest.json
├── plots/
├── summary.json
├── report.html
├── report.pdf
├── operator_notes.md
└── config_snapshot/
```

### 11.2 数据规则

- 原始文件只追加、只读归档；
- CSV 是导出格式，不是唯一真源；
- 派生数据带分析脚本版本和参数；
- 每个 plot 有生成配置；
- 不覆盖旧运行；
- 删除或排除样本必须有原因；
- 正式会话保留完整 metadata；
- dirty worktree 运行必须保存 dirty flag，正式批准前需保存补丁或拒绝批准；
- 文件 hash 写入 summary。

---

## 12. 自动分析与图表

所有分析先执行完整性检查。关键窗口缺帧时，瞬态、SysId、FFT 或 PID 结果默认 `REPEAT` 或带限制输出。

### 12.1 必生成图

1. `trajectory_xy`
   - 目标和实际二维轨迹；
   - 起点、终点、方向箭头；
   - quality 分段；
   - 横向误差着色；
   - 场地边界可选。

2. `position_time`
   - 目标/实际位置；
   - 误差；
   - settle 区间。

3. `velocity_time`
   - 目标/实际线速度；
   - 左右轮速度；
   - 角速度。

4. `acceleration_time`
   - 线加速度；
   - 左右轮加速度；
   - 角加速度；
   - 滑移和饱和标记。

5. `jerk_time`
   - 线 jerk；
   - 角 jerk；
   - 峰值、RMS、累计绝对 jerk；
   - 明确滤波和微分方法。

6. `pid_breakdown`
   - target/measured/error；
   - P/I/D/FF；
   - clamp 前后；
   - saturation；
   - anti-windup；
   - settle state。

7. `motor_health`
   - 六电机速度；
   - 电流；
   - 温度；
   - 同侧 spread；
   - fault 和 invalid。

8. `timing_integrity`
   - raw dt；
   - exec；
   - jitter；
   - p95/p99；
   - overrun；
   - ring depth；
   - drops。

### 12.2 运动与控制指标

至少计算：

- 终态位置和航向误差；
- 横向/纵向误差；
- path RMSE 和最大误差；
- 上升时间；
- 峰值时间；
- 过冲；
- 进入误差带时间；
- 连续 settle 时间；
- 稳态误差；
- IAE：`∫|e|dt`；
- ISE：`∫e²dt`；
- ITAE：`∫t|e|dt`；
- 控制输出 RMS；
- 控制输出 total variation；
- 饱和占比；
- 积分钳位占比；
- P/I/D/FF 能量或绝对贡献占比；
- 请求到响应延迟候选；
- 正反、CW/CCW 和左右差异。

### 12.3 速度、加速度与 jerk 算法

禁止对噪声位置做两次或三次裸差分后直接下结论。

推荐流程：

1. 检查时间戳和缺帧；
2. 对累计位置使用真实时间的局部线性/多项式窗口估计速度；
3. 对速度使用 Savitzky-Golay、局部回归或明确截止频率的低通；
4. 从处理后的速度估计加速度；
5. 从处理后的加速度估计 jerk；
6. 保存窗口、阶数、截止频率和群延迟；
7. 同时保存 raw-derived 和 filtered-derived；
8. 在线滤波与离线零相位滤波分开命名；
9. 离线零相位结果不得直接当作实时控制可实现效果。

### 12.4 FFT、傅里叶级数与频率分析

#### 普通振荡

使用：

- FFT；
- Welch PSD；
- STFT；
- 主峰频率；
- 带内功率；
- 谐波比；
- error、D term、voltage、velocity 的互谱和 coherence。

#### 周期轨迹

对圆、方形周期误差或 8 字重复运行，可计算：

- 傅里叶级数系数；
- 基频和高次谐波；
- 重建误差；
- 左右方向谐波差；
- 周期间一致性。

#### 频率响应

真正的 FRF/Bode 分析只对已知激励有效：

- sine sweep；
- chirp；
- PRBS；
- 受限 relay test。

需输出：

- 增益；
- 相位；
- coherence；
- 带宽候选；
- 共振候选；
- 传输延迟候选。

普通驾驶日志只能说明频谱内容，不能独立证明系统频率响应。

### 12.5 滤波器 A/B

同一原始日志可比较：

- 无滤波；
- EMA；
- 一阶低通；
- 移动平均；
- Savitzky-Golay；
- 中值去离群；
- 可选 Kalman/alpha-beta。

每组报告：

- 噪声 RMS；
- 峰值抑制；
- 群延迟；
- 相位滞后；
- 对 PID D 项的影响；
- 对峰值/settle 结论的影响；
- 与实时可实现性的差异。

不得只以“曲线更平滑”选择滤波器。

### 12.6 SysId

后续启用时，按左右侧和正反方向分析：

```text
V = kS * sign(v) + kV * v + kA * a
```

输出：

- `kS/kV/kA` 候选及单位；
- 时间延迟候选；
- 训练/验证 RMSE、MAE、最大残差；
- R²；
- 参数置信区间或 bootstrap；
- 左右/正反差异；
- 残差随速度、加速度、电池和温度的图；
- 排除样本和原因；
- 参数适用工况；
- `Draft` 状态。

任何拟合结果都不得自动覆盖比赛参数。

---

## 13. 报告

### 13.1 报告内容

自动报告至少包含：

1. 标题和 run ID；
2. 队号、操作员、观察员；
3. robot ID、硬件/接线 revision；
4. software commit、dirty flag、PROS/VEXos；
5. config/schema/hash；
6. 测试定义和工况；
7. 完整性结论；
8. 质量状态覆盖率；
9. 时序、日志和链路指标；
10. 轨迹、速度、加速度、jerk；
11. PID 与输出；
12. 电机、电池和热；
13. FFT/滤波/SysId 可选章节；
14. 异常和排除试次；
15. 自动诊断建议；
16. 人工备注；
17. `PASS / CONDITIONAL PASS / REPEAT / FAIL / NOT TESTED`；
18. 回滚版本和开放风险；
19. 原始数据、脚本和产物 hash；
20. 审批人和日期。

### 13.2 自动诊断边界

系统可以输出：

- “疑似 Kp 过大”；
- “D 项噪声占比高”；
- “存在约 X Hz 主振荡峰”；
- “左右电机响应不一致”；
- “可能存在命令到速度延迟”；
- “关键窗口缺帧，建议重复”。

系统不能直接输出：

- “把 Kp 改成某值一定更好”；
- “该温度一定安全”；
- “该轨迹已 CompetitionApproved”；
- “位姿 Good”，如果机器人上报 quality Invalid；
- “滤波器解决了机械问题”。

参数建议必须带证据、置信度、适用工况和验证计划。

---

## 14. 实施工作包

### WP0：协议和会话契约冻结

交付：

- `telemetry_schema.yaml`；
- packet ID；
- metadata schema；
- run ID 规则；
- 会话状态机；
- golden packets；
- 兼容性规则。

验收：

- C++/Python 对同一 golden packet 编解码完全一致；
- major 不兼容时明确拒绝；
- minor 追加字段可忽略；
- CRC 和截断检测通过。

### WP1：PC 离线骨架

交付：

- Python 项目；
- CLI；
- 新建会话向导；
- metadata 验证；
- artifact 目录；
- 原始包写入；
- golden log 解码。

验收：

- 无机器人也能创建 Draft 会话；
- 所有身份字段规则可测试；
- run ID 不重复；
- 原始文件不可覆盖；
- metadata round-trip 一致。

### WP2：机器人快日志接入

交付：

- LogFrame builder；
- 控制环单次 `tryPush`；
- 固定容量 SPSC；
- drop/high-water；
- TelemetryTask；
- Memory/Fake sink。

验收：

- 控制环无文件、串口和格式化；
- 消费者暂停只产生可计数 drop；
- 输出和时序不因消费者状态改变；
- PC 全套测试通过；
- PROS 构建通过。

### WP3：SD 完整日志

交付：

- binary SD sink；
- 分段文件；
- 文件头/footer；
- 介质错误；
- 后台 flush；
- 恢复策略。

验收：

- SD 缺失、拔出、满盘、慢写不影响控制；
- 截断文件可检测；
- 序列缺口可解释；
- 文件可在 PC 转换为 Parquet/CSV。

### WP4：Controller→PC 实时链路

交付：

- HELLO/ACK；
- COBS/CRC；
- 20/50 Hz compact telemetry；
- event/health；
- 重连；
- 自动降级；
- PC 接收服务。

验收：

- 5/10/20/50 Hz 吞吐基线；
- 长时间运行；
- 断开/重连；
- 错包、乱序、截断；
- 控制时序无显著变化；
- 最终冻结本机链路档位。

### WP5：完整性与转换

交付：

- packet/log integrity；
- usable windows；
- binary→Parquet；
- schema adapter；
- event decoding；
- analysis manifest。

验收：

- 所有故障类型可区分；
- 不静默修补；
- 不兼容 schema 拒绝；
- 派生结果可追到原始包。

### WP6：实时 Dashboard

交付：

- Connect、Session、Live、Integrity 页面；
- 有界曲线窗口；
- quality/fault；
- 连接健康。

验收：

- UI 卡住时接收和原始写入继续；
- 长运行内存有界；
- Invalid 数据不显示成 Good；
- 图形断点与数据缺口一致。

### WP7：自动分析和报告

交付：

- 八类基础图；
- PID 指标；
- FFT/Welch；
- 滤波 A/B；
- compare runs；
- summary JSON；
- HTML/PDF 报告。

验收：

- golden run 结果确定；
- 时间戳重采样有测试；
- 缺帧时正确降级结论；
- 报告含身份、hash 和生成版本。

### WP8：HIL、现场与发布

交付：

- 链路基线；
- 日志压力矩阵；
- Controller 断联；
- SD 故障；
- 低电/热态；
- 正反/CW/CCW；
- 发布报告模板。

验收：

- 未测试项保持 `NOT TESTED`；
- 只有证据充分的指标冻结为本机门限；
- status monitor 不改变任何 capability；
- 比赛参数仍走项目既有批准流程。

---

## 15. 测试计划

### 15.1 PC 单元测试

- metadata 必填和条件必填；
- run ID；
- 文件名安全；
- protocol round-trip；
- CRC；
- COBS；
- schema major/minor；
- sequence gap；
- time regression；
- duplicate/out-of-order；
- filter 边界；
- 非均匀时间导数；
- FFT 重采样；
- 指标公式；
- 报告字段完整。

### 15.2 属性测试

- 任意字节损坏不能生成合法错误帧；
- 任意包切分/合并后接收器能恢复；
- 任意 NaN/Inf 不使分析崩溃；
- 乱序不会被静默重排成连续真值；
- 原始输入不被派生流程修改；
- 相同输入和版本生成相同 summary。

### 15.3 集成测试

- Fake 100 Hz 日志 → 协议 → PC → Parquet → 报告；
- 60 分钟合成运行；
- PC 磁盘慢写；
- UI 冻结；
- 串口间歇丢包；
- 机器人重启；
- run ID 冲突；
- schema 不兼容；
- 文件尾截断；
- 多次重连。

### 15.4 HIL 测试

- Brain 直连 USB；
- Controller 中转；
- 不同实时帧率；
- 最坏 HMI/日志负载；
- Controller 断联；
- PC 休眠/恢复；
- 拔 USB；
- SD 缺失/慢写/满盘；
- 控制环 `dt/exec/jitter` 对比开关 telemetry 前后。

### 15.5 真机测试

仅在对应硬件和安全门通过后：

- 静止基线；
- 手动低速；
- 正反直线；
- CW/CCW；
- Quick Turn；
- 8 字；
- 热态；
- 低电；
- 故障注入；
- 日志与外部测量对齐。

---

## 16. 验收标准

### 16.1 软件硬门

- 原始数据可追溯；
- schema 不兼容时拒绝；
- sequence、时间回退、CRC、截断可检测；
- PC/串口/SD 故障不改变控制输出；
- 控制环无 I/O、格式化、动态分配和无界等待；
- 无第二电机写入者；
- quality 和 age 全程可见；
- 原始文件不被分析程序修改；
- 所有报告包含 run/robot/software/config/analysis 身份。

### 16.2 候选性能指标

以下只是开始 HIL 的候选，不是当前机器冻结门限：

- 控制环保持名义 10 ms，并报告真实分布；
- 正常负载下 `p99(exec)` 候选目标小于名义周期一半；
- 20 Hz 实时遥测连续运行；
- 50 Hz 作为可选档位；
- PC Dashboard 10–20 FPS；
- 任意接收故障后能从下一合法帧恢复；
- 长运行内存和队列有界；
- 报告可从单条 golden run 自动生成。

最终门限必须由 1690X 的 HIL 和现场基线冻结。

---

## 17. 风险登记

| 风险 | 影响 | 缓解 |
|---|---|---|
| 把 667 MHz 当采样率 | 错误频率设计 | 文档和 UI 区分 CPU/控制/传感器/链路频率 |
| Controller 无线吞吐不足 | 丢包、延迟 | 完整日志本地保存，实时包精简、自动降级 |
| 二进制与 printf 混流 | 解码失步 | 禁止混发，事件也结构化 |
| PC 断线 | 实时图中断 | 机器人继续记录，重连分段 |
| SD 慢写 | 环满 | 后台批量、有界环、drop 计数 |
| 派生导数噪声 | jerk/FFT 误判 | 保存原始值，记录滤波和延迟 |
| 位姿未校准 | 错误轨迹 | quality 门，Invalid 不生成性能通过 |
| 身份信息手工填错 | 数据不可追溯 | 机器人真值只读，PC 模板校验、最终审阅 |
| dirty build | 结果难复现 | 自动记录 dirty，正式批准需补丁或拒绝 |
| 自动建议被直接应用 | 安全/稳定风险 | 第一版只读，候选保持 Draft |
| 图形线程拖慢接收 | 丢数据 | 接收/存储与 UI 分进程或独立线程、队列有界 |
| 缺帧后插值掩盖 | 虚假结论 | 图上断点，关键分析 REPEAT |

---

## 18. Definition of Done

系统第一版完成必须同时满足：

- [ ] PC 可选择新建正式/快速会话；
- [ ] 可填写并复用队号、操作员、观察员和工况资料；
- [ ] 机器人身份由握手自动读取且不能被 PC 覆盖；
- [ ] run ID、metadata、audit 和目录可追溯；
- [ ] 机器人每控制拍只做定长内存入环；
- [ ] TelemetryTask 后台独立；
- [ ] SD 和 Controller 实时 sink 可单独失败；
- [ ] Controller→PC 至少一个实测可用档位；
- [ ] PC 原始包保存、完整性检查和 Parquet 转换完成；
- [ ] 实时状态、轨迹、速度、PID 和健康页面完成；
- [ ] 自动生成基础八类图；
- [ ] 自动生成 summary.json 和 report.html；
- [ ] 缺帧、坏 schema、Invalid pose 会阻止错误 PASS；
- [ ] PC/Fake 测试通过；
- [ ] PROS 构建通过；
- [ ] HIL 压力测试有报告；
- [ ] 未验证 capability 保持关闭；
- [ ] 所有修改按项目要求完成 implementation commit 和 CHANGELOG commit。

---

## 19. 推荐的第一实施切片

为最短时间得到可用结果，第一切片只做：

1. PC 新建会话向导；
2. 队号、操作员、观察员、测试类型和备注；
3. Robot HELLO 只读身份；
4. 20 Hz `LIVE_CORE`；
5. 原始包保存；
6. sequence/CRC/时间戳检查；
7. 电机速度、电流、温度、电池、final voltage 和 timing 实时图；
8. 一键停止并生成基础 HTML 报告。

第一切片暂不做：

- 远程调参；
- 完整 SysId；
- PDF；
- 复杂滤波；
- 频率响应；
- CompetitionApproved。

完成第一切片后，再接入完整 100 Hz SD 日志、PID 分项和高级分析。

---

## 20. 项目状态与能力声明

本策划案只完成设计，不代表软件已实现。

| 项目 | 状态 |
|---|---|
| 策划文档 | Implemented |
| PC 会话程序 | NOT IMPLEMENTED |
| 通信协议 | NOT IMPLEMENTED |
| 机器人日志接线 | NOT IMPLEMENTED |
| Controller→PC 吞吐 | NOT TESTED |
| SD 压力 | NOT TESTED |
| PC 自动报告 | NOT IMPLEMENTED |
| HIL | NOT TESTED |
| FieldValidated | NOT TESTED |
| CompetitionApproved | NOT TESTED |

当前所有机器人 capability 状态不因本文档发生变化。特别是 `pose_good`、自动底盘速度、自动运动和比赛路线仍保持关闭。

---

## 21. 参考资料

### 官方平台资料

- VEX V5 Robot Brain：<https://www.vexrobotics.com/276-4810.html>
- VEX Library — Understanding the V5 Robot Brain：<https://kb.vex.com/hc/en-us/articles/360060662352-Understanding-the-V5-Robot-Brain>
- PROS RTOS C API：<https://pros.cs.purdue.edu/v5/pros-4/group__c-rtos.html>
- PROS Multitasking：<https://pros.cs.purdue.edu/v5/pros-4/multitasking.html>
- PROS Wireless Terminal release：<https://pros.cs.purdue.edu/v5/releases/cli3.2.0.html>
- PROS Wireless Upload：<https://pros.cs.purdue.edu/v5/pros-4/wireless-upload.html>

### 项目强制设计资料

- `C:\Users\alexh\Documents\VEX0713\docs2\12-日志测试验收与诊断.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\14-仿真与数值验证.md`
- `C:\Users\alexh\Documents\VEX0713\docs\10-调参实战手册.md`
- `C:\Users\alexh\Documents\VEX0713\docs\README.md`
- `C:\Users\alexh\Documents\override\74000M\74000M\74000pros.md`
- `C:\Users\alexh\Documents\override\74000M\74000M\.agents\skills\understand-74000m\SKILL.md`

---

> 本文由 OpenAI GPT-5.6（Codex）创作。  
> 文档中的频率、带宽和验收值凡未标为官方硬件事实者，均为工程候选或实施建议；正式门限必须由当前机器人、当前软件、当前配置和当前工况的 HIL/现场证据冻结。
