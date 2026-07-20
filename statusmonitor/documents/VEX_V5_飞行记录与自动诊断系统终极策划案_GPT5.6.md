# VEX V5 飞行记录、赛后自动诊断与调参证据系统终极策划案

> 文档版本：2.0 Ultimate  
> 创作：OpenAI GPT-5.6（Codex）  
> 创作日期：2026-07-20  
> 适用项目：74000M / 当前 1690X 样机 VEX V5 PROS C++17 工程  
> 对应计划板块：板块 0、5；为板块 12、22 提供数据和报告基础  
> 当前状态：Implementation Plan / 待实施  
> 当前验证等级：文档 `Implemented`；软件、microSD、USB 导出、HIL 与现场效果均为 `NOT TESTED`

---

## 1. 最终产品定义

本系统不是实时状态监控器，而是一套：

> **机器人飞行记录器 + PC 测试会话管理 + 赛后自动分析 + 人类 GUI + LLM 证据包**

操作者的目标流程是：

```text
PC 新建测试会话并填写队号、操作员和工况
  → 点击“开始记录窗口”
  → 机器人完成一段运动，完整数据写入 microSD（TF）
  → 点击“结束记录窗口”
  → 导入对应日志
  → 自动完整性检查
  → 自动分段、计算、绘图和诊断
  → 输出给人看的 GUI
  → 输出给 LLM 看的 Markdown 证据包
```

系统的首要价值不是“曲线多”，而是让每个结论都能追溯到：

```text
机器人 + 软件 commit + 配置 hash + 工况 + 原始日志
  → 完整性结论
  → 算法版本和参数
  → 指标、图表和异常证据
  → 调参建议和复验计划
```

### 1.1 P0、P1、P2 范围

| 优先级 | 必须交付 |
|---|---|
| P0 | PC 会话身份、机器人完整日志、microSD 导入、完整性检查、基础 GUI、LLM Markdown、原始数据归档 |
| P1 | 轨迹、速度、加速度、jerk、PID、时序、电机健康、运行对比、自动异常定位 |
| P1 | SysId 数据提取、分侧/分方向拟合、训练/验证报告 |
| P2 | Welch PSD/FFT、STFT、滤波器 A/B、周期轨迹傅里叶级数、日志重放、Brain USB 自动导出 |

### 1.2 第一版明确不做

- 不依赖实时 Dashboard；
- 不通过 Controller 链路传完整日志；
- 不要求 PC 通过 Controller 命令 Brain 开始或停止；
- 不从 PC 远程写 PID、驱动电机或改变 capability；
- 不自动把分析候选写入比赛配置；
- 不把插值后的平滑曲线伪装成完整原始数据；
- 不承诺尚未 HIL 的带宽、microSD 写速、定位精度或调参效果；
- 不把一份日志同时用于拟合和最终验证。

---

## 2. 两版原案的吸收与最终裁决

本终极版保留两版原案各自正确且有价值的部分，并删除已被二次核验否定的前提。

### 2.1 保留 Kimi K3 方案的优势

- 正确识别 Brain 直连 USB 与 Controller 中转并非同一种用户通道；
- 正确指出 PROS CLI 3.5.6 的 Controller 无线终端为 RX-only；
- 正确否定 Controller 上的自定义双向 `RUN_BIND/PING/ACK/LOG_CHUNK` 主协议；
- 强调 microSD 全速二进制日志是完整证据源；
- 强调完整性报告必须先于绘图和参数结论；
- 强调 SysId 左右分侧、正反分方向、训练/验证分离；
- 给出清晰的指标族和风险清单。

### 2.2 保留旧 GPT-5.6 方案的优势

- 完整的会话向导、身份信息和状态机；
- PC 人工身份与机器人真值身份分域；
- 728 B 帧长和 100 Hz 码率预算；
- 完整的数据字典、产物目录和只读归档规则；
- GUI、运行对比、自动分析、报告和 LLM 输出设计；
- 导数、FFT、滤波器和 SysId 的方法边界；
- 工作包、测试矩阵、Definition of Done 和能力声明。

### 2.3 删除或修正的内容

| 原设计 | 终极版裁决 |
|---|---|
| Controller→PC 是透明双向自定义协议 | 删除；官方 PROS CLI 路径按 RX-only 处理 |
| PC 通过 Controller 发送 RUN_BIND、PING、分频或 LOG_CHUNK 请求 | 删除 |
| 20/50 Hz 实时状态是 P0/P1 主目标 | 删除；仅保留可选 1–2 Hz 时间信标和状态提示 |
| 完整日志在结束后从 Controller 返回 | 删除；使用 microSD 或运动结束后的 Brain 直连 USB |
| PC 点击开始必须直接命令 Brain | 改为“定义分析窗口”；Brain 连续/自动分段记录 |
| 日志和 UI 以实时链路为中心 | 改为“原始文件优先，导入后分析” |
| 固定宣称 microSD 最大 16 GB 或 32 GB | 官方资料冲突；首发使用共同安全交集 16 GB FAT32，32 GB 待 HIL |
| Markdown 内逐行复制全部原始样本 | 删除；Markdown 保存完整证据索引，原始样本保存在二进制/Parquet |

最终架构结论：

> **Controller 只做可选的单向时间信标；microSD（TF）做完整记录；PC 做身份绑定、导入、分析、GUI 和 LLM 报告；Brain 直连 USB 只作为赛后可选自动导出通道。**

---

## 3. 已核实的平台与项目事实

### 3.1 V5 Brain 与存储

| 事项 | 已核实结论 |
|---|---|
| VEXos 处理器 | ARM Cortex-A9，667 MHz |
| 用户处理器 | 一颗 Cortex-A9 |
| USB | USB 2.0 High Speed 物理接口 |
| 可移动存储 | **microSD 卡，即国内通常所称 TF 卡** |
| 文件系统 | FAT32 |
| PROS 路径 | microSD 文件使用 `/usd/` 前缀 |
| 容量 | VEX 产品页写 16 GB；现行 VEX API 文档写 32 GB，存在官方冲突 |

容量决策：

1. 首发支持清单写为：**16 GB 高耐久 microSD/TF，FAT32**；
2. 32 GB FAT32 仅作为 HIL 候选，不进入首发保证；
3. 每个具体品牌、容量和格式化方法都要做连续写、掉电和重新插入测试；
4. 为避免不同 API 对目录支持的差异，第一版优先使用 `/usd/` 根目录下的安全短文件名。

官方来源：

- V5 Brain 产品规格：<https://www.vexrobotics.com/276-4810.html>
- VEX SDcard API：<https://api.vex.com/v5/home/cpp/Brain/SDcard.html>
- PROS 文件系统：<https://pros.cs.purdue.edu/v5/tutorials/topical/filesystem.html>

### 3.2 Controller→PC 链路边界

在 PROS CLI 3.5.6 的官方实现中：

- PC 通过扩展命令 `0x27` 轮询 Brain 用户 FIFO；
- 单次请求粒度为 64 B；
- 接收端会缓存并重组完整 COBS 消息，所以可承载结构化上行字节；
- `user_fifo_write()` 标注 `Not currently implemented` 并直接返回；
- 官方合并的功能名称即 “wireless terminal (RX only)”。

因此它可以承担：

- 机器人→PC 单向 `TIME_BEACON`；
- `BOOT/FILE/SD/FAULT` 状态提示；
- 可选的精简事件。

它不能作为首发保证承担：

- PC→机器人开始/结束命令；
- run ID 绑定；
- 请求重传；
- 完整日志回收；
- 远程调参；
- 双向握手。

来源：

- PROS CLI `v5_device.py`：<https://github.com/purduesigbots/pros-cli/blob/3.5.6/pros/serial/devices/vex/v5_device.py>
- PROS CLI `v5_user_device.py`：<https://github.com/purduesigbots/pros-cli/blob/3.5.6/pros/serial/devices/vex/v5_user_device.py>
- Wireless terminal RX-only PR：<https://github.com/purduesigbots/pros-cli/pull/114>

### 3.3 频率必须分开

| 名称 | 当前事实或候选 | 用途 |
|---|---:|---|
| CPU 主频 | 667 MHz | 处理器时钟，不是控制频率 |
| 项目控制环 | 名义 100 Hz | 当前 `nominal_period_s = 0.010` |
| 完整快日志 | 候选 100 Hz | 每控制拍一帧，以真实时间戳为准 |
| Controller 时间信标 | 候选 1–2 Hz | PC/Brain 时间窗口粗对齐 |
| PC GUI 重绘 | 10–20 FPS | 导入后交互，与采样率无关 |

所有分析必须使用机器人单调时间戳。不能因为名义周期是 10 ms 就把每个样本强行放在严格等距的 10 ms 网格上。

### 3.4 当前仓库真实状态

当前工程已经具备：

- `LogFrame` schema 1.0；
- 固定容量 `SpscRing`；
- `TelemetrySink/TelemetryDrain`；
- `LogIntegrityTracker`；
- `RawInputReplay`；
- Fake IO、Fake Clock；
- SysId 激励和拟合基础。

当前 ARM 工具链实际核验：

```text
sizeof(robot::LogFrame) = 728 B
728 B × 100 Hz = 72,800 B/s
每分钟 ≈ 4.368 MB
每小时 ≈ 262.08 MB
```

理论容量估算，不含文件头、块 CRC、FAT 开销和安全余量：

| 卡容量 | 100 Hz 728 B 连续记录理论时长 |
|---:|---:|
| 16 GB | 约 61 小时 |
| 32 GB | 约 122 小时 |

当前缺口：

- `src/main.cpp` 尚未创建产品级日志环和 TelemetryTask；
- ControlLoop 尚未组装并推送完整 `LogFrame`；
- 没有 microSD sink、Memory/Fake 产品 sink 或文件格式；
- 没有 PC 会话、导入、分析、GUI 和报告程序；
- `LogFrame` 尚未记录完整 PID P/I/D/FF 分项和参考轨迹；
- 当前配置没有 IMU，`pose_good=false`，二维位姿不能宣称有效；
- 当前样机配置身份为 `1690X`，PC 默认队号 `74000M` 不能覆盖机器人上报身份；
-所有正式 capability 继续保持关闭，自动路线继续 `DoNothing`。

---

## 4. 最终端到端架构

```text
机器人端
==============================================================================

ControlLoop（名义 10 ms）
  ├─ 同拍采集 raw
  ├─ 生成 validated/state/request/actuator/timing/fault
  ├─ 组装定长 LogFrame
  └─ SpscRing.tryPush()                   ← 控制环唯一日志动作
          │
          ▼
TelemetryTask（低优先级后台）
  ├─ 批量写 microSD/TF：完整 100 Hz 块文件
  ├─ 写文件 header/block/footer/CRC
  └─ 可选 1–2 Hz TIME_BEACON → Controller→PC

PC 端
==============================================================================

Session Manager
  ├─ 队号、操作员、观察员、测试类型、工况
  ├─ 点击开始/结束，保存 PC 时间窗口
  └─ 保存 Controller 时间信标，建立粗略 Brain↔PC 时间映射

Importer
  ├─ 首选：microSD/TF 插入电脑
  └─ 可选：运动结束后 Brain USB 直连导出
          │
          ▼
Integrity Gate
  ├─ schema/frame size/CRC/sequence/time/epoch/run
  ├─ 截断、缺帧、陈旧、sink fault
  └─ PASS / CONDITIONAL / REPEAT / FAIL
          │
          ▼
Decode + Parquet + Event Segmentation
          │
          ├─ Human GUI
          ├─ report_for_llm.md
          ├─ metrics.json / events.ndjson
          ├─ plots/
          └─ readonly raw archive
```

### 4.1 控制路径硬边界

必须始终保持：

- 只有 `OutputService` 写驱动电机；
- ControlLoop 不做文件 I/O、串口 I/O、CSV、JSON、字符串格式化或动态分配；
- `tryPush` 满环时丢新帧并计数，永不等待；
- TelemetryTask 不修改估计器、请求、仲裁、安全状态或电机输出；
- microSD 缺失、满盘、慢写或拔出不改变机器人控制；
- PC、Controller、GUI 和报告程序全部属于观测域；
- 原始日志失败可以使试次 `REPEAT`，不能让运行中的机器人失控。

---

## 5. “点击开始/结束”的可实现语义

### 5.1 为什么不能把按钮直接定义成 Brain 命令

当 PC 只通过 Controller 连接时，官方 PROS 链路没有已实现的自定义反向用户通道。因此：

```text
PC 点击 Start/Stop ≠ 已证明能发送到 Brain 的 Start/Stop 命令
```

终极版不掩盖这个事实，而是把按钮定义成：

> **建立和关闭一个 PC 侧分析窗口，并在导入后绑定机器人日志中的对应时间段。**

### 5.2 推荐的连续分段记录模式

机器人端：

1. 程序启动时生成 `boot_id`；
2. TelemetryTask 检测到有效 microSD 后开启分段文件；
3. 每个文件记录固定最大时长或最大体积；
4. Disabled、Driver、Test、Autonomous 等模式边沿写事件，但不阻塞；
5. 文件持续覆盖 PC 点击窗口前后，保证不会因边界估计误差丢掉瞬态；
6. 采用保留策略，而不是静默覆盖旧文件。

PC 端：

1. 点击“开始记录窗口”时创建 `session_id`，保存 `pc_monotonic_ns`；
2. 保存最近的 `TIME_BEACON{boot_id, robot_time_us, file_id, frame_sequence}`；
3. 点击“结束记录窗口”时保存结束时间；
4. 导入后根据同一 `boot_id/file_id` 建立时间映射；
5. 初始裁剪保留建议候选 `前 2 s + 后 2 s`，最终值由 HIL 冻结；
6. 自动用请求边沿、速度阈值、模式事件和 command lifecycle 精确分段；
7. 报告同时保存“人工窗口”和“自动识别运动窗口”。

这种方式满足 PC 点击体验，同时不需要 PC→Brain 下行。

### 5.3 精确打点的可选方式

若某项测试要求 Brain 时钟上的精确起止：

- 使用 Controller 按键组合生成 `RECORD_MARK_START/STOP` 事件；
- 或由自动命令、SysId runner、测试状态机直接写事件；
- 或运动结束后使用 Brain 直连 USB 的双向模式。

PC 按钮仍负责身份、会话和分析窗口，不直接获得运动控制权限。

### 5.4 用户体验状态

PC 页面只显示必要状态：

```text
Draft → Armed → WindowRecording → WindowClosed
      → WaitingForImport → Importing → Checking
      → Analyzing → Complete / RepeatRequired
```

这不是实时状态监控；录制期间只需要：

- 会话是否已建立；
- 时间信标是否最近可用；
- 用户点击窗口时长；
- 导入提示；
- 明确说明“完整数据仍以 microSD 文件为准”。

---

## 6. 机器人日志文件设计

### 6.1 文件命名

第一版使用 microSD 根目录中的短文件名：

```text
/usd/L<boot8>_<segment4>.V5L
/usd/L<boot8>_<segment4>.MAN
```

示例：

```text
/usd/L8A31D902_0007.V5L
```

PC 导入后再使用可读的长目录名。这样避免 V5/PROS/VEXcode 对子目录和文件名支持差异。

### 6.2 文件结构

```text
FileHeader
  magic
  file_format_major/minor
  log_schema_major/minor
  frame_size_bytes
  endian_marker
  boot_id
  segment_id
  robot_id_hash
  config_hash
  software_commit_hash/truncated identity
  start_robot_time_us

Block[0..N]
  block_magic
  block_sequence
  first_frame_sequence
  frame_count
  payload_bytes
  payload[frame_count × frame_size]
  crc32

FileFooter（正常关闭时存在）
  total_frames
  first/last sequence
  first/last robot_time_us
  producer_drop_count
  sink_failure_count
  final_block_sequence
  footer_crc32
```

规则：

- `LogFrame` 仍是定长、可平凡复制的内部记录；
- 文件格式和 LogFrame schema 分开版本化；
- 首版不在 Brain 上压缩，避免额外 CPU 和不可预测延迟；
- PC 导入后生成 Parquet 并可压缩归档；
- 文件缺 footer 时仍尝试恢复到最后一个 CRC 正确的完整块；
- 恢复不等于试次有效，报告必须标记截断；
- `frame_size_bytes` 必须与 ARM ABI 实际值匹配；
- 建议在实现时增加 `static_assert(sizeof(LogFrame) == 728)`；schema 变化时同步更新。

### 6.3 环形缓冲和批量写

- 单生产者：ControlLoop；
- 单消费者：TelemetryTask；
- 环容量按最大允许写暂停窗口计算；
- TelemetryTask 每次最多取固定批量；
- 使用预分配块缓冲；
- flush 周期和最大单次写固定；
- 不逐帧 flush；
- 写失败后有界退避；
- 介质恢复时开启新 segment，不伪装成连续文件；
- 所有 producer drop、sink discard、CRC、open/write/flush/close 错误写入状态和下一个可用文件。

---

## 7. 最低完整数据字典

必须记录因果链，而不是只记录最终曲线。

### 7.1 身份与时序

- magic、schema、frame size；
- `time_us`、sequence、mode epoch、boot/segment/run hash；
- `raw_dt`、control math dt、exec、jitter；
- sensor/request/actuator age；
- overrun、ring depth、high watermark、producer/sink drop；
- test phase、command lifecycle、事件位。

### 7.2 raw

- Controller 原始轴、按钮、连接和 competition mode；
- 电池电压；
- 六电机逐端口位置、速度、电流、温度、实际电压、flags/faults、API 成功位；
- IMU rotation、gyro rate、calibrating、API 成功位；
- tracking wheel 原始累计位置/速度/API 成功位；
- 采集开始/结束时间或帧内读取跨度。

### 7.3 validated/state

- 每电机有限数、范围、新鲜度、方向、跳变和同侧一致性；
- 左右侧稳健聚合、有效电机 mask 和 spread；
- 位姿、机体速度、左右轮距离和速度；
- translation/heading/pose quality、age 和 estimator mode；
- 滑移指标和传感器残差。

### 7.4 request/control/actuator

- 原始目标、规划目标位置/速度/加速度；
- target/measured/error；
- P/I/D/FF 分项；
- 未限幅和限幅控制器输出；
- 积分状态、积分门、钳位、anti-windup；
- request source、owner、lease、TTL、reject reason；
- unallocated/allocated/final 左右电压；
- saturation、derate 因子、applied limits；
- 最终逐电机映射、write attempted/write ok、last written sequence；
- stop mode、settle/timeout/stall 状态。

### 7.5 fault/event

- active、latched、enter、exit fault；
- severity、safety state、affected motor mask；
- first seen、duration、occurrence、recovery；
- mode/epoch/command start/end；
- marker start/stop；
- microSD insert/remove/open/write/close；
-人工中止、碰撞、碰线、滑移和备注时间点。

当前 schema 1.0 尚缺 PID 分项、参考轨迹和部分事件字段。增加字段前必须冻结 schema 1.1/2.0 策略，并提供旧日志适配器。

---

## 8. PC 会话、身份与归档

### 8.1 会话向导

正式会话分五步：

1. 团队与人员；
2. 测试定义；
3. 工况与设备；
4. 记录窗口策略；
5. 审阅与创建。

最低字段：

| 分组 | 字段 |
|---|---|
| 人工身份 | `team_number`、operator、observer、analyst、notes |
| 测试 | test_case_id、类型、方向、重复序号、目标、主要变化变量、训练/验证标签 |
| 工况 | surface、location、ambient、payload、battery_id、轮胎/磨损/清洁 |
| 机器人真值 | robot_id、hardware/wiring revision、config schema/hash、calibration revision |
| 软件真值 | software commit、dirty、build type、compiler、PROS、VEXos、log schema |
| 记录 | session_id、PC start/end、boot_id、segment/file、时间映射质量 |

### 8.2 身份分域

```text
PC 人工填写：
  team_number / operator / observer / environment / notes

机器人日志真值：
  robot_id / software / config / schema / ports / capability
```

规则：

- PC 可以给档案起别名，但不能覆盖日志中的 robot_id；
- 当前文件夹名是 74000M，而当前样机配置是 1690X，两者必须分别保存；
-身份不一致时允许导入，但完整性页必须报警；
- 正式报告不得用空值或默认值伪造机器人身份；
- dirty build 必须保存 dirty flag，正式批准前保存补丁或拒绝批准。

### 8.3 运行目录

```text
statusmonitor/artifacts/<date>/<team>/<robot_id>/<session_id>/
├── metadata.json
├── audit.jsonl
├── raw/
│   ├── imported_segment_0007.v5l
│   └── original_hashes.json
├── integrity/
│   ├── integrity_report.json
│   └── usable_windows.json
├── derived/
│   ├── samples.parquet
│   ├── events.parquet
│   ├── metrics.json
│   └── analysis_manifest.json
├── plots/
├── gui_summary.json
├── report_for_llm.md
├── operator_notes.md
└── config_snapshot/
```

原始文件只读保存；任何清洗、重采样、滤波、排除和派生均产生新文件并记录算法版本与参数。

---

## 9. 导入与完整性硬门

所有分析的第一步必须是完整性检查。

### 9.1 检查项

1. 文件 magic、格式版本、endian；
2. log schema major/minor；
3. `frame_size_bytes`；
4. block sequence 和 CRC32；
5. frame sequence 连续；
6. 时间戳单调、重复、回退和异常间隔；
7. mode epoch 合法变化；
8. boot/segment/robot/config/run 身份一致；
9. 文件尾和 footer；
10. producer drop、sink discard 与 sequence gap 的关系；
11. 原始设备、request、actuator 是否陈旧；
12. NaN/Inf、非法枚举、越界端口和质量状态。

### 9.2 问题分类

- 生产者丢帧；
- 环消费者丢弃；
- microSD 文件损坏或截断；
- 传感器陈旧；
- request/actuator 陈旧；
- schema/解析错误；
- 会话绑定或身份不一致；
- PC 时间窗口映射不确定。

### 9.3 结论规则

| 结论 | 含义 |
|---|---|
| PASS | 适用硬门通过，关键窗口完整 |
| CONDITIONAL PASS | 非安全限制明确，仍可对指定指标分析 |
| REPEAT | 关键窗口缺帧、窗口绑定不可靠或工况失控 |
| FAIL | 安全硬门或冻结验收线失败 |
| NOT TESTED | 没有足够证据 |

即使是 `REPEAT`，系统仍可生成诊断图，但不得生成参数批准结论。

---

## 10. 自动分段

导入后先把长日志切成可解释阶段：

```text
PreIdle
→ RequestRise / ProfileAcceleration
→ Cruise / Tracking
→ Deceleration
→ Settling
→ PostIdle
→ Success / Timeout / Stalled / Interrupted
```

分段证据按优先级：

1. command/test lifecycle 事件；
2. Brain 端 marker；
3. request source/owner/target 边沿；
4. mode epoch；
5. 速度、输出和误差的有滞回阈值；
6. PC 人工窗口。

自动分段必须：

- 保存算法版本和阈值；
- 允许用户在 GUI 中查看但不能覆盖原始数据；
- 人工修正以 audit 事件保存；
- 所有指标写明使用哪个 window。

---

## 11. 自动分析与调参指标

### 11.1 每次必生成的基础图

1. 实际二维轨迹，若有目标则叠加目标；
2. 目标/实际位置和误差；
3. 目标/实际线速度、左右轮速度、角速度；
4. 加速度和角加速度；
5. jerk 和角 jerk；
6. PID/FF 分项、未限幅/限幅输出；
7. 六电机电流、温度、速度和同侧 spread；
8. 电池、最终电压、饱和和 derate；
9. raw dt、exec、jitter、overrun、ring/drop；
10. fault/event 时间线。

位姿质量无效时：

- 轨迹页显示 `POSE INVALID / NOT AVAILABLE`；
- 仍显示原始电机、请求、输出、时序和故障；
- 不输出路径跟踪性能 PASS。

### 11.2 PID 指标

- rise time；
- peak time；
- overshoot；
- settling time；
- steady-state error；
- settle 后标准差；
- zero crossing/目标穿越次数；
- IAE、ISE、ITAE；
- 输出 RMS 和 total variation；
- 饱和占比和最长连续饱和；
- integral clamp 占比和积分峰值；
- P/I/D/FF 绝对贡献；
- 请求→输出和请求→速度延迟候选；
- 主振荡频率、阻尼候选和振荡衰减率；
- 正/反、CW/CCW、左右侧差异。

诊断不能只根据一条曲线直接改 PID。顺序必须是：

```text
日志完整性
→ 机械/端口/单位
→ 传感器质量和延迟
→ 请求/仲裁/安全
→ 饱和和电池
→ 最后才是 PID/FF 参数
```

### 11.3 速度、加速度和 jerk

禁止对噪声位置连续裸差分两次或三次后直接下结论。

推荐：

1. 使用真实时间戳；
2. 对累计位置做局部线性/多项式窗口估计速度；
3. 保留 API 速度、raw-derived 和 filtered-derived 三种来源；
4. 用 Savitzky-Golay、局部回归或明确截止频率的低通估计加速度；
5. 从处理后的加速度估计 jerk；
6. 报告窗口、阶数、截止频率、边界策略和群延迟；
7. 离线零相位结果标记为 `offline-only`，不得当作实时可实现滤波器。

jerk 输出：

- 峰值；
- RMS；
- p95/p99；
- 累计绝对 jerk；
- 按加速、巡航、减速、settle 分段；
- 噪声和缺帧敏感性警告。

### 11.4 频率分析

普通振荡优先使用：

- Welch PSD；
- FFT；
- STFT；
- error、D term、voltage、velocity 的互谱与 coherence；
- 主峰频率、带内功率和谐波比。

前置条件：

- 关键窗口完整；
- 明确重采样方法；
- 明确窗函数、窗口长度、重叠和频率分辨率；
- 100 Hz 采样的奈奎斯特频率是 50 Hz；
- 普通驾驶日志只能说明频谱内容，不能单独证明闭环 FRF/Bode。

傅里叶级数仅用于：

- 圆、8 字等周期轨迹；
- 重复误差的谐波分解；
- 闭合轨迹的周期一致性。

它不作为任意点到点轨迹的默认分析方法。

### 11.5 滤波器 A/B

同一原始日志可比较：

- 无滤波；
- EMA/一阶低通；
- 移动平均；
- Savitzky-Golay；
- 中值去离群；
- alpha-beta/Kalman 候选。

每组输出：

- 噪声 RMS；
- 峰值抑制；
- 群延迟/相位滞后；
- 对 D 项、过冲、settle 和主频结论的影响；
- 是否实时因果可实现；
- 参数适用采样率。

“曲线更平滑”不是选择滤波器的充分理由。

### 11.6 SysId

模型：

```text
V = kS × sign(v) + kV × v + kA × a
```

要求：

- 左右分侧；
- 正反分方向；
- quasistatic 和 dynamic；
- 训练与验证分离；
- 排除饱和、降额、滑移、陈旧和故障样本并记录原因；
- 报告 RMSE、MAE、最大残差、R²、残差结构、bootstrap 区间和重复性；
- 拟合结果只生成 `Draft` 参数 diff；
- 不自动写入机器人。

### 11.7 机械、能源、热和实时性

至少计算：

- 同侧电机速度/电流/温度 spread；
- 单电机异常持续时间；
- 电池负载压降；
- 电压饱和占比；
- 各 derate 因子占比；
- 峰值/均值/RMS 电流；
- 温升率、最高温度和恢复；
- raw dt/exec/jitter 的均值、std、p95、p99、最坏值；
- overrun rate、最长连续 overrun；
- ring high watermark、producer/sink drop；
- 日志开/关对控制时序的 A/B 影响。

---

## 12. 给人看的 GUI

### 12.1 页面

1. **Home / Session**
   - 新建、继续、打开历史；
   - 队号、操作员、机器人、测试类型和工况模板。

2. **Record Window**
   - Start/Stop；
   - 当前 PC 窗口时长；
   - 最近时间信标及其质量；
   - 说明完整数据保存在 microSD；
   - 导入操作提示。

3. **Import**
   - 自动扫描可移动盘；
   - 显示 boot/segment/时间范围/robot/config；
   - 自动推荐与会话重叠的文件；
   - 复制、hash、只读归档和断点恢复。

4. **Integrity**
   - PASS/CONDITIONAL/REPEAT/FAIL；
   - 缺帧、CRC、截断、身份、时间映射和可用窗口；
   - 不完整位置在时间轴上显示断点。

5. **Overview**
   - 数据可信度；
   - 控制响应；
   - 定位；
   - 电机/电池/热；
   - 实时性；
   - 主要异常和建议复验。

6. **Plots**
   - 轨迹、运动学、PID、电机、时序、故障、频谱；
   - 光标联动；
   - 点击异常跳转到同一时间点所有曲线。

7. **Compare Runs**
   - 按时间、运动进度或路径进度对齐；
   - 参数、软件、配置和工况差异；
   - 叠加图、统计和效应量。

8. **Report**
   - GUI 摘要；
   - LLM Markdown 预览；
   - 导出目录；
   - 人工结论、限制和批准状态。

### 12.2 Overview 卡片示例

```text
Data integrity       PASS
Pose quality         NOT AVAILABLE
PID overshoot        4.2%  [above current baseline]
Settling time        0.68 s
Output saturation    31.4%
Main oscillation     4.8 Hz
Left/right spread    8.1%  [inspect right motor 2]
Battery sag          1.17 V
Control exec p99     2.4 ms
Recommendation       Check sensor delay before increasing Kd
Evidence             12.40–13.85 s
```

### 12.3 GUI 硬规则

- 接收、导入和分析不运行在 UI 主线程；
- 原始文件大于内存时按列/时间窗口读取；
- 图形使用降采样视图，但指标使用完整有效数据；
- quality Invalid 不能显示为绿色；
- 缺口画断点，不用插值伪装；
- 每张图显示 run、robot、schema、单位、数据来源和滤波版本；
- 自动建议必须显示证据、置信度和复验动作。

---

## 13. 给 LLM 的详细证据包

### 13.1 为什么不能把每一帧都塞进 Markdown

60 秒完整日志约：

```text
728 B × 100 Hz × 60 s = 4.368 MB 二进制
```

展开为带列名的 Markdown/CSV 后会膨胀数倍，并超过多数 LLM 的有效上下文。逐行复制会降低分析质量，而不是提高完整性。

因此“详细记录全部信息”定义为：

> Markdown 记录全部身份、字段语义、完整性、指标、事件、异常证据窗口和原始文件索引；逐帧真值保存在带 hash 的二进制/Parquet 中。

### 13.2 `report_for_llm.md` 结构

```markdown
---
report_schema:
session_id:
robot_id:
software_commit:
config_hash:
log_schema:
analysis_version:
integrity_verdict:
---

# Executive conclusion
# Scope and test question
# Identity and environment
# Data integrity and usable windows
# Signal dictionary and units
# Timeline and automatic segmentation
# Control/PID metrics
# Trajectory and motion metrics
# Motor, battery, thermal and timing metrics
# Frequency and filter analysis
# Anomalies
## A-001
- evidence window
- observed signals
- earliest abnormal layer
- competing hypotheses
- confidence
- recommended next test
# Parameter candidates
# Restrictions and NOT TESTED items
# Artifact manifest and SHA-256
# Suggested machine queries
```

### 13.3 LLM 侧文件

```text
llm/
├── report_for_llm.md
├── data_dictionary.md
├── timeline.md
├── metrics.json
├── events.ndjson
├── evidence/
│   ├── A-001_12.40-13.85s.csv
│   └── A-002_28.10-30.00s.csv
└── artifact_manifest.json
```

关键原则：

- 每个异常提供小而完整的证据窗口；
- Markdown 给出 Parquet 列名、时间范围和查询方法；
- 不让 LLM 猜单位、符号、滤波或质量；
- 自动建议写为 hypothesis，不写成保证；
- 所有图和派生结果带生成参数；
- 原始日志和报告均写 SHA-256。

---

## 14. PC 技术栈与目录

### 14.1 推荐技术栈

首版使用单一 Python 技术栈，降低实现和部署成本：

| 领域 | 选型 |
|---|---|
| Python | 3.11 或项目冻结版本 |
| GUI | PySide6 |
| 交互曲线 | pyqtgraph |
| 列式处理 | Polars |
| 数值 | NumPy、SciPy |
| Parquet | PyArrow |
| 静态图 | Matplotlib |
| 数据模型 | Pydantic |
| 报告 | Jinja2 + Markdown |
| 本地索引 | SQLite |
| 测试 | pytest、Hypothesis |
| 打包 | PyInstaller |

不建议首版采用 React/Tauri + Python 双进程，因为会增加协议、打包和调试面。GUI 和分析稳定后再评估。

### 14.2 推荐目录

```text
statusmonitor/
├── documents/
├── protocol/
│   ├── log_file_schema.yaml
│   ├── telemetry_beacon_schema.yaml
│   └── golden/
├── pc/
│   ├── pyproject.toml
│   ├── src/statusmonitor/
│   │   ├── app.py
│   │   ├── sessions/
│   │   ├── importers/
│   │   ├── integrity/
│   │   ├── storage/
│   │   ├── segmentation/
│   │   ├── analysis/
│   │   ├── plotting/
│   │   ├── reports/
│   │   └── llm_pack/
│   └── tests/
└── artifacts/                 # gitignore
```

CLI：

```text
statusmonitor new-session
statusmonitor start-window <session>
statusmonitor stop-window <session>
statusmonitor scan-media
statusmonitor import <session>
statusmonitor check <run>
statusmonitor analyze <run>
statusmonitor compare <run...>
statusmonitor report <run>
statusmonitor doctor
```

---

## 15. 实施工作包

### WP0：契约和事实冻结

交付：

- 文件格式、LogFrame schema 策略、事件 ID；
- metadata、integrity、metrics 和 LLM report schema；
- session/boot/segment/run 标识关系；
- 16 GB microSD 首发支持策略；
- golden files 和故障样本。

验收：

- ARM `LogFrame=728 B` 被测试锁定；
- major/minor 兼容规则明确；
- 截断、CRC、错 endian、错 schema golden 文件齐全；
- PC 无机器人即可解析 golden file。

### WP1：PC 离线导入、完整性和会话骨架

交付：

- Python 项目和 CLI；
- 会话向导、身份模板和 SQLite 索引；
- microSD 扫描与文件复制；
- SHA-256、只读归档；
- 解析器和完整性报告；
- 二进制→Parquet。

验收：

- 不接机器人也能完成 golden log 全流程；
- run/session 不覆盖；
- 身份错配报警；
- 截断文件恢复到最后合法块；
- 关键窗口缺帧输出 REPEAT。

### WP2：机器人 LogFrame builder 和内存链

交付：

- 控制拍 `makeLogFrame`；
- 产品级 SPSC；
- Memory/Fake sink；
- TelemetryTask；
- drop/high-water/sink counters；
- 需要的 schema 字段扩展。

验收：

- 控制环只做定长复制和 tryPush；
- 消费者暂停不影响输出；
- 环满语义和计数正确；
- PC 测试通过；
- PROS ARM 构建通过。

### WP3：microSD 完整记录

交付：

- `/usd/` 检测；
- 文件 header/block/footer/CRC；
- 批量写、flush、轮转和退避；
- 插拔、满盘、慢写、截断处理；
- boot/segment manifest；
- 1–2 Hz 可选时间信标。

验收：

- 16 GB 目标卡连续记录至少 60 分钟；
- microSD 拔出、写保护、满盘和慢写不改变控制；
- PC 导入与 ARM 写出逐字段一致；
- 100 Hz 文件无静默缺帧；
- 关闭失败和掉电截断可检测。

### WP4：PC Start/Stop 窗口和自动绑定

交付：

- Start/Stop UI；
- PC monotonic timestamp；
- TIME_BEACON 接收和时间映射；
- boot/segment 文件推荐；
- 前后 guard window；
- 自动运动分段；
- Controller/command marker 支持。

验收：

- 无下行也能完成 session→文件绑定；
- 时间信标丢失时明确降级为人工选择；
- 自动分段误差有 golden 测试；
- 人工修改保留 audit。

### WP5：基础分析和报告

交付：

- 基础十类图；
- PID、运动、健康和时序指标；
- GUI summary；
- `report_for_llm.md`；
- metrics/events/manifest；
- evidence window 提取。

验收：

- 相同输入和版本生成确定性指标；
- 缺帧、Invalid pose 和非法值正确降级；
- 图表单位、来源、滤波和窗口齐全；
- LLM 报告可追到所有原始 artifact。

### WP6：高级分析

交付：

- Welch/FFT/STFT；
- 滤波器 A/B；
- jerk 稳健估计；
-周期轨迹傅里叶级数；
- SysId；
- compare runs；
- 日志重放。

验收：

- 非均匀时间和重采样测试；
- 频率分辨率/奈奎斯特边界测试；
- C++/Python SysId 固定向量对拍；
- 训练/验证分离；
- 结果只生成 Draft 建议。

### WP7：可选 Brain USB 自动导出

交付：

- Brain 直连 USB user port 的只读文件导出协议；
- PC 请求、块号、CRC、重传和断点续传；
- 运动结束后自动识别；
- 不影响 Controller-only 主流程。

验收：

- 只在 Brain 直连 USB 时启用；
- 断线重传和 hash 一致；
- Controller 中转时不误判为支持下行；
- 导出任务不影响控制时序。

### WP8：HIL、现场和发布

交付：

- microSD 卡型白名单；
- 时序 A/B；
- 故障注入；
- 正反/CW/CCW/冷热/高低电矩阵；
- 测试 SOP；
- 发布和回滚报告。

验收：

- 未测试保持 NOT TESTED；
- capability 不因记录系统自动解锁；
- 只有本机重复数据冻结阈值；
- 比赛配置仍走原项目批准流程。

---

## 16. 推荐里程碑与人力估算

以下为一名熟悉 Python/C++ 的开发者的工程估算，不是承诺：

| 里程碑 | 范围 | 估算 |
|---|---|---:|
| M0 | WP0 契约、golden 和 PC 解析原型 | 3–5 人日 |
| M1 | WP1 PC 会话、导入、完整性、Parquet | 5–8 人日 |
| M2 | WP2–WP3 机器人日志和 microSD | 6–10 人日 |
| M3 | WP4 窗口绑定、自动分段 | 4–7 人日 |
| M4 | WP5 GUI 和双报告 | 8–12 人日 |
| M5 | WP6 高级分析和运行对比 | 8–15 人日 |
| M6 | WP7 可选 USB 导出 | 4–8 人日 |
| M7 | WP8 HIL、压力和现场冻结 | 5–10 人日 |

最短可用 MVP：

```text
M0 + M1 + M2 + M4 的基础子集
≈ 20–30 人日
```

MVP 不需要：

- Controller 双向协议；
- 实时 Dashboard；
- USB 自动导出；
- FFT/SysId 全套；
- PDF。

---

## 17. 测试矩阵

### 17.1 PC 单元/属性测试

- 文件 header/block/footer round-trip；
- CRC、截断、字节损坏、错 endian；
- schema major/minor；
- sequence gap、duplicate、out-of-order、time regression；
- NaN/Inf 和非法枚举；
- metadata 条件必填和身份冲突；
- run/session/文件名不重复；
- 非均匀时间导数；
- filter 边界和群延迟；
- FFT/Welch 重采样；
- 自动分段；
- 报告字段和 artifact hash；
- 任意原始文件不被分析过程修改。

### 17.2 集成测试

- Fake 100 Hz → LogFrame → 文件 → import → Parquet → GUI/LLM；
- 60 分钟合成运行；
- 文件尾任意位置截断；
- microSD 文件重复和跨 boot；
- PC 低磁盘空间；
- UI 冻结；
- 分析进程失败后恢复；
- 旧 schema 日志；
- 多 run compare；
- 无时间信标时人工绑定。

### 17.3 HIL

- 16 GB microSD/TF FAT32；
- 不同品牌和写速等级；
- 插入、拔出、写保护、满盘、慢写；
- TelemetryTask 暂停；
- 环满；
- 1–2 Hz Controller 时间信标；
- Controller 断开和重连；
- Brain USB 直连导出候选；
- telemetry 开/关时 `raw_dt/exec/jitter` A/B；
- 掉电后的截断恢复。

### 17.4 真机

仅在对应硬件门通过后：

- 静止基线；
- 低速手动；
- 正反直线；
- CW/CCW；
- Quick Turn、换向、8 字；
- PID step/profile；
- SysId；
- 热态、低电和负载；
- 故障注入；
- 外部人工测量对齐；
- 多次重复和独立验证。

---

## 18. Definition of Done

第一版只有同时满足以下项目才算完成：

- [ ] PC 可新建正式和快速会话；
- [ ] 队号、操作员、观察员、测试和工况可模板化；
- [ ] PC 人工身份与机器人日志身份严格分域；
- [ ] Start/Stop 窗口在无 PC→Brain 下行时仍可用；
- [ ] 机器人每控制拍只做定长 LogFrame 入环；
- [ ] TelemetryTask、microSD 和控制路径隔离；
- [ ] 16 GB microSD/TF FAT32 通过目标卡型 HIL；
- [ ] 728 B/100 Hz 文件连续记录通过；
- [ ] microSD 缺失、拔出、慢写、满盘不改变控制；
- [ ] 文件格式可检测 CRC、截断、缺帧、错 schema 和身份冲突；
- [ ] PC 可导入、hash、只读归档、转 Parquet；
- [ ] 完整性硬门先于全部分析；
- [ ] GUI 可输出清晰状态、曲线、异常证据和建议；
- [ ] LLM Markdown 包含全部身份、字段、指标、事件和 artifact 索引；
- [ ] 原始逐帧数据保存在二进制/Parquet，不被修改；
- [ ] Invalid pose 不生成轨迹性能 PASS；
- [ ] PID、jerk、FFT、滤波和 SysId 结论标明方法和限制；
- [ ] 未验证 capability 继续关闭；
- [ ] PC 测试和 PROS ARM 构建通过；
- [ ] HIL 报告和回滚流程齐全；
- [ ] 所有实现提交和 CHANGELOG 符合 74000M 项目规则。

---

## 19. 风险登记

| 风险 | 影响 | 缓解 |
|---|---|---|
| PC Start/Stop 被误解为 Brain 命令 | 会话边界错误 | UI 明确“分析窗口”，使用连续日志和 guard window |
| Controller 无反向用户通道 | 无法远程开始/停止 | 不依赖下行；精确打点用 Controller/command 事件 |
| Controller 上行不稳定 | 时间映射变差 | 只作为可选信标；允许无信标人工绑定 |
| 官方 microSD 容量文档冲突 | 兼容性不确定 | 首发限定 16 GB FAT32；32 GB 单独 HIL |
| microSD 慢写/拔出 | 丢帧或截断 | SPSC、批写、CRC、退避、分段和压力测试 |
| 728 B schema 扩展 | 帧长和容量变化 | static_assert、版本化、golden、重算预算 |
| 位姿未校准 | 错误轨迹结论 | quality 门；当前显示 NOT AVAILABLE |
| jerk 放大噪声 | 误判冲击 | 稳健导数、方法记录、敏感性分析 |
| FFT 被误当 FRF | 错误控制结论 | 普通日志只报告 PSD；FRF 需已知激励 |
| LLM 上下文过大 | 分析质量下降 | Markdown 证据索引 + 小证据窗口 + Parquet 真源 |
| 自动建议被直接应用 | 稳定/安全风险 | 只生成 Draft、证据和复验计划 |
| GUI 卡死 | 用户误以为丢数据 | 导入/分析/GUI 解耦，原始文件优先 |
| dirty build | 不可复现 | metadata 强制记录并保存补丁或拒绝批准 |
| 旧原始文件被覆盖 | 证据丢失 | run/session 唯一、只读归档、hash |

---

## 20. 当前能力声明

本策划案只完成终极架构和实施计划，不代表功能已经实现。

| 项目 | 当前状态 |
|---|---|
| 终极策划案 | Implemented |
| 当前 LogFrame/SPSC/Integrity 骨架 | Implemented / PC-tested status 以仓库测试为准 |
| 产品级 LogFrame 接线 | NOT IMPLEMENTED |
| microSD sink | NOT IMPLEMENTED |
| PC 会话和导入 | NOT IMPLEMENTED |
| GUI | NOT IMPLEMENTED |
| LLM 报告 | NOT IMPLEMENTED |
| 16 GB microSD HIL | NOT TESTED |
| 32 GB microSD | NOT SUPPORTED UNTIL TESTED |
| Controller 时间信标 | NOT IMPLEMENTED / NOT TESTED |
| Brain USB 文件导出 | OPTIONAL / NOT IMPLEMENTED |
| pose_good | false |
| autonomous capabilities | false |
| CompetitionApproved | NOT TESTED |

本文不会改变任何机器人 capability，也不会解锁自动运动或比赛路线。

---

## 21. 参考资料

### 项目权威资料

- `C:\Users\alexh\Documents\VEX0713\docs2\00-目标范围与设计准则.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\02-硬件配置单位与HAL.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\12-日志测试验收与诊断.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\13-参考工程骨架与落地顺序.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\14-仿真与数值验证.md`
- `C:\Users\alexh\Documents\VEX0713\docs2\15-来源与事实核验.md`
- `C:\Users\alexh\Documents\VEX0713\docs\10-调参实战手册.md`
- `C:\Users\alexh\Documents\override\74000M\74000M\docs\HARDWARE_COMMISSIONING.md`

### 官方资料

- VEX V5 Robot Brain：<https://www.vexrobotics.com/276-4810.html>
- VEX V5 SDcard：<https://api.vex.com/v5/home/cpp/Brain/SDcard.html>
- PROS 文件系统：<https://pros.cs.purdue.edu/v5/tutorials/topical/filesystem.html>
- PROS Extended API：<https://pros.cs.purdue.edu/v5/extended/apix.html>
- PROS Wireless Terminal：<https://pros.cs.purdue.edu/v5/releases/cli3.2.0.html>
- PROS CLI RX-only 实现：<https://github.com/purduesigbots/pros-cli/pull/114>

### 受评审原案

- `statusmonitor/documents/telemetry-status-monitor-plan.md`，作者 Kimi K3；
- `statusmonitor/documents/VEX_V5_PC状态监控与调参系统完整策划案_GPT5.6.md`，作者 OpenAI GPT-5.6（Codex）。

---

> 本文由 OpenAI GPT-5.6（Codex）创作。  
> 本文吸收两版原案的优势，并以 VEX/PROS 官方资料、当前 74000M 仓库实现和项目 `docs2` 证据规则为最终裁决依据。任何尚未完成本机 HIL 的容量、带宽、频率、阈值和性能均明确保持 `NOT TESTED`，不得升级为比赛保证。
