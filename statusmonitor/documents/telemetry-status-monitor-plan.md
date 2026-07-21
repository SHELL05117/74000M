# 74000M 遥测状态监控与记录分析系统策划案

> **作者：Kimi K3**
> **日期：2026-07-20**
> **项目：74000M / VEX0713（VEX V5 PROS/C++17）**
> **版本：v1.0**
> **状态：草案（Draft）— 待团队评审与 M0 实测确认**

---

## 1. 文档信息

| 项目   | 内容                                                     |
| ---- | ------------------------------------------------------ |
| 文档名称 | 74000M 遥测状态监控与记录分析系统策划案                                |
| 作者   | Kimi K3                                                |
| 日期   | 2026-07-20                                             |
| 项目   | 74000M / VEX0713                                       |
| 版本   | v1.0                                                   |
| 状态   | 草案，M0 里程碑开始前冻结为 v1.1                                   |
| 适用范围 | 机器人端遥测固件扩展 + PC 端记录/分析工具链 + 比赛归档流程                     |
| 阅读顺序 | 第 2–4 章为共识基础；第 5–7 章为实施方案；第 8–9 章为执行清单；第 10–11 章为边界与出处 |

**术语约定**

- 「已核实事实」：标注来源（URL 或文档路径），可直接作为设计依据。
- 「设计决策/建议」：标注【设计决策】，是本策划案提出的方案选择，可评审修改。
- 「待实测/待确认」：调研材料中无权威数字或代码中尚未锁定的事项，必须在对应里程碑出口条件中落实。

---

## 2. 背景与目标

### 2.1 用户需求

团队需要一套「机器人实时遥测 → PC 记录分析」系统，支撑调参与比赛数据分析：

1. **调试场景**：Brain 通过 USB 有线直连 PC（手柄保持无线驾驶，不受线缆影响），机器人实时回传位姿与运行参数到 PC。
2. **记录场景**：PC 端有记录系统，每次读完数据自动生成：
   - 机器人运动轨迹二维图；
   - 速度-时间图、加速度-时间图、加加速度（jerk）-时间图；
   - PID 震荡曲线（目标 vs 实际、输出、积分等）。
3. **可选扩展**：傅里叶级数轨迹拟合、滤波器对比实验等。
4. **身份档案（特别要求）**：PC 端程序必须提供「身份信息记录」功能——可新建一条会话/身份档案，填写队号（如 74000M）、操作员、机器人 ID、日期、电池编号、场地/工况、备注等信息，并与每次采集的数据绑定归档。

### 2.2 可行性结论

**结论：可行，且与项目既有架构高度兼容。**

- 硬件侧：V5 Brain 直连 PC 的 USB 提供独立 user port（用户 stdio 虚拟串口），与程序上传用的 system port 互不干扰；社区已有 30 Hz JSON 位姿串流的世锦赛先例（已核实，见 §3.3）。
- 项目侧：代码库已有定长 `LogFrame`、无锁 SPSC 环、完整性跟踪器、SysId 拟合工具等资产（已核实，见 §4.1），主要缺口是「sink 实现 + PROS 任务接线 + PC 端工具链」，全部为软件工作，不涉及硬件改动。
- 约束侧：项目文档体系（docs2/12）已强制规定了日志架构、完整性报告与会话可追溯要求，本系统的 PC 端功能（完整性报告、metadata.json、归档目录）正是这些文档要求的落地实现，方向完全一致。

### 2.3 目标分层

| 层级     | 目标                                                    | 对应章节            |
| ------ | ----------------------------------------------------- | --------------- |
| 核心（P0） | USB 实时遥测 + SD 全速落盘 + 完整性报告 + 会话身份绑定归档                 | §5、§6、§7.1、§7.2 |
| 主要（P1） | 实时仪表盘（轨迹/v/a/jerk/PID 曲线）、离线分析、SysId 拟合闭环             | §7.2、§7.3       |
| 扩展（P2） | FFT 频谱分析、傅里叶级数轨迹拟合、滤波器对比实验、AdvantageScope CSV 导出、日志重放 | §7.4            |

---

## 3. 已核实的平台事实

### 3.1 V5 Brain 硬件规格（已核实）

| 事实          | 数值                                                       | 来源                                                                                                                            |
| ----------- | -------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| 内部 SoC      | Xilinx Zynq XC7Z010（FPGA + 双核 Cortex-A9）                 | https://snikolaj.com/2023/02/25/vex-robotics-v5-brain-analysis/                                                               |
| VEXos 处理器   | 1× Cortex-A9 @ 667 MHz + 2× Cortex-M0 @ 32 MHz + 1× FPGA | https://www.vexrobotics.com/276-4810.html ；https://kb.vex.com/hc/en-us/articles/360060662352-Understanding-the-V5-Robot-Brain |
| 用户处理器       | 1× Cortex-A9（标称 1333 MIPS，官方按单核计）；VEXos 与用户程序各占一个 A9 核   | 同 VEX KB                                                                                                                      |
| RAM / Flash | 128 MB / 32 MB，8 个用户程序槽，SD 卡扩展（FAT32）                    | 同 VEX KB                                                                                                                      |
| USB         | USB 2.0 High Speed（480 Mbit/s）                           | 同 VEX KB                                                                                                                      |

### 3.2 USB 串口通道（已核实）

Brain 直连 PC 的 USB 枚举**两个虚拟串口**（来源：https://vexide.dev/blog/posts/serial-deep-dive/）：

- **system port**：V5 二进制串行协议，用于程序上传/文件传输；
- **user port**：用户程序 stdio，即 `pros terminal` 看到的输出。**本系统实时遥测使用此口。**

关键推论：

1. USB CDC 虚拟串口的「波特率」设置基本无实际约束，吞吐瓶颈不在波特率而在 PROS 内核缓冲。**PROS stdout 实测最大吞吐无权威公开数字 → 列为 M0 第一项实测任务**（递增帧率 + PC 端序号缺口统计）。
2. **缓冲溢出是已知问题**：不限频会出现 `Serial buffer full!`（来源：https://github.com/ros-drivers/rosserial/issues/591）。因此实时流必须节流分频（见 §6.4）。
3. 使用 user port 自定义二进制协议前，必须先执行 `pros::c::serctl(SERCTL_DISABLE_COBS, NULL)` 关闭 PROS 默认的 COBS 复用（来源：PROS APIX 文档 https://pros.cs.purdue.edu/v5/extended/apix.html ；社区先例见 §3.3）。

### 3.3 社区先例（已核实）

- **3151A（2025 世锦赛队伍）**：用 `printf` + `fflush` 以约 **30 Hz** 经 user port 推 JSON 位姿包，需先关闭 COBS。来源：https://www.aadishv.dev/robotics-5 。→ 证明 user port 实时遥测在比赛级使用中可行；JSON 文本格式都能跑 30 Hz，二进制格式余量更大。
- **V5SerialPlotter**：PC 串口实时绘图器，printf 行格式，可作 PC 端实时绘图参考实现。来源：https://github.com/adityanarayanan03/V5SerialPlotter 。
- **AdvantageScope（FRC 工具）**：支持导入 CSV 做 2D/3D 位姿场地图和轨迹回放。来源：https://docs.advantagescope.org/overview/log-files/ 。【设计决策】采用「自研 PC 工具 + 导出 AdvantageScope 兼容 CSV」双轨。
- **WPILib SysId** 为 FRC 专属不可直接用，但 SysId 算法本质是最小二乘拟合，项目代码 `calibration_tools.hpp` 已有等价实现（见 §4.1），无需移植。

### 3.4 手柄无线通道不能作主遥测通道（已核实结论）

PC→USB→手柄→VEXnet→Brain 的连接确实存在（无线下载用），但该通道：

- 只有一个等价 system port 的串口，**无 user port**；
- 取用户输出需主机轮询 FIFO，带宽与延迟远不如直连 USB。

**结论（已核实，来源同 vexide deep dive）：手柄 VEXnet 通道不适合做实时遥测主通道，仅保留为低速应急备用（如线缆不可用时读取关键健康摘要）。** 主遥测 = Brain 直连 USB user port；手柄保持无线驾驶不受影响。

### 3.5 项目文档强制要求（已核实，docs2 为权威）

以下要求直接构成本系统的设计约束，来源为 `C:\Users\alexh\Documents\VEX0713`（记为 DOC_ROOT）：

1. **控制优先于记录**（docs2/12 §1.1, §2.1–2.4）：日志/SD/串口失败不得阻塞或改变控制计算；控制环内严禁 printf、文件 I/O、字符串格式化、动态分配、等待消费者；控制环每拍只做一次 tryPush。
2. **快/慢日志分级**：快日志 = 每控制拍一帧（名义 10 ms / 100 Hz，以真实时间戳为准）；慢遥测 = 按 sequence 分频 20–100 ms；练习时允许 20 Hz 实时发到串口绘图仪（docs/10 §3.3）。
3. **SPSC 环语义**：生产者永不等待，满环丢新帧并计数 `log_dropped_total`；必须做「暂停消费者」压力测试。
4. **LogHeader 必备字段**：magic、schema_major/minor、frame_size_bytes、time_us（单调）、sequence（严格递增）、mode_epoch、run_id_hash；schema 不兼容时离线工具必须明确拒绝。
5. **完整因果链七层**：raw→validated→state→request→actuator→timing→fault，逐电机展开，禁止只存均值。
6. **落盘方式**：数值转文本与文件 I/O 只能后台执行；建议紧凑二进制落盘 + 离线转 CSV；SD 缺失/满盘只计数退避不卡顿。
7. **PC 端完整性报告（P0 第一步）**：magic/schema/帧长校验、sequence 连续、时间戳单调、mode_epoch 合法边沿、丢帧计数一致、文件末帧完整、CSV 列/有限数/枚举合法、run metadata 一致；区分五类问题：生产者丢帧 / 消费者或文件丢失 / 传感器陈旧 / 请求执行器陈旧 / 解析错误；**关键窗口缺帧的试次默认无效（REPEAT）**。
8. **会话可追溯（metadata 必备，docs2/12 §4.1）**：run_id、test_case_id、robot_id、软硬 revision、software_commit + dirty flag、VEXos/PROS 版本、schema、config_hash、轮/齿比/质量/重心、场地/温度/轮胎、电池 ID 与起止状态、电机起止温度、**操作员/日期**。推荐目录 `artifacts/<date>/<robot_id>/<run_id>/{metadata.json, fast_log.bin, integrity_report.json, summary.json, plots/, operator_notes.md, config_snapshot/}`；原始文件只读归档，禁止覆盖。
9. **SysId（docs2/11 §7–11）**：模型 V = kS·sign(v) + kV·v + kA·a，分侧分方向；Quasistatic 正/反 + Dynamic 正/反，多次重复 + 独立验证试次；最小采集字段含 time_us/sequence/逐侧命令电压（实际物理电压）/位置/速度/加速度/逐电机电流温度/IMU/电池/降额/饱和标志/raw_dt/exec/质量/故障位；走快日志 100 Hz；报告训练/验证 RMSE/MAE/最大残差/R²/残差分布/参数重复性；**禁止 12/Vbattery 放大**。
10. **一行摘要与预警（docs/10 §3.2）**：每条运动一行 `SETTLED t=settle时间 err=终点误差 ov=过冲 i_pk=积分峰值 sat=饱和占比`；预警：i_pk 逼近 i_max、sat>60%、ov 变号。
11. **四条最有诊断力曲线（docs/10 §2）**：目标 vs 实际位置、输出 u、积分项、滤波后速度。
12. **SOP 量化判据（docs/10 §1）**：过冲<3%、终态 ±5 mm、输出峰值≤0.95、σ<3 mm、电池 11.5 vs 12.6 V 表现一致、SysId R²>0.99、IMU 静置 1 min 漂移<1°。
13. **统计要求**：均值/std/中位/p95/p99/最坏值、按方向/冷热/电量分组；jerk 与 FFT 文档未覆盖 → 属扩展项，jerk 需注意加速度噪声警告（docs2/12 §8）。
14. **PC 重放链（docs2/12 §10.2, docs2/14 §7）**：raw 快日志 + 真实 time_us → SensorValidator → Odometry → 与原 state 对比，输出逐字段首次分歧；旧故障日志成回归集。

---

## 4. 现状盘点

### 4.1 已有资产（已核实，路径相对项目根 `C:\Users\alexh\Documents\override\74000M\74000M`）

| 资产              | 位置                                                      | 说明                                                                                                                                                                                                                                                                                                                                                                                                          |
| --------------- | ------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 定长 POD LogFrame | `include/robot/telemetry/log_frame.hpp`                 | magic `0x374D3030`（"7M00"）、schema 1.0、`static_assert(std::is_trivially_copyable_v<LogFrame>)` 已锁定。字段：LogHeader + 左右各 3 电机样本（位置/速度/电流/温度/电压/故障）+ IMU + battery_V + 位姿(x,y,θ)/机体速度(vx,vy,ω)/质量 + RequestLog + ActuatorLog（三级电压/derate/写入结果）+ TimingLog（raw_dt/math_dt/exec/jitter/age/overrun/ring_depth/log_dropped_total）+ FaultLog。**帧长约 712 字节（由字段推算，待 static_assert 锁定确认）→ 100 Hz 全速原始码率约 71 KB/s（推算值）** |
| 无锁 SPSC 环       | `include/robot/telemetry/spsc_ring.hpp`                 | 满环丢新帧并计数，语义符合 docs2/12。**但产品级实例不存在（只有测试实例）**                                                                                                                                                                                                                                                                                                                                                                |
| 遥测任务骨架          | `include/robot/telemetry/telemetry_task.hpp`            | `TelemetrySink` 抽象接口（`write(frames,count)->bool`）+ `TelemetryDrain` 批量出队。**无任何 sink 实现，无 PROS 遥测任务**                                                                                                                                                                                                                                                                                                        |
| 完整性跟踪           | `include/robot/telemetry/integrity.hpp`                 | `LogIntegrityTracker`（sequence 缺口/时间回退/epoch 回退），PC 端可复用同款逻辑                                                                                                                                                                                                                                                                                                                                                |
| 重放              | `include/robot/telemetry/replay.hpp`                    | `RawInputReplay`                                                                                                                                                                                                                                                                                                                                                                                            |
| SysId 激励发生器     | `include/robot/calibration/characterization_runner.hpp` | Quasistatic/Dynamic × Forward/Reverse，安全闸门齐全                                                                                                                                                                                                                                                                                                                                                                |
| SysId 拟合        | `include/robot/calibration/calibration_tools.hpp`       | `SysIdSample` + `fitFeedforward()`（kS/kV/kA 最小二乘，训练/验证分离），**header-only 平台无关，PC 端可直接复用或移植**                                                                                                                                                                                                                                                                                                                 |

### 4.2 缺口清单（已核实）

| 编号  | 缺口                                                       | 影响              | 解决阶段              |
| --- | -------------------------------------------------------- | --------------- | ----------------- |
| G1  | `src/main.cpp` 无遥测接线（无产品级 SPSC 环实例、无 TelemetryTask 创建）   | 遥测链路不存在         | M0/M1             |
| G2  | 全库无 serial/printf/SD/文件 I/O 代码                           | 无任何 sink 出口     | M1/M2             |
| G3  | LogFrame 无 PID p/i/d/ff 分量字段（PidResult 在控制器出口存在）         | 无法绘制 P/I/D 分量曲线 | M2 前置（schema 1.1） |
| G4  | 当前 cycle 不跑里程计                                           | 位姿字段暂为空/低质量     | 依赖板块 8 解锁         |
| G5  | 帧长 712 B 为推算值，未用 `static_assert(sizeof(LogFrame)==…)` 锁定 | 协议帧长不确定         | M0                |
| G6  | PROS stdout 吞吐上限未知                                       | 实时流码率无法预算       | M0 第一项实测          |
| G7  | PC 端工具链不存在                                               | 记录/分析/归档无从谈起    | M1–M4             |

---

## 5. 系统总体架构

### 5.1 设计原则

1. **控制优先于记录**（硬约束）：遥测链路的任何故障（串口断开、SD 拔出、缓冲区满、PC 未连接）不得阻塞、延迟或改变控制计算与电机输出。
2. **单生产者单消费者**：控制环每拍仅 `tryPush` 一次；所有昂贵操作（格式化、I/O、COBS 编码）在后台 TelemetryTask 完成。
3. **双 sink 互补**：SD 卡存全速二进制（事后精析），USB 串流降频摘要（实时观察），两者各自独立失败、互不影响。
4. **schema 演进可控**：帧头带 schema major/minor，PC 端对不兼容 schema 明确拒绝。

### 5.2 架构图（ASCII）

```text
机器人端 (V5 Brain, PROS/C++17)
=================================

 ControlLoop (10 ms 名义, 100 Hz)
   │  每拍一次 tryPush(LogFrame)          ← 控制环内唯一遥测动作
   │  满环 → 丢新帧, log_dropped_total++
   ▼
 SpscRing<LogFrame, N>  (无锁 SPSC, 定长帧)
   │
   │  TelemetryDrain::drainBatch()        ← 后台任务, 批量出队
   ▼
 TelemetryTask (PROS task, 低优先级)
   │
   ├──► SdBinarySink ──► /usd/<run_id>/fast_log.bin
   │       (全速 100 Hz 紧凑二进制; SD 缺失/慢写/满盘只计数退避)
   │
   └──► UsbStreamSink ──► USB user port (stdout, 需先 SERCTL_DISABLE_COBS)
             (COBS 定界 + CRC16; 分频 20–30 Hz 摘要帧; 溢出丢帧计数)

   [备用] VEXnet 手柄通道: 仅低速健康摘要轮询, 非实时主通道

PC 端 (Windows, Python)
=================================

 USB COM 口 ◄── COBS 解码 + CRC16 校验 + sequence 缺口统计
   │
   ▼
 接收解码层 (pyserial 后台线程 → 线程安全队列)
   │
   ├──► 会话管理层: 身份档案(队号/操作员/机器人ID/日期/电池/场地/备注)
   │       → metadata.json, 与每次采集绑定
   │
   ├──► 实时仪表盘 (pyqtgraph): 轨迹/v/a/jerk/PID/温度电流/dt 抖动
   │
   ├──► 录制器: 原始二进制落盘 fast_log.bin (只读归档)
   │
   └──► 离线分析 (numpy/scipy/matplotlib):
           P0 完整性报告 → P1 轨迹/运动学/PID/SysId → P2 FFT/傅里叶/滤波器对比

 归档目录:
 artifacts/<date>/<robot_id>/<run_id>/
   ├── metadata.json        (会话身份 + run metadata, docs2/12 §4.1 清单)
   ├── fast_log.bin         (原始二进制, 只读, 禁止覆盖)
   ├── integrity_report.json
   ├── summary.json
   ├── plots/
   ├── operator_notes.md
   └── config_snapshot/
```

---

## 6. 通信协议设计

### 6.1 帧格式（二进制线帧）

【设计决策】USB 线帧与 SD 落盘帧复用同一 LogFrame 有效载荷，外层加线帧头尾：

```text
USB 线帧 (COBS 定界, 0x00 为帧界):
┌──────────────────────────────────────────────────────────┐
│ LogHeader (magic/schema/frame_size/time_us/sequence/     │
│            mode_epoch/run_id_hash)                       │
│ payload: LogFrame 其余字段 (电机/IMU/位姿/请求/执行器/    │
│          时序/故障)                                       │
│ crc16_ccitt(payload bytes)                               │
└──────────────────────────────────────────────────────────┘
→ COBS 编码 → 0x00 分隔符 → 写入 stdout (fflush 按批)
```

- **magic/schema**：`0x374D3030` / 1.x，PC 端第一步校验，不兼容明确拒绝（docs2/12 强制）。
- **sequence**：严格递增，PC 端用缺口统计实时评估丢帧率；与帧内 `log_dropped_total` 交叉核对以区分「生产者丢帧」与「串口传输丢失」（对应完整性报告五类问题中的两类）。
- **CRC16-CCITT**：覆盖帧内全部字节；COBS 保证 0x00 只出现在帧界，帧同步可靠。
- **COBS**：需先 `SERCTL_DISABLE_COBS` 关闭 PROS 默认复用（§3.2），然后应用层自行 COBS。【设计决策】放弃 PROS 默认 COBS 复用（其协议语义不透明），采用应用层自控 COBS，PC 端解码实现约 30 行 Python。

### 6.2 帧类型

【设计决策】schema 1.x 定义两类线帧：

| 帧类型          | 频率       | 内容                                                               | 去向                           |
| ------------ | -------- | ---------------------------------------------------------------- | ---------------------------- |
| FULL（全量帧）    | 100 Hz   | 完整 LogFrame（712 B 级）                                             | 仅 SD；USB 仅在 M0 实测证明带宽足够时分频开启 |
| SUMMARY（摘要帧） | 20–30 Hz | 位姿/速度/电压子集 + PID 分量 + 时序统计 + 故障位 + log_dropped_total（目标 ≤ 256 B） | USB 实时流（也可混入 SD）             |

### 6.3 码率预算

- 全速 100 Hz × ~712 B ≈ **71 KB/s（推算值，待 G5 锁定帧长后重算）**。
- USB 2.0 HS 物理层 480 Mbit/s 远非瓶颈；瓶颈在 PROS 内核缓冲与调度（§3.2）。**PROS stdout 安全持续吞吐未知 → M0 实测。**
- 【设计决策】预算分配策略：
  - SD 全速 71 KB/s：SD 卡 FAT32 连续写通常远高于此，但**SD 实际写速待实测**（M1 出口条件）；
  - USB 摘要帧 25 Hz × 256 B ≈ 6.4 KB/s：显著低于 3151A 已验证的 30 Hz JSON 先例负载，风险低；
  - 若 M0 实测证明 stdout 可持续 ≥ 80 KB/s，则可选开启 USB 全速帧模式（调试特写场景，SD 卡缺席时的降级方案）。

### 6.4 分频与背压策略

1. **生产者侧（控制环）**：不感知分频，永远每拍 tryPush；满环丢帧计数——这是 docs2/12 的硬语义，不可协商。
2. **消费者侧（TelemetryTask）**：按 `sequence % k == 0` 抽稀给 USB sink；SD sink 不抽稀。
3. **USB sink 背压**：使用非阻塞写（`SERCTL_NOBLKWRITE`，参考 PROS APIX），写不下即丢该帧并计数 `usb_dropped_total`（加入 SUMMARY 帧上报）；**严禁阻塞等待 PC**——PC 掉线时机器人行为必须与连线时完全一致。
4. **SD sink 背压**：写超时/失败计数退避（如失败后 1 s 内不再尝试），不卡顿（docs2/12）。

---

## 7. PC 端功能规格（重点章节）

### 7.1 会话与身份管理（用户特别要求，P0）

**功能目标**：每次采集的数据都能回答「谁、哪天、用哪台机器人、哪块电池、在什么场地、做了什么测试、备注了什么」。

**功能清单**：

1. **身份档案（Profile）CRUD**：
   
   - 新建/选择/编辑/复制档案；档案持久化为 `profiles.json`（PC 端用户目录或工具目录）。
   
   - 档案字段（【设计决策】，对齐 docs2/12 §4.1 metadata 清单）：
     
     | 字段              | 必填  | 说明                                      |
     | --------------- | --- | --------------------------------------- |
     | team_number     | 是   | 队号，如 `74000M`                           |
     | operator        | 是   | 操作员姓名/代号                                |
     | robot_id        | 是   | 机器人 ID（须与固件 `RobotIdentity` 一致，采集时交叉校验） |
     | date            | 自动  | 会话日期（自动生成，可改）                           |
     | battery_id      | 是   | 电池编号                                    |
     | field_condition | 否   | 场地/工况（练习场/比赛场/垫子批次等）                    |
     | tire_condition  | 否   | 轮胎状态（docs2/12 §4.1 要求项）                 |
     | ambient_temp    | 否   | 场地温度（docs2/12 §4.1 要求项）                 |
     | notes           | 否   | 备注                                      |

2. **采集会话（Run）绑定**：
   
   - 每次开始录制必须选择一个档案 + 填写 `test_case_id`（如 `sysid_qs_fwd_left`、`pid_step_1m`）；
   - 自动生成 `run_id`（【设计决策】：`YYYYMMDD-HHmmss_<test_case_id>` 短哈希），并写入机器人端 run 开始/结束标记帧（若固件支持）或仅由 PC 侧按录制窗口切分；
   - 落盘 `metadata.json`：档案全字段 + 固件上报的 run metadata（software_commit + dirty flag、VEXos/PROS 版本、schema、config_hash、轮/齿比/质量/重心、电池起止电压、电机起止温度）——固件上报项在 schema 1.1 中以 RUN_META 帧下发，未接入前 PC 侧留空并标注「待固件补充」。

3. **归档规则**（docs2/12 §4.1 强制）：
   
   - 目录结构：`artifacts/<date>/<robot_id>/<run_id>/`（§5.2）；
   - `fast_log.bin` 录制完成后设为只读；**禁止覆盖已有 run_id 目录**，重名即报错要求换 test_case_id；
   - `operator_notes.md` 提供录制结束后的快速备注入口（模板预填档案字段）。

**验收标准**：

- 创建档案 → 录制 30 s → 目录结构与文件齐全，metadata.json 通过 jsonschema 校验；
- 不选档案无法开始录制（UI 硬约束）；
- 同一 run_id 目录二次录制被拒绝；
- metadata.json 中 robot_id 与固件上报不一致时产生醒目警告（但允许保存，记为 integrity 问题）。

### 7.2 实时仪表盘（P1，依赖 M2）

【设计决策】技术形态：Python + pyqtgraph 单窗口多页签，串口后台线程接收、GUI 线程 30 Hz 刷新。

| 页签   | 内容                                                                      | 数据来源帧                            | 备注                                                                             |
| ---- | ----------------------------------------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------ |
| 轨迹   | 二维 x-y 图，叠加目标路径（若有轨迹跟随）；可选场地图底图                                         | 位姿字段                             | G4 未解锁前显示「位姿不可用」占位                                                             |
| 运动学  | v-t、a-t、jerk-t 三曲线                                                      | 机体速度(vx,vy,ω) + 离线差分             | **jerk 曲线固定叠加噪声警告条**（docs2/12 §8：jerk 文档未覆盖、注意加速度噪声）；提供滤波选项（无/savgol/移动平均）并排对比 |
| PID  | 目标 vs 实际位置、输出 u、积分项、P/I/D/ff 分量、滤波后速度                                   | RequestLog + schema 1.1 PID 分量字段 | 对齐 docs/10 §2「四条最有诊断力曲线」；**P/I/D 分量依赖 G3 修复（schema 1.1）**                      |
| 电机健康 | 6 电机温度/电流/电压曲线 + derate 状态                                              | 电机样本 + ActuatorLog               | 超温/降额阈值线标注                                                                     |
| 实时性  | dt 抖动直方图、raw_dt/math_dt/exec 时序、overrun/ring_depth/log_dropped_total 计数 | TimingLog                        | p50/p95/p99 实时统计                                                               |
| 摘要   | 每条运动结束自动生成一行 `SETTLED t=… err=… ov=… i_pk=… sat=…`（docs/10 §3.2 格式）     | 全帧滑窗                             | **预警规则（docs/10 §3.2）**：i_pk 逼近 i_max → 黄色；sat>60% → 橙色；ov 变号 → 红色              |

**验收标准**：

- 25 Hz 摘要流下 GUI 不卡顿，sequence 缺口率 < 0.1%（M0 实测码率内）；
- 串口拔插后自动重连且丢帧计数连续可解释；
- SETTLED 行与预警颜色规则有单元测试覆盖。

### 7.3 离线分析（P0 完整性 → P1 核心 → P1 SysId）

**P0 — 完整性报告（docs2/12 强制第一步，任何分析前置）**

输入 `fast_log.bin` + `metadata.json`，输出 `integrity_report.json`：

- magic/schema/帧长校验（schema 不兼容 → 明确拒绝，退出码非零）；
- sequence 连续性、时间戳单调性、mode_epoch 合法边沿；
- 丢帧计数一致性（sequence 缺口数 vs 帧内 `log_dropped_total` 增量）；
- 文件末帧完整性（截断帧 → 标注并丢弃）；
- 若导出 CSV：列/有限数/枚举合法性；
- run metadata 一致性（robot_id、config_hash 与帧内 run_id_hash 匹配）；
- **问题分类五类**：生产者丢帧 / 消费者或文件丢失 / 传感器陈旧 / 请求执行器陈旧 / 解析错误；
- **判定规则：关键窗口（如 SysId 激励段、一次自动运动段）缺帧 → 该试次默认无效，结论 = REPEAT**（docs2/12 强制）。

**P1 — 轨迹 / 运动学 / PID 分析**

- 二维轨迹图（叠加目标路径、起终点标记、误差带）；
- v/a/jerk-t 曲线组（jerk 默认带滤波开关与噪声警告）；
- PID 震荡曲线：目标 vs 实际、u、积分、P/I/D 分量；
- 逐次运动 SETTLED 摘要表 + 预警标记；
- 统计输出（docs2/12）：均值/std/中位/p95/p99/最坏值，按方向/冷热（电机起始温度分组）/电量（电池电压分组）分组；
- SOP 量化判据自动核对（docs/10 §1）：过冲<3%、终态 ±5 mm、输出峰值≤0.95、σ<3 mm、11.5 vs 12.6 V 一致性、IMU 静置漂移<1°——每项输出 PASS/FAIL/NOT TESTED。

**P1 — SysId 拟合闭环**

- 从快日志提取 `SysIdSample` 等价字段（time_us/sequence/逐侧命令电压/位置/速度/加速度/逐电机电流温度/IMU/电池/降额/饱和标志/raw_dt/exec/质量/故障位——docs2/11 最小字段清单）；
- 拟合 V = kS·sign(v) + kV·v + kA·a，分侧分方向，训练/验证分离；
- 输出 RMSE/MAE/最大残差/R²/残差分布/多次重复参数一致性（docs2/11 强制报告项）；
- 【设计决策】实现路径二选一：① C++ `fitFeedforward()` 经 ctypes/pybind11 编译为 PC 动态库直接复用（首选，保证与固件算法逐位一致）；② Python/numpy 重实现最小二乘（fallback，需用固定测试向量与 C++ 版对拍）；
- 拟合结果回填：生成参数 diff 报告，由人工经 HMI/配置流程写回机器人——**本系统不直接改机器人参数**（安全边界）。

### 7.4 扩展功能（P2，均为超出 docs2 基线的增强项，明确标注）

| 功能                       | 说明                                                                                  | 依据/风险                                                                            |
| ------------------------ | ----------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| FFT 频谱分析                 | 对 PID 误差/速度信号做 FFT：识别震荡频率、估算阻尼比、定位噪声主频 → 指导滤波器截止频率设计                                | **docs 未覆盖，属扩展**；采样率 100 Hz → 奈奎斯特 50 Hz，高于此的机械振动不可见，报告中必须标注                     |
| 傅里叶级数轨迹拟合                | 用有限项傅里叶级数拟合闭合/周期轨迹，评估轨迹平滑性与可复现性                                                     | 用户点名扩展项；纯离线，无安全影响                                                                |
| 滤波器对比实验                  | butter/savgol/移动平均等离线作用于同一日志，对比相位延迟与噪声抑制；结论经人工评审后回填机器人滤波参数                          | 离线验证 → 人工回填，禁止自动下发                                                               |
| AdvantageScope 兼容 CSV 导出 | 导出位姿/遥测 CSV，用 AdvantageScope 做 2D/3D 场地图回放（§3.3 双轨策略）                               | 格式细节待查官方文档对齐，列为 M4 任务                                                            |
| 日志重放对比                   | raw 快日志 + 真实 time_us → SensorValidator → Odometry → 与原 state 对比，输出逐字段首次分歧；旧故障日志成回归集 | docs2/12 §10.2、docs2/14 §7 强制方向；复用 `replay.hpp` 逻辑，建议以 C++ PC 测试目标实现而非 Python 重写 |

### 7.5 技术栈建议（【设计决策】）

| 层         | 选型                                                             | 理由                                                |
| --------- | -------------------------------------------------------------- | ------------------------------------------------- |
| 串口        | Python + pyserial                                              | 生态成熟，COBS/CRC 实现简单                                |
| 实时绘图      | pyqtgraph                                                      | 大数据量实时刷新性能优于 matplotlib                           |
| 离线分析      | numpy + scipy + matplotlib                                     | scipy.signal 覆盖 FFT/butter/savgol；matplotlib 出报告图 |
| schema 校验 | pydantic 或 jsonschema                                          | metadata.json / integrity_report.json 校验          |
| SysId 拟合  | ctypes/pybind11 复用 C++ `fitFeedforward()`（首选）或 numpy 重实现（对拍验证） | 保证与固件算法一致性                                        |
| 打包        | 单目录脚本起步，后期 PyInstaller 打包 exe                                  | 队员零环境可用                                           |

目录建议（【设计决策】）：`statusmonitor/pc_tool/` 下分 `serial_rx.py / cobs_crc.py / frames.py / profiles.py / recorder.py / dashboard.py / analysis/integrity.py / analysis/kinematics.py / analysis/pid_report.py / analysis/sysid_fit.py / export/advantagescope.py`。

---

## 8. 对调参有意义的参数指标清单

### 8.1 PID 整定组

| 指标         | 定义/来源                        | SOP 判据                             |
| ---------- | ---------------------------- | ---------------------------------- |
| 过冲 ov      | 峰值超调占目标比（docs/10 §1/§3.2）    | < 3%                               |
| settle 时间  | 进入并保持在误差带内所需时间（docs/10 §3.2） | 越短越好，按运动类型建基线                      |
| 终点误差 err   | settle 时刻残余误差（docs/10 §1）    | 终态 ±5 mm                           |
| 稳态 σ       | settle 后位置标准差（docs/10 §1）    | < 3 mm                             |
| 积分峰值 i_pk  | 运动全程积分项最大值（docs/10 §3.2）     | 逼近 i_max → 预警（积分饱和风险）              |
| 饱和占比 sat   | 输出处于限幅的时间占比（docs/10 §3.2）    | > 60% → 预警；输出峰值 ≤ 0.95（docs/10 §1） |
| ov 变号      | 反复穿越目标（震荡标志）（docs/10 §3.2）   | 出现 → 红色预警                          |
| P/I/D 分量曲线 | schema 1.1 新增字段（G3）          | 分量形状诊断：D 过大噪声毛刺、I 爬升慢→ki 不足        |

### 8.2 前馈 / SysId 组

| 指标                  | 定义/来源                                    | SOP 判据              |
| ------------------- | ---------------------------------------- | ------------------- |
| kS/kV/kA            | V = kS·sign(v)+kV·v+kA·a，分侧分方向（docs2/11） | 多次重复参数一致性好          |
| 训练/验证 RMSE、MAE、最大残差 | 拟合质量（docs2/11）                           | 验证集不显著劣于训练集         |
| R²                  | 拟合优度（docs/10 §1）                         | > 0.99              |
| 残差分布                | 残差直方图/正态性（docs2/11）                      | 无明显结构（结构性残差 = 模型缺项） |
| 逐侧不对称性              | 左/右 kV、kA 差异                             | 显著差异 → 查机械/摩擦       |

### 8.3 滤波器设计组

| 指标        | 定义/来源                       | 用途           |
| --------- | --------------------------- | ------------ |
| 滤波后速度信噪比  | 滤波前后速度方差比（docs/10 §2 第四条曲线） | 选滤波器类型/参数    |
| 相位延迟      | 滤波器引入的滞后拍数                  | 过大 → 闭环不稳    |
| 噪声主频      | FFT 噪声谱峰（P2 扩展）             | 定截止频率        |
| jerk 幅值分布 | 加速度差分（注意噪声警告，docs2/12 §8）   | 评估 S 曲线/滤波需求 |

### 8.4 轨迹跟踪组

| 指标           | 定义/来源                           | 用途                      |
| ------------ | ------------------------------- | ----------------------- |
| 横向/纵向跟踪误差    | 轨迹误差分解                          | 调跟踪器增益                  |
| 曲率段误差        | 按路径曲率分组的误差                      | 查角速度限制是否合理              |
| 终点 settle 质量 | 复用 8.1 判据                       | 路线逐段验收                  |
| 电池分组一致性      | 11.5 V vs 12.6 V 表现（docs/10 §1） | 验证无 12/Vbattery 放大后的一致性 |

### 8.5 实时性审计组

| 指标                 | 定义/来源                | 判据                                         |
| ------------------ | -------------------- | ------------------------------------------ |
| t_exec p50/p95/p99 | TimingLog.exec 分布    | **p99 < 0.5 × T_nominal（5 ms）——遥测接入后必须满足** |
| dt 抖动              | raw_dt 直方图           | 无系统性长尾                                     |
| overrun 计数         | TimingLog.overrun    | 应为 0                                       |
| ring_depth 峰值      | TimingLog.ring_depth | 远低于环容量，无单调增长                               |
| log_dropped_total  | 生产者丢帧计数              | 正常工况应为 0；非零必须可解释                           |
| usb_dropped_total  | USB sink 丢帧计数（本策划新增） | 摘要流内应接近 0                                  |

### 8.6 安全健康组

| 指标            | 定义/来源                     | 判据                      |
| ------------- | ------------------------- | ----------------------- |
| 电机温度（起止 + 峰值） | 电机样本（docs2/12 §4.1）       | 超阈值 → derate 应已触发且日志可解释 |
| 电机电流峰值/均值     | 电机样本                      | 卡死/滑移诊断                 |
| 电池起止电压        | battery_V（docs2/12 §4.1）  | 分组统计基础                  |
| 故障位/derate 事件 | FaultLog/ActuatorLog      | 所有保护必须由日志解释（docs2/12）   |
| IMU 静置漂移      | 静置 1 min 角度变化（docs/10 §1） | < 1°                    |

---

## 9. 里程碑计划

> 总原则：遥测不得影响控制环（验收矩阵见 §9.6，每个里程碑出口都必须重跑该矩阵）。M0 先行是因为一切码率决策都依赖实测数字。

### M0 — 吞吐实测与协议原型（纯实验，不改产品代码）

**内容**：

1. 独立 PROS 测试程序（不改 main.cpp）：递增帧率（10/20/30/50/100 Hz）经 user port 发送带序号的固定长度帧（先关 COBS），PC 端 Python 脚本统计序号缺口率与持续吞吐 → **确定 stdout 安全码率上限**。
2. `static_assert(sizeof(LogFrame) == N)` 锁定帧长（G5），重算全速码率。
3. COBS + CRC16 编解码的 PC 端原型与单元测试。

**出口条件**：产出实测报告（安全持续吞吐 KB/s、缺口率拐点）；帧长锁定值写入策划案 v1.1；协议原型可环回自测。

### M1 — SD 全速落盘 + PC 解析

**内容**：

1. main.cpp 接线：产品级 SPSC 环实例 + TelemetryTask（G1）；
2. `SdBinarySink`：100 Hz 全帧落盘 `/usd/<run_id>/fast_log.bin`，SD 缺失/慢写/满盘只计数退避（G2 之一）；
3. PC 端：二进制解析器 + **P0 完整性报告**（§7.3）+ 会话/身份档案管理（§7.1）+ 归档目录落地；
4. 暂停消费者压力测试（docs2/12 强制）：kill TelemetryTask 后控制环时序与输出不变。

**出口条件**：SD 连续录制 ≥ 5 min 零解析错误；完整性报告通过且五类问题分类逻辑有测试；拔 SD 卡 30 s 内控制环 p99(t_exec) 不恶化；身份档案 → metadata.json → 归档目录全流程走通。

### M2 — USB 实时流 + 仪表盘

**内容**：

1. schema 1.1：LogFrame 增加 PID p/i/d/ff 分量字段（G3），PC 端按 minor 版本兼容解析，major 不符明确拒绝；
2. `UsbStreamSink`：COBS+CRC16、25 Hz SUMMARY 帧、非阻塞写 + usb_dropped_total（G2 之二）；
3. 实时仪表盘全部页签（§7.2），含 SETTLED 摘要与三色预警；
4. 里程计未解锁（G4）期间，轨迹页签显示占位，不阻塞其余页签。

**出口条件**：25 Hz 摘要流连续 10 min 缺口率 < 0.1%；拔插 USB 自动重连；仪表盘各曲线与 SD 落盘数据对拍一致（同 run 比对）；验收矩阵 §9.6 通过。

### M3 — SysId 闭环

**内容**：

1. characterization_runner 激励与快日志采集打通（docs2/11 字段清单核对）；
2. PC 端 SysId 拟合（ctypes 复用 `fitFeedforward()` 或对拍过的 Python 版）+ 完整报告（RMSE/MAE/最大残差/R²/残差分布/参数重复性）；
3. 参数 diff 报告 → 人工回填流程演练；
4. 电池 11.5 V vs 12.6 V 一致性核对（docs/10 §1）。

**出口条件**：一次完整 SysId 流程（激励 → 采集 → 完整性 PASS → 拟合 → R² 报告）端到端走通；关键窗口缺帧试次被正确判为 REPEAT；R²>0.99 的判据可自动输出 PASS/FAIL（数值本身待真机，离线用合成数据验证流程）。

### M4 — 比赛归档流程 + 扩展项

**内容**：

1. 归档流程 SOP 化：赛前档案模板（队号 74000M）、每场自动 run_id、赛后一键打包（含 config_snapshot、operator_notes）；
2. AdvantageScope 兼容 CSV 导出；
3. P2 扩展按优先级排期：FFT 频谱分析 → 滤波器对比实验 → 傅里叶级数轨迹拟合 → 日志重放对比（C++ PC 目标）；
4. 旧故障日志整理为回归集（docs2/12 §10.2）。

**出口条件**：比赛日全流程演练一次（档案 → 录制 → 完整性 → 摘要 → 归档打包）≤ 2 min/场；至少 FFT 与 CSV 导出可用；归档包可被第三方（教练）在无工具环境下读懂目录结构。

### 9.6 遥测不干扰控制环验收矩阵（每个里程碑出口必跑）

| 测试     | 方法                    | 通过判据                                                |
| ------ | --------------------- | --------------------------------------------------- |
| 最坏负载时序 | 遥测双 sink 全开 + 满速激励运动  | **p99(t_exec) < 0.5 × T_nominal（5 ms）**，overrun = 0 |
| 暂停消费者  | 冻结 TelemetryTask 60 s | 控制输出与时序不变；log_dropped_total 增长可解释；恢复后无雪崩            |
| SD 拔出  | 运动中拔卡 30 s 再插回        | 控制环 p99 不恶化；SD sink 计数退避；恢复后续写或换文件，不崩溃              |
| USB 断开 | 运动中拔线 30 s 再插回        | 控制环无感知；usb_dropped_total 计数；重连后流恢复                  |
| 满环注入   | 强制消费者慢速               | 生产者丢新帧不丢旧语义正确，计数一致                                  |

---

## 10. 风险与未核实项

| 编号  | 事项                        | 状态       | 处置                                                        |
| --- | ------------------------- | -------- | --------------------------------------------------------- |
| R1  | PROS stdout 最大持续吞吐无权威公开数字 | **待实测**  | M0 第一项任务；在实测前 USB 全速帧模式不得默认开启                             |
| R2  | LogFrame 帧长 712 B 为字段推算值  | **待确认**  | M0 用 `static_assert(sizeof(LogFrame))` 锁定并重算码率            |
| R3  | SD 卡 FAT32 连续写速与慢卡行为      | **待实测**  | M1 出口条件含 ≥ 5 min 连续录制；建议备两张不同品牌卡对比                        |
| R4  | LemLib 等外部库日志格式兼容性        | **未评估**  | 本系统自研协议，不依赖 LemLib；若未来引入 LemLib 需单独评估其日志导出对接              |
| R5  | jerk 与 FFT 超出 docs 文档基线   | **扩展项**  | 全部归入 P2；jerk 曲线固定带噪声警告（docs2/12 §8）；FFT 结果标注 50 Hz 奈奎斯特上限 |
| R6  | 当前 cycle 不跑里程计（G4）        | 依赖板块 8   | 位姿类功能先占位，板块 8 解锁后自动生效；不阻塞 M0–M2 其余部分                      |
| R7  | schema 1.1 加 PID 分量字段改变帧长 | 设计内      | 与 M0 帧长锁定同步冻结；schema minor 升级，PC 端兼容解析                    |
| R8  | PC 掉线/高负载时 GUI 积压         | 设计内      | 接收线程有界队列 + 丢帧计数；仪表盘可降级为只录不画                               |
| R9  | 手柄 VEXnet 备用通道带宽延迟        | 已核实不适合实时 | 仅低速健康摘要；不投入开发量，列入「未来可选」                                   |

---

## 11. 附录：来源清单

### 11.1 网络来源（已核实）

1. V5 Brain 拆解分析（Zynq XC7Z010）：https://snikolaj.com/2023/02/25/vex-robotics-v5-brain-analysis/
2. V5 Brain 官方产品规格（276-4810）：https://www.vexrobotics.com/276-4810.html
3. VEX KB — Understanding the V5 Robot Brain（处理器/RAM/Flash/USB）：https://kb.vex.com/hc/en-us/articles/360060662352-Understanding-the-V5-Robot-Brain
4. vexide — Serial Deep Dive（双虚拟串口、手柄通道轮询 FIFO）：https://vexide.dev/blog/posts/serial-deep-dive/
5. 3151A 遥测先例（30 Hz JSON、SERCTL_DISABLE_COBS）：https://www.aadishv.dev/robotics-5
6. rosserial issue #591（Serial buffer full!）：https://github.com/ros-drivers/rosserial/issues/591
7. PROS APIX（serctl、usd 等扩展 API）：https://pros.cs.purdue.edu/v5/extended/apix.html
8. V5SerialPlotter（PC 实时绘图参考实现）：https://github.com/adityanarayanan03/V5SerialPlotter
9. AdvantageScope 日志格式文档：https://docs.advantagescope.org/overview/log-files/

### 11.2 项目文档（DOC_ROOT = C:\Users\alexh\Documents\VEX0713）

- `docs2/12-日志测试验收与诊断.md`（§1.1, §2.1–2.4, §4.1, §8, §10.2）——日志架构、完整性报告、metadata、jerk 警告、重放链
- `docs2/11-系统辨识底盘调校与参数管理.md`（§7–11）——SysId 模型、流程、最小采集字段、报告项
- `docs2/14-仿真与数值验证.md`（§7）——重放回归集
- `docs/10-调参实战手册.md`（§1, §2, §3.2, §3.3）——SOP 判据、四条诊断曲线、SETTLED 摘要与预警、20 Hz 串口绘图仪实践
- `docs/HARDWARE_COMMISSIONING.md`——硬件 commissioning 顺序（本系统真机阶段遵循）

### 11.3 项目代码（PROJECT_ROOT = C:\Users\alexh\Documents\override\74000M\74000M）

- `include/robot/telemetry/log_frame.hpp`（LogFrame / LogHeader，magic 0x374D3030，schema 1.0）
- `include/robot/telemetry/spsc_ring.hpp`（SPSC 环）
- `include/robot/telemetry/telemetry_task.hpp`（TelemetrySink / TelemetryDrain）
- `include/robot/telemetry/integrity.hpp`（LogIntegrityTracker）
- `include/robot/telemetry/replay.hpp`（RawInputReplay）
- `include/robot/calibration/characterization_runner.hpp`（SysId 激励）
- `include/robot/calibration/calibration_tools.hpp`（SysIdSample / fitFeedforward）
- `src/main.cpp`（组合根，遥测接线点）

---

*— 策划案撰写_KimiK3，2026-07-20。本策划案中所有「待实测/待确认」项必须在对应里程碑出口前落实，任何实测数字回填时连同测量方法与日期一并更新本文档。*
