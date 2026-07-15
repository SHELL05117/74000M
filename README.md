# 74000M 机器人程序

本项目当前面向 **1690X 样机**的 VEX V5 六马达差速底盘底层与自动控制工程，使用 **PROS + C++17**。

> 当前状态：已生成可下载的 **4 V 限压真机调试版本**，可用左摇杆单杆 Arcade 做架空和低速驾驶测试。官方 `hardware_output`、`driver_control` 与全部自动能力仍保持 `false`，因为端口方向、断控、TTL、停车和热/电流尚未完成 HIL 验收；`autonomous()` 始终执行 `DoNothing`。

## 1. 当前机器人配置

### 1.1 身份与能力

| 项目 | 当前值 | 修改位置 |
|---|---|---|
| 队号 | `1690X` | [`make1690XCommissioningConfig()`](include/robot/config/robot_config.hpp) 中 `config.identity` 的第 1 项 |
| 机器人 ID | `1690X` | 同上第 2 项 |
| 机器人名称 | `1690X SAMPLE` | 同上第 3 项；它不是队号 |
| 配置 schema | `2` | 同上及运行时 `kExpectedConfigSchema` |
| 底盘 | 六马达差速底盘 | [`HardwareConfig`](include/robot/config/robot_config.hpp) |
| 自动线路 | `DoNothing` | `RobotConfig::selected_route` |
| 正式真机输出 | 锁定 | `RobotCapabilities::hardware_output == false` |
| 受限试车输出 | 非场控下可用，最大 `4 V` | [`CommissioningControlCycle`](include/robot/manual/commissioning_arcade.hpp) |

能力必须按下面的依赖顺序解锁：

```text
hardware_output
  -> driver_control / pose_good
  -> autonomous_chassis_velocity
  -> autonomous_motion
  -> competition_routes
```

完整真机解锁流程见 [`docs/HARDWARE_COMMISSIONING.md`](docs/HARDWARE_COMMISSIONING.md)，当前样机的操作步骤见 [`docs/1690X_FIRST_DRIVE.md`](docs/1690X_FIRST_DRIVE.md)。

### 1.2 已录入的六个底盘马达

| 位置 | Port | cartridge | 外传动路径 | 电机转/轮转 | 理论轮速 | `reversed` |
|---|---:|---:|---|---:|---:|---:|
| 左前 LA | 11 | 600 RPM | 36T → 48T | `4/3` | 450 RPM | `true` |
| 左中 LM | 12 | 200 RPM | 36T → 12T；同轴 36T → 48T | `4/9` | 450 RPM | `false` |
| 左后 LB | 13 | 600 RPM | 36T → 48T | `4/3` | 450 RPM | `false` |
| 右前 RA | 1 | 600 RPM | 36T → 48T | `4/3` | 450 RPM | `false` |
| 右中 RM | 2 | 200 RPM | 36T → 12T；同轴 36T → 48T | `4/9` | 450 RPM | `true` |
| 右后 RB | 3 | 600 RPM | 36T → 48T | `4/3` | 450 RPM | `true` |

两种路径均得到 450 RPM 理论轮速，因此设计上不会因 cartridge 不同而互相对抗。运行时配置位于 [`make1690XCommissioningConfig()`](include/robot/config/robot_config.hpp)，硬件清单位于 [`config/hardware_profile.yaml`](config/hardware_profile.yaml)。YAML 是可追溯清单，程序不会自动读取它；修改接线或齿数时应同步更新两处并更新测试。

## 2. 构建与测试

### 2.1 PC 全量测试

```powershell
cmake -S . -B build/pc -DBUILD_TESTING=ON
cmake --build build/pc --config Release --parallel
ctest --test-dir build/pc -C Release --output-on-failure
```

### 2.2 PROS 构建

```powershell
make
```

PC 测试通过只代表离线逻辑正确，不代表真机方向、几何、保护阈值或自动线路已经批准。

### 2.3 当前样机试车操作

1. 第一次必须把底盘架空，观察员能立即点击 Brain 的停止/禁用按钮。
2. 下载并进入 Driver Control；确保没有连接场控。
3. 左摇杆保持中立，同时按住 `L1+L2+R1+R2` 1 秒，然后全部松开。
4. 左摇杆 `Y` 控制前后，`X` 控制转向；当前上限固定为 4 V。
5. 任意异常立即按 `B`。`B` 是锁存停车，必须禁用再重新启用并重新执行解锁组合键。
6. 先确认六个轮端方向、无机械对抗和 Brake 行为，再落地低速测试。

控制器断联、场控连接、模式变化、请求超时、输出帧超时或非有限数都会走 Brake 停车。没有 IMU，所以航向辅助、位姿和自动控制全部关闭。

## 3. 目录速查

| 目录 | 内容 |
|---|---|
| [`src/main.cpp`](src/main.cpp) | PROS 生命周期与 1690X 试车组合根 |
| [`include/robot/robot.hpp`](include/robot/robot.hpp) | 项目公共头文件总入口 |
| [`include/robot/config/`](include/robot/config) | 身份、端口、几何、校准、周期和能力配置 |
| [`include/robot/platform/`](include/robot/platform) | PROS/Fake IO 与单位边界 |
| [`include/robot/drive/`](include/robot/drive) | 请求、运动学、安全门和唯一输出服务 |
| [`include/robot/manual/`](include/robot/manual) | 摇杆整形、Slew、曲率驾驶和航向辅助 |
| [`include/robot/odometry/`](include/robot/odometry) | 驱动编码器/Tracking Wheel + IMU 里程计 |
| [`include/robot/commands/`](include/robot/commands) | Command、Subsystem、Scheduler 和命令组合 |
| [`include/robot/autonomy/`](include/robot/autonomy) | 自动原语、轨迹、跟踪器、路线注册和自动安全 |
| [`include/robot/hmi/`](include/robot/hmi) | 路线/参数注册、选择、事务和持久化 |
| [`tests/`](tests) | PC 单元、故障注入和离线闭环测试 |
| [`docs/`](docs) | 当前实现、真机调试和验证说明 |

核心代码使用 SI 单位：长度 `m`、角度 `rad`、时间间隔 `s`、电压 `V`；`TimeUs` 时间戳使用微秒。degree、RPM 和 mV 只允许在平台边界转换。

## 4. 常用 API

通常只需要包含：

```cpp
#include "robot/robot.hpp"
```

### 4.1 配置与单位

| API | 用途 | 定义位置 |
|---|---|---|
| `robot::make1690XCommissioningConfig()` | 生成当前样机的能力锁定配置 | [`robot_config.hpp`](include/robot/config/robot_config.hpp) |
| `robot::makeOfflineRobotConfig()` | 上述函数的兼容别名 | 同上 |
| `robot::validateConfig(config, robot_id, schema)` | 检查身份、端口、几何、校准和能力链 | 同上 |
| `robot::motorFromForwardFlag(port, rpm, forward, motor_rev_per_wheel_rev)` | 转换方向标志并记录逐马达轮端传动 | 同上 |
| `robot::nominalWheelRpm(motor)` | 由 cartridge 与外传动计算理论轮速 | 同上 |
| `robot::units::degreesToRadians()` | degree 转 rad | [`units.hpp`](include/robot/core/units.hpp) |
| `robot::units::rpmToRadiansPerSecond()` | RPM 转 rad/s | 同上 |
| `robot::units::voltsToMotorMillivolts()` | V 转 PROS mV，并做边界处理 | 同上 |

### 4.2 PROS 生命周期

[`src/main.cpp`](src/main.cpp) 提供以下标准入口：

| 回调 | 责任 |
|---|---|
| `initialize()` | 初始化硬件、启动一次性的持久任务、启动非阻塞校准 |
| `disabled()` | 请求 Disabled 模式，撤销旧 owner 并安全停车 |
| `competition_initialize()` | 赛前 HMI/自动选择准备，不启动第二套控制任务 |
| `autonomous()` | 强制 Disabled 并执行 `DoNothing` |
| `opcontrol()` | 非场控下进入受限 `Test`；场控连接时保持 Disabled |

不要在 `autonomous()` 或 `opcontrol()` 内新增直接写马达的阻塞循环。生命周期回调只应切换模式或投递事件。

### 4.3 请求、调度与输出

| API | 常用方法 | 作用 |
|---|---|---|
| [`robot::StaticScheduler<N>`](include/robot/commands/scheduler.hpp) | `schedule()`、`tick()`、`cancel()`、`cancelAll()` | 分配 requirement 和 owner lease，运行非阻塞命令 |
| [`robot::DriveRequestSink`](include/robot/commands/request_sink.hpp) | `beginFrame()`、`publish()`、`read()` | 每拍接收一条经过 owner 校验的底盘请求 |
| [`robot::DriveRequestArbiter`](include/robot/commands/drive_request_arbiter.hpp) | `select()` | 按模式、epoch、TTL、来源、能力和租约选择请求 |
| [`robot::SafetyGate`](include/robot/drive/safety_gate.hpp) | `apply()` | 最终限幅、Slew、降额和停车语义 |
| [`robot::OutputService`](include/robot/drive/output_service.hpp) | `tick()` | 唯一允许持有可写 `DriveIO&` 的执行器写入者 |

底盘输出的唯一合法路径是：

```text
Command / ManualDrive
  -> DriveRequest
  -> Scheduler / Arbiter
  -> Drivetrain
  -> SafetyGate
  -> ActuatorFrame
  -> OutputService
  -> DriveIO / PROS Motor
```

常用请求 payload：

- `DriverCurvaturePayload`：驾驶员曲率/Quick Turn/航向辅助；
- `ChassisVelocityPayload`：自动底盘速度请求，单位 `m/s`、`rad/s`；
- `WheelVoltagePayload`：只允许受控 Test 或已批准自动闭环使用；
- `BrakePayload`：`Coast`、`Brake` 或 `Hold` 停车。

### 4.4 手动、里程计与控制器

| API | 常用方法 | 定义位置 |
|---|---|---|
| `robot::ManualDrive` | `update()`、`reset()` | [`manual_drive.hpp`](include/robot/manual/manual_drive.hpp) |
| `robot::CommissioningControlCycle` | 1690X 的 4 V 单杆 Arcade 真机调试链 | [`commissioning_arcade.hpp`](include/robot/manual/commissioning_arcade.hpp) |
| `robot::Odometry` | `update()`、`requestReset()` | [`odometry.hpp`](include/robot/odometry/odometry.hpp) |
| `robot::EngineeringPid` | `update()`、`reset()` | [`engineering_pid.hpp`](include/robot/control/engineering_pid.hpp) |
| `robot::ChassisVelocityController` | `update()`、`reset()` | [`chassis_velocity_controller.hpp`](include/robot/control/chassis_velocity_controller.hpp) |
| `robot::calculateFeedforward()` | 计算 `kS + kV*v + kA*a` | [`feedforward.hpp`](include/robot/control/feedforward.hpp) |

### 4.5 自动运动与线路

| API | 用途 | 定义位置 |
|---|---|---|
| `DriveDistanceCommand` | 前进/后退指定距离 | [`motion_commands.hpp`](include/robot/autonomy/motion_commands.hpp) |
| `TurnToHeadingCommand` | 转到目标航向 | 同上 |
| `DriveArcCommand` | 按圆弧运动 | 同上 |
| `DriveToPoseCommand` | 驱动到目标位姿 | 同上 |
| `SequentialCommandGroup` | 顺序执行命令 | [`command_groups.hpp`](include/robot/commands/command_groups.hpp) |
| `ParallelCommandGroup` | 等所有子命令完成 | 同上 |
| `RaceCommandGroup` | 任一子命令结束即结束 | 同上 |
| `DeadlineCommandGroup` | deadline 完成即结束 | 同上 |
| `WaitCommand` / `TimeoutCommand` | 非阻塞等待/超时装饰 | 同上 |
| `FixedTrajectoryGenerator` | 从固定容量路径点生成时间轨迹 | [`trajectory.hpp`](include/robot/autonomy/trajectory.hpp) |
| `TrajectoryTracker` | 跟踪轨迹并发布 `ChassisVelocityPayload` | [`trajectory_tracker.hpp`](include/robot/autonomy/trajectory_tracker.hpp) |
| `AutonomousRouteRegistry` | 将稳定 `RouteId` 解析为命令工厂 | [`route_registry.hpp`](include/robot/autonomy/route_registry.hpp) |

所有自动命令都必须是非阻塞状态机，只发布 `DriveRequest`，不能直接访问马达。

### 4.6 HMI 路线和参数

| API | 常用方法 | 定义位置 |
|---|---|---|
| `RouteRegistry<N>` | `valid()`、`find()`、`doNothing()` | [`registry.hpp`](include/robot/hmi/registry.hpp) |
| `SelectionManager<N>` | `stage()`、`confirm()`、`lockForEnable()` | [`selection_manager.hpp`](include/robot/hmi/selection_manager.hpp) |
| `ParameterRegistry<N>` | `valid()`、`find()` | [`registry.hpp`](include/robot/hmi/registry.hpp) |
| `ParameterTransaction<N>` | `stage()`、`apply()`、`rollback()` | [`parameter_transaction.hpp`](include/robot/hmi/parameter_transaction.hpp) |

路线选择必须遵守 `Draft -> Confirmed -> Locked`。启用边沿只执行锁定快照；未确认、ID 无效、未实现、未批准或质量不足时自动回退 `DoNothing`。

## 5. 更换队号

队号的权威运行时配置在 [`make1690XCommissioningConfig()`](include/robot/config/robot_config.hpp)：

```cpp
config.identity = {
    "1690X",        // team_number：修改这一项
    "1690X",        // robot_id：独立字段
    "1690X SAMPLE", // robot_name：独立字段
    "commission",
    0, 2, 0
};
```

修改步骤：

1. 将第 1 项 `team_number` 改为新队号；当前固定缓冲最多容纳 **7 个可见字符**。
2. 不要因为队号改变就自动修改 `robot_id` 或 `robot_name`；它们是独立身份字段。
3. 若 Brain/Controller 存在单独写死的显示字符串，再同步修改；正常 HMI 应从 `RobotIdentity::team_number` 读取。
4. 更新明确验证队号的测试 fixture，不要修改与机器人名称有关的 `74000M` 断言。
5. 运行全量 PC 测试。

查找所有可能位置：

```powershell
rg -n '"1690X"|team_number|kExpectedRobotId' include src config tests docs
```

## 6. 添加、更换和选择自动线路

### 6.1 当前限制

当前生产 [`autonomous()`](src/main.cpp) 固定显示并执行 `DO NOTHING`，生产组合根尚未接入实际 `RouteRegistry`。完整线路注册示例目前在 [`tests/test_command_groups_routes.cpp`](tests/test_command_groups_routes.cpp)，它是测试示例，不是比赛线路表。

`RobotConfig::selected_route` 也不是绕过审批的开关：当 `competition_routes == false` 时，任何非 `DoNothing` 的选择都会被 `validateConfig()` 拒绝。

### 6.2 新增一条线路

1. **分配永久 ID**：在 [`include/robot/ui/registry_ids.hpp`](include/robot/ui/registry_ids.hpp) 的 `RouteIds` 中新增稳定 `RouteId`。ID 发布后不复用。
2. **实现命令**：用自动原语、轨迹跟踪器和固定容量命令组构造静态 `Command` 对象。不得在控制拍动态分配，也不得使用阻塞 `while`。
3. **添加描述符**：在生产路线表中添加 `RouteDescriptor`，填写短名、长名、联盟、起点、预期起始位姿、质量要求和标签。
4. **添加工厂**：增加同一 ID 的 `RouteFactoryDescriptor`，工厂只返回已经构造好的命令对象。
5. **更新 revision**：路线表内容变化后递增 registry revision，避免旧持久化选择指向新含义。
6. **先保持锁定**：初期设置 `implemented=false`、`competition_approved=false`，先完成 UI、PC、仿真和 HIL 测试。
7. **逐条批准**：只有该线路已通过真机与场地验收，才设置 `implemented=true`、`competition_approved=true`。
8. **接入选择器**：Disabled 状态用 `SelectionManager::stage()` 和 `confirm()`；启用边沿用 `lockForEnable()` 获取不可变选择。

描述符示意：

```cpp
constexpr robot::RouteId kRedNear = 0x0101;

{kRedNear,
 "R-NEAR",                 // <= 12 个 ASCII 字符
 "Red Near",
 robot::kAllianceRed,
 robot::kStartNear,
 {start_x_m, start_y_m, start_heading_rad},
 robot::RouteQualityRequirement::FullPoseGood,
 0,
 false,                     // 完成实现与测试后才改 true
 false}                     // CompetitionApproved 后才改 true
```

上例中的起点变量不能用猜测值。路线工厂和描述符 ID 必须一一对应；任何缺失或验证失败都应解析为 `DoNothingCommand`。

### 6.3 比赛中更换已批准线路

正确流程是：

```text
Disabled
  -> 选择 Alliance / StartSide / RouteId
  -> stage()
  -> confirm()
  -> Competition enable
  -> lockForEnable()
  -> AutonomousRouteRegistry::resolve()
  -> Scheduler::schedule()
```

不要通过修改数组下标、临时全局变量或在 `autonomous()` 中写 `if/else` 直接执行函数。线路选择与线路执行必须分离。

## 7. 常量与调参位置

### 7.1 当前真正生效的基础配置

| 参数组 | 字段 | 修改位置 | 当前状态/来源 |
|---|---|---|---|
| 身份 | `team_number`、`robot_id`、revision | [`make1690XCommissioningConfig()`](include/robot/config/robot_config.hpp) | 队号/样机 ID 已设为 1690X；revision 待冻结 |
| 马达 | port、`reversed`、`cartridge_rpm`、逐马达 `motor_rev_per_wheel_rev` | 同上 | 六个底盘马达与齿数已录入，仍需架空点动确认 |
| 传感器 | `ImuConfig::installed`、Rotation 配置 | `HardwareConfig` / 同上 | 外部传感器均未安装 |
| 名义几何 | 轮径、名义轮距 | `GeometryConfig` / 同上 | 2.75 in / SolidWorks 6 in，均未场地校准 |
| 校准几何 | 左右 `m_per_motor_rad`、有效轮距 | `CalibrationConfig` / 同上 | 待真机标定 |
| 控制周期 | `nominal_period_s`、数学 dt 范围 | `RuntimeConfig` / 同上 | 10 ms 是建议起点 |
| TTL | `request_ttl_us`、`output_ttl_us` | `RuntimeConfig` / 同上 | 40 ms 是离线起点 |
| 电压上限 | `max_command_voltage_V` | `ElectricalConfig` / 同上 | 当前试车硬限制 4 V；正式上限不得超过 12 V |
| 能力开关 | `RobotCapabilities` | `RobotConfig` / 同上 | 当前全部锁定 |

[`config/hardware_profile.yaml`](config/hardware_profile.yaml) 中的 `null` 表示仍需实机或 CAD 数据，不应随意填入“看起来合理”的值。

### 7.2 运行时、安全和驾驶参数结构

| 参数类别 | 主要字段 | 定义位置 |
|---|---|---|
| 调度时序 | `period_ms` | [`ControlLoopConfig`](include/robot/runtime/control_loop.hpp) |
| dt 与 deadline | `nominal_period_s`、`min/max_math_dt_s`、`deadline_dt_s` | [`TimingConfig`](include/robot/runtime/timing_monitor.hpp) |
| 请求仲裁 | `max_request_ttl_us` | [`DriveRequestArbiterConfig`](include/robot/commands/drive_request_arbiter.hpp) |
| 输出看门狗 | `frame_ttl_us`、`max_voltage_V`、`stale_stop_mode` | [`OutputServiceConfig`](include/robot/drive/output_service.hpp) |
| 安全门 | 电压上限、请求 TTL、输出 Slew、三种停车模式 | [`SafetyGateConfig`](include/robot/drive/safety_gate.hpp) |
| 输出 Slew | `rise_V_per_s`、`fall_V_per_s`、`max_dt_s` | [`OutputSlewConfig`](include/robot/drive/output_slew.hpp) |
| 摇杆曲线 | `center_offset`、`deadband`、`cubic_weight` | [`AxisShapeConfig`](include/robot/manual/input_shaping.hpp) |
| 手动驾驶 | 轴映射、输入 Slew、曲率、Quick Turn、TTL | [`ManualDriveConfig`](include/robot/manual/manual_drive.hpp) |
| 样机试车 | 解锁组合键、B 停车、左 Y/X、4 V、Slew、TTL | [`CommissioningArcadeConfig`](include/robot/manual/commissioning_arcade.hpp) |
| 航向辅助 | 进入/退出门、`kP/kD`、最大修正、质量要求 | [`HeadingAssistConfig`](include/robot/manual/heading_assist.hpp) |
| 传感器校验 | 范围、变化率、年龄、冻结、恢复样本 | [`SensorValidatorConfig`](include/robot/sensors/sensor_validator.hpp) |
| 里程计 | 布局、轮比例、轮距、tracking 外参、滑移确认 | [`OdometryConfig`](include/robot/odometry/odometry.hpp) |
| 电机保护 | 温度、电流、卡死进入/恢复阈值 | [`MotorProtectionConfig`](include/robot/health/motor_protection.hpp) |
| 故障规则 | 严重度、进入/恢复延时、降额、锁存 | [`FaultRuleConfig`](include/robot/health/fault_manager.hpp) |

### 7.3 自动控制参数结构

| 参数类别 | 主要字段 | 定义位置 |
|---|---|---|
| PID | `kp/ki/kd`、积分区/钳位、D 低通、死区、输出和 dt 范围 | [`PidConfig`](include/robot/control/engineering_pid.hpp) |
| 前馈 | 分侧/正反 `kS`、`kV`、`kA` | [`DirectionalFeedforwardGains`](include/robot/control/feedforward.hpp) |
| 轮速闭环 | 轮距、最大轮速/加速度/电压、状态年龄、左右 PID/前馈 | [`ChassisVelocityControllerConfig`](include/robot/control/chassis_velocity_controller.hpp) |
| 自动原语 | 线/角运动约束、误差增益、降级比例、TTL | [`MotionPrimitiveConfig`](include/robot/autonomy/motion_commands.hpp) |
| 结束条件 | error/velocity band、settle、timeout、stall | [`MotionTerminationConfig`](include/robot/control/termination.hpp) |
| 轨迹生成 | 速度、加减速度、角速度、抓地、电压、轮距、采样间隔 | [`TrajectoryConstraints`](include/robot/autonomy/trajectory.hpp) |
| 轨迹跟踪 | 纵横/航向增益、偏差门、曲率、降级/滑移比例 | [`TrajectoryTrackerConfig`](include/robot/autonomy/trajectory_tracker.hpp) |
| 自动安全 | 降级比例、最大偏差、宽限时间、状态年龄、TTL | [`AutonomySafetyConfig`](include/robot/autonomy/safety_supervisor.hpp) |
| SysId | ramp、dynamic step、最大电压/时间/距离、TTL | [`CharacterizationConfig`](include/robot/calibration/characterization_runner.hpp) |
| 路线解析 | 起始位置/航向容差 | [`RouteResolutionConfig`](include/robot/autonomy/route_registry.hpp) |

这些头文件定义的是参数**结构和合法范围**，不代表已经存在生产参数实例。当前许多数值只出现在 `tests/` 中用于验证算法；修改测试值不会调节真机。生产参数应在组合根中集中构造，并通过 `ParameterRegistry`、revision 和校准记录管理。

## 8. 调参原则

1. 先确认端口、方向、单位和机械无干涉，再调控制参数。
2. 先完成轮径/轮距和传感器质量校准，再调自动控制。
3. 先做分侧、正反 SysId，再调轮速 PID。
4. 每次修改绑定 `robot_id`、config/calibration revision、commit 和日志数据集。
5. 参数训练数据与最终验证数据分开。
6. 测试中的示例值、CAD 值和社区默认值不能直接标记为比赛参数。
7. `Apply` 只改变本次运行值，`Save` 才持久化；两者都不会自动获得 `CompetitionApproved`。

真机调试的完整顺序、记录要求和 capability 解锁门见 [`docs/HARDWARE_COMMISSIONING.md`](docs/HARDWARE_COMMISSIONING.md)。

## 9. 不可破坏的安全规则

- 全工程只有 `OutputService` 可以写驱动马达；
- Command、自动线路、HMI 和测试不能直接持有或写 `pros::Motor`；
- 自动逻辑不得使用阻塞运动 `while` 循环；
- 不得使用 `12 / battery_V` 放大物理电压请求；
- 有意停车和陈旧帧停车必须走 `stop()`/`brake()`，不能用 `move_voltage(0)` 代替；
- 所有跨任务请求和帧必须带 timestamp、sequence、mode epoch 和 TTL；
- 真机未验证前，相关 capability 保持 `false`，自动选择保持 `DoNothing`。
