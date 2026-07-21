# VEX V5 六马达底盘程序

本项目当前面向VEX V5 六马达差速底盘底层与自动控制工程，使用 **PROS + C++17**。
当前子系统属于Override 2026-2027赛季的子系统，底层逻辑可泛用于其他V5机器人。



当前版本：v0721.001 测试版

## 1. 构建与测试

### 1.1 PC 全量测试

```powershell
cmake -S . -B build/pc -DBUILD_TESTING=ON
cmake --build build/pc --config Release --parallel
ctest --test-dir build/pc -C Release --output-on-failure
```

### 1.2 PROS 构建

```powershell
make
```

PC 测试通过只代表离线逻辑正确，不代表真机方向、几何、保护阈值或自动线路已经批准。

### 1.3 当前样机试车操作

1. 第一次必须把底盘架空，观察员能立即点击 Brain 的停止/禁用按钮。
2. 下载并进入 Driver Control；确保没有连接场控。
3. 左摇杆保持中立；程序进入 Driver Control 后立即可以驾驶，不需要组合键解锁。
4. 默认双摇杆 Curvature：左摇杆 `Y` 控制前后，右摇杆 `X` 控制左右；可通过 `kCommissioningCurvatureInputMode` 切回左摇杆 `Y/X` 单摇杆模式。油门采用线性曲线，满杆会在一个名义 10 ms 控制帧内达到 12 V 命令上限。V5 电池满电端电压可高于 12 V，但 PROS `move_voltage` 的合法命令上限仍是 12000 mV，程序不会写 12.8 V 或做 `12 / battery_V` 放大。
5. 任意异常立即按住 `B` 或直接禁用。按住 `B` 时六个底盘马达立即 `Coast`；它不锁存，松开后会按当前摇杆位置恢复输出。
6. 先用约 10% 杆量确认六个轮端方向、无机械对抗和 `Coast` 行为，再落地；`Coast` 会自由滑行，全幅测试必须预留更长的停车距离。

摇杆回中、按住 `B`、控制器断联、场控连接、禁用、模式变化、请求超时、输出帧超时或非有限数都会走 `stop(Coast)` 停车。这里的 `Coast` 只决定零输出时的自由滑行行为；摇杆非零时仍使用 `move_voltage` 主动驱动。该策略只作用于六个底盘马达，不修改其他机构马达。没有 IMU，所以航向辅助、位姿和自动控制全部关闭。

## 2. 目录速查

| 目录                                                   | 内容                                |
| ---------------------------------------------------- | --------------------------------- |
| [`src/main.cpp`](src/main.cpp)                       | PROS 生命周期与当前试车组合根                 |
| [`include/robot/robot.hpp`](include/robot/robot.hpp) | 项目公共头文件总入口                        |
| [`include/robot/config/`](include/robot/config)      | 身份、端口、几何、校准、周期和能力配置               |
| [`include/robot/platform/`](include/robot/platform)  | PROS/Fake IO 与单位边界                |
| [`include/robot/drive/`](include/robot/drive)        | 请求、运动学、安全门和唯一输出服务                 |
| [`include/robot/manual/`](include/robot/manual)      | 摇杆整形、Slew、曲率驾驶和航向辅助               |
| [`include/robot/odometry/`](include/robot/odometry)  | 驱动编码器/Tracking Wheel + IMU 里程计    |
| [`include/robot/commands/`](include/robot/commands)  | Command、Subsystem、Scheduler 和命令组合 |
| [`include/robot/autonomy/`](include/robot/autonomy)  | 自动原语、轨迹、跟踪器、路线注册和自动安全             |
| [`include/robot/hmi/`](include/robot/hmi)            | 路线/参数注册、选择、事务和持久化                 |
| [`tests/`](tests)                                    | PC 单元、故障注入和离线闭环测试                 |
| [`docs/`](docs)                                      | 当前实现、真机调试和验证说明                    |

核心代码使用 SI 单位：长度 `m`、角度 `rad`、时间间隔 `s`、电压 `V`；`TimeUs` 时间戳使用微秒。degree、RPM 和 mV 只允许在平台边界转换。

## 3. 常用 API

通常只需要包含：

```cpp
#include "robot/robot.hpp"
```

### 3.1 配置与单位

| API                                                                          | 用途                                 | 定义位置                                                        |
| ---------------------------------------------------------------------------- | ---------------------------------- | ----------------------------------------------------------- |
| 当前试车配置工厂                                                                     | 生成当前样机的能力锁定配置                      | [`robot_config.hpp`](include/robot/config/robot_config.hpp) |
| `robot::makeOfflineRobotConfig()`                                            | 上述函数的兼容别名                          | 同上                                                          |
| `robot::validateConfig(config, robot_id, schema)`                            | 检查身份、端口、几何、校准和能力链                  | 同上                                                          |
| `robot::motorFromReversedFlag(port, rpm, reversed, motor_rev_per_wheel_rev)` | 直接记录 VEX/PROS 马达构造器的 `reversed` 标志 | 同上                                                          |
| `robot::motorFromForwardFlag(port, rpm, forward, motor_rev_per_wheel_rev)`   | 转换方向标志并记录逐马达轮端传动                   | 同上                                                          |
| `robot::nominalWheelRpm(motor)`                                              | 由 cartridge 与外传动计算理论轮速             | 同上                                                          |
| `robot::units::degreesToRadians()`                                           | degree 转 rad                       | [`units.hpp`](include/robot/core/units.hpp)                 |
| `robot::units::rpmToRadiansPerSecond()`                                      | RPM 转 rad/s                        | 同上                                                          |
| `robot::units::voltsToMotorMillivolts()`                                     | V 转 PROS mV，并做边界处理                 | 同上                                                          |

### 3.2 PROS 生命周期

[`src/main.cpp`](src/main.cpp) 提供以下标准入口：

| 回调                         | 责任                               |
| -------------------------- | -------------------------------- |
| `initialize()`             | 初始化硬件、启动一次性的持久任务、启动非阻塞校准         |
| `disabled()`               | 请求 Disabled 模式，撤销旧 owner 并安全停车   |
| `competition_initialize()` | 赛前 HMI/自动选择准备，不启动第二套控制任务         |
| `autonomous()`             | 强制 Disabled 并执行 `DoNothing`      |
| `opcontrol()`              | 非场控下进入受限 `Test`；场控连接时保持 Disabled |

不要在 `autonomous()` 或 `opcontrol()` 内新增直接写马达的阻塞循环。生命周期回调只应切换模式或投递事件。

### 3.3 请求、调度与输出

| API                                                                              | 常用方法                                           | 作用                                   |
| -------------------------------------------------------------------------------- | ---------------------------------------------- | ------------------------------------ |
| [`robot::StaticScheduler<N>`](include/robot/commands/scheduler.hpp)              | `schedule()`、`tick()`、`cancel()`、`cancelAll()` | 分配 requirement 和 owner lease，运行非阻塞命令 |
| [`robot::DriveRequestSink`](include/robot/commands/request_sink.hpp)             | `beginFrame()`、`publish()`、`read()`            | 每拍接收一条经过 owner 校验的底盘请求               |
| [`robot::DriveRequestArbiter`](include/robot/commands/drive_request_arbiter.hpp) | `select()`                                     | 按模式、epoch、TTL、来源、能力和租约选择请求           |
| [`robot::SafetyGate`](include/robot/drive/safety_gate.hpp)                       | `apply()`                                      | 最终限幅、Slew、降额和停车语义                    |
| [`robot::OutputService`](include/robot/drive/output_service.hpp)                 | `tick()`                                       | 唯一允许持有可写 `DriveIO&` 的执行器写入者          |

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

### 3.4 手动、里程计与控制器

| API                                | 常用方法                                 | 定义位置                                                                                       |
| ---------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------ |
| `robot::ManualDrive`               | `update()`、`reset()`                 | [`manual_drive.hpp`](include/robot/manual/manual_drive.hpp)                                |
| `robot::CommissioningControlCycle` | 12 V 可切换单/双摇杆 Curvature / Quick Turn 真机调试链 | [`commissioning_curvature.hpp`](include/robot/manual/commissioning_curvature.hpp)          |
| `robot::Odometry`                  | `update()`、`requestReset()`          | [`odometry.hpp`](include/robot/odometry/odometry.hpp)                                      |
| `robot::EngineeringPid`            | `update()`、`reset()`                 | [`engineering_pid.hpp`](include/robot/control/engineering_pid.hpp)                         |
| `robot::ChassisVelocityController` | `update()`、`reset()`                 | [`chassis_velocity_controller.hpp`](include/robot/control/chassis_velocity_controller.hpp) |
| `robot::calculateFeedforward()`    | 计算 `kS + kV*v + kA*a`                | [`feedforward.hpp`](include/robot/control/feedforward.hpp)                                 |

### 3.5 自动运动与线路

| API                              | 用途                               | 定义位置                                                                      |
| -------------------------------- | -------------------------------- | ------------------------------------------------------------------------- |
| `DriveDistanceCommand`           | 前进/后退指定距离                        | [`motion_commands.hpp`](include/robot/autonomy/motion_commands.hpp)       |
| `TurnToHeadingCommand`           | 转到目标航向                           | 同上                                                                        |
| `DriveArcCommand`                | 按圆弧运动                            | 同上                                                                        |
| `DriveToPoseCommand`             | 驱动到目标位姿                          | 同上                                                                        |
| `SequentialCommandGroup`         | 顺序执行命令                           | [`command_groups.hpp`](include/robot/commands/command_groups.hpp)         |
| `ParallelCommandGroup`           | 等所有子命令完成                         | 同上                                                                        |
| `RaceCommandGroup`               | 任一子命令结束即结束                       | 同上                                                                        |
| `DeadlineCommandGroup`           | deadline 完成即结束                   | 同上                                                                        |
| `WaitCommand` / `TimeoutCommand` | 非阻塞等待/超时装饰                       | 同上                                                                        |
| `FixedTrajectoryGenerator`       | 从固定容量路径点生成时间轨迹                   | [`trajectory.hpp`](include/robot/autonomy/trajectory.hpp)                 |
| `TrajectoryTracker`              | 跟踪轨迹并发布 `ChassisVelocityPayload` | [`trajectory_tracker.hpp`](include/robot/autonomy/trajectory_tracker.hpp) |
| `AutonomousRouteRegistry`        | 将稳定 `RouteId` 解析为命令工厂            | [`route_registry.hpp`](include/robot/autonomy/route_registry.hpp)         |

所有自动命令都必须是非阻塞状态机，只发布 `DriveRequest`，不能直接访问马达。

### 3.6 HMI 路线和参数

| API                       | 常用方法                                    | 定义位置                                                                       |
| ------------------------- | --------------------------------------- | -------------------------------------------------------------------------- |
| `RouteRegistry<N>`        | `valid()`、`find()`、`doNothing()`        | [`registry.hpp`](include/robot/hmi/registry.hpp)                           |
| `SelectionManager<N>`     | `stage()`、`confirm()`、`lockForEnable()` | [`selection_manager.hpp`](include/robot/hmi/selection_manager.hpp)         |
| `ParameterRegistry<N>`    | `valid()`、`find()`                      | [`registry.hpp`](include/robot/hmi/registry.hpp)                           |
| `ParameterTransaction<N>` | `stage()`、`apply()`、`rollback()`        | [`parameter_transaction.hpp`](include/robot/hmi/parameter_transaction.hpp) |

路线选择必须遵守 `Draft -> Confirmed -> Locked`。启用边沿只执行锁定快照；未确认、ID 无效、未实现、未批准或质量不足时自动回退 `DoNothing`。

## 4. 添加、更换和选择自动线路

### 4.1 当前限制

当前生产 [`autonomous()`](src/main.cpp) 固定显示并执行 `DO NOTHING`，生产组合根尚未接入实际 `RouteRegistry`。完整线路注册示例目前在 [`tests/test_command_groups_routes.cpp`](tests/test_command_groups_routes.cpp)，它是测试示例，不是比赛线路表。

`RobotConfig::selected_route` 也不是绕过审批的开关：当 `competition_routes == false` 时，任何非 `DoNothing` 的选择都会被 `validateConfig()` 拒绝。

### 4.2 新增一条线路

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

### 4.3 比赛中更换已批准线路

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

## 5. 常量与调参位置

### 5.1 当前真正生效的基础配置

| 参数组  | 字段                                                                                   | 修改位置                                                                              | 当前状态/来源                                                                |
| ---- | ------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| 身份   | `robot_id`、schema、revision                                                           | [`robot_config.hpp`](include/robot/config/robot_config.hpp)                       | 当前样机身份与修订仅在配置源码和硬件清单维护                                                 |
| 马达   | port、`reversed`、`cartridge_rpm`、逐马达 `motor_rev_per_wheel_rev`                        | 同上                                                                                | 已按“前推后退”反馈翻转六个方向，仍需架空复测确认                                              |
| 传感器  | `ImuConfig::installed`、Rotation 配置                                                   | `HardwareConfig` / 同上                                                             | 外部传感器均未安装                                                              |
| 名义几何 | 轮径、名义轮距                                                                              | `GeometryConfig` / 同上                                                             | 2.75 in / SolidWorks 6 in，均未场地校准                                       |
| 校准几何 | 左右 `m_per_motor_rad`、有效轮距                                                            | `CalibrationConfig` / 同上                                                          | 待真机标定                                                                  |
| 控制周期 | `nominal_period_s`、数学 dt 范围                                                          | `RuntimeConfig` / 同上                                                              | 10 ms 是建议起点                                                            |
| TTL  | `request_ttl_us`、`output_ttl_us`                                                     | `RuntimeConfig` / 同上                                                              | 40 ms 是离线起点                                                            |
| 电压上限 | `max_command_voltage_V`                                                              | `ElectricalConfig` / 同上                                                           | 当前 commissioning 上限 12 V；不得超过 V5 12 V 命令边界                             |
| 试车驾驶 | `kCommissioningCurvatureInputMode`、`throttle_shape`、`curvature_gain`、`quick_turn_gain`、`quick_turn_enter/exit_throttle` | [`commissioning_curvature.hpp`](include/robot/manual/commissioning_curvature.hpp) | 默认左 Y 油门、右 X 转向的双摇杆 Curvature；可切回左 Y/X 单摇杆；低油门自动 Quick Turn |
| 试车加速 | 输入 rise/fall、`output_slew` rise/fall                                                 | 同上                                                                                | 激进候选 100/s、1200 V/s；名义一个 10 ms 控制帧内达到 12 V，待新一轮 HIL 复测                 |
| 能力开关 | `RobotCapabilities`                                                                  | `RobotConfig` / 同上                                                                | 当前全部锁定                                                                 |

[`config/hardware_profile.yaml`](config/hardware_profile.yaml) 中的 `null` 表示仍需实机或 CAD 数据，不应随意填入“看起来合理”的值。

### 5.2 运行时、安全和驾驶参数结构

| 参数类别            | 主要字段                                                      | 定义位置                                                                               |
| --------------- | --------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| 调度时序            | `period_ms`                                               | [`ControlLoopConfig`](include/robot/runtime/control_loop.hpp)                      |
| 启动自检            | `minimum_duration_us`、`sensor_timeout_us`（当前 3 s / 5 s）   | [`StartupSelfCheckConfig`](include/robot/platform/hardware_self_test.hpp)          |
| Controller 位姿刷新 | `kControllerHmiPeriodMs`（当前 100 ms，每次最多写一行）               | [`src/main.cpp`](src/main.cpp)                                                     |
| dt 与 deadline   | `nominal_period_s`、`min/max_math_dt_s`、`deadline_dt_s`    | [`TimingConfig`](include/robot/runtime/timing_monitor.hpp)                         |
| 请求仲裁            | `max_request_ttl_us`                                      | [`DriveRequestArbiterConfig`](include/robot/commands/drive_request_arbiter.hpp)    |
| 输出看门狗           | `frame_ttl_us`、`max_voltage_V`、`stale_stop_mode`          | [`OutputServiceConfig`](include/robot/drive/output_service.hpp)                    |
| 安全门             | 电压上限、请求 TTL、输出 Slew、三种停车模式                                | [`SafetyGateConfig`](include/robot/drive/safety_gate.hpp)                          |
| 输出 Slew         | `rise_V_per_s`、`fall_V_per_s`、`max_dt_s`                  | [`OutputSlewConfig`](include/robot/drive/output_slew.hpp)                          |
| 摇杆曲线            | `center_offset`、`deadband`、`cubic_weight`                 | [`AxisShapeConfig`](include/robot/manual/input_shaping.hpp)                        |
| 手动驾驶            | 轴映射、输入 Slew、曲率、Quick Turn、TTL                             | [`ManualDriveConfig`](include/robot/manual/manual_drive.hpp)                       |
| 样机试车            | 可切换左 Y/X 单摇杆或左 Y + 右 X 双摇杆 Curvature、低油门自动 Quick Turn、B 按住 Coast、12 V、Slew、TTL | [`CommissioningCurvatureConfig`](include/robot/manual/commissioning_curvature.hpp) |
| 航向辅助            | 进入/退出门、`kP/kD`、最大修正、质量要求                                  | [`HeadingAssistConfig`](include/robot/manual/heading_assist.hpp)                   |
| 传感器校验           | 范围、变化率、年龄、冻结、恢复样本                                         | [`SensorValidatorConfig`](include/robot/sensors/sensor_validator.hpp)              |
| 里程计             | 布局、轮比例、轮距、tracking 外参、滑移确认                                | [`OdometryConfig`](include/robot/odometry/odometry.hpp)                            |
| 电机保护            | 温度、电流、卡死进入/恢复阈值                                           | [`MotorProtectionConfig`](include/robot/health/motor_protection.hpp)               |
| 故障规则            | 严重度、进入/恢复延时、降额、锁存                                         | [`FaultRuleConfig`](include/robot/health/fault_manager.hpp)                        |

### 5.3 自动控制参数结构

| 参数类别  | 主要字段                                     | 定义位置                                                                                       |
| ----- | ---------------------------------------- | ------------------------------------------------------------------------------------------ |
| PID   | `kp/ki/kd`、积分区/钳位、D 低通、死区、输出和 dt 范围      | [`PidConfig`](include/robot/control/engineering_pid.hpp)                                   |
| 前馈    | 分侧/正反 `kS`、`kV`、`kA`                     | [`DirectionalFeedforwardGains`](include/robot/control/feedforward.hpp)                     |
| 轮速闭环  | 轮距、最大轮速/加速度/电压、状态年龄、左右 PID/前馈            | [`ChassisVelocityControllerConfig`](include/robot/control/chassis_velocity_controller.hpp) |
| 自动原语  | 线/角运动约束、误差增益、降级比例、TTL                    | [`MotionPrimitiveConfig`](include/robot/autonomy/motion_commands.hpp)                      |
| 结束条件  | error/velocity band、settle、timeout、stall | [`MotionTerminationConfig`](include/robot/control/termination.hpp)                         |
| 轨迹生成  | 速度、加减速度、角速度、抓地、电压、轮距、采样间隔                | [`TrajectoryConstraints`](include/robot/autonomy/trajectory.hpp)                           |
| 轨迹跟踪  | 纵横/航向增益、偏差门、曲率、降级/滑移比例                   | [`TrajectoryTrackerConfig`](include/robot/autonomy/trajectory_tracker.hpp)                 |
| 自动安全  | 降级比例、最大偏差、宽限时间、状态年龄、TTL                  | [`AutonomySafetyConfig`](include/robot/autonomy/safety_supervisor.hpp)                     |
| SysId | ramp、dynamic step、最大电压/时间/距离、TTL         | [`CharacterizationConfig`](include/robot/calibration/characterization_runner.hpp)          |
| 路线解析  | 起始位置/航向容差                                | [`RouteResolutionConfig`](include/robot/autonomy/route_registry.hpp)                       |

这些头文件定义的是参数**结构和合法范围**，不代表已经存在生产参数实例。当前许多数值只出现在 `tests/` 中用于验证算法；修改测试值不会调节真机。生产参数应在组合根中集中构造，并通过 `ParameterRegistry`、revision 和校准记录管理。

## 6. 调参原则

1. 先确认端口、方向、单位和机械无干涉，再调控制参数。
2. 先完成轮径/轮距和传感器质量校准，再调自动控制。
3. 先做分侧、正反 SysId，再调轮速 PID。
4. 每次修改绑定 `robot_id`、config/calibration revision、commit 和日志数据集。
5. 参数训练数据与最终验证数据分开。
6. 测试中的示例值、CAD 值和社区默认值不能直接标记为比赛参数。
7. `Apply` 只改变本次运行值，`Save` 才持久化；两者都不会自动获得 `CompetitionApproved`。

真机调试的完整顺序、记录要求和 capability 解锁门见 [`docs/HARDWARE_COMMISSIONING.md`](docs/HARDWARE_COMMISSIONING.md)。

## 7. 不可破坏的安全规则

- 全工程只有 `OutputService` 可以写驱动马达；
- Command、自动线路、HMI 和测试不能直接持有或写 `pros::Motor`；
- 自动逻辑不得使用阻塞运动 `while` 循环；
- 不得使用 `12 / battery_V` 放大物理电压请求；
- 有意停车和陈旧帧停车必须走 `stop()`/`brake()`，不能用 `move_voltage(0)` 代替；
- 所有跨任务请求和帧必须带 timestamp、sequence、mode epoch 和 TTL；
- 真机未验证前，相关 capability 保持 `false`，自动选择保持 `DoNothing`。
