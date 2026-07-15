# HMI 离线实现与待真机验收

状态：`PCValidated`（离线接口冻结），不代表 `HILValidated`、`FieldValidated` 或比赛批准。

## 已离线实现

- Brain 与 Controller 共用只读 `HmiModel` 快照；渲染层不读取电机、传感器或可变配置对象。
- 输入转换为带时间戳、序号和模式 epoch 的 `HmiEvent`。固定容量队列合并导航事件；关键事件不会静默覆盖，满队列时明确计数拒绝。
- 自动路线区分草稿、确认和使能锁定。未确认、未实现或未批准的路线在锁定时回退稳定 ID `DoNothing`。
- Brain 与 Controller 对自动选择、参数和维护域使用互斥编辑租约；租约在超时、epoch 变化或来源断开时失效。
- 参数注册表只公开 ID、单位、范围、步长、访问级别、应用策略和持久化策略，不向 UI 暴露配置字段地址。
- 参数编辑先暂存；只有 Disabled、非场控、Bench 解锁、静止、模型新鲜、租约/epoch/TTL/版本均有效时，才由后端一次性原子应用。应用失败不产生部分写入。
- Apply 与 Save 分离。持久化采用显式小端二进制格式、schema、机器人 ID、注册表 revision、generation、profile state 和 CRC32；双槽写入后完整读回校验，损坏时回退上一有效 generation。
- Controller 输出固定为 3 行、每行最多 19 个可见 ASCII 字符；Brain 布局约束为 480×240，导航触摸目标至少 40×36。

## 已执行的 PC 验证

`tests/test_hmi.cpp` 覆盖：

1. 路线草稿/确认/锁定与 `DoNothing` 回退；
2. 事件合并、关键事件保留和丢弃计数；
3. 双端编辑冲突、租约超时、epoch 与断开回收；
4. 场控连接、陈旧 revision、原子应用和应用策略合并；
5. 双槽 generation 选择、CRC 损坏回退和机器人 ID 隔离；
6. Controller 全页面定宽 ASCII 格式及 Brain 几何/触摸目标边界。

当前全套 PC 结果：100/100 通过。PROS GNU++17 固件编译通过。

## 机器完成后必须验收

| 验收项 | 真机证据 | 当前状态 | 解锁影响 |
|---|---|---|---|
| Brain 实际字体、颜色、截断和阳光下可读性 | 页面照片/录像及操作者检查表 | 未测试 | 不得标记 HMI HIL 通过 |
| 触摸坐标、边缘命中、去抖和长按确认 | 逐目标点击记录、误触/漏触统计 | 未测试 | 不得开放维护危险动作 |
| Controller 3×20 刷新与按钮导航 | 所有页面录像、按钮矩阵记录 | 未测试 | Controller 编辑入口保持锁定 |
| Brain/Controller 同时编辑与掉线恢复 | 双端竞争、断线、超时、epoch 切换日志 | 未测试 | 不得宣称无 split-brain |
| 场控连接和 Disabled/Enabled 权限矩阵 | Competition Switch/Field Control 状态日志 | 未测试 | 场控下调参和维护能力保持锁定 |
| SD 插拔、写满、断电和损坏恢复 | 双槽文件、CRC、generation、读回日志 | 未测试 | 持久化 profile 不得比赛批准 |
| HMI 任务耗时和队列压力 | p50/p95/p99、最大值、high-watermark、drop/reject | 未测试 | 不得提高显示刷新率或队列负载 |
| 参数应用后的控制器 reset/next-enable/reboot 语义 | 每个 ApplyPolicy 的状态与输出日志 | 未测试 | 参数只保持 Draft/Bench 候选 |

所有对应能力继续由 `config/hardware_profile.yaml` 的 capability 锁控制。详细上机顺序、记录字段和停止条件见 `docs/HARDWARE_COMMISSIONING.md`；不得用预设数据把上述“未测试”改写为通过。
