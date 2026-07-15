# 自动路线编排与注册（离线）

状态：组合器、机构请求与 RouteId 解析 `PCValidated`；比赛路线 capability 仍关闭。

## 固定容量组合器

- Sequential：前一命令完成后才初始化下一命令；
- Parallel：所有命令完成才完成，子命令 requirement 必须互斥；
- Race：第一个结束的子命令确定结果，其余 Interrupted；
- Deadline：deadline 完成即结束，其余 Interrupted；
- Wait：无 subsystem requirement 的非阻塞计时命令；
- Timeout：装饰单个命令，超时确定失败并结束子命令；
- Conditional：只在 initialize 读取一次条件，启用后分支不可变。

组合器把 Scheduler 分配给组的同一 owner lease 传给子命令，组的 requirements 是子命令并集；因此并行子命令不能产生第二个 drivetrain writer。所有数组固定容量，无动态分配或阻塞等待。

## 机构与路线

`InstantMechanismCommand` 只发布带 mechanism ID、action、value、TTL、owner 和 epoch 的类型化请求，不接触机构硬件。

`AutonomousRouteRegistry` 通过稳定 RouteId 线性查找 descriptor 和 factory，不以数组下标执行函数。解析时同时验证：

- 锁定选择有效、未回退、mode epoch 与 registry revision 一致；
- 路线已实现且 competition approved；
- alliance/start 兼容；
- 当前质量满足要求；
- 当前起始位姿及锁定快照与 descriptor 一致；
- factory 存在并成功返回固定对象。

任一失败均返回注入的 `DoNothingCommand`；它持续发布 Brake。机器完成前没有路线能获得 CompetitionApproved，因此真实 `competition_routes=false`、selected route=`DoNothing` 保持不变。
