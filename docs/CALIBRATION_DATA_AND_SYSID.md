# 74000M 校准数据、SysId 与调校执行说明

本文件对应计划板块 12。当前机器未完工，因此这里只冻结数据格式、拟合工具、
试验状态机和证据要求；所有机械、HIL、场地结论均为 **NOT TESTED**，不得把测试
夹具中的合成系数写入 `config/hardware_profile.yaml`。

## 参数身份与状态

每个候选参数必须同时记录：schema、calibration revision、robot ID hash、软件
commit、训练数据集、独立验证数据集、工况 hash、来源和验证等级。训练/验证数据
ID 必须不同。新拟合只能生成 `Draft`；没有真机证据不得升级到
`BenchValidated/FieldValidated/CompetitionApproved`。

建议每次数据集目录包含：

```text
manifest.json       机器人、软件、机械、场地、电池、温度和试验限制
frames.bin          定长 LogFrame 或可校验导出
exclusions.csv      被排除样本、固定原因位和人工备注
fit_result.json     系数、单位、训练/验证指标、残差和工具版本
approval.md         PASS/FAIL/REPEAT/NOT TESTED、审批和回滚 revision
```

## 真机执行顺序

1. 按 `HARDWARE_COMMISSIONING.md` 完成机械、端口、方向、单位和唯一写者验收。
2. 正反长直线分别拟合左右距离比例，保留未参与拟合的距离/速度作为验证集。
3. CW/CCW 多圈拟合有效轮距；tracking wheel 先校距离比例，再拟合 offset。
4. 用独立批次执行 CW/CCW UMBmark；不以一次闭合替代统计。
5. 分侧生成正/反 quasistatic 与 dynamic 数据。所有电压必须走
   `Test request → Arbiter → SafetyGate → OutputService`。
6. 固定规则排除无效、陈旧、饱和、降额、滑移、碰撞、超期和不完整样本。
7. 拟合 `V = kS sign(v) + kV v + kA a`，分别报告训练/验证 RMSE、MAE、
   最大残差、排除率和左右/正反分组结果。
8. 再调抓地/Slew、输入曲线、Quick Turn 和航向 PD，最后做热态/低电复验。

## 当前离线证据

| 项目 | 当前状态 | 真机缺口 |
|---|---|---|
| 固定容量数据集与线性回归 | PCValidated | 用本机日志复验数值稳定性 |
| 距离/轮距/tracking offset 符号 | PCValidated | 正反/CW/CCW 独立基准 |
| 三参数前馈拟合与验证集指标 | PCValidated | 分侧正反 QS/Dynamic 数据 |
| 非阻塞 Test 请求状态机 | PCValidated | HIL 急停、距离/时间边界 |
| 机械、几何、SysId、抓地和驾驶参数 | NOT TESTED | 完整真机 SOP |
| 参数 FieldValidated/CompetitionApproved | NOT TESTED | 场地矩阵、审批与回滚 |

物理电压命令禁止使用 `12 / battery_V` 放大；电池只作为工况、裕量、饱和和
降额证据记录。
