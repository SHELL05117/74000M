# 74000M VEX 飞行记录 PC 系统使用手册

> 程序与本文由 OpenAI GPT-5.6（Codex）创作。

## 1. 首次安装

在项目根目录打开 PowerShell：

```powershell
cd statusmonitor\pc
.\setup.ps1
.\run_statusmonitor.ps1
```

系统使用独立 `.venv`，不会修改机器人固件或比赛参数。若只使用命令行：

```powershell
.\.venv\Scripts\statusmonitor.exe doctor
```

`doctor` 必须显示 GUI、Parquet 和 V5L ABI 可用；磁盘空间不足时先清理 PC
归档盘，禁止删除 TF 卡上唯一一份尚未导入的数据。

### 界面说明

0.2 版采用已选定的 J 方案：暖白背景、瑞士网格、单一 VEX 红和编号侧栏。
红色只表示当前页面或主要操作，绿色只表示通过/健康状态。八个页面可直接点击，
也可用 `Ctrl+1` 至 `Ctrl+8` 切换。界面仅负责离线观察与分析，不会向机器人
发送控制指令。

## 2. 一次标准工作流

1. 在 **Home / Session** 新建会话，至少填写队号、操作员和测试类型。
2. 需要 PC 侧人工时间参考时，在 **Record Window** 点 Start。它只记 PC 时间，
   不会命令机器人。
3. 操作手在 Controller 长按 Y 至少 3 秒，松开后由 Brain 预检 TF 并开始记录；
   完成运动后长按 Y 至少 1 秒并松开停止。
4. 取出 microSD/TF 插入电脑，在 **Import** 选择盘符并扫描。
5. 核对机器人 ID、storage sequence、帧数和完整性，选择会话后导入。
6. **Integrity** 必须先查看。`REPEAT/FAIL` 仍可画诊断图，但禁止批准性能参数。
7. 在 **Overview** 点 Analyze。程序生成指标、异常、11 类静态图和 LLM 报告。
8. **Plots** 可按完整 Parquet 样本交互查看；缺口不会插值连接。
9. **Compare Runs** 只比较已分析运行，并同时保留 software/config/工况差异。
10. **Report** 查看或打开 `report_for_llm.md`，把整个 `llm/` 目录和需要的
    Parquet/原始文件交给 LLM，而不是复制数百万行 Markdown。

## 3. 产物解释

每次运行目录结构：

```text
artifacts/<date>/<team>/<robot>/<session>/<run>/
├── metadata.json                 # 人工身份 + 机器人真值身份
├── audit.jsonl                   # 导入、分析、报告操作审计
├── raw/
│   ├── imported_segment_*.v5l    # SHA-256 校验后的只读原始副本
│   └── original_hashes.json
├── integrity/
│   ├── integrity_report.json
│   └── usable_windows.json
├── derived/
│   ├── samples.parquet           # 完整逐帧列式数据
│   ├── metrics.json
│   ├── events.ndjson
│   └── analysis_manifest.json
├── plots/                        # 11 类 PNG
├── gui_summary.json
├── report_for_llm.md
└── llm/
    ├── data_dictionary.md
    ├── timeline.md
    ├── metrics.json
    ├── events.ndjson
    ├── evidence/*.csv
    └── artifact_manifest.json
```

- `.V5L`：正常 footer 关闭的机器人原始文件。
- `.TMP`：异常掉电/拔卡/未完成关闭的原始文件；系统只恢复到最后 CRC 正确块，
  完整性通常为 `REPEAT`。
- `samples.parquet`：逐帧完整信息的查询版本；不是原始真值替代品。
- `metrics.json`：确定性分析指标。
- `events.ndjson`：每行一个事件，便于程序和 LLM 流式读取。
- `report_for_llm.md`：全部身份、方法、限制、异常证据和 artifact 索引；不复制
  每一帧。

## 4. 结论边界

- 当前 schema 3.1 没有有效 pose 时，二维轨迹明确显示 `NOT AVAILABLE`。
- 当前控制栈没有 PID 分项时，不输出伪造的 PID 震荡/过冲结论。
- 没有校准的轮径/传动配置时，电机 `rad/s` 不会冒充底盘 `m/s`。
- SysId 需要物理电压、校准线速度、加速度、phase 标签以及训练/验证拆分；
  条件不足时只报告缺口。
- Welch/FFT 是频谱内容，不是闭环 FRF/Bode。
- Savitzky–Golay 离线导数和滤波明确标为 offline-only。
- 所有建议是 hypothesis；程序永不写入机器人参数，也不解锁 capability。

## 5. 命令行速查

```powershell
statusmonitor new-session --team 74000M --operator Alex --test-type manual
statusmonitor scan-media E:\
statusmonitor check E:\FLIGHT\1690X\R000001_T0000123456\DATA.V5L
statusmonitor import <session-id> E:\FLIGHT
statusmonitor analyze <run-id>
statusmonitor compare <run-id-1> <run-id-2>
statusmonitor report <run-id>
statusmonitor gui
```

命令退出码：`0` 成功；`2` 表示完整性未通过或 `doctor` 依赖不完整；`1` 表示
输入、I/O、数据库或程序错误。
