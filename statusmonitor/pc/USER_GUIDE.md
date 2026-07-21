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

0.4 版切换为深色工程仪器界面：暖调石墨深色表面按窗口底、页面、卡片逐级分层，
全局只使用单一信号琥珀作为强调色；判定一律使用语义色——PASS 绿、
CONDITIONAL PASS 琥珀（绝不渲染为绿色）、FAIL/REPEAT 红、NOT TESTED/不可用
灰。首页仍只提供“新建 TF 记录”“继续会话”“历史记录”三个入口，业务流程与
之前一致。技术工具仅在选定会话后显示，界面始终只负责离线观察与分析，不会向
机器人发送控制指令。

## 2. 一次标准工作流

1. 在首页点击 **新建 TF 记录**，按顺序填写“会话信息”和“实验条件”。
2. 操作手在 Controller 长按 Y 至少 3 秒，松开后由 Brain 预检 TF 并开始记录；
   完成运动后长按 Y 至少 1 秒并松开停止。
3. 取出 microSD/TF 插入电脑，在向导第三步选择盘符并扫描；核对机器人 ID、
   storage sequence、帧数和完整性，取消不需要导入的记录。
4. 点击 **导入、分析并查看结果**。加载页会依次完成完整性检查、只读归档、
   指标/图表分析与 LLM 信息包生成，结束后自动打开结果中心。
5. 在结果中心查看 **运行总览、图表分析、完整性、LLM 信息包、运行对比**。
   `REPEAT/FAIL` 仍可用于诊断，但禁止批准性能参数。
6. 需要 PC 侧人工时间参考时，使用结果中心的 **会话工具**；它只记 PC 时间，
   不会命令机器人。
7. 要加入下一批数据，从首页点 **继续会话**；要复盘已完成数据，点 **历史记录**
   后选择会话并点击 **查看图表与信息**。
8. LLM 报告可打开 `report_for_llm.md`；把整个 `llm/` 目录和需要的
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
