# 74000M 飞行记录与诊断系统 — 前端视觉系统设计规范

> 本文件是 PC 前端重设计的唯一视觉权威。实施者必须逐项落实，不得自行替换配色/字体/布局方向。
> 目标气质：成熟、昂贵、可信赖的企业级机器人调试与数据诊断工具；工业艺术来自结构、比例、
> 精度、材质层次与交互细节。**禁止**齿轮、警戒条、六边形、碳纤维、霓虹、金属拉丝、蓝紫渐变、
> 大圆角卡片、彩色投影等装饰符号。

## 1. 设计方向：深色工程仪器控制台

理由：专业机器人/测控软件（测功机台架、CAN 分析、示波器、数控面板）普遍采用深灰哑光控制台：
长时间实验室/赛场使用不刺眼；让数据（图表、读数、判定）成为画面中最亮的元素；
哑光分层 + 发丝线传达"仪器"而非"消费 App"；单一信号灯色建立克制的识别度。

整体观感参考：精密测量仪器的面板 —— 深色阳极氧化铝的层级、丝印发丝线、一盏琥珀色信号灯。

## 2. 色彩系统（QSS 令牌，ui_style.py 顶部定义）

表面层级（暖调石墨，避免冷蓝灰）：
- SHELL  `#14161A` 窗口最底层
- CANVAS `#191C21` 页面背景
- PANEL  `#1F232A` 卡片/面板/输入底
- RAISED `#262B33` 悬停、浮起元素、输入框底
- OVERLAY `#2D333D` 按下/激活
- LINE   `#32383F` 结构发丝线（1px）
- LINE_SOFT `#282D34` 次级分隔线（表格行线）

文字：
- TEXT      `#E8E9EC` 主文字
- TEXT_DIM  `#A3A9B2` 次要文字/说明
- TEXT_FAINT `#6B7280` 元信息/占位/禁用

强调色（信号琥珀 —— 仪器指示灯；全局唯一强调色，克制使用）：
- ACCENT       `#E8A33D` 主按钮底、激活导航、焦点框、图表主系列
- ACCENT_HOVER `#F2B45A`
- ACCENT_PRESS `#C9882C`
- ACCENT_TEXT  `#171A1E` 琥珀底上的文字（深色保证可读性）
- ACCENT_SOFT  `rgba(232,163,61,0.14)` 选中底色（QSS 用 #2E2A20 近似实心色）

语义色（判定/状态，绝不美化 FAIL/REPEAT/NOT TESTED）：
- OK   `#57B98A` PASS / 可用
- WARN `#D9A13B` CONDITIONAL PASS
- BAD  `#DE6E62` FAIL / REPEAT / 错误
- IDLE `#6B7280` NOT TESTED / 不可用 / 草稿
- 语义 chip 底色：对应色 12% 透明度的实心近似（OK:#223129 WARN:#332D1D BAD:#3A2624 IDLE:#262A30）

图表系列色（数据用色，与 UI 铬件分离）：
- S1 `#E8A33D`（主系列琥珀）S2 `#7EA8C9`（钢蓝）S3 `#57B98A` S4 `#DE6E62` S5 `#A3A9B2`
- 图表底 `#14161A`，网格 `rgba(255,255,255,0.06)`，轴文字 TEXT_FAINT

## 3. 字体与排版

- 族：`"Noto Sans SC","Microsoft YaHei UI","Microsoft YaHei","Segoe UI",sans-serif`（沿用现有加载逻辑）
- 数字/读数/JSON/日志：`"Cascadia Mono","Consolas",monospace`；指标数值一律等宽字体
- 字阶（weight 只用 400/500/600，禁止 800+）：
  - 11px 元信息、kicker（英文部分 letter-spacing 2px、大写）
  - 12px 说明文字（TEXT_DIM）
  - 13px 正文/控件（行高舒适）
  - 14px/600 小节标题
  - 17px/600 卡片标题
  - 22px/500 页面标题（克制，不用粗黑）
  - 26px/500 等宽 指标读数
  - 30px/500 首页主标题
- 中文排版：标签与控件垂直居中对齐；说明文字行距宽松；冒号后留半角空格

## 4. 空间与材质

- 8px 栅格：页面边距 32~40，卡片内边距 20~24，控件间距 12/16，表单行距 12
- 圆角：结构面板 0；按钮/输入框/chip 3px（不能再大）
- 无投影；层级靠底色明度差 + 1px 发丝线表达
- 所有分隔用 1px LINE/LINE_SOFT，不用 2px 以上粗线（强调条除外：2px ACCENT）

## 5. 控件语言

按钮：
- 主按钮：ACCENT 底 / ACCENT_TEXT 字 / 无边框 / 3px / min-height 32 / padding 6 18 / 13px 600；
  hover ACCENT_HOVER，pressed ACCENT_PRESS，disabled 底 #3A3F47 字 TEXT_FAINT
- 次按钮：透明底 / 1px LINE 边 / TEXT 字；hover 边 #49505A + 底 RAISED；pressed 底 OVERLAY
- 幽灵按钮（role="quiet"）：无边框 / TEXT_DIM；hover TEXT + 底 RAISED
- 危险不单独设红色按钮（本软件无破坏性操作）

输入（QLineEdit/QComboBox/QPlainTextEdit）：
- 底 PANEL / 1px LINE 边 / 3px / padding 6 10 / min-height 32；focus 边 ACCENT；
  placeholder TEXT_FAINT；下拉箭头区无边框、hover 底 RAISED；下拉列表 PANEL 底 LINE 边，
  选中项 ACCENT_SOFT 底 TEXT 字

表格（QTableWidget）：
- 底 CANVAS / 无外框或 1px LINE / 无网格线；行高 ~34；alternate `#1C2026`；
  行分隔 1px LINE_SOFT；表头 11px TEXT_FAINT、底线 1px LINE、左对齐、padding 8 10；
  行 hover 底 RAISED；选中行底 ACCENT_SOFT + 文字 TEXT（不反白）；
  完整性/严重性列用语义色文字（PASS→OK、CONDITIONAL→WARN、FAIL/REPEAT→BAD、其余→IDLE），
  由代码设置 item foreground，绝不把缺失值染成 OK

标签页（结果中心 QTabWidget）：
- 下划线式：tab 无底无边，TEXT_DIM 13px，padding 10 14；选中 TEXT 色 + 底部 2px ACCENT；
  pane 顶部 1px LINE

进度条：高 4px，轨 LINE_SOFT，chunk ACCENT，无文字；不确定模式即扫描感

滚动条：宽 10，透明轨，把 #3A414A，hover #4A525E，无箭头

QToolTip：RAISED 底 / TEXT / 1px LINE / padding 6 8

QMessageBox：PANEL 底

## 6. 全局框架

顶栏（高 52，PANEL 底，下 1px LINE）：
- 左：8×8 ACCENT 小方块（QLabel 固定尺寸底色）+ 间隔 10 + "74000M · 飞行记录与诊断"（13px 600 TEXT），
  整块可点击回首页（沿用现有 topBrand 行为）
- 右：离线徽章 chip（OK 语义：文字 "离线分析 · 不控制机器人"，11px，IDLE 底或 OK 浅底 + OK 字，
  前面一个小圆点 "●"）＋ 间隔 12 ＋ 版本元信息 "v0.4.0 · 离线版"（11px TEXT_FAINT）。
  **移除 "GPT-5.6" 字样**。

底栏状态条（高 26，SHELL 底，上 1px LINE_SOFT，11px TEXT_FAINT）：
- 左：数据目录真实路径（repo.settings.artifacts，程序运行时可得，不许编造）
- 右："PC 时间标记仅记录本地时间 · 不下发机器人"
- 用 QStatusBar 或底部小条实现；不得显示伪造的连接/机器人状态

## 7. 页面规格（业务流程不变，只重做视觉与布局）

### 7.1 首页 HomePage
- 左右两区：左栏宽 380~420，右区弹性
- 左栏（垂直居中偏上，padding 48）：
  kicker "VEX V5 · TF FLIGHT RECORD"（11px ACCENT，letterspacing）；
  主标题 "飞行记录与诊断系统"（30px/500 TEXT）；
  副述一行（12px TEXT_DIM）："面向机器人研发与工程测试的离线记录导入、完整性校验与数据诊断。"；
  下方 1px LINE；元信息表（11px）：版本 / 数据目录 / 分析引擎版本（均来自真实变量 version/settings，
  没有就不显示该行）
- 右区：三张纵向排列的操作卡（不是横向三列），每张高 ~96、全宽：
  自定义 QWidget（objectName "actionCard"，可点击，cursor Pointer）：
  左侧 2px 竖条（featured=ACCENT，其余 LINE）；内部左起：大号序号 "01"（20px 等宽 TEXT_FAINT）、
  标题（16px/600 TEXT）、描述（12px TEXT_DIM 一行）、右侧 "→"（16px TEXT_FAINT，hover 时 ACCENT）；
  hover：底 RAISED + 竖条变 ACCENT；featured 卡竖条恒 ACCENT
- 三卡文案与回调不变：新建 TF 记录（featured）/ 继续会话 / 历史记录
- 保留 `self.action_buttons` 列表属性（放三张卡），供测试断言

### 7.2 向导 SessionWizardPage
- 顶部：幽灵按钮 "← 返回首页" + 流程标题（22px）
- 主体左右分栏：左步骤轨宽 230（PANEL 底，右 1px LINE）：
  三个步骤块（序号等宽 12px TEXT_FAINT + 名称 13px）：active → 名称 TEXT + 左 2px ACCENT 竖条；
  未完成 → TEXT_FAINT；已完成 → TEXT_DIM。底部放一句 12px TEXT_FAINT 提示
  （"带 * 为必填；机器人身份以 TF 原始记录为准"）
- 右内容区：卡片化表单面板（PANEL + 1px LINE + padding 28）：
  步骤标题 17px/600 + 说明 12px TEXT_DIM + QFormLayout（标签 13px TEXT_DIM 右对齐，行距 14）；
  必填星号用 ACCENT 色
- 导入步骤：路径输入 + "选择目录…"（次）+ "扫描记录"（主）同行；扫描进度条其下；
  候选表格列不变；完整性列文字按语义着色
- 底部控制条（上 1px LINE，padding 12 0 0）：左 "上一步"（次），右 主按钮
  文案逻辑不变（"下一步" / "导入、分析并查看结果"）
- 保留全部现有属性名与校验逻辑（team/operator/test_type/...、start_new、continue_session、
  _set_step、next_step、import_and_analyze 等）；步骤数 3 不变

### 7.3 加载页 LoadingPage
- 垂直居中窄栏（max-width ~520）：
  kicker "OFFLINE PIPELINE"（11px ACCENT letterspacing）；
  动态标题（22px/500，保留 QTimer 打点动画与 start/stop API）；
  detail 12px TEXT_DIM；
  细进度条（4px 不确定）；
  静态工序清单一行（12px TEXT_FAINT，等宽）："完整性校验 → 只读归档 → 指标与图表分析 → LLM 信息包"
  （描述性文字，非实时进度，不构成造假）
- 保留 `self.timer` 属性与 isActive 语义

### 7.4 会话选择 SessionPickerPage（继续会话 / 历史记录）
- 顶行：幽灵 "← 返回首页" + 标题 22px；其下 description 12px TEXT_DIM
- 会话表格：列不变；行高 36；双击打开保留；选中行 ACCENT_SOFT
- 底行：空态提示（TEXT_FAINT）+ 右侧主按钮（文案模式逻辑不变）
- set_mode/refresh/open_selected 逻辑与文案不变

### 7.5 结果中心 ResultsPage
- 顶行：幽灵 "← 首页" + 会话标题（22px）+ summary（12px TEXT_DIM，同行或下一行）；
  右侧 "当前记录" 标签 TEXT_FAINT + 运行选择 Combo（min-width 280）+ "继续导入 TF 记录"（次按钮）
- QTabWidget 下划线式，6 个 tab 与顺序不变：运行总览/图表分析/完整性/LLM 信息包/运行对比/会话工具
- 运行总览 OverviewPage：
  指标网格 4 列（窄窗口允许 2 列换行）：每个指标 = 仪表读数卡（PANEL + 1px LINE，padding 14 16，
  上 label 11px TEXT_FAINT，下 value 22px 等宽 TEXT；"不可用" 用 IDLE 色且字重不变，不得显示 0）；
  完整性指标值按 verdict 语义着色（PASS→OK 等，CONDITIONAL PASS 不得渲染成 OK）；
  异常表列不变，"严重性" 列按语义着色；保留 "开始分析 / 重新生成" 主按钮于卡片行右上
- 图表分析 PlotsPage：
  工具行：运行 Combo（隐藏逻辑保留）、信号 Combo（min-width 240）、"打开静态图目录"（次）；
  pyqtgraph：背景 #14161A，网格 alpha 0.06，轴笔 LINE / 轴文字 TEXT_FAINT，
  主系列 ACCENT 宽 1.8；标题 12px TEXT_DIM；时间缺口保持断开（现状逻辑不动）
- 完整性 IntegrityPage：判定横幅（PANEL + 左 3px 语义条 + 语义色 20px/600 文案）+
  "原始完整性报告 · JSON" 小节标题 + 只读等宽文本（12px，PANEL 底）
- LLM 信息包 ReportPage：工具行按钮（"重新生成报告" 主 /"打开 Markdown" 次）+ 等宽只读文本
- 运行对比 ComparePage：多选列表（PANEL 底，选中 ACCENT_SOFT）+ 主按钮 + 等宽输出
- 会话工具 RecordWindowPage：小节标题 + 会话 Combo + 两个按钮（主 "记录 PC 开始时间" /
  次 "记录 PC 结束时间"）+ 状态卡（PANEL + 1px LINE，12px TEXT_DIM）；
  保留"只记录 PC 本地时间、不下发命令"说明文案

## 8. 必须保留的接口契约（GUI 测试与业务依赖）

- 窗口标题 `VEX V5 飞行记录与诊断系统`；`main()` 行为与 `STATUSMONITOR_SMOKE_TEST` 环境变量不变
- MainWindow 属性：`tabs/home_page/wizard_page/picker_page/results_page/loading_page` 及现有方法签名
- HomePage.action_buttons（3 项，顺序与回调不变；卡片需暴露标题文本供测试断言，
  例如 `card.title_text` 或子控件 objectName "actionCardTitle"）
- wizard_page.steps(QStackedWidget, count==3)、`next_button` 文案逻辑、`step_labels`、
  `show_candidates`、`table`（7 列、首列复选）等
- picker_page.title / open_button / set_mode；loading_page.timer；results_page.views.count()==6
- Worker/QThreadPool 后台执行模型、错误回退路径、去重（重复点击不并发冲突）一律不动
- VERDICT_LABELS / SESSION_STATUS_LABELS / DATASET_ROLE_LABELS / TEST_TYPE_ITEMS 语义不变

## 9. 数据真实性红线（实施时逐条自查）

- 缺失数据显示 "不可用"（IDLE 色），禁止显示 0；时间缺口不插值；FAIL/REPEAT/NOT TESTED 不隐藏
- CONDITIONAL PASS 用 WARN 色，绝不渲染成 OK
- 截图用合成数据必须经独立临时 STATUSMONITOR_HOME，禁止写入真实用户库
- 顶栏/底栏/首页元信息只显示程序内真实可得的值（版本、数据目录），禁止伪造机器人/连接状态
