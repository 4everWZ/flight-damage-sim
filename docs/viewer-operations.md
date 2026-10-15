# 可视化测试切片：启动与操作

这是现有 C++ 核心的工程 viewer，用于检查操纵、气动／结构损伤与遥测。它不是完成的游戏。只有 C++ 积分飞行状态；Python 发送输入请求并绘制返回状态，不设置飞机姿态、速度或额外力矩。

## 最小启动

### Windows PowerShell

在仓库根目录执行。需要 PATH 中有支持 C++20 的 `g++`，以及可创建虚拟环境、带 Tk 的 Python。以下命令只构建 viewer 所需的 C++ 程序，不依赖 `make`，也不激活或修改全局 Python 环境：

```powershell
New-Item -ItemType Directory -Force build, results | Out-Null
g++ -std=c++20 -O2 -g -Wall -Wextra -Wpedantic -Iinclude src/core.cpp src/propeller.cpp src/live.cpp -o build/flight-live.exe
# 确认编译成功后继续；PowerShell 不会自动在外部命令失败时停止。
python -m venv .venv-viewer
.venv-viewer/Scripts/python.exe -m pip install --no-cache-dir -r requirements-viewer.txt
.venv-viewer/Scripts/python.exe -c "import tkinter; print('Tk', tkinter.TkVersion)"
.venv-viewer/Scripts/python.exe tools/viewer.py --smoke
$env:MPLBACKEND = 'TkAgg'
.venv-viewer/Scripts/python.exe tools/viewer.py
```

每条命令成功后再执行下一条；冒烟测试应输出 `PASS`，并生成下文的 JSON 和 PNG。已有 `.venv-viewer` 时可跳过创建；依赖已安装时可跳过安装。若 `python3` 在 Windows 上指向应用商店别名，使用这里的 `python` 和虚拟环境解释器路径。

### bash / Linux 桌面

以下命令在仓库根目录、使用 bash 的桌面环境执行。需要 C++20 GCC、GNU make、Python 3 和该 Python 的 Tk 绑定。Matplotlib 安装在仓库局部虚拟环境，环境目录已忽略，不修改全局 Python：

```sh
make -j2 build/flight-live
python3 -m venv .venv-viewer
.venv-viewer/bin/python -m pip install --no-cache-dir -r requirements-viewer.txt
.venv-viewer/bin/python -c 'import tkinter; print("Tk", tkinter.TkVersion)'
MPLBACKEND=TkAgg .venv-viewer/bin/python tools/viewer.py
```

缺少 `tkinter` 时，需要使用带 Tk 的 Python 或自行配置系统 Tk；pip 的 Matplotlib 依赖不会安装系统 Tk。Linux GUI 还需要桌面显示连接。本次交付明确使用 TkAgg，因为其 FocusOut 接口已接入；其他交互 backend 未作为交付入口。退出时关闭窗口，刷新定时器与待处理按键回调停止，C++ 子进程随之关闭。若子进程在 0.5 秒内不响应退出，先 terminate，再在必要时 kill，并回收进程和通信管道。清理只针对 viewer 自己启动的核心进程。

### 播放性能与退出检查

viewer 复用 cells、力箭头、航迹点与结构条形对象；TkAgg 缓存静态背景，仅重绘动态对象。窗口缩放或相机变化导致完整重绘时会刷新缓存。HUD 的 `FPS` 显示相邻刷新回调的实际频率；刷新以约 30 FPS 为目标，并扣除本帧执行时间，不在绘制结束后额外等待完整 33 ms。物理仍为 120 Hz，不以减少物理 tick 或 cells 换取 FPS。

依赖上限现在是 `matplotlib<3.11`：本机 3.11.2 的 Agg 逐字形文字绘制对长遥测 HUD 开销显著较高。已有环境也应重新执行安装命令，以应用此约束。

2026-10-15 Windows 本机定向测量：旧路径重建图表约 47.9 ms、完整渲染约 423.0 ms，四个物理 tick 加通信约 7.5 ms。复用对象、局部重绘、静态帮助文字与 Matplotlib 3.10.9 后，Agg 分项中位数约 2.0 ms 更新、38.7 ms 渲染、7.0 ms 四 tick 通信／积分。原生 Tk 短运行含缩放与实际调度约 13.8 FPS；这是本机自动检查，不是所有硬件达到 30 FPS 或人工手感通过的保证。原生窗口关闭后 Python 测试入口正常退出、核心进程回收、定时器停止。

退出回归可在仓库根目录执行；第二条需要可用的桌面 Tk，会短暂创建测试窗口：

```powershell
.venv-viewer/Scripts/python.exe tests/live_client_tests.py
.venv-viewer/Scripts/python.exe tests/viewer_gui_tests.py
```

Linux 对应使用 `.venv-viewer/bin/python`。前者涵盖正常退出、重复清理、核心已退出，以及子进程不响应退出命令的强制终止；后者运行真实 Tk 定时器和缩放／重绘，再关窗检查核心退出和定时器停止。VS Code 的 Python 语言服务属于编辑器进程，不由 viewer 关闭。

Cloud 无桌面时可运行：

```sh
mkdir -p results
.venv-viewer/bin/python tools/viewer.py --smoke
```

生成 `results/viewer-input-summary.json` 和 `results/viewer-smoke.png`，后者标注 **OFFLINE AGG INPUT TEST · HUMAN GUI NOT VERIFIED**。图像是自动事件测试后的离线渲染，不代表真人操控通过。CI 同时把这两个文件保存为 `viewer-slice-evidence`，可在相应 Actions 运行页面下载；生成结果不提交到源码仓库。

## 操作

先点击画布获取键盘焦点。在右上角 heading/elevation 面板左键点击以设置鼠标 aim。heading 0° 表示世界北向，90° 表示东向；elevation 请求空中相对气流的飞行轨迹角，不是机头角或高度锁定。请求仍受实际推力、升力、失速和剩余控制能力约束。

| 输入 | 操作 |
|---|---|
| ← / → | 左／右滚转请求 |
| ↑ / ↓ | 抬头／低头请求 |
| Q / E | 左／右偏航请求 |
| W / S | 增／减油门请求，范围 0–1 |
| I / K | 增／减俯仰配平请求 |
| J / L | 减／增滚转配平请求 |
| U / O | 减／增偏航配平请求 |
| PgUp / PgDn | 增／减目标桨转速比，范围 0.5–1.1；不会直接设置实际 RPM |
| G | 恒速调速器开／关；关闭时保持当前桨距 |
| M | flightpath → legacy → off 控制器比较 |
| Space | 暂停／继续 |
| R | 清除损伤并重新初始化配平；保持所选控制器、调速器开关和暂停状态 |
| F2 | 去掉左外翼 8/32 个 cell 的气动面积，保留结构连接 |
| F3 | 断开左副翼控制链 |
| F4 | 左根部前梁剩余截面设为 30%，由真实载荷推进失效 |
| F5 | 断开升降舵控制链 |
| F6 | 发动机可用功率降至 50% |
| F7 | 卡死当前桨距，调速器不能绕过卡死 |

配平值是归一化输入，viewer 限制为 ±0.9；HUD 显示请求值，实际舵面角还受真实控制链、压力／行程／铰链与损伤限制。键盘只接管当前按住的滚转／俯仰／偏航轴。相反按键同时按下仍接管该轴，净请求为零；松开后按核心的 0.25 秒平滑交还给 Instructor。油门、配平和 RPM 按键不接管这三个操纵轴。

失去画布键盘焦点、Alt-Tab 或指针移出画布会清理 held input 并暂停。恢复时先松开旧按键，点击画布，再按 Space；操纵键需要松开后重新按，避免旧 held input 在 reset/pause 后继续作用。若失焦期间丢失了 key-release 事件，按下再松开该键一次即可解除其保护锁存。暂停冻结物理 tick；按键损伤／控制器／重置命令仍可在当前 tick 边界应用。暂停和重置清除时间余额；恢复不补跑暂停时长。慢渲染时每帧最多 12 个固定 tick，墙钟播放会变慢，不改变 1/120 秒物理步，也不积累长时间追赶队列。

所有 Matplotlib 默认快捷键均已禁用，避免 Q 退出、S 保存、G 网格、箭头浏览和配平键缩放的冲突。窗口工具栏的鼠标工具仍可用，但可能改变画布焦点；恢复操作按上段执行。

## 看哪些遥测

左图显示当前姿态、附着 cells 和抽样局部气动力箭头；红色表示丢失气动面积，橙色表示局部失速。它围绕当前 CG 显示，没有地形／追尾相机，也不绘制脱离部件。绘图轴采用右手 ENU（x 东、y 北、z 上），由核心 NED（北、东、下）转换，避免原“北／东／上”排列造成的镜像。Q 请求机头左偏航，E 请求机头右偏航；这是相对飞机自身的方向，不是固定的屏幕左右。气动力箭头每帧归一化，仅表示方向和当帧相对大小。

右上红色 + 是 aim，蓝色圆点是由核心速度减去风得到的实际空中航迹；图例显示实际 heading/elevation 数值。超出面板范围时圆点会被裁切，数值仍显示。它不以机头姿态代替速度方向。右下显示结构连接的实际等效应力／极限强度，虚线为 1，横轴显示截断于 3；超过 1 并不代表瞬时断裂，塑性应变和连通性决定后续结果。HUD 显示模拟时间、空速、高度、G、实际／目标 RPM、桨距、限制状态、tip Mach、实际舵面、trim 请求、逐轴 direct 权重和脱离部件数。暂停时 HUD 的 ownership 是上一个物理 tick 的状态，恢复后的物理 tick 才开始交还。

默认 `validation-bem.airframe` 是未标定的合成验证体。可用 `--controller legacy` / `--controller off` 比较同一推进模型上的 guidance；`--airframe validation.airframe` 使用保留的旧推进模型，此时 G/F7 无调速器作用。旧模型三个失败反例仍见 `tools/propulsion_experiments.py`。

## 建议人工检查

默认使用 `validation-bem.airframe` 和 `flightpath` 控制器。先完成冒烟测试，再按下表检查；各损伤项之间按 R 重置，避免多个损伤混合导致难以判断。若当前暂停，点击画布后按 Space 继续。

| 检查 | 操作 | 预期结果／需要记录 |
|---|---|---|
| 干净配平 | 启动后不输入，观察约 10 秒模拟时间 | 高度、空速接近初值；记录明显漂移、抖动或错误信息 |
| 鼠标航迹 | 在右上面板点击 heading 90°、elevation 0° 附近，观察约 30 秒模拟时间 | 蓝点逐渐跟随红色 aim，实际舵面变化；记录响应延迟、过冲、空速和高度变化。允许暂时掉高，这不是高度锁定 |
| 逐轴接管 | 保持 aim，按住左箭头，再同时按右箭头，随后全部松开；再试俯仰和 Q/E | 只有对应轴的 direct 权重变为 1；相反键同时按时净请求为零但仍接管；松开后约 0.25 秒交还 |
| 焦点／暂停／重置 | 按住操纵键时分别试 Space、R、Alt-Tab 和指针移出，再松键、点击画布、恢复 | 无粘键；失焦暂停；暂停期间模拟时间冻结，恢复无补跑突跳；R 清损伤并保持控制器／调速器／暂停选择 |
| 连续键／切换键 | 长按 W/S、配平和 PgUp/PgDn；另长按 G、M | 连续请求受范围限制；G/M 每次实际按下只切换一次。记录键盘布局、重复或漏键 |
| 气动／控制损伤 | 分别重置后试 F2、F3、F5，并与干净状态执行相同输入 | F2 左外翼出现红色缺损且不直接脱落；F3/F5 降低相应实际控制能力，Instructor 不能绕过断链 |
| 发动机／桨距损伤 | 分别重置后试 F6、F7；F7 后改变油门或目标 RPM | F6 功率能力下降；F7 桨距保持卡死值，目标 RPM 仍可变化，实际 RPM 不保证跟随目标 |
| 结构弱化 | 重置后 F4，比较正常飞行与拉杆时的结构利用率、失效和脱离数 | 弱化由实际载荷推进；不保证一次低速拉杆就断翼。可重复的延迟失效由下文自动场景检查 |
| 播放性能 | 观察模拟时间与墙钟时间，调整窗口大小并持续操纵 | 记录能否接近实时、输入延迟及卡顿；慢渲染允许模拟变慢，物理步长保持不变 |

反馈可直接按这个格式发回，不必另建报告：`系统／Python 与 Matplotlib 版本／backend／键盘布局；检查项；操作顺序；预期与实际；是否可重现；模拟时间与关键 HUD 数值`。附截图或报错有助于定位。不要把离线图和自动断言当作人工手感结论。

## 完整自动回归（bash / GNU make 环境）

如需复核核心与模型证据，在已有 GCC、GNU make、Python 的 Linux／WSL 环境执行；viewer 使用该环境自己的 `.venv-viewer/bin/python`，不要与 Windows 虚拟环境混用：

```sh
python3 tools/experiments.py
make -j2 test
python3 tools/instructor_experiments.py
python3 tools/propulsion_experiments.py
python3 tools/rotor_experiments.py
.venv-viewer/bin/python tools/viewer.py --smoke
# 可选：绘制实验曲线
.venv-viewer/bin/python tools/plot_results.py
```

所有命令应正常退出；最新判据和数值分别见 `results/summary.json`、`instructor-summary.json`、`propulsion-summary.json`、`rotor-summary.json` 和 `viewer-input-summary.json`。旧推进模型的失败假设是保留的反例，不能要求其物理假设全部成立；入口脚本自身的验收失败才是回归异常。自动回归通过仍不代表完整 v0.1 或历史飞机真实性通过。

## spec 覆盖与剩余核心工作

完整逐项状态由[当前实施计划](plans/codex-cloud-first-iteration.md#original-acceptance-mapping)维护；[当前决策](decisions/current-decisions.md)维护来源、实验与假设。原 51 节 spec 保持不变。

已实现并有自动证据：6DoF／120 Hz、152 个分布 cells／局部失速、逐轴仲裁与实际受损舵面、配平、独立气动／结构／系统拓扑、截面／刚度／强度导致的重分配与延迟失效、连通性分离与质量／CG／惯量变化、有符号桨载荷／风车启动／有限桨距调速、耦合转子力学和诊断 transport。当前 viewer 的输入生命周期、固定 tick 和 Agg 渲染已测试。

尚缺或仍为 reduced／未标定模型：局部材料去除的质量账本、真实跨翼展梁／剪切／轴向载荷与材料断裂标定、脱离部件二次断裂／部件间气流、动态失速／下洗／襟翼、桨叶压缩性／tip／swirl／Re 修正、历史飞机数据、顺桨及更完整发动机／系统损坏。碰撞、弹道／爆炸／火灾、多机型、成品天空地形／镜头和真人 GUI 手感不属于已完成结果。

用户已在桌面修正并授权合并 PR #1，随后恢复有界核心迭代。上述人工操控检查仍独立于自动物理证据；继续核心研发不代表完整游戏或历史拟真已验收。

脱离部件现在按自身 cells／CG／速度／角速度计算气动力并积分，控制分支失去驾驶员连接，机械卡死保留或按中立回落；当前 viewer 仍只绘制母体。需要诊断时可读取 live JSON 的新增 `detached` 数组（各部件 pose、气动力／矩、舵面连接、cell 气流与载荷），或运行 `python tools/detachment_experiments.py` 并用 `plot_results.py` 查看 `detached-flight.png`。后者包含同初态弹道对照，属于离线实验，不是人工操控验收。

## 显式材料质量实验（依赖 PR #2）

`validation-material.airframe` 是同一 BEM 验证体的局部质量离散版本，初始总质量／CG／惯量保持一致。它把左外翼一个质量块拆成保留载体与两个命名材料元，没有新增机型。通过 `build/flight-live data/validation-material.airframe` 进入现有行协议后，可发送 `damage material left_outer_forward .2`：留下该元 20%，去除的 8 kg 按原刚体速度场形成单独碎料刚体；`materials` 返回库存，`detached[].material` 标识碎料来源，空名称表示原结构部件。`debris` 数量包含两者。重复同一剩余值不重复扣账，增大剩余值会被拒绝，R/reset 恢复初始定义。

该事件不自动改气动面积、梁截面或连接，也不施加爆炸冲量。当前没有新增材料 GUI 快捷键，viewer 也不绘制碎料几何。复现核心实验使用 `.venv-viewer/bin/python tools/material_experiments.py`（Windows 对应虚拟环境的 `Scripts/python.exe`），结果为 `results/material-summary.json`。质量／动量／瞬时能量与固定配平响应有通过证据；强滚转闭环的小材料效应在 120/240 Hz 的 10% 一致性假设失败，报告会保留此限制。
