# 可视化测试切片：启动与操作

这是现有 C++ 核心的工程 viewer，用于检查操纵、气动／结构损伤与遥测。它不是完成的游戏。只有 C++ 积分飞行状态；Python 发送输入请求并绘制返回状态，不设置飞机姿态、速度或额外力矩。

## 最小启动

以下命令在仓库根目录、使用 bash 的桌面环境执行。需要 C++20 GCC、GNU make、Python 3 和该 Python 的 Tk 绑定。Matplotlib 安装在仓库局部虚拟环境，环境目录已忽略，不修改全局 Python：

```sh
make -j2 build/flight-live
python3 -m venv .venv-viewer
.venv-viewer/bin/python -m pip install --no-cache-dir -r requirements-viewer.txt
.venv-viewer/bin/python -c 'import tkinter; print("Tk", tkinter.TkVersion)'
MPLBACKEND=TkAgg .venv-viewer/bin/python tools/viewer.py
```

缺少 `tkinter` 时，需要使用带 Tk 的 Python 或自行配置系统 Tk；pip 的 Matplotlib 依赖不会安装系统 Tk。Linux GUI 还需要桌面显示连接。本次交付明确使用 TkAgg，因为其 FocusOut 接口已接入；其他交互 backend 未作为交付入口。退出时关闭窗口，C++ 子进程随之关闭。

Cloud 无桌面时可运行：

```sh
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

左图显示当前姿态、附着 cells 和抽样局部气动力箭头；红色表示丢失气动面积，橙色表示局部失速。它围绕当前 CG 显示，没有地形／追尾相机，也不绘制脱离部件。气动力箭头每帧归一化，仅表示方向和当帧相对大小。

右下显示结构连接的实际等效应力／极限强度，虚线为 1，横轴显示截断于 3；超过 1 并不代表瞬时断裂，塑性应变和连通性决定后续结果。HUD 显示模拟时间、空速、高度、G、实际／目标 RPM、桨距、限制状态、tip Mach、实际舵面、trim 请求、逐轴 direct 权重和脱离部件数。暂停时 HUD 的 ownership 是上一个物理 tick 的状态，恢复后的物理 tick 才开始交还。

默认 `validation-bem.airframe` 是未标定的合成验证体。可用 `--controller legacy` / `--controller off` 比较同一推进模型上的 guidance；`--airframe validation.airframe` 使用保留的旧推进模型，此时 G/F7 无调速器作用。旧模型三个失败反例仍见 `tools/propulsion_experiments.py`。

## 建议人工检查

1. 从干净配平开始，点击不同 heading/elevation；观察实际速度方向、舵面与能量变化，记录延迟、过冲和手感，不假设所有轨迹请求可达。
2. 鼠标 aim 保持不变，单轴键盘接管、同时按相反键、松开交还；检查其他轴仍由 Instructor 操作。
3. 按住操纵键时暂停、重置和 Alt-Tab，松开旧键再恢复；确认无粘键、重复切换或追赶突跳。检查 Tk 的平台按键重复、PgUp/PgDn 和焦点行为。
4. 调整油门、三个 trim 与目标 RPM，再分别注入 F2/F3/F5/F6/F7，观察真实剩余能力。F4 不保证一次低速拉杆就断翼；必须由载荷和渐进失效产生结果。
5. 记录实际显示 backend、键盘布局、是否能持续实时播放，以及操纵响应是否便于理解。不要把离线图和自动断言当作人工手感结论。

## spec 覆盖与剩余核心工作

完整逐项状态由[当前实施计划](plans/codex-cloud-first-iteration.md#original-acceptance-mapping)维护；[当前决策](decisions/current-decisions.md)维护来源、实验与假设。原 51 节 spec 保持不变。

已实现并有自动证据：6DoF／120 Hz、152 个分布 cells／局部失速、逐轴仲裁与实际受损舵面、配平、独立气动／结构／系统拓扑、截面／刚度／强度导致的重分配与延迟失效、连通性分离与质量／CG／惯量变化、有符号桨载荷／风车启动／有限桨距调速、耦合转子力学和诊断 transport。当前 viewer 的输入生命周期、固定 tick 和 Agg 渲染已测试。

尚缺或仍为 reduced／未标定模型：局部材料去除的质量账本、真实跨翼展梁／剪切／轴向载荷与材料断裂标定、脱离部件气动／控制链分配、动态失速／下洗／襟翼、桨叶压缩性／tip／swirl／Re 修正、历史飞机数据、顺桨及更完整发动机／系统损坏。碰撞、弹道／爆炸／火灾、多机型、成品天空地形／镜头和真人 GUI 手感不属于已完成结果。

这是待审阅里程碑。本轮到此停住，不自动扩展完整游戏；下一步真正需要人工的是上述桌面操控和显示检查。
