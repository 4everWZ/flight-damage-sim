# WWII Prop Combat Flight Sim Core — Specification v0.1

**Status:** Draft / Canonical Engineering Spec  
**Working name:** `FlightLab`  
**Primary platform:** PC  
**Engine:** Unity 6  
**Implementation language:** C#  
**Primary objective:** 建立可复用于多种二战螺旋桨战斗机的飞行、操作、气动损伤与结构损伤模拟核心。

---

# 1. Project Goal

本项目不是首先制作一个完整空战游戏，而是建立一个可以反复复用的 **WWII propeller aircraft simulation core**。

核心目标只有三个：

1. 建立足够可信、可扩展的二战螺旋桨飞机 Flight Model；
2. 建立类似 War Thunder Air RB 的 Mouse Aim + Keyboard Maneuver 操作体系；
3. 建立以真实物理后果为核心，而非整体 HP / 模块 HP 为核心的气动与结构损伤模型。

最终希望做到：

```text
Aircraft Definition
        ↓
Generic Prop Sim Core
        ↓
不同飞机表现出不同：
- 飞行性能
- 操纵特性
- 失速特性
- 高速操纵
- 发动机/螺旋桨行为
- 结构承载能力
- 损伤后飞行特性
```

新增飞机原则上应主要通过：

```text
aircraft data
aero tables
geometry
engine data
control geometry
structural definition
```

完成，而不是新增大量：

```text
if aircraft == Bf109
if aircraft == Spitfire
```

之类专用逻辑。

---

# 2. Core Design Principles

## 2.1 Simulation First

游戏表现必须来自 simulation state。

禁止使用以下机制作为核心模拟：

```text
transform.Rotate(...)
transform.position = ...
if wingHP <= 0:
    detachWing()
```

飞机姿态变化必须来自：

```text
forces
moments
mass
inertia
control surfaces
propulsion
aerodynamics
```

损伤必须通过改变上述系统的状态产生结果。

---

## 2.2 No Global Aircraft HP

飞机不存在：

```text
aircraftHP
wingHP
tailHP
```

作为主要死亡机制。

结构部件可以存在：

```text
remainingSection
remainingStiffness
yieldStrength
ultimateStrength
crack/damage state
```

但它们表示物理状态，而不是游戏意义上的生命值。

飞机不存在统一：

```text
isDead
```

核心 simulation 只回答：

```text
Can it generate lift?
Can it generate thrust?
Can it maintain structural integrity?
Can control surfaces still generate authority?
Can the aircraft reach a stable flight state?
```

---

## 2.3 Damage Does Not Directly Kill Aircraft

Damage system 不负责决定：

```text
Aircraft destroyed.
```

Damage system 只允许：

```text
remove material
reduce stiffness
reduce strength
damage aero surfaces
damage control links
damage engine systems
cause leaks
cause fire
detach physically disconnected components
```

飞机最终失控、解体、坠毁或继续飞行，是其他模拟系统自然计算的结果。

---

## 2.4 Control Input Does Not Directly Control Attitude

玩家输入不能直接控制飞机姿态。

正确路径：

```text
Player Input
     ↓
Control Intent
     ↓
Instructor / Direct Control Logic
     ↓
Elevator / Aileron / Rudder / Throttle
     ↓
Aerodynamic + Propulsion Forces
     ↓
Rigid Body
     ↓
Aircraft Motion
```

---

# 3. Scope

## 3.1 v0.x Core Scope

必须实现：

- 6DoF rigid-body aircraft motion
- propeller aircraft aerodynamics
- piston engine + propeller
- stall behavior
- asymmetric aerodynamic forces
- control surfaces
- Mouse Aim Instructor
- Keyboard Maneuver
- Mouse/Keyboard mixed control arbitration
- trim
- aero surface damage
- control surface damage
- simplified structural load system
- structural weakening
- delayed structural failure under load
- component detachment
- debug damage injection
- basic projectile penetration
- deterministic simulation scenarios
- data-driven aircraft definitions

---

## 3.2 Explicit Non-Goals

v0.x 不要求：

- multiplayer
- matchmaking
- tech tree
- economy
- campaign
- ground vehicles
- ships
- detailed cockpit interaction
- clickable cockpit
- VR
- realistic pilot physiology
- full electrical simulation
- full hydraulic simulation
- high fidelity CFD
- full finite element simulation
- photorealistic maps
- large AI battles
- hundreds of aircraft

这些不得阻塞 Sim Core 开发。

---

# 4. System Architecture

```text
                    Game Layer
 ┌───────────────────────────────────────┐
 │ Rendering                             │
 │ Camera                                │
 │ Audio                                 │
 │ UI                                    │
 │ Input Devices                         │
 │ Scene / Terrain                       │
 └───────────────────┬───────────────────┘
                     │
                     ▼
               Input Abstraction
                     │
                     ▼
 ┌───────────────────────────────────────┐
 │               Sim Core                │
 │                                       │
 │ Flight Instructor                     │
 │ Flight Controls                       │
 │ Engine / Propeller                    │
 │ Aerodynamics                          │
 │ Rigid Body State                      │
 │ Structural Solver                     │
 │ Damage Model                          │
 │ Ballistics                            │
 │ Aircraft Systems                      │
 └───────────────────┬───────────────────┘
                     │
                     ▼
               Simulation State
                     │
            ┌────────┴────────┐
            ▼                 ▼
        Rendering         Telemetry
```

Sim Core SHOULD 尽可能避免直接依赖：

```text
UnityEngine.GameObject
MonoBehaviour
Transform
Renderer
Animator
```

Unity adapter 可以负责：

```text
Simulation state
      ↓
Unity Rigidbody / Visual Transform
```

这样未来允许更换：

- rigid body backend
- rendering backend
- headless test runner

而不重写 Flight Model。

---

# 5. Simulation Timing

核心 simulation 使用：

```text
Fixed timestep: 120 Hz
dt = 1 / 120 s
```

Rendering 独立于 simulation frequency。

第一阶段允许：

```text
Unity Rigidbody
```

作为刚体积分 backend。

Flight Model 必须输出：

```text
Force
Torque
ForceAtPosition
```

不得输出：

```text
DesiredRotation
DesiredPosition
```

未来 SHOULD 可以替换为自定义 6DoF integrator，而无需修改 Aero / Instructor API。

---

# 6. Input Architecture

输入必须首先转换为统一的 `PilotIntent`。

例如：

```text
PilotIntent
{
    mouseAimDirection

    pitchInput
    rollInput
    yawInput

    throttle
    brake
    trimPitch
    trimRoll
    trimYaw

    directPitchActive
    directRollActive
    directYawActive
}
```

物理按键不得进入 Flight Model。

例如：

```text
W
S
MouseX
MouseY
```

只能存在于 Input Adapter。

所有绑定必须可配置。

---

# 7. Control Modes

至少支持三类控制行为。

## 7.1 Mouse Aim

类似 WT RB。

玩家控制：

```text
Desired Flight Direction
```

而不是控制：

```text
Elevator
Aileron
Rudder
```

流程：

```text
Mouse
   ↓
Aim Vector
   ↓
Instructor
   ↓
Desired body rates / attitude
   ↓
Flight Controller
   ↓
Control Surfaces
```

Instructor MUST 通过真实飞机控制面操作飞机。

禁止直接产生：

```text
aircraft torque cheat
rotation correction force
artificial attitude lock
```

---

# 8. Flight Instructor

Instructor 分成两层。

## 8.1 Guidance Layer

输入：

```text
Current Flight Direction
Mouse Aim Direction
Aircraft Attitude
Velocity
AoA
Sideslip
```

输出：

```text
desiredRoll
desiredPitchRate
desiredYawRate
```

---

## 8.2 Control Layer

输入：

```text
desired body state
actual body state
control authority
current aerodynamic state
```

输出：

```text
aileronCommand
elevatorCommand
rudderCommand
```

推荐采用：

```text
outer-loop guidance
        ↓
inner-loop body-rate controller
```

而不是直接 attitude PID 控制所有内容。

---

# 9. Instructor Responsibilities

Instructor SHOULD 提供：

- mouse aim tracking
- coordinated turns
- sideslip suppression
- reasonable rudder coordination
- basic stall prevention
- basic excessive-AoA avoidance
- soft G limiting
- roll stabilization
- damaged-aircraft compensation
- trim assistance

Instructor MUST NOT：

- 强行保证飞机不失速
- 强行保证飞机不进入 spin
- 生成飞机本身不存在的 control authority
- 无视 damaged control surfaces
- 无视 actuator limits
- 无视 aerodynamic limits

例如：

```text
Desired roll rate = 60 deg/s

Available roll rate after damage = 18 deg/s
```

Instructor 可以：

```text
aileron = 100%
rudder assist = active
```

但不得人为产生剩余：

```text
42 deg/s
```

---

# 10. Keyboard Maneuver

Keyboard Maneuver 是核心功能，不作为 Mouse Aim 的附属快捷键实现。

它必须允许玩家直接请求：

```text
pitch
roll
yaw
```

例如：

```text
Pitch Up
Pitch Down

Roll Left
Roll Right

Yaw Left
Yaw Right
```

实际 key bindings 不属于 Sim Core。

---

# 11. Mouse + Keyboard Arbitration

Keyboard maneuver SHOULD 与 Mouse Aim 同时存在，而不是要求切换整个 control mode。

采用 **per-axis ownership**。

例如玩家按下 Roll Left：

```text
ROLL axis:
Keyboard owns

PITCH axis:
Instructor may continue controlling

YAW axis:
Instructor may continue coordinating
```

即：

```text
Mouse Aim
       │
 ┌─────┼─────┐
Pitch Roll   Yaw
  │    │      │
  │ Keyboard │
  │ override │
  ▼    ▼      ▼
control surfaces
```

---

## 11.1 Axis Override

当某个 direct-control input 激活：

```text
abs(input) > threshold
```

对应 axis 的 Instructor authority MUST 被释放。

例如：

```text
Roll Left = 1.0

Instructor roll output ignored.
```

Instructor MAY 使用：

```text
rudder
pitch
```

帮助完成协调机动。

---

## 11.2 Keyboard Release

玩家释放 direct input 后：

```text
Keyboard authority
      ↓
handoff blend
      ↓
Instructor authority
```

必须提供：

```text
handoffTime
```

避免瞬间切换造成 control surface snap。

初始建议：

```text
0.15–0.30 seconds
```

具体值后续通过手感测试调整。

---

# 12. Keyboard Maneuver Philosophy

键盘必须能够用于：

- hard elevator pull
- maximum roll
- rudder maneuver
- scissors
- defensive break
- vertical maneuver
- stall-edge maneuver

键盘不能只是：

```text
“让 Mouse Aim target 往某个方向移动”
```

而必须真正控制飞机。

这允许玩家：

```text
Mouse Aim
```

负责：

```text
general flying
aiming
navigation
```

而：

```text
Keyboard
```

负责：

```text
maximum-performance maneuvering
```

---

# 13. Aerodynamic Model

不采用实时 CFD。

采用：

# Distributed Aero Cell Model

飞机被划分成多个 aerodynamic cells。

例如：

```text
Left Wing

ROOT                         TIP

[A01][A02][A03][A04][A05]
[A06][A07][A08][A09][A10]
```

典型战斗机第一版目标：

```text
80–200 aerodynamic cells
```

不设为硬编码限制。

---

# 14. Aero Cell State

每个 Aero Cell 至少保存：

```text
position
orientation

area
chord
span

liftCurve
dragCurve
momentCurve

stallAngle
postStallCurve

controlSurfaceType
controlDeflection

surfaceIntegrity
surfaceRoughness
effectiveArea

attached
```

---

# 15. Local Airflow

每个 aero cell 必须独立计算 local airflow：

```text
Vlocal =
    aircraft linear velocity
  + angular velocity × cell position
  + wind
  + propwash
```

由此计算：

```text
AoA
sideslip
dynamic pressure
```

然后：

```text
Lift
Drag
Pitching Moment
```

最后将力作用在 cell 对应位置。

这样自然产生：

```text
roll moment
pitch moment
yaw moment
```

---

# 16. Stall

Stall MUST 是局部 aerodynamic state。

禁止：

```text
if AoA > X:
    aircraft.stalled = true
```

不同 aero cells 可以：

```text
stalled
attached
partially separated
```

因此允许自然产生：

- asymmetric stall
- wing drop
- partial stall
- spin entry

不同飞机可以通过 aero data 表现不同失速特征。

---

# 17. Control Surfaces

控制面必须作为 aero cells 或 aero cell modifiers 实现。

包括：

```text
aileron
elevator
rudder
flap
slat
```

控制效率依赖：

```text
airspeed
dynamic pressure
deflection
surface integrity
hinge integrity
control link state
```

而不是固定：

```text
rollPower = 10
```

---

# 18. High-Speed Control Behavior

模型 MUST 支持：

```text
control stiffening
reduced control effectiveness
compressibility correction
```

v0.1 不要求精确跨音速空气动力学，但高速状态不能保持完全线性 control authority。

---

# 19. Propeller Aircraft Core

Propeller Sim Core 至少包含：

```text
Piston Engine
Propeller
Engine Torque
Propeller Thrust
Propeller Drag
Propwash
P-factor approximation
```

后续可以增加：

```text
gyroscopic precession
mixture
supercharger stages
carburetor behavior
boost limits
engine overheating
```

---

# 20. Engine Model

Engine SHOULD 使用数据驱动输出，例如：

```text
power = f(
    rpm,
    throttle,
    altitude,
    manifold pressure
)
```

第一版不要求逐气缸 thermodynamic simulation。

必须能够表现：

```text
power loss with altitude
rpm dependency
throttle response
engine damage
engine shutdown
```

---

# 21. Propeller Model

需要支持至少：

```text
fixed pitch
constant speed propeller
```

Propeller output：

```text
thrust
reaction torque
```

必须影响 rigid body。

这使得低速高功率状态能够产生：

```text
torque roll
yaw tendency
```

---

# 22. Trim

必须支持：

```text
pitch trim
yaw trim
roll trim
```

但具体飞机只能使用真实存在的 trim capability。

Aircraft Definition 可以声明：

```text
trim.pitch = true
trim.roll = false
trim.yaw = adjustableTabOnly
```

受损飞机允许通过 trim 找到新的 equilibrium。

这对轰炸机与重损伤飞机尤其重要。

---

# 23. Damage Architecture

Damage 分成三个不同 topology：

```text
AERODYNAMICS
STRUCTURE
SYSTEMS
```

它们必须保持独立。

---

# 24. Aerodynamic Damage

Aero damage 可以改变：

```text
effectiveArea
lift efficiency
drag
surface roughness
stall characteristics
control authority
```

例如蒙皮爆炸：

```text
Cell A08

effectiveArea      1.0 → 0.55
dragMultiplier     1.0 → 2.8
liftEfficiency     1.0 → 0.62
```

Flight Model 不需要知道：

```text
“飞机被击中了”
```

它只读取新的 aero state。

---

# 25. Structural Model

v0.x 不实现完整实时 FEM。

使用：

# Lightweight Structural Load Graph

主要结构包括：

```text
Front Spar
Rear Spar
Wing Box / Skin
Ribs
Fuselage Frames
Longerons
Tail Attachments
Control Surface Hinges
Engine Mounts
```

结构元素拥有：

```text
stiffness
section
yieldStrength
ultimateStrength
temperature
remainingSection
plasticDamage
connection state
```

---

# 26. Load Path

Aero Cell 产生：

```text
local aerodynamic force
```

映射到：

```text
Rib / Skin
      ↓
Spar
      ↓
Wing Root
      ↓
Fuselage
```

Structural Solver 负责计算：

```text
bending
torsion
load margin
```

---

# 27. Load Redistribution

结构损伤不能简单表示：

```text
wingStrength -= 20%
```

结构受损后，负载应该根据剩余 stiffness 重新分配。

例如：

```text
Before damage:

Front Spar  55%
Rear Spar   30%
Skin        15%
```

前梁损坏：

```text
After damage:

Front Spar  32%
Rear Spar   44%
Skin        24%
```

因此损伤可能产生：

```text
secondary overload
progressive failure
```

---

# 28. Structural Failure

结构失效必须取决于：

```text
remaining structure
+
current load
```

所以以下情况必须可能发生：

### Case A

飞机中弹：

```text
spar damaged
```

但：

```text
1G level flight
```

仍然能够正常飞行。

随后：

```text
5G maneuver
```

才发生：

```text
wing failure
```

---

### Case B

大量蒙皮被炸毁：

```text
large aero degradation
```

但主要 spar 完整。

结果：

```text
aircraft difficult to control
drag dramatically increased
```

但：

```text
wing remains attached
```

---

### Case C

外观损伤不大：

```text
small entry hole
```

但弹药直接破坏主承力结构。

结果可能：

```text
immediate structural failure
```

---

# 29. Structural Detachment

组件脱落只能由：

```text
connection failure
```

触发。

例如：

```text
Wing Outer Section
```

与：

```text
Wing Inner Section
```

之间所有关键 structural connections 失效后：

```text
detach
```

而不是：

```text
damage > threshold → detach
```

---

# 30. Damage and Aerodynamics Feedback Loop

必须形成：

```text
Aerodynamic Forces
        ↓
Structural Loads
        ↓
Structural Damage
        ↓
Geometry / Surface State
        ↓
Aerodynamic Forces
```

例如：

```text
damaged spar
↓
reduced stiffness
↓
wing deformation
↓
aero characteristics change
↓
different loads
```

v0.1 可以采用简化 deformation。

---

# 31. Control System Damage

控制链必须独立建模。

例如：

```text
Pilot Command
     ↓
Control Link
     ↓
Control Surface
```

Control Link 可以是：

```text
cable
rod
hydraulic actuator
mechanical linkage
```

二战飞机第一阶段主要：

```text
cable / rod
```

---

损伤可能产生：

```text
reduced range
increased friction
delay
partial response
stuck position
complete disconnect
```

例如：

```text
Elevator command = +0.8

Damaged linkage output = +0.32
```

Instructor 不允许绕过这个限制。

---

# 32. Crew and Re-Trim

v0.x 不实现完整 crew simulation。

但大型飞机设计必须允许：

```text
trim adjustment
engine shutdown
prop feathering
power redistribution
```

使严重受损飞机存在新的稳定飞行状态。

大型飞机的韧性应来自：

```text
redundancy
structure
size
multiple engines
control authority
crew compensation
```

而不是：

```text
larger HP pool
```

---

# 33. Ballistics

Ballistics 在 Sim Core 稳定后加入。

Projectile 至少包含：

```text
mass
diameter
velocity
drag
projectile type

AP capability
HE mass
incendiary capability
fragment behavior
```

碰撞结果：

```text
penetrate
stop
ricochet
detonate
fragment
```

---

# 34. Projectile Interaction

Projectile 不调用：

```text
ApplyDamage(50)
```

正确流程：

```text
Projectile
      ↓
Geometry Intersection
      ↓
Material / Thickness
      ↓
Penetration
      ↓
Remaining Energy
      ↓
Next Component
```

---

# 35. HE Damage

HE SHOULD 产生：

```text
blast
fragmentation
local structural damage
surface destruction
```

因此大口径 HE 可以：

```text
remove a large skin area
```

却不一定：

```text
break a spar
```

另一枚位置极佳的 HE：

```text
detonates near spar
```

则可能直接造成：

```text
catastrophic structural failure
```

---

# 36. Debug Damage Injection

Ballistics 完成之前必须提供 Debug Damage Tool。

开发者可以直接选择：

```text
Destroy Aero Cell
Damage Front Spar
Cut Elevator Cable
Remove Aileron
Reduce Engine Power
Damage Propeller
Set Fuel Leak
```

例如：

```text
F1 normal

F2 remove left-wing aero cells 30%

F3 disconnect left aileron

F4 damage front spar 60%

F5 remove elevator authority

F6 engine power 50%
```

这允许 Flight / Instructor / Structure 独立开发。

---

# 37. Debug Visualization

必须提供：

## Flight

```text
IAS
TAS
Mach
Altitude

AoA
Sideslip

Pitch Rate
Roll Rate
Yaw Rate

G load
```

## Instructor

```text
Aim Direction
Desired Roll
Desired Pitch Rate
Desired Yaw Rate

Actual Rates

Aileron command
Elevator command
Rudder command

Axis ownership
```

## Aerodynamics

能够显示每个 cell：

```text
Lift
Drag
AoA
stall state
damage state
```

推荐使用：

```text
force vectors
```

可视化。

---

## Structure

必须能够显示：

```text
load
load margin
remaining stiffness
remaining strength
failed connections
```

支持 structural heatmap。

---

# 38. Aircraft Data Architecture

每架飞机必须尽可能数据驱动。

推荐目录：

```text
Aircraft/
└── Bf109F4/
    ├── aircraft.yaml
    ├── mass.yaml
    ├── controls.yaml
    ├── structure.yaml
    ├── engine.yaml
    ├── propeller.yaml
    │
    ├── aero/
    │   ├── wing.csv
    │   ├── tail.csv
    │   └── fuselage.csv
    │
    ├── geometry/
    │
    └── tests/
        └── reference.yaml
```

具体序列化格式可以在实现阶段调整。

核心要求是：

**飞机参数不能散落在 C# gameplay code 中。**

---

# 39. No Aircraft-Specific Core Logic

禁止：

```text
if (aircraft == Bf109)
{
    ApplyBf109Physics();
}
```

特殊设计必须通过 component 实现。

例如 Bf 109 自动前缘缝翼：

```text
LeadingEdgeSlatComponent
```

任何拥有相同机构的飞机都可以复用。

---

# 40. Initial Aircraft Validation Set

不立即制作大量飞机。

## Stage A — Core Validation

### Bf 109 F-4

用于验证：

- general fighter FM
- high power propeller effects
- automatic leading-edge slats
- energy fighting
- stall behavior
- compact high-performance airframe

---

### Spitfire Mk IX

用于验证：

- different wing geometry
- high turn performance
- different control response
- different stall characteristics

---

### Fw 190 A-series

用于验证：

- high roll authority
- roll response across speed
- different control philosophy
- radial engine / different mass distribution

---

## Stage B — Extended Validation

### A6M-series

重点：

```text
low wing loading
excellent low-speed maneuverability
light structure
large control surfaces
```

---

### P-51D

重点：

```text
higher speed flight
energy retention
high-speed control behavior
different wing characteristics
```

如果同一个 Sim Core 能在不加入 aircraft-specific hacks 的情况下支持这些飞机，说明架构基本成立。

---

# 41. Aircraft Validation

不追求第一版就达到商业飞行模拟器精度。

每架飞机至少验证：

```text
maximum speed vs altitude
climb rate
stall speed
basic turn behavior
roll response
energy loss
acceleration
engine power vs altitude
control authority vs airspeed
```

数据存在不确定性时：

```text
record uncertainty
```

不得为了“感觉对”偷偷加入不可解释的 magic multiplier。

允许通过数据调参。

不允许通过架构 hack 调参。

---

# 42. Flight Feel Validation

历史性能数据只能验证：

```text
performance envelope
```

无法完全验证操作感觉。

因此允许建立：

```text
Qualitative Flight Tests
```

例如：

```text
high-speed dive recovery
low-speed scissors
stall entry
stall recovery
rolling reversal
rudder kick
hard break turn
vertical loop
```

目标不是复刻某个游戏，而是保证：

```text
control response
```

来自模拟本身。

---

# 43. Core Test Scenarios

以下场景必须能够自动运行。

## TEST-FM-001

```text
Trimmed level flight
```

要求飞机在无输入时保持合理稳定状态。

---

## TEST-FM-002

```text
Full elevator pull
```

验证：

```text
AoA
stall
G
energy loss
```

---

## TEST-FM-003

```text
Full aileron roll
```

记录：

```text
roll rate vs speed
```

---

## TEST-CTRL-001

Mouse Aim 90° heading change。

飞机必须通过：

```text
control surfaces
```

完成。

禁止 artificial rotation。

---

## TEST-CTRL-002

Mouse Aim 飞行期间按：

```text
full roll left
```

Keyboard 必须取得 roll axis authority。

---

## TEST-CTRL-003

释放 keyboard 后：

```text
Instructor
```

平滑恢复控制。

---

# 44. Damage Acceptance Scenarios

## TEST-DMG-001 — Aero Surface Loss

移除：

```text
25% left outer wing aero cells
```

预期：

```text
asymmetric lift
increased drag
natural rolling tendency
Instructor attempts compensation
```

---

## TEST-DMG-002 — Control Damage

损坏：

```text
left aileron
```

预期：

```text
reduced roll authority
```

Instructor 不得 magically restore roll performance。

---

## TEST-DMG-003 — Structural Weakening

对 left main spar 造成严重但非完全破坏的损伤。

要求：

```text
1G flight survives
```

随后：

```text
high-G maneuver
```

导致 progressive structural failure。

---

## TEST-DMG-004 — Large Skin Damage

大量 aero surface / skin 损坏。

但：

```text
main structural load path survives
```

预期：

```text
bad aerodynamics
heavy drag
trim difficulty
```

但机翼仍然连接。

---

## TEST-DMG-005 — Critical Structural Hit

外观损伤较小。

但直接破坏：

```text
critical spar section
```

在高载荷下允许快速 wing separation。

---

# 45. Determinism

给定：

```text
same initial state
same input
same aircraft data
same damage event
same random seed
```

simulation SHOULD 得到可重复结果。

所有 fragmentation / random damage 行为使用明确 seed。

这样可以：

```text
regression testing
bug reproduction
balance comparison
```

---

# 46. Performance Philosophy

正确顺序：

```text
Correct behavior
      ↓
Profiling
      ↓
Optimization
```

不得提前因为性能担忧而把核心模型简化为 HP system。

潜在优化方向：

```text
Burst
Jobs
SoA data layout
lower-frequency structural updates
sleeping inactive components
batched aero calculations
```

例如未来允许：

```text
Flight Dynamics      120 Hz
Aero                  120 Hz
Instructor            120 Hz
Structure              60 Hz
Systems                30 Hz
Thermal / Fire         20 Hz
```

但所有 frequency 必须根据稳定性与性能测量决定。

---

# 47. Development Order

开发顺序固定为：

```text
Phase 0
Simulation foundation

Phase 1
Clean-aircraft aerodynamics

Phase 2
Direct keyboard flight control

Phase 3
Mouse Aim Instructor

Phase 4
Mouse + keyboard mixed control

Phase 5
Aero damage

Phase 6
Structural loads

Phase 7
Structural damage

Phase 8
Control system damage

Phase 9
Projectile / penetration

Phase 10
Second and third aircraft

Phase 11
Combat prototype
```

不得因为：

```text
guns
graphics
map
AI
UI polish
```

提前跳过核心 simulation。

---

# 48. First Playable Prototype

第一个 playable prototype 只需要：

```text
one aircraft
simple sky
simple terrain
third-person camera
mouse aim
keyboard maneuver
throttle
debug HUD
damage injection
```

不需要：

```text
enemy aircraft
weapons
hangar
menu
progression
```

Prototype 成功标准：

> 飞机本身已经“值得飞”。

---

# 49. Sim Core Success Criteria

Sim Core v0.1 被认为成立，需要同时满足：

### Flight

飞机能够通过 aerodynamic forces 稳定飞行。

### Control

Mouse Aim 可以自然操纵飞机。

### Keyboard

Keyboard 可以进行 full-authority combat maneuvers。

### Hybrid Input

Mouse Aim 与 Keyboard 能够无模式切换共同使用。

### Stall

飞机能够自然进入和恢复 stall。

### Damage

非对称损伤自然产生飞行后果。

### Structure

飞机可能：

```text
survive damage at low load
```

但：

```text
fail later under high load
```

### Reusability

至少三种明显不同的二战战斗机使用同一个 Sim Core。

### Architecture

不得依赖：

```text
per-aircraft hard-coded physics
global aircraft HP
direct attitude control
scripted wing destruction
```

才能满足上述目标。

---

# 50. Implementation Rule

任何新的 simulation behavior 在加入核心前必须回答：

```text
What physical state causes this?

What component owns this behavior?

What input does it consume?

What state does it modify?

Can another aircraft reuse it?

Does this behavior bypass an existing physical subsystem?
```

如果一个实现只能解释为：

```text
“这样玩起来比较像”
```

而无法对应明确模拟状态，应优先重新设计，而不是立即加入 magic correction。

---

# 51. Guiding Example

目标中的飞机损伤过程应该能够自然表示为：

```text
20 mm shell
      ↓
wing skin penetration
      ↓
detonation near front spar
      ↓
skin area destroyed
front spar section reduced
      ↓
local lift changes
local drag increases
structural stiffness decreases
      ↓
Instructor compensates
      ↓
aircraft remains controllable
      ↓
pilot performs hard 5G turn
      ↓
damaged spar overloads
      ↓
load redistributes
      ↓
rear spar + skin overloaded
      ↓
wing structural connection fails
      ↓
outer wing separates
```

任何一步都不需要：

```text
wingHP = 0
```

这就是本项目 Flight / Damage Sim Core 的基本设计目标。
