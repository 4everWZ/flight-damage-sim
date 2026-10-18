# User requirements: relevant original excerpts

These are project-relevant quotations from the recovered design conversation, followed by the current instruction. Unrelated website-generation discussion is excluded. The historical comments about commercial games state the user's preferences and impressions; they are not verified claims about those games' source code.

## Historical source

[Original conversation](https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd)

### Excerpt 1

Source: https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=2700c935-897c-4485-8a14-7c6a546c561c
Created: 2026-09-23T10:20:47.425333Z

其实我想做il2 掠食之翼的空战模块化损伤【比战争雷霆更细致】 + 战争雷霆RB 的这种操作逻辑。

### Excerpt 2

Source: https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=c24e323d-1535-47cd-8fa4-590fcb9407f9
Created: 2026-09-23T10:20:47.440750Z

wt 现在其实很简化，飞机其实就是每个模块的全是有各自的血条，所以 要不然 轰炸机和混凝土堡垒一样，f14把 aim54打光了 都炸不死一个B17
要不然就脆的和纸一样
IL2 是真的做了 轰炸机的结构 抗性 不是靠血条来模拟。
高射炮  或者大口径高爆弹依然能炸掉一块儿蒙皮，但是机组还能重新配平，薄壳弹直接打中翼梁也能炸断整个机翼这种。

### Excerpt 3

Source: https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=3fe6c22e-161e-43d2-bbc9-05186c73cedf
Created: 2026-09-23T10:20:47.447326Z

可以，没毛病，但是，拿啥做呢
气动 损伤 还有飞控和操作逻辑 这些是重中之重。
我只是玩的多XD 没做过大型游戏XD

### Excerpt 4

Source: https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=2612f07b-3448-4acd-a9e9-32c32fc5205a
Created: 2026-09-23T10:24:38.505000Z

good
关于操作，wt 还有就是键盘机动，这也是核心。飞控这个东西，其实非常的难写，不过，我们就是玩玩，所以，就二战那几个知名的飞机先做出来验证各个核心逻辑就可以，螺旋桨飞机的sim core 出来了，这样可以套更多的飞机XD
来写份spec吧

## Current instruction

Received in the project request on 2026-10-14. Quoted verbatim:

之前我曾经说要参考 warthunder 和IL2的毁伤 飞控 操作逻辑做个prototype吗 先从核心的飞控和毁伤引擎开始吧 找一下之前chat的生成的资料 创建一个repo 用codex cloud 去研发测试. 先做核心引擎飞控，然后搭配到现成的工具里能简单可视化操作和测试的 这也是一个并行任务，也按类似 auto research 的路子搞下去，但是，这个核心的操作，飞控，毁伤不能削弱。cpp还是python都可以，看适配度.

## Specification provenance

The preserved [FlightLab v0.1 specification](../spec/flight-sim-core-v0.1.original.md) comes from the [assistant response requested by the user](https://chatgpt.com/c/8c79d2da-f148-8328-b951-93766a55c0cd?messageId=83e5c30d-f0ce-4700-9449-9095619ddfbc). It is an assistant-authored engineering proposal. Recovery removed only the surrounding chat introduction, closing commentary, and writing wrapper; the 51-section specification body is unchanged. The original document's status label does not imply that its implementation or physical accuracy has been verified.

## Priority clarification, 2026-10-18

The user asked whether the reported structural parameters were based on actual material physics, then clarified: “抓大放小，可以先参考二战经典战斗机，比如bf109 spitfire 系列”. Their follow-up permits weapon/performance parameter tables and rejects building an entire physically complete world as a prerequisite. Aircraft core components, flight controls/aerodynamics, damage-driven functional degradation and coupling, and high-speed/overload breakup remain central; the preceding spar tests are examples, not the complete damage scope.

Use specific reference variants and keep measured/reference-informed/assumed values distinguishable. This direction does not require full finite-element analysis or historical certification, and does not authorize HP, direct attitude changes or scripted overspeed separation. Existing synthetic fixtures remain explicit engineering tests rather than silently becoming a historical aircraft. PR14-specific merge approval applies only to PR14; PR15 remains draft unless separately authorized.
