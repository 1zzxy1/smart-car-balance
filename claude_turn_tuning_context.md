# 单车低速转弯调试上下文给 Claude

更新时间：2026-05-07

## 当前目标

当前车已经能直线，正在探索 `1.00m/s` 极限低速下的转圈/8 字路线。

目标不是先追求高速稳定，而是让低速下 `MISSION_OPEN_TURN_ANGLE_DEFAULT=6.0f` 附近也能稳定转弯，不再在转弯后半段出现舵机 `servo_output=-1300` 饱和后倒车。

## 当前仓库状态

工程路径：

```text
D:\work\3_smart_Car\this_is_my_project
```

当前 `master` 最新提交：

```text
a7675e3 README PID 参数表同步本轮 P0/P1 改动
```

最近重要提交：

```text
a7675e3 README PID 参数表同步本轮 P0/P1 改动
325cf90 P1：放宽角度环输出 + 同步下调 gyro Kp + 收紧 steering
768653a 开机静止平均锁零 + SW2 触发加静止守卫
1708c32 角速度环限幅对齐 + anti-windup + 输入截断放宽
11a6cf0 角度环 Kp 8.3→22 收紧
b0bff45 添加19列遥测转31列工具
```

注意：仓库里有一些未跟踪的 `tools/*.py`、`tools/*.csv`、`tools/*.txt` 分析产物，当前没有纳入提交。

## 当前关键参数

`code/app/schedule.c`：

```c
#define MISSION_GO_DISTANCE_M       (9.0f)
#define MISSION_RUN_SPEED_MPS       (1.00f)
#define MISSION_RUN_SPEED_MIN_MPS   (0.50f)
#define MISSION_RUN_SPEED_MAX_MPS   (2.00f)
#define MISSION_OPEN_TURN_ANGLE_DEFAULT (6.0f)
#define MISSION_OPEN_TURN_ANGLE_MIN     (1.0f)
#define MISSION_OPEN_TURN_ANGLE_MAX     (12.0f)
#define MISSION_RELOCK_ERROR_DEG    (30.0f)
#define MISSION_LEAN_BLEED_MS       (200U)
#define MISSION_BACK_HEADING_HOLD_MS (250U)
#define MISSION_FIRST_TURN_DEG      (360.0f)
#define MISSION_FULL_TURN_DEG       (360.0f)
#define MISSION_YAW_PROGRESS_SIGN   (-1.0f)
#define MISSION_TURN_RAMP_MS        (300U)
```

`code/app/balance_app.c`：

```c
#define BALANCE_STEERING_KP         (0.15f)
#define BALANCE_STEERING_KI         (0.0f)
#define BALANCE_STEERING_KD         (0.05f)
#define BALANCE_STEERING_OUT_LIMIT  (1.5f)

#define BALANCE_ANGLE_KP            (22.0f)
#define BALANCE_ANGLE_KI            (0.005f)
#define BALANCE_ANGLE_KD            (-0.3f)
#define BALANCE_ANGLE_OUT_LIMIT     (250.0f)
#define BALANCE_ANGLE_INT_LIMIT     (30.0f)

#define BALANCE_GYRO_KP             (5.2f)
#define BALANCE_GYRO_KI             (0.0f)
#define BALANCE_GYRO_KD             (0.0f)
#define BALANCE_GYRO_OUT_LIMIT      (1300.0f)
#define BALANCE_GYRO_INT_LIMIT      (200.0f)
#define BALANCE_GYRO_INPUT_LIMIT    (1500.0f)
#define BALANCE_GYRO_LPF_ALPHA      (0.20f)
```

## 当前任务状态机

`code/app/schedule.c`：

```c
typedef enum
{
    MISSION_IDLE = 0,
    MISSION_GO_STRAIGHT,
    MISSION_TURN_RAMP,
    MISSION_OPEN_TURN,
    MISSION_LEAN_BLEED,
    MISSION_BACK_HEADING
} mission_state_enum;
```

状态含义：

- `0 IDLE`：电机关闭或任务复位。
- `1 GO_STRAIGHT`：直行到 `MISSION_GO_DISTANCE_M`。
- `2 TURN_RAMP`：新增进弯渐入状态，`expect_angle` 用 `300ms` 从 `0` 线性爬到当前 `TURN`。
- `3 OPEN_TURN`：关闭航向环，固定 `expect_angle=TURN` 开环压弯。
- `4 LEAN_BLEED`：出弯溜泄，`expect_angle` 从当前值线性降到 `0`。
- `5 BACK_HEADING`：短暂锁住目标航向，然后继续下一次反向整圈。

## 串口遥测格式

当前 19 字段：

```text
tick,state,dist,speed,yaw,prog,rem,exp,ta,pit,af,tg,gy,so,pwm,akp,ad,turn,run
```

字段解释：

- `tick`：毫秒计时。
- `state`：mission 状态编号。
- `dist`：累计距离。
- `speed`：速度。
- `yaw`：当前 yaw。
- `prog`：本次转弯累计进度角。
- `rem`：本次转弯剩余角。
- `exp`：`expect_angle`。
- `ta`：`target_angle`。
- `pit`：pitch。
- `af`：angle feedback。
- `tg`：target gyro。
- `gy`：gyro_y。
- `so`：servo output。
- `pwm`：servo pwm。
- `akp`：angle Kp。
- `ad`：angle D。
- `turn`：当前开环压弯角。
- `run`：目标速度 m/s。

## 本轮关键实测文件

用户上传：

```text
D:\download\转弯数据\4.txt
```

该数据是新增 `MISSION_TURN_RAMP` 后的实车记录。文件是串口助手导出的原始文本，里面有时间戳插入和断行错位，需要先清洗再分析。

清洗后有效记录约 `1785` 条，出现状态：

```text
state 0: 160
state 1: 887
state 2: 32
state 3: 706
```

没有出现 `state=4 LEAN_BLEED` 和 `state=5 BACK_HEADING`，说明车在进入出弯逻辑之前已经失稳/撞停。

数据内出现多个 `AD` 值：

```text
AD=-0.30
AD=-0.35
AD=-0.40
AD=-0.45
AD=-0.50
```

其中用户重点关注 `AD=-0.5` 多次跑的表现。

## 对 4.txt 的核心结论

结论：当前问题不是 `AD` 的主问题，也不是进弯瞬间阶跃问题。`MISSION_TURN_RAMP` 生效了，进弯瞬间没有直接把舵机打满。真正的问题在 `OPEN_TURN` 后半段：`expect_angle=6°` 保持太久，车还没进入 `LEAN_BLEED`，pitch 已经滚大，最后平衡环被逼到 `servo_output=-1300`。

### TURN_RAMP 生效证据

`state=2 TURN_RAMP` 阶段：

```text
expect_angle: 0 -> 5.56
servo_output: 约 -391 ~ +478
没有 -1300 饱和
```

这说明新增的进弯渐入状态有效，`0° -> 6°` 阶跃不是当前主因。

### OPEN_TURN 后段失稳证据

`AD=-0.5` 的关键片段：

```text
progress≈259.8°
remaining≈100.2°
pitch≈9.21°
target_gyro≈-124
servo_output=-1300
```

随后：

```text
progress≈271.2°
remaining≈88.8°
pitch≈18.03°
target_gyro=-150
servo_output=-1300
```

最后：

```text
progress≈329°
remaining≈31°
pitch≈60°
仍未进入 LEAN_BLEED
```

也就是说，`MISSION_RELOCK_ERROR_DEG=30°` 太晚了。车在 `remaining≈100°` 时已经第一次饱和，等到 `remaining≈30°` 基本救不回来。

## 当前判断

`servo_output=-1300` 是结果，不是根因。

更像是任务层给了一个过长时间的 `expect_angle=6°`，导致角度环后半程持续要求大倾角，pitch 越滚越大；平衡环只能一直给反向舵机，最终撞到 `-1300`。

所以现在不建议继续盲调 `AD`。`AD=-0.5` 可能比 `-0.3` 稍好，但主矛盾是任务状态机的出弯卸载太晚。

## 建议下一步方案

不要只靠 `MISSION_RELOCK_ERROR_DEG=30°` 一刀切进入 `LEAN_BLEED`。建议增加“出弯卸载过渡”逻辑。

推荐第一版逻辑：

```text
OPEN_TURN 前半段：
remaining > 100°
expect_angle = TURN

OPEN_TURN 后半段：
remaining 100° -> 50°
expect_angle 从 TURN 线性降到 0

接近目标：
remaining <= 50°
进入 LEAN_BLEED
```

对应新增宏可以类似：

```c
#define MISSION_TURN_UNLOAD_START_ERROR_DEG  (100.0f)
#define MISSION_TURN_UNLOAD_END_ERROR_DEG    (50.0f)
```

意图：

- 前半段继续用 `TURN=6°` 保证转弯半径。
- 后半段提前卸载 `expect_angle`，避免 pitch 滚到 60°。
- 不再等到 `remaining=30°` 才卸载。
- `LEAN_BLEED` 仍保留，作为最后缓冲。

## 修改注意事项

用户要求：

- 每次代码修改后需要 commit 并 push。
- 不要误合并远端；如果 push 出现分叉，先问用户。
- 不要把无关未跟踪分析文件混入提交。
- 用户正在探索极限低速，不要擅自恢复到 `1.5m/s` 中速标准。

推荐提交粒度：

```text
1. 添加 OPEN_TURN 后段 expect_angle 卸载逻辑
2. 如需要，再单独提交串口字段补充或 README 同步
```

## 当前不建议优先做的事

- 不建议继续只扫 `AD`，因为 `4.txt` 已经显示 `AD=-0.5` 仍在 `remaining≈100°` 附近饱和。
- 不建议直接把 `MISSION_RELOCK_ERROR_DEG` 改回很大值后就开航向环，因为之前改大过可能更差；应该先在 `OPEN_TURN` 内部做连续卸载，而不是突变。
- 不建议再增大 `TURN`，因为 `TURN=6°` 已经能在后半段把 pitch 推到 60°。
- 不建议优先改速度，当前问题在低速目标下的任务层转弯卸载时机。

