# 串口协议 v1

v0.2.18取消P调参范围。`TURN CONFIG <kp_centi> [tolerance_cd]` 接受 kp_centi=0～INT32_MAX，这是协议存储容量；P=0关闭航向输出，角度容差范围保持1～1000。控制乘法使用64位并在缩窄前按任务最高轮速饱和。配置仍要求底盘/控制任务静止，回报实际值，无静默截断。

v0.2.17 新增 `straight=1`、`CHASSIS STRAIGHT <FWD|BACK|LEFT|RIGHT> <steps> <sps> <accel>`；起步记录新航向，行驶时共享 P/容差纠偏。CHASSIS 状态增加 `assist target_cd error_cd fault`，巡航直线自动使用同一运动底层。停止/ARM0清除待发送直线命令，IMU失效不自动续跑。详情见 [辅助直线](STRAIGHT.md)。

v0.2.16 新增 `turn_precision=1`，`TURN CONFIG <kp_centi> <tolerance_cd>` 原子应用共享P和角度容差；仅底盘、巡航、转角均静止时接受，P1～20000、容差1～1000，分别代表P×100、容差度数×100。返回TURN_STAT实际kp_centi/tolerance_cd；旧的单参数CONFIG保留容差。

v0.2.15 新增 `turn_tuning=1 aux_live=1 servo1_pin=PH9 servo2_pin=PH6`，`TURN CONFIG <P×100>`、`TURN_STAT kp_centi tolerance_cd=20`，舵机回复报告 pin/timer。AUX CONFIG 允许已使能但轴静止时修改范围，保留参考和当前位置检查。

v0.2.14 新增 `nav_path=1` 和 `NAV BEGIN/WP/COMMIT/START <path_id>`。Nano 规划整条路线，H743 校验并自主执行；裸 START 也要求已提交路线。上传与停止语义见 [完整路径协议](NAVIGATION.md)。

v0.2.10 新增 `turn=1` 和 `TURN ZERO/MOVE REL/MOVE ABS/STAT/STOP`，使用 H723 航向独立闭环转角；详见[转角协议与基准](TURN.md)。

v0.2.8 新增 `aux_control=1` 和 M5/M6 `AUX STAT/ZERO/CONFIG/STEP/GOTO/HALT`；动态累计位置范围由板端检查。命令、字段和参考边界见 [辅助轴协议](AUXILIARY.md)。

v0.2.5 新增 `LIFT INIT/STAT/DIR/MOVE`；v0.2.6 的 HELLO 声明 `lift=1 lift_max=8000`，并反转 M5 物理 DIR 电平。上位机在握手后自动 INIT，同一次 MCU 上电内只建立一次最高点零位；M5 所有运动（包括原始 MOVE）受 0～8000 累计行程和显式方向配置限制。命令、应答及错误码见 [M5 协议与使用](LIFT.md)。

v0.2.4 为 S1/S2 分别声明 270°/180° 行程；v0.2.2 新增 `SERVO`，详见下方“舵机命令”。v0.2 新增 `CHASSIS MOVE/JOG/HALT/STAT/CONFIG`，详细语义见 [底盘协议扩展](CHASSIS.md)。帧格式保持 v1；HELLO 新增 `chassis=1` 用于 GUI 能力检测。原有原始 MOVE/WHEELS 接口不套用底盘方向反转。

115200 baud，8N1，无流控。通道：CH340X → USART1 PA9(TX)/PA10(RX)，以及无线 → UART7 PA15(TX)/PA8(RX)。v0.2.1 两路独立组帧、共享序号去重缓存，全部协议应答双路发送；仅一个主动控制上位机，另一端可被动观察，详见 [双串口说明](WIRELESS.md)。ASCII 帧，所有数值使用十进制，状态字段另有说明除外。

```text
@<sequence> <body>*<CRC16>\n
```

`sequence`：1–4294967295。CRC16-CCITT-FALSE：poly=0x1021，init=0xFFFF，refin/refout=false，xorout=0。校验范围为 `<sequence> <body>` 的 ASCII 字节，不含 `@`、`*`、四位十六进制 CRC 或换行。编码及解码实现在 `host/protocol.py` 和 `firmware/src/protocol.c`。

接收兼容 CRLF。发送帧总长必须小于 384 字节；请求正文小于 128 字节。不支持非 ASCII、嵌入 NUL 或 `*`。坏 CRC 不应答，坏帧计数加一；接收溢出丢弃到下一换行；不完整帧超过 200 ms 未收到新字节后丢弃。

## 请求与应答

| 正文 | 含义 | 应答示例正文 |
|---|---|---|
| `HELLO` | 版本、能力握手 | `OK cmd=HELLO fw=0.2.7 protocol=1 chassis=1 servo=2 ... lift=1 lift_max=8000 nav=1` |
| `PING` | 更新心跳租约 | `OK cmd=PING` |
| `INFO` | 只读芯片与时钟信息 | `OK cmd=INFO cpu_hz=400000000 ...` |
| `STAT` | 运行资源、LED、错误计数 | `OK cmd=STAT uptime_ms=... heap_free=...` |
| `LED <green> <blue>` | 0/1 控制绿蓝通道 | `OK cmd=LED led=3` |
| `ARM <0或1>` | 解除/申请运动使能 | `OK cmd=ARM armed=1` |
| `STOP` | 所有轴停止，解除使能 | `OK cmd=STOP armed=0` |
| `MOVE <axis> <steps> <sps> <accel>` | 单轴有限行程 | `OK cmd=MOVE state=accepted axis=5 move_id=...` |
| `WHEELS <s1> <s2> <s3> <s4> <accel>` | 四轮速度原子提交 | `OK cmd=WHEELS state=accepted` |
| `AXIS <axis>` | 查询单轴快照 | `OK cmd=AXIS axis=5 pos=... sps=... remaining=... active=... move_id=...` |
| `SERVO <通道> <角度>` | 舵机定位 | `OK cmd=SERVO ch=1 angle=135.0 us=1500` |
| `SERVO STAT` | 全部舵机快照 | `OK cmd=SERVO_STAT ch=1 angle=135.0 us=1500 on=1 OK cmd=SERVO_STAT ch=2 angle=90.0 us=1500 on=1` |

错误应答为 `ERR code=<原因>`，仍使用原请求 sequence。语法错误、越界值、未使能、轴忙、运动中反向都会拒绝；不会静默限制成另一个速度。轴编号为 1–7，LED 位掩码 bit0=绿、bit1=蓝，红色固定。

## 命令生命周期

`MOVE` 的 OK 表示**接受**。通过 `AXIS` 查询对应 `move_id`：`active=0 && remaining=0` 表示有限行程脉冲发送完成。`active=0 && remaining>0` 表示提前停止。此版本用轮询反馈，不发送异步完成事件。原始 MOVE 不排队；NAV 支持已提交的完整路线缓冲。

`pos` 是有符号 32 位脉冲位置，`sps` 是当前计划速度（整数化），均不代表编码器测量。计数溢出会触发运动故障，不能无限跨越 INT32 边界。

## 舵机命令

抓取点：`SERVO 2 90` → `angle=90.0 us=1500`；`SERVO 2 120` → `angle=120.0 us=1833`。两者均按新版 0–180°标尺，分别对应旧 360°标尺的 180°/240°。

`SERVO <通道> <角度>`：通道为 1–`servo`（当前 2，v0.2.13 起 S1=PH9 / S2=PH6）。角度是**度**，可写整数（`135`、`90`、`0`）或带一位小数（`13.5`、`135.5`）；带小数点才表示小数，`135` 永远表示 135.0 度而不是 13.5 度。S1 行程 0–270°，S2 行程 0–180°。应答回显**已写入的目标**角度与脉宽（不是舵机位置反馈），`us` 就是比较寄存器的微秒值，可直接和示波器对照。

- 超出行程返回 `ERR code=ANGLE_RANGE`，参数写错（含通道号越界）返回 `ERR code=SERVO_BAD_ARGUMENT`；都不静默截断成邻近角度。
- 分辨率 0.5°：脉宽表按半度步进，`90.5` 会落到 90.5°，更细的小数会被向下取整到半度。
- `SERVO STAT` **把每一路各写一条 `OK cmd=SERVO_STAT ch=…` 放在同一个正文里，用空格分隔**。上位机字段解析器只保留最后一个同名键，所以按通道读取时必须从原始正文里逐条解析（`host/static/app.js` 的 `SERVO_FRAME` 正则即为此）。
- 舵机**不需要 ARM，也不被 `STOP` 停止**。初始化后输出 50 Hz 脉冲：S1 默认 90°（1167 µs），S2 默认 90°（1500 µs），两路同频同相。
- `HELLO` 报 `servo=2 servo_travel=2700 servo1_travel=2700 servo2_travel=1800`。每路 `servoN_travel` 的单位为 0.1°；兼容字段 `servo_travel` 仍指 S1。新版上位机优先使用每路字段，连接旧固件时回退到公共字段。
- S2 的软件换算：`SERVO 2 45/90/135/180` 分别输出 `1000/1500/2000/2500 µs`；`SERVO 2 180.5`、`SERVO 2 270`、`SERVO 2 360` 与 `SERVO 1 360` 均拒绝。500–2500 µs 的物理端点需实测校准。

只有新 `PING` 更新心跳。INFO/STAT 查询、重复请求重放都不刷新租约。申请 ARM 前必须有最近 1000 ms 内的 PING；再次 ARM 不能清除时序故障。

## 重试规则

上位机串行处理一个在途请求，300 ms 超时后使用**完全相同的字节和编号**重试一次；仍无应答则关闭连接。

板端缓存最近 **8 个**请求的原始正文和应答。同号同正文返回原应答、不重复执行；同号不同正文返回 `SEQUENCE_REUSE`。缓存淘汰后不再提供去重保证；复位后缓存也会丢失。因此不能无限期重试运动命令，也不能跨设备复位重发旧运动。正常 GUI 重连后重新握手并保持电机未使能。

GUI 心跳由通信线程发送，周期 250 ms。STOP 清除尚未发送的 GUI 请求并优先发送；已经在途的请求仍可能先完成。板端租约是通信在线检测，不是独立硬件急停。

## 系统状态字段

- `uptime_ms`：32 位 RTOS tick，1 ms/tick，约 49.7 天回绕。
- `heap_free` / `heap_min`：48 KiB FreeRTOS heap_4 堆的当前/历史最小剩余字节数，不是全片 SRAM 余量。
- `stack_comm` / `stack_motion` / `stack_super`：三个任务历史最小未用栈空间，单位字节。
- `tasks`：含 idle 任务的数量；初始为 4。
- `rx` / `rx_err` / `tx_err`：两路 UART 合计接收字节、接收错误/环形缓冲溢出、发送等待超时。
- `bad` / `replay`：协议坏帧与重放命中次数。
- `timeout` / `motion_fault`：心跳超时次数和锁存运动时序/计数故障数。
- `reset`：启动时捕获的 RCC_RSR 十六进制原值；`uid` 为芯片 UID 高字在前的 24 位十六进制字符串。

尚未实现 CPU 占用率、温度、供电电压、传感器读数；GUI 不显示伪造的实时硬件测量值。
