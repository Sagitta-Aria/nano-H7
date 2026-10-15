# STM32H743 机器人控制板

Waiken STM32H743IIT6 CB V1.5 的寄存器级固件（CMSIS + FreeRTOS，无 HAL / 无 CubeMX），
配一套 Python 上位机：Web 控制台（标准库 + 原生前端，无构建步骤）和等价的 Tk 后备界面。
串口协议为文本行 + CRC，带心跳租约和重放保护。

版本史见 [CHANGELOG.md](CHANGELOG.md)，当前为 v0.2.18。

## 当前状态

已实现：七轴独立 STEP/DIR 与加减速、四轮联合下发、底盘平移/旋转、陀螺仪闭环转角、
田字格坐标巡航、M5 升降与 M6 旋转、两路定位舵机、光流位移累计、USB 摄像头拍照、
板级资源查询、心跳超时停车。

未实现 / 未验证，排进度前先读这段：

- 坐标是**下发脉冲的累计**，不是编码器位置，也没有做光流融合定位。
- 轮径、轮距、减速比、细分数未确定，所以只有 step/s 接口，没有整车 m/s 的麦轮逆运动学。
- 舵机 500–2500 µs 是软件假设，机械端点未校准；舵机型号未确认。
- 未做电机波形、驱动器电平、电源的实机验证。硬件 EN、限位开关、自动回零、
  驱动故障反馈、编码器闭环、原生 USB CDC、参数持久化均未启用。
- 避障只有预留接口，没有绕障。
- 模拟模式的资源数据是演示值。

`ARM` 是软件运动许可，不控制驱动器硬件使能。上电默认禁止 STEP 输出；舵机 PWM 上电即输出初始位置。

## 跑上位机

```powershell
python -m host.webserver          # 启动并打开 http://127.0.0.1:8770/
python -m host.webserver --demo   # 接模拟板，不需要硬件
```

HTTP 页面只用标准库；真实串口需要 `host/requirements.txt` 里的 pyserial。也可双击 `host/run_web.bat`。

- 默认只绑 `127.0.0.1`。`--host 0.0.0.0` 会把**没有认证的电机控制**暴露给整个网络，只在可信链路上用。
- 同一时刻只有一个页面能控制板子，其余自动只读；控制页关闭或卡死几秒后控制权自动转移。
- 关页面不断开板卡；`Ctrl+C` 退出前会先发 `STOP`。
- 其他参数：`--connect COM5` 直接开串口，`--port` 换端口，`--exit-on-close`，`--auto-devices`（按 UID 自动重连 H743/H723）。

Tk 后备界面功能等价：`python -m host.gui --demo`（需要带 Tkinter 的 Python）。

## 编译固件

```powershell
python tools/build_firmware.py                              # 默认 D:\keil\ARM\ARMCC\bin
python tools/build_firmware.py --compiler "C:\Keil_v5\..."  # 其他机器
```

ARM Compiler 5 + MicroLIB + 专用 scatter。输出 `firmware/build/robot_h743.{hex,bin,axf,map}`。
脚本**只编译，不烧录**。Keil 工程入口 `firmware/MDK-ARM/Robot_H743.uvprojx`。

时钟 CPU 400 MHz / AHB 200 MHz / APB 100 MHz / TIM5、TIM8 200 MHz。
不用 480 MHz 是因为它只在部分芯片修订版上可用。电源配置按本板的外部 VCORE 旁路供电设计，
**不能直接移植到其他 H743 板卡**。

## 七轴引脚

| 轴 | STEP | 通道 | DIR | 最大 step/s |
|---|---|---|---|---:|
| M1 左后 | PI5 | TIM8_CH1 | PI8 | 5000 |
| M2 右后 | PI6 | TIM8_CH2 | PI3 | 5000 |
| M3 左前 | PI7 | TIM8_CH3 | PI4 | 5000 |
| M4 右前 | PI2 | TIM8_CH4 | PC13 | 5000 |
| M5 升降 | PH10 | TIM5_CH1 | PC2 | 3000 |
| M6 旋转 | PH11 | TIM5_CH2 | PG11 | 3000 |
| M7 | PH12 | TIM5_CH3 | PB9 | 3000 |

反转掩码 **5（M1/M3 反转）**，2026-10-11 用户实车确认，已是固件默认值。

## 舵机

下表按 [firmware/src/servo.c](firmware/src/servo.c) 的 `hw[]` 表，这是唯一权威来源：

| 协议通道 | 引脚 | 定时器 | 固件角度坐标 |
|---|---|---|---|
| `SERVO 1` | PH9 | TIM12_CH2 | 0–270° |
| `SERVO 2` | PH6 | TIM12_CH1 | 0–180° |

两路共用 TIM12，50 Hz，500–2500 µs。上位机把物理角度换算成固件坐标后下发，
界面上 S1 物理行程按 180°、S2 按 270° 显示。2026-10-14 用户实物确认协议通道 1 对应实物舵机 1。

> **已知不一致**：[host/servo_config.py](host/servo_config.py#L8)、[host/static/app.js](host/static/app.js#L759)、
> [host/static/index.html](host/static/index.html#L117) 四处把 S1 标成 PH6、S2 标成 PH9，与固件相反。
> 通道选择和角度换算都是对的，错的只有显示用的引脚标签。接线请以上表为准，不要照界面上的标签接。

舵机**不受 `ARM` 门控，也不被 `STOP` 停止**——它们是位置输出，必须持续收到 50 Hz 脉冲才能保持位置。
角度越界返回 `ERR code=ANGLE_RANGE`，不静默截断。

**两路舵机必须独立 5V 供电，不要从板上 3.3V 取电**，堵转大电流会把 MCU 一起拉死。信号地与板子共地。
方向反了请改 [firmware/include/app_config.h](firmware/include/app_config.h) 的脉宽常量或机械反装，不要用负角度。

加一路舵机：在 `servo.c` 的 `hw[]` 加一行、`SERVO_CHANNELS` 加一，协议和页面会自动跟上。
TIM12 只有 CH1/CH2，第三路要换定时器（见 `PIN_ALLOCATION.md` 的 S3）。

## 其他外设

- **LED**：PB0 绿、PB1 蓝，低电平有效。红色通道经电阻固定接地，**上电常亮**，没有软件开关。
- **串口**：USART1 板载 COM Type-C / CH340X；UART7 无线（PA8 接无线 TX，PA15 接无线 RX）。
  两路独立组帧，应答向两路发送。115200 8N1。
- **光流**：UP-F7-QX01 接 USART3，PB11 接模块 TX、PB14 接模块 RX，模块 5V、UART 3.3V。
- **陀螺仪**：达妙 DM-MC02（H723）走独立 USB 虚拟串口接上位机，由上位机转发航向给 H743。
- **摄像头**：直接接上位机，仅 Web 版提供，按需拍照、无持续视频流。

## 协议边界

- 非零目标为整数 step/s，最低 1。0 用于四轮减速停止。
- `MOVE` 每轴同时只接受一条命令；忙时拒绝，不排队。单次步数绝对值 ≤ 10000000；
  M5 另受相对最高点 0～8000 的累计行程限制。
- 加速度 1–20000 step/s²，四轮默认 2000（软件初值，需实机调整）。
- `WHEELS` 原子提交四轮目标。运动中直接反向会被拒绝，须先减速到零。
- `STOP` 立即禁止后续上升沿、补完当前高电平后关闭并解除使能。它**不是**规划减速停车，
  需要平滑停车先发 `WHEELS 0 0 0 0 <加速度>`。
- 心跳 250 ms，板端 1000 ms 超时执行 `STOP`；上位机重连不会自动恢复运动。
- 比较更新错过期限会锁存故障并关闭全部轴，需重启板子清除。

完整协议见 [docs/PROTOCOL.md](docs/PROTOCOL.md)。

## 接板测试顺序

1. 用 SWD 烧录 HEX。
2. 数据线接板上 **COM** Type-C（不是标 USB 的原生口）。
3. 上位机刷新串口，选 CH340 对应的 COM 口连接。
4. 验证绿/蓝灯、运行时间递增、固件版本和 UID。
5. 没装电机可以停在这里——LED 和资源查询不需要使能运动。
6. 接电机前先用逻辑分析仪核 STEP/DIR：步数、方向、频率、启停完整脉冲、并发输出。

## 测试

```powershell
python -m pip install --target .testdeps -r tools/test-requirements.txt
python tools/build_firmware.py
python -m unittest discover -s tests -p "test_*.py" -v
python -m tests.gui_smoke
node tests/test_servo_frontend.cjs      # 前端契约测试，另有数个 *_browser.cjs
```

ARM 测试用 Unicorn 执行**真实编译出的 AXF 函数**，验证协议、限速、计数、回绕、停止逻辑和舵机脉宽表；
只模拟必要的寄存器访问语义，**不是**整板仿真。前端没有浏览器自动化环境，用页面契约测试代替：
逐一核对脚本用到的每个元素 id、CSS 类和 API 路由，方向矩阵与 ARM 测试共用同一份期望值。

这些都不能代替电机、波形、电源和通信的实机验证。

## 目录

```text
firmware/include/   配置、接口、FreeRTOSConfig
firmware/src/       时钟/GPIO、串口、协议、运动引擎、底盘、巡航、转角、舵机、光流
firmware/vendor/    CMSIS、FreeRTOS（保留原始许可证，见 docs/THIRD_PARTY.md）
firmware/MDK-ARM/   Keil 工程
host/               通信层、Web 控制台（static/ 前端）、Tk 控制台、协议、模拟板
tests/              ARM 二进制逻辑测试、Python 测试、GUI 冒烟、前端契约测试
tools/              编译与部署脚本
docs/               协议、架构、各子系统说明、验证记录
```

`PIN_ALLOCATION.md` 是原始输入资料；已核实的问题与实现选择见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。
