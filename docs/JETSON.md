# Jetson 侧部署与运行（第一阶段）

本页记录把上位机通信层搬到 Jetson 盒子后的状态、验证结果和待办。

## 2026-10-14：上位机重启按钮（后台待加载）

- 连接栏新增“重启上位机”。控制页确认后请求 `/api/host/restart`；已使能、轴运动、转角/巡航或待执行配置期间拒绝重启。接受请求后关闭设备并重新启动当前 Python 程序，沿用原启动参数和保存文件；页面通过 `/api/host/state` 的实例编号确认恢复后自动刷新。
- 同步4个文件并逐项校验 SHA-256；Nano Python 3.8 的158项测试及PTY三阶段通过，包括独立SIM进程的真实重启和解除使能检查。浏览器验证取消、拒绝、等待断线/旧实例、自动刷新、只读禁用和1440/390 px布局；实际Nano页面新增按钮可见，无JavaScript异常。
- 加载前发现现场电机已使能且四轮正在运动，因此未执行服务重启。新版后台仍待用户停止并取消使能后加载；已请求用户回复“已停止”。未下发实车运动命令。
- 备份 `/home/a/h743/backups/host_restart_button_20261014_203121/`；证据见 [validation_nano_host_restart_button_20261014.json](validation_nano_host_restart_button_20261014.json)。

## 2026-10-14 19:20：修正错误的舵机通道交换

- 用户实物反馈：点击上位机 S1，动的是实物 S2。根因是 19:04 版本按 `HELLO` 引脚标签交换了发送通道；日志与回归检查复现 S1 点击生成 `SERVO 2`。该版本的板端回读不能证明实物编号正确。
- 已撤销交换，保持用户实测的 **S1→协议通道1，S2→协议通道2**。显示按用户指定 S1/PH6/0–180°、S2/PH9/0–270°，只换算角度坐标：现有固件 S1 的目标120°发送 `SERVO 1 180`（1833 µs），S2 的0°发送 `SERVO 2 0`（500 µs）。状态由同一通道脉宽换算显示，固件引脚文字不再决定舵机编号。
- 新增用户问题的回归检查，修正前失败、修正后通过。Web/Tk 模拟联调通过；Nano 同步5个文件，46项部署哈希一致，153项远端测试及PTY三阶段通过。
- 19:20:00 重启用户服务，PID5184→6901，active。H743/H723重新连接；H743回报通道1为180.0/1833 µs，通道2为0.0/500 µs。现场H743已由用户更新到0.2.18，本次代理未烧录。
- 在Nano实际页面测试点击处理器，并在浏览器拦截命令，确认S1生成 `SERVO 1 180`、S2生成 `SERVO 2 0`；没有把测试点击命令发送给设备。页面状态显示120.0°/0.0°，桌面/390px布局通过、无JS错误。实物对应关系采用用户反馈，没有用协议回读替代实物检查。
- 部署期间现场 `auxiliary.json` 的M6范围由-3200改为-4000；保留重启前最新文件，并额外备份，重启前后哈希一致。巡航与设备配对文件保持一致。备份 `/home/a/h743/backups/servo_channel_fix_20261014_191629/`；记录 [validation_nano_servo_channel_fix_20261014.json](validation_nano_servo_channel_fix_20261014.json)。

## 2026-10-14 19:04：舵机编号与连接自动定位

- 已同步 7 个更新/新增文件，部署清单 46 项 SHA-256 全部一致；Nano Python 3.8 的 153 项回归、三阶段伪串口验证通过。
- 上位机 S1 对应 PH6/TIM12_CH1、0–180°，连接后自动定位 120°；S2 对应 PH9/TIM12_CH2、0–270°，连接后自动定位 0°。显示编号与固件协议通道独立，命令、状态与脉宽换算均按引脚映射。
- 19:04:14 重启用户服务 `h743-web.service`，PID 1761→5184，active。重启前 H743 已恢复连接，七轴静止、巡航/转角空闲；重启后解除电机使能。H743 实物回读确认 PH6 为 120.0°/1833 µs、PH9 为 0.0°/500 µs，H723 streaming。
- 实物页面 1440/390 px 检查通过，无 JS 错误；浏览器验证仅查看页面，未发送控制命令。刷新网页不重复初始化，设备重新连接时自动定位。
- `auxiliary.json`、`navigation.json`、`device_links.json` 与部署前 SHA-256 一致。原程序备份 `/home/a/h743/backups/servo_display_20261014_190017/`。本次只更新上位机，现场 H743 固件仍 0.2.16。
- 记录见 [validation_nano_servo_display_20261014.json](validation_nano_servo_display_20261014.json)。

## 2026-10-14 18:24：取消 P 调参范围

- Nano同步9个更新文件，SHA-256逐文件核验；150项远端回归通过，伪串口3阶段通过。上位机允许P500/1000等值，P0关闭航向输出，不再采用0.01～200人为范围。
- 重启前确认未使能、七轴静止、无活动转角/巡航。18:24:11加载后台，PID12834→14610，active；H743/H723反馈恢复，仍未使能；三份现场参数哈希不变。
- 现场H743仍0.2.16。P范围在旧板端仍受限制，必须更新到本次0.2.18并重连才能使用超出旧范围的设置。代理未烧录、未修改实车P或发运动命令。
- 修改前远端包备份 ~/h743/backups/p_gain_20261014/，本地源码与固件备份 backups/p_gain_20261014/。记录见 [validation_p_gain_20261014.json](validation_p_gain_20261014.json)。

## 2026-10-14 18:08：辅助直线后台已加载

- 同步12个更新/新增文件，部署清单44项；SHA-256逐文件核验。Nano Python3.8 的148项回归通过，伪串口3阶段通过，未操作实车运动。
- 重启前再次确认未使能、7轴静止、巡航/转角无活动任务。18:08:25重启 h743-web.service，PID10764→12834，active；H743重连、H723航向可用，重启后仍未使能。
- auxiliary.json/navigation.json/device_links.json 三份现场参数哈希不变。修改前远端包备份 ~/h743/backups/gyro_straight_20261014/。
- 实车固件已由用户更新至0.2.16；辅助直线实际执行及巡航直线纠偏需要0.2.17。页面使用 straight=1 功能位提示升级，无此功能位不提交辅助直线命令。固件仅构建，代理未烧录。
- 记录见 [validation_gyro_straight_20261014.json](validation_gyro_straight_20261014.json)，使用见 [STRAIGHT.md](STRAIGHT.md)。
盒子：`a@192.168.55.1`（USB-C 直连），主机名 `1`，Ubuntu 20.04.6 LTS / L4T 35.4.1（JetPack 5.1.2）/ **Python 3.8.10**。

## 2026-10-14 17:24：用户授权重启加载新版后台

- 用户明确回答“现在重启”，于17:24:27重启 h743-web.service，active，PID1755→10764。H743/H723重新连接，H723 streaming，电机未使能；未恢复旧运动。
- 新辅助轴apply及转角接口存在，无令牌请求被正常拒绝；三份现场参数哈希保持一致。M5仍需在最高点确认升降方向并应用范围；M6已建立参考但仍需应用范围，未替用户自动修改参考/方向或机构范围。
- 现场固件仍0.2.15，新增可调角度容差需烧录0.2.16；本次未烧录。记录见 [validation_nano_restart_20261014.json](validation_nano_restart_20261014.json)。

## 2026-10-14：v0.2.16 共享陀螺仪参数

- 已同步三页共享P/角度容差输入、协议/API和实际板端回读；P0.01～200、容差0.01～10°，默认20/0.2°。对应固件0.2.16已生成，未烧录。
- 10文件更新、41文件哈希一致，140项远端测试和PTY三阶段通过。现场辅助轴/巡航/设备配置哈希均未变化，备份 /home/a/h743/backups/heading_tuning_20261014/。
- 只读检查实车已由用户更新0.2.15，P20；仍遵守用户等待调试结束再重启的要求，服务PID1755、14:35:14启动时间保持不变。新后台待重启加载；两项可调参数需0.2.16固件并刷新页面。证据见 [validation_heading_tuning_20261014.json](validation_heading_tuning_20261014.json)。

## 2026-10-14：v0.2.15 控制参数修复

- S1/PH9/270°、S2/PH6/180°保留；握手与状态确认物理映射。Web辅助轴保存并应用，新范围经板端确认后保存；新增可调P，默认20，完成误差±0.2°。对应固件要求0.2.15，HEX已生成但未烧录。
- 同步10个文件，40个文件哈希匹配；137项Nano测试和PTY三阶段通过。备份 `/home/a/h743/backups/control_fixes_20261014/`，三份现场参数未覆盖，哈希不变。
- 仍遵守用户等待调试结束的要求，未重启服务（PID1755、14:35:14启动保持不变）；新版后台待重启加载，当前实车固件仍为0.2.13。页面文件已同步，完整新功能需重启后台、更新固件后使用。证据见 [validation_control_fixes_20261014.json](validation_control_fixes_20261014.json)。

## 2026-10-14：v0.2.14 Nano 完整路线规划

- 已同步 Nano/PC 规划、BEGIN/WP/COMMIT 上传、H743 执行所需共享代码与 Web 页面。39 个部署文件与本地 SHA-256 一致，更新 9 个文件。后台启动前查询最新 NAV STAT，规划完整五腿任务；新巡航要求实际固件 `nav_path=1`，旧固件界面显示升级提示。
- Nano Python 3.8 的 135 项测试及 PTY 三阶段通过。测试中出现原有 camera-reader 退出时的 ProcessLookupError 日志，摄像头文件未修改；不影响本次路径测试结果。原程序备份 `/home/a/h743/backups/nano_path_executor_20261014/`，三份保存参数的哈希与备份一致，速度 2500 step/s 保留。
- **按用户明确要求，暂不重启 `h743-web.service`，等待实车调试结束。** 当前服务进程仍运行原后台，代码文件已更新；现场 H743 仍是 0.2.13、H723 正在 streaming。检查时实车已重新使能，未发送新的停止、使能或运动命令。
- 新固件 0.2.14 AXF/HEX/BIN 已在本地生成，未烧录。下一步需在调试结束后重启上位机服务，并更新 H743 固件，才可使用完整路径上传。光流接线、位置融合和动态绕障保留为后续阶段。证据见 [validation_nano_path_executor_20261014.json](validation_nano_path_executor_20261014.json)。

## 2026-10-14：田字格巡航与舵机引脚对调

- 已通过 `nano` 同步本地最新共享上位机及 Web 页面，新增 `navigation_grid.py`，更新田字格参数编辑/预览、巡航固件能力检查和 S1/PH9/TIM12_CH2、S2/PH6/TIM12_CH1 标注。模拟固件版本为 0.2.13。
- 38 个部署文件与本地 SHA-256 一致，其中 8 个更新。Nano 上 124 项 Python 测试、PTY 三阶段验证通过。旧程序备份在 `/home/a/h743/backups/web_grid_servo_20261014_143000/`。
- `h743-web.service` 已于 14:32 重启，active/enabled，保留 `--auto-devices`。页面 <http://192.168.1.197:8770/> 的舵机引脚标注、田字格六个坐标输入及 1440/390 px 布局检查通过，无 JavaScript 错误；浏览器检查未发送设备或运动命令。
- Nano 的 `navigation.json`、`auxiliary.json`、`device_links.json` 与部署前 SHA-256 一致；旧巡航参数在内存自动补齐通道坐标，保留原速度 2500 step/s。服务启动后 H743/H723 离线，系统串口枚举为空，自动连接持续等待设备。
- 本次只更新上位机，没有烧录 H743。田字格巡航要求实际固件具备 `nav_grid=1`，舵机对调要求烧录 0.2.13。证据见 `validation_nano_grid_servo_20261014.json`。

## 2026-10-13：v0.2.11 光流上位机

- 通过 SSH 别名 `nano`（Wi-Fi `192.168.1.197`）同步新版共享上位机，补齐 `turn.py`、`turn_sim.py`、`optical_flow.py` 的部署清单。Web 新增“光流位移”页，支持两种坐标、时段记录及换算校准。
- 37 个部署文件按 SHA-256 核对，更新其中 13 个。Nano Python 3.8.10 的 124 项测试通过；PTY 三阶段通过，握手显示模拟固件 `0.2.11 flow=1`。
- `h743-web.service` 已于 20:44 重启，active/enabled，继续使用 `--auto-devices`。页面 <http://192.168.1.197:8770/> 的光流坐标选择、离线按钮状态及桌面/390 px 布局通过浏览器检查，无 JS 错误。实际 H743/H723 当前离线，服务等待自动识别；浏览器验证没有发送设备或电机命令。
- Nano 的 `navigation.json`、`auxiliary.json`、`device_links.json` 保持原 SHA-256。原程序备份：`/home/a/h743/backups/web_flow_20261013_204037/`。
- v0.2.11 HEX/BIN 已存入 `~/h743/releases/v0.2.11/`，与本地哈希一致；本次没有烧录 H743。读取光流需要 H743 运行支持 `flow=1` 的固件。接线与校准见 [OPTICAL_FLOW_F7.md](OPTICAL_FLOW_F7.md)。部署证据见 [validation_nano_flow_20261013.json](validation_nano_flow_20261013.json)。

## 2026-10-11：Nano USB 摄像头按需拍照

- 页面 <http://192.168.1.197:8770/> 新增“摄像头”，默认来源为 Nano，读取 `/dev/video0` 的 **USB 2.0 Camera: HD USB Camera**；`video1` 为元数据节点，不列为可选摄像头。
- 使用顺序：打开摄像头 → 选择分辨率并应用 → 点击“拍照 · 获取一帧” → 查看照片 → 关闭摄像头。开机/打开网页不自动开启摄像头，打开设备后不持续读取/编码/传输照片；只有快门动作生成和传输一帧，设备驱动内部可能仍采集并缓存，关闭设备后释放 USB 资源。照片保留在浏览器，切换分辨率后需重新拍照。
- Nano 已安装 `v4l2-ctl` 与 OpenCV 4.9.0，用户 `a` 在 `video` 组。设备支持 320×240、640×480、800×600、1024×768、1280×720、1280×1024、1920×1080；界面仅列支持的档位。采集在可终止的子进程中，JPEG 保留最新一张，摄像头打开/关闭/拍照受页面控制权限制，无网络视频流或自动重试快门。
- 640×480 与 1920×1080 实物拍照通过，两次点击恰好两次图片请求，等待期间照片序号不变，镜像、关闭释放已验证。H743 `timeout=0`、错误计数为 0，H723 `streaming`；Nano Python 3.8 的 108 项测试通过，原有 PTY 三阶段通过。
- 修改前备份 `/home/a/h743/backups/web_camera_20261011_180809/`；部署清单 33 个文件逐项 SHA-256 复核。服务已重启载入新接口，H743/H723 自动重连成功，保持解除使能。坐标参数和芯片配对文件与备份哈希一致。
- 实物记录 `docs/validation_nano_camera_20261011.json`，截图 `tmp/nano_camera_desktop.png`、`tmp/nano_camera_mobile.png`。本机也使用相同的按需拍照界面，来源可选择浏览器本机摄像头；本机模式仍需 localhost 或 HTTPS。

## 2026-10-11：H723 图形与双设备自动连接

页面控制权修复：浏览器每 2 秒主动续租；旧页面超过 6 秒不响应会让出控制权。
权限通知按页面分别发送，避免服务端已经交权而新页面仍显示只读。
多个正常页面同时开启时，使用“接管控制”切换操作页面；接管会先发送 STOP
停止 H743 运动并解除使能，H723 数据继续显示。升级后需要刷新已打开的页面。

- 当前页面：<http://192.168.1.197:8770/>。新增“陀螺仪 · H723”页，包含带坐标轴的姿态曲线、X/Y 加速度曲线和 XY 加速度矢量图。
- 用户服务已改为 `--auto-devices`，不再默认 SIM，保持 active/enabled。无需打开网页，后台每 2 秒枚举并分别连接 H743/H723；设备缺席或异常断线时持续重试，串口编号改变后重新枚举。
- 只尝试对应 USB 类型，再核对板型/协议/芯片 UID。H723 必须先收到 IMU1，再通过随机挑战确认 DM-MC02/H723/BMI088；J-Link 和其他类型不作探测。首次确认的 UID 保存在 `host/device_links.json`，后续拒绝不同芯片替换。
- 实物已自动握手：H743 v0.2.8 `/dev/ttyUSB0`，UID `3637333530335109002A003A`；H723 v1.1.1 `/dev/ttyACM0`，UID `002600483134510438373332`。Nano 连续采集 1029 帧，约 100.17 Hz，坏帧 0。
- 主动关闭两条实物串口后均自动重新握手；服务重启后沿用配对并自动连接。实物物理拔插未另做测试；合成传输测试覆盖设备消失、再次出现和端口编号变化。
- H743 重连会 STOP/解除使能，只恢复通信，不恢复运动、巡航或接线确认。页面可以取消“自动连接设备”暂停重试，设置持久保存；需要保持断开时先暂停再断开。
- Nano Python 3.8 的 96 项测试及 PTY 三阶段通过；Edge 实物页面验证坐标轴、实时数值、只读权限及暂停设置。30 个部署文件按 SHA-256 核对；巡航参数哈希保持不变。
- 旧版备份：`/home/a/h743/backups/web_h723_20261011_173307/`。部署记录：本地 `docs/validation_nano_h723_20261011.json`；截图与详细实物记录：`D:\stm32H7\tmp\h723_usb_debug_20261011`。

## 2026-10-11：辅助轴页面、勾选与使能修复部署

- 用户实测确认默认反转掩码 5（M1/M3），新 HEX/BIN 已放到 `~/h743/releases/v0.2.8/` 并核对哈希；模拟器源码也同步默认 5。未烧录，现有真板运行时掩码仍以实际回读为准。
- 当前 Wi-Fi 为 `Bird-EFB7`，Nano 地址 `192.168.1.197`；页面 <http://192.168.1.197:8770/>。电脑 SSH 转发 <http://127.0.0.1:18770/> 已重建。
- 共享代码和 Web 页面已同步，包含 M5/M6 设置卡片、DIR 勾选草稿及 ARM/STOP 会话状态更新。21 个文件哈希一致，Nano 66 项测试和 PTY 三阶段通过；独立 SIM 浏览器验证通过。
- 旧版备份 `/home/a/h743/backups/web_aux_20261011_153317/`。保存的坐标参数未覆盖；辅助轴参数仍由 Nano 页面独立保存，与电脑 Tk 保存文件分开。
- 用户服务 active/enabled，重启仍默认 SIM。使用实物时先断开 SIM，再选 `/dev/ttyUSB0` 连接；实物升级前握手为 v0.2.7，AUX 新接口需要 v0.2.8，本次未烧录。

## 2026-10-10：v0.2.7 Web 上位机与 Wi-Fi

- 已部署坐标巡航模块、Web 页面和共享通信模块，20 个部署文件逐一通过 SHA-256 比对。修改前备份：`~/h743/backups/host_nav_20261010/`。
- Nano 已连接电脑当前 Wi-Fi `vivo S18 Pro`，NetworkManager 保存配置并启用自动重连；当日 WLAN 地址 `10.12.194.155`（DHCP，可能变化），外网 ping 2/2 成功。
- USB 页面：<http://192.168.55.1:8770/>；同 Wi-Fi 页面：<http://10.12.194.155:8770/>。拔掉 USB 后，电脑 SSH 别名 `nano` 改走 Wi-Fi `10.12.194.155`，`nano-usb` 保留 USB `192.168.55.1`；Wi-Fi SSH 登录已验证。
- 用户服务 `~/.config/systemd/user/h743-web.service` 已启用，`Linger=yes`，开机及退出 SSH 后保持运行。当前使用 `--demo`，未发现 STM32 串口设备。
- 参数保存在 Nano 的 `~/h743/host/navigation.json`。首次同步默认 O/A/B/C/D 和 3000 ms 停留；部署脚本不覆盖该文件，保留 Nano 上修改的参数。
- Nano Python 3.8：64 项 agent/control/webapp 测试通过；PTY 三阶段串口模拟通过。Edge 浏览器通过参数保存/刷新、建系、完整五段模拟巡航、停止及桌面/手机布局检查，结束后恢复默认参数并解除使能。
- v0.2.7 HEX/BIN 已放入 `~/h743/releases/v0.2.7/`，哈希与本地一致；本次未烧录。实车使用坐标功能需 STM32 运行 v0.2.7。

```bash
systemctl --user status h743-web.service
systemctl --user restart h743-web.service
systemctl --user stop h743-web.service
```

本地服务模板为 `tools/h743-web.service`，当前使用 `--auto-devices`。接入实物后后台自动握手；本地不加该参数时仍使用手动连接。

## 已完成的验证（2026-10-01）

| 项 | 结果 |
|---|---|
| USB-C 直连网络 | `192.168.55.1`，ping 0% 丢包；SSH 握手连续 25/25 成功 |
| 免密 SSH | `ssh nano` 直接登录，无需密码（`~/.ssh/config` 中 `nano` 别名，User=a） |
| 盒子有外网 | 是（`ping 1.1.1.1` 通，RTT ~120ms），但这与串口控制无关 |
| pyserial | 3.5，装于 `~/.local`（`pip3 install --user`，未动系统包） |
| 无界面客户端 | `host/agent.py` 部署到 `~/h743/host/`，Python 3.8 上握手成功 |
| 链路预演脚本 | `host/verify_link.py`，盒子侧 15 秒连续轮询 **9/9 通过**（0 失败、板端 timeout=0） |
| 心跳压测 | 盒子侧 30 秒：**118 次 PING / 0 失败**，板端 `timeout=0` |
| 值守模式 | 盒子侧 40 秒：0 重连、0 失败、板端 `timeout=0` |
| 链路自愈 | 单测覆盖：掐断链路后 `ensure_link()` 恢复并重做握手；watchdog 自动恢复且板端不记超时 |
| **真实串口路径** | `tests/pty_link_check.py`：伪终端 + 真实 `serial.Serial`，盒子侧 **3 阶段全过**（9/9 预演、6 条命令往返、值守 timeout=0）；打开参数/权限拒绝/设备缺失路径另有单测覆盖 |
| **负载下的租约** | `tests/lease_stress.py`：最坏 `tx_max_gap=0.434s`（loadavg 10.9 / 24 压载）；4K OpenCV 单帧 1049.6ms 时心跳间隙仍仅 0.261s |
| 测试套件 | `tests/test_agent.py` 31 项 + `tests/test_control.py` 16 项全过（盒子侧同样 47/47） |
| 控制 API | `host/control.py` 盒子侧实测：握手、`ARM`、有限行程按板端 `move_id` 判完成、`hold_velocity` 1.6s 续租 15 次后归零 |
| 部署一致性 | `tools/deploy_host.py` 按 SHA-256 逐个比对，拷贝后再复核一遍 |

**尚未验证**：真实 STM32 串口通信（板子还没接到盒子上）。PTY 检查走的是真实 pyserial 代码路径，
但它证明的是**盒子侧管线**（打开设备、115200 8N1、读超时、半帧重组），不证明电气电平、CH340X 与真实 UART。

## 部署到盒子

```powershell
python tools\deploy_host.py --check   # 只报告哪些文件与盒子不一致
python tools\deploy_host.py           # 同步 → 复核哈希 → 盒子侧单测 → PTY 真实串口检查
```

**每次改完 `host/` 下的共享文件（`protocol.py` / `transport.py` / `simulator.py`）都要跑一次**，
否则盒子上的副本会静默落后于工作区。这个坑已经踩过一次：`protocol.py` 里的
Python 3.9+ 写法在 Windows 上永远不报错，一上盒子就 `AttributeError`。

## 目录布局

```text
~/h743/host/     共享通信、control、navigation、webapp、webserver、static/、navigation.json
~/h743/tests/    agent/control/webapp 单测与 PTY 验证脚本
~/h743/releases/v0.2.7/  robot_h743.hex robot_h743.bin
```

部署共享通信层和 Web 上位机。`gui.py` / `ui.py` / `chassis_panel.py` 依赖 Tkinter，盒子侧不需要，也未部署；
`transport.py` 只在非 `SIM` 端口时才 `import serial`，所以无 GUI 环境下不会碰到 Tk。

## 运行

```bash
ssh nano
cd ~/h743

# 不接硬件自检（模拟板，验证协议与心跳链路）
python3 -m host.agent --demo --check

# 心跳稳定性压测（板端租约 1000ms，上位机每 250ms 一次 PING）
python3 -m host.agent --demo --json --heartbeat-seconds 30

# 接上 STM32 之后（CH340X 在盒子上通常枚举为 /dev/ttyUSB0）
python3 -m host.agent --port /dev/ttyUSB0 --check
python3 -m host.agent --port /dev/ttyUSB0          # 交互式命令行
python3 -m host.agent --port /dev/ttyUSB0 --script cmds.txt

# 链路预演：完整流程 + 连续心跳，接板前后都可以跑
python3 -m host.verify_link --demo --seconds 20
python3 -m host.verify_link --port /dev/ttyUSB0 --seconds 60

# 值守模式：长时间保持租约，链路断了自动重连；接板后的验收压测用它
python3 -m host.agent --port /dev/ttyUSB0 --watchdog-seconds 1800
python3 -m host.agent --port /dev/ttyUSB0 --json --watchdog-seconds 1800
```

值守模式退出时会打印汇总：`reconnects` 是链路重建次数，`board_timeouts` 是**板端自己记的**
租约超时次数。**只要 `board_timeouts` 不为 0，就说明 Jetson 的心跳在负载下迟到过**——
这是第一阶段唯一诚实的验收指标。退出码 1 表示确实出现超时或状态查询失败。

`verify_link.py` 做 9 项检查：握手 `protocol=1`、`INFO` 可读、LED 开/关各一次往返（应答里的 `led` 字段必须等于期望值）、
心跳有效后 `ARM 1` 被接受、持续轮询无失败、板端 `timeout=0`、`bad`/`rx_err` 计数为 0、`STOP` 解除使能。
**接上真板后，`timeout=0` 这一项才是第一阶段真正的验收指标**——它证明 Jetson 的心跳在负载下没迟到。

### 应答匹配是按标签，不是按命令正文

`transport.Connection.exchange(body, tag=...)` 会把 `tag` 原样带回对应的 `reply` 事件。
`Agent.request()` 每次生成唯一标签，因此**不可能把心跳自己的 `PING` 应答当成调用方的应答**。
（按正文匹配的旧设计有约 5.8% 的概率发生这种错配，是实测出来的。）
心跳与握手产生的应答没有标签，只会被 `submit()` 收集，`request()` 永不领取。

## 负载下的租约余量（实测）

板端租约是 1000ms。`tests/lease_stress.py` 在盒子本地制造负载，并在**串口写出的那一刻**打时间戳——
租约是在 PING 到达板子时刷新的，所以只有发送侧的间隙才有意义（用"收到应答"的时刻去算会得出偏大的假数字，
这一点我踩过：曾据此误报 1.118s 的缺口）。

| 场景 | loadavg | 最大发送间隙 | 板端 timeout |
|---|---:|---:|---:|
| 空载对照 20s | 0.23 | — | 0 |
| 6 压载 25s | 2.54 | — | 0 |
| 12 压载 20s | 5.71 | — | 0 |
| **24 压载 20s** | **10.90** | — | **0** |
| 6 压载 + 4 个 GIL 线程 25s | 5.73 | 0.434s | 0 |
| 12 压载 + 4 GIL 线程 + `nice +10` 25s | 9.15 | 0.311s | 0 |
| 6 压载 + 2 个 OpenCV 线程 1080p 25s | 4.73 | 0.261s | 0 |
| **6 压载 + 4 个 OpenCV 线程 4K 25s** | 7.69 | **0.261s** | **0** |

结论：在这台盒子上**我没能把租约压破**，最坏发送间隙 0.434s 对 1000ms 有约 2.3 倍余量，
连"推理进程优先级更高"（`nice +10`）也扛住了。

最后一行的意义最大：OpenCV 单帧最坏耗时 **1049.6ms，本身已超过租约**（`cv_max_frame_ms`），
但心跳间隙仍只有 0.261s。原因是 **OpenCV 在内部会释放 GIL**（多线程后端），
所以"一次很慢的 C 运算"并不会挡住心跳。**我原先的假设是错的**——曾以为 C 层运算会全程独占 GIL；
实测否定。真正会挡住心跳的是"长时间持锁且不释放"的 C 扩展（例如自己写的 `while` 循环里不放锁、
或某些未释放 GIL 的回调），这类代码需要在接入真实视觉管线时单独检查。

`lease_stress.py` 会如实报告 `cv_max_frame_ms` 与 `tx_max_gap_s` 两项，两者一起看才有意义：
前者说明视觉这一步有多慢，后者说明它是否真的妨碍了心跳。

## 视觉代码怎么调用（host/control.py）

`host/agent.py` 是传输层（请求/应答、重连）。**上层任务脚本应该用 `host/control.py`**，
它把两个最容易出人命的错误堵死了：

```python
from host.control import Control

board = Control("/dev/ttyUSB0")      # 或 Control("SIM") 做无硬件演练
board.connect()
board.require_protocol("1")          # 协议版本不符就拒绝开工

board.arm()
board.move_axis(5, steps=200, sps=2000, accel=4000)
board.wait_axis(5, move_id=..., timeout=10)   # 用板子自己的 remaining/active 判完成

board.hold_velocity((0, 0, 300, -300), seconds=1.6)   # 连续转 1.6 秒，结束自动归零
board.stop()
board.close()
```

状态字段返回的是 **int** 而不是协议里的字符串；`_ok` / `_body` / `_sequence` 保留，用于判断命令是否被接受。

两处安全设计，都有单测锁定：

1. **运动指令永不自动重发。** `MOVE` / `WHEELS` / `ARM` / `STOP` / `CHASSIS` 只发一次。
   链路中断时抛 `ControlError("... was not confirmed")`，由调用方重新读状态再决定——
   自动重发可能造成重复的物理动作。查询类指令才允许跨重连重试。
2. **`hold_velocity()` 失败即停车。** 固件租约只认得 PING，**不会因为 PING 而续期轮速目标**，
   所以持续运动必须反复重发目标。`hold_velocity` 每一轮都重发，并且无论正常结束、
   抛异常还是被 Ctrl-C 打断，`finally` 里都会发一次四轮归零。调用方卡死时结果是"停下"，
   而不是"照着最后一条指令继续跑"。

`hz` 必须大于 1（租约是 1 Hz），默认 10 Hz，即每个租约窗口内续 10 次。

盒子侧实测（`SIM`，Python 3.8.10）：

```
HELLO: {'fw': '0.2.0', 'protocol': '1', 'chassis': 1, 'serial': 'dual'}
armed: 1
move accepted: accepted move_id: 1160161255
move finished: pos=200 remaining=0 active=0
hold_velocity: {'seconds': 1.6, 'renewals': 15, 'hz': 10.0, 'speeds': (0, 0, 300, -300)}
status after: {'armed': 1, 'timeout': 0, 'bad': 0, 'rx_err': 0}
```

## 接 STM32 之前必须先解决串口权限

用户 `a` **不在 `dialout` 组**（盒子上的 `dialout` 组存在但成员为空），
所以打开 `/dev/ttyUSB0` 会得到 `Permission denied`。二选一：

```bash
# 方案一：加入 dialout 组（推荐，一次到位，需重新登录生效）
sudo usermod -aG dialout a

# 方案二：不改系统组，每次提权运行
sudo python3 -m host.agent --port /dev/ttyUSB0 --check
```

## 接线

盒子 USB-A → STM32 板上的 **COM Type-C**（板载 CH340X → USART1 PA9/PA10），115200 8N1。
第二路 UART7（PA8/PA15）是无线调试口，与盒子这条有线链路互不干扰；固件应答会双发，
因此 PC 端仍可用原 GUI 通过无线口被动观察。

## 已知边界

- 板端心跳超时 1000ms 后执行 `STOP` 并解除 ARM。Jetson 侧若被视觉任务抢占 CPU，
  `transport.py` 的心跳线程一旦迟发超过 1 秒，板子会自行停止——这是设计行为，不是故障。
- 当前协议**只有 `CHASSIS JOG` 有自己的租约**（400ms），`MOVE` / `WHEELS` 没有。
  上位机卡死但 PING 线程仍在时，板子会保持最后一条速度指令继续跑。
  这一条在进入"Jetson 自主决策"阶段前必须处理。
- 反馈是轮询式的：没有异步完成事件，`AXIS` / `CHASSIS STAT` 查询 `move_id` 判断是否完成。
- 盒子侧 Python 3.8 无法升级（JetPack 5 定版），因此 `host/` 下的代码必须保持 3.8 兼容：
  不要使用 `str.removeprefix` / `removesuffix`（3.9+）、海象运算符以外的 3.10+ 语法或 `X | None` 注解。
  已修复 `protocol.py` 与 `simulator.py` 中的两处 `remove*fix` 调用。
- **一次丢帧不再终结链路**（已解决）：`transport.exchange()` 重试一次仍失败时工作线程会退出，
  原先这会让 `Agent.connect()` 永久卡在 "already connected"。现在 `Agent.ensure_link()`
  会重连并重做 HELLO/PING/INFO，`run_watchdog()` 与 `request_resilient()` 在此之上自动恢复。
  **只有幂等命令会被自动重发**：`PING` / `STAT` / `HELLO` / `INFO` / `AXIS` / `LED` / `SERVO` / `CHASSIS STAT`。
  `MOVE` / `WHEELS` / `ARM` / `STOP` / `CHASSIS MOVE|JOG` **一律不重发**——重发可能造成重复的物理动作，
  且链路中断后板端的 8 条去重缓存已经不可依赖。对非幂等命令调用 `request_resilient` 会直接抛 `ValueError`。
  重连本身永远不会让电机转起来：板端 1000ms 租约早已超时并执行了 `STOP` + 解除使能。
- `request()` 超时**抛 `AgentError`**，链路已死则抛 `LinkLost`（`AgentError` 的子类）。
  返回 `None` 会让调用方把"没应答"和"板子回了空"混为一谈。超时默认 1.5 秒。
