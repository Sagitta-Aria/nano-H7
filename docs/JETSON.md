# Jetson 侧部署与运行（第一阶段）

本页记录把上位机通信层搬到 Jetson 盒子后的状态、验证结果和待办。

## 2026-10-22：NX 面板一键启动部署

NX 上电整屏直接进入两轮搬运启动页，可选 O/X 起点，一个按钮跑完整流程，不再需要浏览器或调试页。

- 新增 `host/launcher.py`（800×480 Tk 全屏启动页：O/X 选择、开始/停止、连接与阶段状态）、`tools/h743-launcher.service`（用户级 systemd，`DISPLAY=:0`，`Restart=always`）、`docs/LAUNCHER.md`、`tests/test_launcher.py`。
- `/api/competition/start` 新增 `start_station`，按名字解析 O/X，只作用于本次运行，不改写 `navigation.json` 的 `start_station`。
- 会话：启动页用独立 token 打开 `/api/events` 注册并每 1.5 s 心跳，持有控制权；浏览器调试页在空闲时仍可“接管控制”，面板下次按键会重新 claim。运行期间启动页取消置顶并下沉，把最前层让给摄像头预览与任务码卡，结束/故障后自动回到最前。
- 备份 `~/h743/backups/launcher_20261022_1414/`（host、docs、tests、tools，323 个文件）。
- 部署 6 个文件（`host/launcher.py`、`host/competition.py`、`host/webserver.py`、`tests/test_launcher.py`、`tools/h743-launcher.service`、`docs/LAUNCHER.md`），SHA-256 与本地一致，远端 `py_compile` 通过；远端 `test_launcher` + `test_competition` 共 16 项通过。
- 服务：`h743-web.service` 重启 PID 4273；`h743-launcher.service` enabled + active，实屏截图确认整屏启动页，右上角显示“H743 已连接 · 未使能 · 固件 0.2.29”。
- 安全验证：按面板同一调用链（join → claim → start X）返回“巡航配置里没有 X 点，请先在调试页添加并保存”，`competition` 仍为 idle、未使能、七轴静止；未启动实车任务。
- 待办：盒子上巡航配置目前只有 O/A/B/C/D，没有 X 站点，所以启动页的 X 按钮灰显；需要在调试页“添加 X 点”再保存（或按同一接口添加）后才会可选。

验证记录见 [validation_nx_launcher_20261022.json](validation_nx_launcher_20261022.json)。

## 2026-10-22：C/D 任务后位姿校准部署

通过 `nx-bird` 定向更新 7 个文件：`host/mission.py`、`host/material_flow.py`、`host/navigation_runner.py`、`host/webapp.py`、`docs/MATERIAL_FLOW.md`、`tests/test_ring_task.py`、`tests/test_material_flow.py`。C/D 视觉任务结束后的位置与航向都按视觉实测重新校准。

- 位置：接管导航权限时先发 `NAV POSE`（在视觉动作之前），摆正/靠近/抓放/转盘的全部指令脉冲由 H743 里程积分累计；任务结束按「到站点坐标 − 当前坐标」的实测差分段正交回位，残余 ≤20 脉冲后 `NAV RESTORE` 到站点。`right_steps` 不再参与退出（字段保留，退出点恒等于到站点）。
- 航向：视觉平行后 `δ = heading_cd − 配置到达姿态`，`|δ| > 6°` 直接报错；否则 δ 加到后续每个 `NAV EXEC` 的下发角。板端地图原点与 yaw 原点不动，站点姿态表不变；无偏置时下发角与配置角完全一致。重建起点（NAV RESET）时偏置清零。
- 备份 `~/h743/backups/pose_recalib_20261022_1322/`（host、docs、tests，296 个文件）。
- 7 个文件 SHA-256 与本地一致，远端 `py_compile` 通过。
- 服务重启 PID 5467→8729，active，13:27:02 启动；H743 已连接（0.2.29、`nav_reference=1`）、H723 streaming、七轴静止未使能；二维码 worker 自动恢复；`navigation_runner` 遥测已带 `bias_cd`（当前 0）。
- 远端回归 83 项通过（`test_ring_task`、`test_material_flow`、`test_mission`、`test_competition`、`test_webapp`）。
- 本地全量 698 项仅剩 6 个已知 `imu_stale`ARM 断言失败。
- 未烧录固件、未执行实车任务。现场需确认两件事：C/D 收尾多出的回位段（1800/2150 脉冲量级）与每站的实际位姿是否吻合；视觉平行后的 δ 是否稳定落在 ±6° 内。

验证记录见 [validation_nx_pose_recalib_20261022.json](validation_nx_pose_recalib_20261022.json)。

## 2026-10-22：任务码全屏卡部署

通过 `nx-bird` 定向更新 4 个文件：`host/camera.py`、`host/camera_monitor.py`、`tests/test_task_code_overlay.py`、`docs/MATERIAL_FLOW.md`。两轮搬运锁定二维码任务码后，NX 本地 800×480 屏整屏切换为纯白底、黑色任务码卡片，任务结束自动恢复相机预览。

- 备份 `~/h743/backups/task_code_card_20261022_1235/`（host、docs、tests，294 个文件）。
- 4 个文件拷贝后 SHA-256 与本地一致，远端 `py_compile` 通过。
- 服务重启 PID 1807→5467，active，12:36:13 启动；H743 已连接（**固件 0.2.29、`nav_reference=1`**）、H723 streaming、七轴静止未使能；二维码 worker（`/dev/video2`）自动恢复。
- 远端回归：`tests.test_task_code_overlay` + `tests.test_camera_monitor_modes` 共 11 项通过。
- 部署后 `RecognitionMonitor.draw()` 实测：有任务码时整屏纯白占比 96.6%、任务码单行居中、字高 55 px（下限 12 px）、占宽 696 px；清空任务码后回到正常预览（纯白占比 0.17%）。
- 真机整屏截图未取得：NX 桌面当时挂着"软件更新器"弹窗占用前台，OpenCV 窗口未到最前。不影响功能，现场使用前建议关掉该弹窗。
- 未烧录固件、未启动实车任务。

验证记录见 [validation_nx_task_code_card_20261022.json](validation_nx_task_code_card_20261022.json)。

## 2026-10-21：坐标巡航草稿区移除与坐标编辑部署（补做中断的部署）

通过 `nx-bird` 定向更新 6 个上位机文件：`host/navigation.py`、`host/navigation_runner.py`、`host/webapp.py`、`host/static/index.html`、`host/static/app.js`、`host/static/styles.css`。这轮原本是上一场会话结尾执行的部署，那次停在"开始定向部署"之后：文件已备好、哈希已核对，拷贝没有发生。本次补做。

坐标巡航页不再有草稿图和"应用范围到草稿"按钮，场地左/下边界、宽、高、中间竖/横通道距边界可直接编辑并保存；站点暂时离开通道线也可保存，建立起点与启动巡航前才严格校验。部署前线上 `index.html` 仍含 `nav-field-apply`，`app.js` 仍含 `previewMission` 与 `applyFieldEditor`，与本轮本地版本不同。

- 部署前预检：服务 active（PID 4997），`armed=false`，七轴静止（active 全 0），导航 idle，无任务/比赛在跑；固件 0.2.28，`INFO` 无 `nav_reference`，因此代码继续拒绝 X 起点建系。
- 备份 `~/h743/backups/nav_editing_20261021_20261021_2049/`（host、tests、tools、docs、CHANGELOG，316 个文件）。
- 6 个文件 scp 后逐个 SHA-256 与本地一致；远端 `py_compile` 通过。
- 服务重启 PID 4997→5835，active，启动时间 20:54:05；H743 已连接、H723 streaming，`armed=false`，七轴静止，导航 idle。
- 页面核验：`curl` 首页 200、75100 字节；`nav-field-apply=0`、`previewMission=0`、`applyFieldEditor=0`、`nav-field-editor=1`、`function fieldGrid=1`、`readFieldGrid=2`。
- 10 份现场 JSON 的 SHA-256 与部署前完全一致（device_links、control_settings、auxiliary、mission、navigation、b_pickup、camera_alignment、material_flow、pick_place、cameras）。
- NX 端逐模块回归：test_stations、test_webapp、test_webpage、test_competition、test_path_upload、test_mission、test_center_navigation、test_pick_place、test_task_code 全部通过；test_navigation、test_navigation_timing、test_navigation_regressions 因 NX 未安装 `unicorn`（ARM 仿真依赖）无法导入，与本次改动无关。
- 未烧录固件、未执行实车运动；X 起点仍被 `nav_reference` 门禁拒绝。
- 另有 9 个文件与本地不一致，本轮未动：`host/webserver.py`、`host/servo_config.py`、`host/competition.py`、`host/static/competition.html`、`host/static/competition.js`、`host/simulator.py`、`host/navigation_sim.py`、`tests/test_webapp.py`、`tests/test_servo_connection.py`。

验证记录见 [validation_nx_navigation_editing_deploy_20261021.json](validation_nx_navigation_editing_deploy_20261021.json)。

## 2026-10-21：二维码放置顺序与到顶等待部署

通过 `nx-bird` 定向更新 `host/task_code.py`、`host/pick_place.py` 及三个对应测试文件。物料从转盘反向取出时，颜色和放置目标同步反序，保持二维码同一位置的颜色与目标配对；摆放取料夹紧后，先确认升降轴停止且脉冲位置为0，再等待500ms，才允许旋转轴回0。NX隔离回归95项全部通过，并在正式目录核对两批任务映射、等待步骤和五个文件哈希。服务 active（PID 12414），H743 connected、H723 streaming，七轴静止且未使能；10份主机JSON及6份颜色阈值哈希不变。备份 `/home/a/h743/backups/qr_order_top_wait_20261021`，未执行实物任务或烧录；机械到顶情况仍需实车验证。[验证记录](validation_nx_qr_order_top_wait_20261021.json)。

## 2026-10-21：B点识别后延迟抓取

通过 `nx-bird` 定向更新5个程序/页面文件及B点测试模块；正式两轮流程与独立B点测试在连续3帧确认目标色后等待默认2000ms再合爪，可在“B点取料参数”中调为0～60000ms。NX隔离回归91项验证通过（两项隔离工具路径缺失修复后复测通过）；在线网页1440/390px确认延迟字段和默认值，无溢出或JavaScript错误。服务 active（PID 11878），H743 connected、H723 streaming，七轴静止且已解除使能；10份主机JSON和6份颜色阈值哈希不变。备份 `/home/a/h743/backups/b_delay_20261021`，未执行实物抓取或烧录。[验证记录](validation_nx_b_delay_20261021.json)。

## 2026-10-20：第二次D点叠放部署

通过 `nx-bird` 定向部署12个程序/页面文件及17个测试模块，NX隔离测试248项全通过，Python编译及文件/HTTP模块哈希一致。服务重启后 active（PID 18291），H743已连接、H723 streaming，七轴静止且未使能；九份现场JSON哈希不变。网页新增独立下降高度 `d_second_drop_down=-1`（待标定），B点参数隔离、共享点距2150/4230、1440/390px页面检查通过。第二次D使用第一层颜色记忆和圆环定位，不运行数字OCR。备份 `/home/a/h743/backups/d_second_20261020_165945`，未烧录或启动实车任务。[验证记录](validation_nx_d_second_20261020.json)。

当前 SSH 入口为 `ssh nx`，自动尝试 USB `192.168.55.1`、固定 Wi-Fi `192.168.1.196` 和 vivo 热点 `10.12.194.155`。指定链路可用 `nx-usb`、`nx-bird`、`nx-vivo`。本机代理脚本已更名为 `~/.ssh/nx_proxy.py`，项目部署脚本使用 `nx`；下方历史记录中的 `nano` 和旧 Wi-Fi 地址不再作为当前连接配置。

## 2026-10-18：USB 部署启动调度、参数保存与单键比赛页

- 通过 `nx-usb` 安装 25 个文件，基于 NX 现场源码合并本次修复；保留原有相机、识别和调试视觉界面。路线上传与任务指令优先，应答唤醒调度；默认首页只有启动按钮，调试入口 `/debug`。
- `h743-web.service` 已重启并 active，PID 5222。在线浏览器确认单键页面、参数回填和调试页面正常，无 JS 错误；未点击启动或执行实车运动。
- 六份原有 JSON 的 SHA-256 全部保持一致，路线速度 3000、加速度 5000 保留；新增 `control_settings.json` 保存更新前板端回读的转向 `[2000,20,200]` 和底盘反转 `[5]`。升降方向尚未设置，因此未编造方向。
- NX 隔离目录首轮 337 项，332 通过、1 跳过、4 条旧断言失败；同步本地已验证的调参标签和大偏差行为测试后，46 项补测通过，覆盖全部失败用例。PTY 三阶段通过。
- H723 已恢复 streaming；H743 的 CH340 在更新期间从 USB 列表消失，用户明确选择暂不接回。因此本轮只完成 NX 更新，未烧录 H743，最后读到的固件仍为 0.2.25。启动提速已加载；固件端偏航恢复和转向确认变化仍需以后烧录。
- 备份 `/home/a/h743/backups/startup_027_20261018_202130/`；记录见 [validation_nx_startup_20261018.json](validation_nx_startup_20261018.json)。

## 2026-10-18：抓取与放置部署

- 通过 `nx-bird`（192.168.1.196）部署 16 个抓放相关程序、页面、测试与说明文件。自动 `nx` 入口本次选中不可用链路，改走已验证主机密钥的 Wi-Fi 入口。以 NX 现有源码为基线，仅合并抓放相关变更，未将其他未部署的视觉调参改动一并覆盖。
- 服务重启后 active，PID 6036；H743 0.2.25、H723 航向与扫码相机在线，七轴静止、未使能，新模块 idle。现场五份 JSON 参数 SHA-256 保持一致；默认抓取/放置下降 9200、旋转 -1750，19 项参数可见。
- NX 隔离目录回归运行 318 项，1 项跳过，发现 1 条旧测试仍期待对调前的 S1 命令；同步现有通道修正后，任务/抓放/任务码 50 项补测全部通过。PTY 三阶段通过，在线页面 1440/390 px 无溢出或 JavaScript 错误，页面检查未发送运动请求。
- 备份 `/home/a/h743/backups/pick_place_20261018_180349/`；证据见 [validation_nx_pick_place_20261018.json](validation_nx_pick_place_20261018.json)。未烧录、未启动抓放流程；连接初始化沿用 S1=120°、S2=0°。板端 M5 仍为 0～8000，M6 尚未建立零位且范围 0～0，需操作员设置实际范围后再试运行。

## 2026-10-16：重复零速度与 STEP 收尾拒绝兼容修复

- 持久记录捕获 16:17:20、16:18:10、16:19:14 三次错误，全部是 `WHEELS 0 0 0 0 accel` 在此前已被接受后再次被拒绝；加速度 2000 和 4000 均出现，当前主机仍显示使能，板端无锁存运动故障。此前的旧 idle 应答修复没有覆盖该零速度重复路径。
- 运行现有 Keil ARMCC5 AXF 的 ARM 模拟器重放：连续轮速运行，STEP 高电平时请求零速度并减速到零，四轮 active=15、armed=1、fault=0；重复零速度返回 `DISARMED_BUSY_OR_REVERSE`，补完下降沿后同一指令返回 OK。源码 `motion_wheels()` 对 active && stopping 的更新拒绝包含零速度，和现场日志一致。
- NX 串口线程对同一速度代次的零目标收到成功 ACK 后不再周期重发；继续正常心跳、250 ms 控制租约监督和实际停车查询。新指令和非零速度保留原发送方式，STOP/清除租约清空零速度缓存。同方向再次启动也须等实际停稳，防止抢占 STEP 收尾。固件与现场加速度参数均不修改。
- 部署前备份 `/home/a/h743/backups/ring_sequence_20261016_162407/`。验证记录见 [validation_nx_zero_stop_fix_20261016.json](validation_nx_zero_stop_fix_20261016.json)。
- 本地 422 项（跳过 1 项）、相机专项 62 项、NX 端 251 项及 PTY 三阶段通过。72 项部署清单哈希一致；设备、辅助轴及导航三份 JSON 未变化，相机参数在部署期间被外部更新，本次部署不复制 JSON，已保留最新值：上限 3000、最低 600、Kp=1.2、Kd=0.1、加速度 4000、搜索速度 2000。服务 PID 12215，H743/H723/摄像头在线，七轴静止且未使能；未发起实车运动、未烧录固件。

## 2026-10-16：四轮速度拒绝与停车应答竞态修复

- 现场错误保留在数字 1 的 approach 阶段：`DISARMED_BUSY_OR_REVERSE`，板端无锁存运动故障。NX systemd 日志只含服务启动信息，指令日志在内存中且接口只返回最近 120 条，故障前指令已被后续 `MOTION STAT` 覆盖，不能从旧日志确定现场当时的全部时序。
- 在 NX 更新前版本的模拟串口中注入延迟 idle 应答，准确复现相同错误：先查询空闲，新帧恢复运动，旧 idle 应答误清方向保护，再收到反方向微调，被板端拒绝。另复现零速度尚未 ACK 就查询停车的缺口。搜索有限步长中锁定数字的交接回归确认已有 HALT、原运动编号、实际空闲、TURN ZERO、视觉速度的正确顺序。
- 修复使用每条速度目标的代次和零速度 ACK 屏障：当前零速度已被板端确认才查询实际空闲；空闲应答须匹配同一代次且当前仍在 braking/settling，才可清方向保护和进入下一阶段。旧 idle/旧零速度 ACK 都不能释放当前换向。有限运动到视觉靠近交接会重置停止证据。
- 速度指令被拒绝时，STOP 前保存原指令、视觉、方向保护、期望轮速、最近 80 条请求/应答及板端缓存到 `host/logs/camera_alignment_faults.jsonl`，接口保留 `fault_context`。这些是指令与缓存证据，不是 ZDT 编码器反馈。
- 本地全量 420 项测试通过（跳过 1 项），相机/圆列专项 60 项、网页三种宽度通过。部署备份 `/home/a/h743/backups/ring_sequence_20261016_160517/`，运行核验见 [validation_nx_wheel_reverse_fix_20261016.json](validation_nx_wheel_reverse_fix_20261016.json)。
- NX 远端 249 项回归及 PTY 三阶段通过，72 项部署文件 SHA-256 一致。服务已加载修复，PID 9324，H743/H723 和摄像头在线；七轴静止、未使能。四份现场 JSON 参数哈希均未变化，本次不烧录固件、不发起实车运动。

## 2026-10-16：摆正丢圆恢复与三个数字逐点校准

- 修复摆正期间丢圆后停留在 align 阶段直到超时、转向时未持续取帧的问题。连续三个新鲜帧丢失双圆时停止转向，确认原运动编号和四轮空闲后重新前后搜索；只接受前进方向重获双圆，停车后再完整前进 500 脉冲，静置并用新帧重新连线摆正。搜索仍受 ±X 边界约束，额外 500 脉冲可能越出搜索边界；一轮失败和总流程超时仍停车。
- 1、2、3 均须完成视觉 PID 靠近、连续三帧死区、航向和实际停止确认。1 对准后原地等 3 秒，再向前找并对准 2；2 完成后再等 3 秒，向前找并对准 3；3 完成后结束。等待期间不发运动命令且不累计下一数字票数。
- 本地全量 415 项测试通过（跳过 1 项），相机闭环专项 62 项、网页本地三种宽度检查通过。新增回归先复现丢圆停滞和缺少等待，再验证转向停止、恢复搜索方向、完整 500 脉冲、新鲜帧重新摆正、三个数字均靠近并等待。
- NX 端 244 项回归与 PTY 三阶段通过，72 项部署文件 SHA-256 一致。服务已重启加载，PID 5659；H743/H723 和摄像头恢复在线，七轴静止且未使能。实际页面在 1440/760/390 px 无溢出或 JavaScript 错误，检查拦截全部写请求。
- 本次保留现场 JSON 参数，不烧录 MCU、不启动车辆自动流程。更新前备份 `/home/a/h743/backups/ring_sequence_20261016_152658/`；部署与运行核验见 [validation_nx_ring_recovery_20261016.json](validation_nx_ring_recovery_20261016.json)。

## 2026-10-16 15:01：圆列流程检查与数字 1 靠近修复

- 保留用户接受的先后退再前进、可调搜索范围。修复运动中双圆结果被忽略的问题：连续三帧发现双圆先 HALT，核对剩余脉冲和实际空闲，再用停车后新帧确认后转向。数字搜索中发现当前目标也按同样方式停车复核。
- 新增数字 1 靠近阶段：三帧确认 1 后，记录摆正后的 H723 朝向，复用视觉 PID 与目标偏移；三帧进入死区、航向合格且四轮实际停止后才开始找 2。丢失 1 先置零速度，三秒仍丢失则停止。实际识别到的 2/3 也显示到网页，不能越序推进。
- 修复识别流切换未更新控制代次的问题，迟到的旧线程错误不再误停新流。回归复现以上三个问题后修复；本地 408 项测试通过（跳过 1 项），相机专项 91 项、远端 237 项和 PTY 三阶段通过，实际网页三种宽度检查通过。离线 GPU 同帧三圆及数字 1/2/3 识别通过。
- 服务 PID 9305，H743/H723 连接正常，现场四份 JSON 参数哈希保持一致，包含用户当前 Y 偏移 −30。七轴静止、未使能，未发起实车运动。重启后发现 NX 没有 `/dev/video*`，摄像头离线，无法核对实拍；部署文件 72 项 SHA-256 一致。
- 备份 `/home/a/h743/backups/ring_sequence_20261016_145807/`；记录：[validation_nx_ring_fixes_20261016.json](validation_nx_ring_fixes_20261016.json)。

## 2026-10-16 14:39：圆列搜索、平行校正与 1→2→3 识别已部署

- 更新 18 个程序/页面/测试文件，72 项部署清单 SHA-256 全部一致。圆环模式在 ±X 脉冲内分段前后搜索同帧双圆，按圆心连线和安装角调整底盘至平行，再从后方寻找 1、向前依次确认 2、3。默认 X=2000、步长 200、速度 300 step/s，网页可调。使用现有固件的有限脉冲直线与 H723 闭环转向接口，本次未烧录。
- 14:39 重启用户级 `h743-web.service`，PID 1604→6616，active。H743 v0.2.21、H723 和 `/dev/video0` 1280×720 相机自动恢复；NX 800×480 实屏保持全屏显示。部署后车辆未使能、七轴静止，未启动实车圆列流程。
- `device_links.json`、`auxiliary.json`、`navigation.json`、`camera_alignment.json` 的哈希保持不变。保留现场 Kp=5、加速度 3000、速度上限 2000 及圆环模式；缺少的安装角 +90° 和圆列参数仅在内存补全，显式保存才写回。
- 232 项远端回归和 PTY 三阶段通过。NX 原 GPU 环境的 1280×720 合成三圆验证：连线误差 0°，数字 1/2/3 分别识别成功，置信度约 0.75/0.98/1.00，单帧处理约 337/264/245 ms。实际网页在 1440/760/390 px 无横向溢出或 JavaScript 异常，验证期间拦截全部写请求。
- 更新前备份 `/home/a/h743/backups/ring_sequence_20261016_143530/`；验证记录：[validation_nx_ring_sequence_20261016.json](validation_nx_ring_sequence_20261016.json)。实车转向方向、透视影响和搜索范围仍需现场验证，见 [CAMERA_ALIGNMENT.md](CAMERA_ALIGNMENT.md)。

## 2026-10-16 13:46：恢复 NX 识别数据显示与触屏切换

- 修复新版默认相机窗口漏接原数字/色块显示的问题。恢复右侧中文触屏按钮、GPU 数字与置信度、颜色、FPS、原图坐标、偏移控制点、X/Y 偏差。800×480 显示保留 16:9 画面，界面缩放不改变原图坐标。
- 复用旧 `VisionModes` 和 GPU 引擎，消费同一相机进程的最新原始帧，不另开相机。待机持续识别；对准期间暂停预览推理，显示闭环实际使用的识别报告，结束后恢复数字/色块预览。按钮只切换预览，对准目标仍由网页配置决定。
- 13:46:08 重启 `h743-web.service`，PID 25703，active。相机子进程使用原 `/home/a/vision-env/bin/python`；主服务仍用系统 Python。备份 `/home/a/h743/backups/monitor_restore_20261016_134022/`，全部现场 JSON 保持原哈希，包括用户修改后的 Kp=5、加速度=3000、速度上限=2000。
- 211 项远端回归、PTY 三阶段、最终 49 项本地相机专项通过。NX GPU 离线合成图验证数字 1（置信度 0.77）、红色色块和切换时复用 GPU；中心 `(640,330)`、控制点 `(640,360)`、误差 `(0,0)`。实屏截图确认触屏按钮和实时状态；现场镜头无靶标，显示未找到目标。未发起实车对准或运动。
- 记录：[validation_nx_monitor_restore_20261016.json](validation_nx_monitor_restore_20261016.json)。

## 2026-10-16 13:29：视觉 PID 四轮速度闭环与 NX 默认实时显示

- 摄像头一取消固定步数、单轴试探和反复 TURN MOVE。NX 根据新帧的 X/Y 像素误差计算 PID 平移速度，叠加启动航向的陀螺仪 P 纠偏，按已有轮序与反向掩码混合成四轮连续 `WHEELS` 速度。进入死区停车确认；轮子换向前先减速并确认实际停止。
- 已写入速度上限 2000、最低靠近 300 step/s、加速度 6000 step/s²、死区 5 px、PID 10/0/0.15。保存原目标类型、偏移与方向。其他现场 JSON 配置哈希不变，备份 `/home/a/h743/backups/visual_pid_20261016_132206/`。
- 四轮目标使用 NX 控制循环续期的 250 ms 租约，独立于串口 PING。视觉中断先请求零速度，3 秒无有效结果 STOP；航向失效、断连、运动故障均停止。停止/完成取消旧目标和迟到回调。
- NX 显示器默认全屏显示 `/dev/video0` 的 1280×720 实时画面，实际桌面 800×480。服务启动自动打开相机并退出 GNOME 活动概览；显示与识别共享单一原始帧采集，网页拍照独立，不自动启动识别或运动。物理屏幕截图确认是实时全屏画面，服务已启用开机启动。
- 13:29:40 重启 `h743-web.service`，最终 PID 21859，active，H743/H723 自动重连，实车未使能、七轴静止。本次未烧录或发起实车对准。
- 60 项部署文件哈希一致；210 项远端回归与 PTY 三阶段通过，最终相机/PID/串口租约专项 48 项通过。模拟四轮位移反馈可双轴收敛，覆盖持续速度、接近减速、换向、失去反馈及控制循环冻结停车；浏览器和 1440/760/390 px 布局检查通过。
- 记录：[validation_nx_visual_pid_20261016.json](validation_nx_visual_pid_20261016.json)；用法：[CAMERA_ALIGNMENT.md](CAMERA_ALIGNMENT.md)。

## 2026-10-16 12:46：NX 连续识别与网页照片拆分

- 修正摄像头一对准依赖单次拍照、JPEG 和后续串口查询导致首帧过期的问题。NX 原始帧采集、连续识别与控制独立运行；网页“拍照”只有查看用途，识别不调用快门、编码 JPEG 或上传本机照片，对准期间仍可手动拍照。
- 相机采集持续读取，只保留最新帧；控制只采用运动完成后采集的新帧。偶发慢帧或重复结果先丢弃等待下一帧，3 秒没有新鲜结果才停止；单调时钟与帧编号共同检查。对准运行时减少无关页面轮询，保留使能与故障检查。圆环阈值计算改用 OpenCV 通道极值，像素分割与原规则一致，原始分辨率及坐标保持不变。
- 201 项远端回归和 PTY 三阶段通过；最终相机专项 42 项通过，包含阈值逐像素一致性和 Python 3.8 相机退出竞争检查。浏览器验证拍照不触发识别、NX 识别不调用拍照或移动，实际页面三种宽度无 JS 异常。
- 实际 `/dev/video0`、1280×720 在零拍照请求下连续输出 8 次识别，再单独拍照，识别继续输出；结果帧龄约 343–406 ms。相同真实连续帧另接内存 SIM 控制板，在无浏览器条件下完成 3 次模拟靠近，未丢帧、未请求额外照片。没有对实车使能或发送运动指令，未验证实车收敛效果。
- 12:46:32 重启 `h743-web.service`，PID 11020→15975，active。H743 与 H723 已重连、解除使能、七轴静止。现场配置哈希不变。
- 备份 `/home/a/h743/backups/continuous_alignment_20261016_124117/`；记录 [validation_nx_continuous_alignment_20261016.json](validation_nx_continuous_alignment_20261016.json)。

## 2026-10-16 12:20：按目标像素方向靠近，移除相机对准标定要求

- 按用户纠正，摄像头一改为 `(X+偏移X,Y+偏移Y)` 控制点靠近画面中心，默认 `(X,Y+30)`。像素偏差只判断方向和死区，速度决定每段靠近大小；进入死区停车并连续三帧确认，H723 保持启动时车头方向。四系数、矩阵和标定确认要求已从摄像头入口删除，主界面只需调偏移、速度和死区。
- 更新 6 个主机/页面/测试文件，完整部署清单 58 项哈希一致。NX Python 3.8 的 195 项回归与 PTY 三阶段通过；合成双轴收敛、色块/圆环及 `Y+30` 正号验证通过。实际页面的 1440/760/390 px 布局通过，无 JS 异常，未发送实车运动指令。
- 12:20:05 重启 `h743-web.service`，PID 8055→11020，active。H743 v0.2.21 和 H723 自动重连、七轴静止、解除使能；未烧录、未打开真实相机或运行实车对准。
- 原设备配对、辅助轴、路线和相机配置文件哈希不变；旧相机配置在内存中兼容转换，保留偏移、速度、死区，显式保存时才更新磁盘。原任务编排配置保留。本轮替代下述首版相机矩阵要求，使用说明见 [CAMERA_ALIGNMENT.md](CAMERA_ALIGNMENT.md)。
- 备份 `/home/a/h743/backups/camera_tracking_20261016_121733/`；记录 [validation_nx_camera_tracking_20261016.json](validation_nx_camera_tracking_20261016.json)。

## 2026-10-16：场地尺寸编辑与摄像头一独立对准已部署

- 按用户“部署”指令同步 15 个更新/新增文件，完整清单 58 项 SHA-256 与本地一致。坐标巡航可直接编辑宽、高及中间通道位置；摄像头一增加色块/圆环识别、参考点偏差和独立车辆对准。巡航挂起机制保留，尚未自动接入新摄像头入口，见 [CAMERA_ALIGNMENT.md](CAMERA_ALIGNMENT.md)。
- NX Python 3.8 的 193 项回归及 PTY 三阶段通过；系统 OpenCV 4.9.0 / NumPy 1.23.5 合成色块和圆环检测通过。Edge 检查实际页面 1440/760/390 px 六个布局，无 JS 异常或横向溢出，未发送设置/运动请求。
- `h743-web.service` 已重启，PID 1623→8055，active。H743 现有 v0.2.21 和 H723 自动重连、七轴静止、解除使能；本次未烧录、未打开真实相机、未执行实车对准。
- 原 `device_links.json`、`auxiliary.json`、`navigation.json` 哈希保持一致，原视觉环境及模型未修改。摄像头标定仍为未确认/零矩阵，实测确认后才允许车辆对准。独立对准后再次巡航前需重建 O。
- 更新前备份 `/home/a/h743/backups/camera_field_20261016_115533/`；记录 [validation_nx_camera_field_20261016.json](validation_nx_camera_field_20261016.json)。

## 2026-10-15：任务编排、视觉会话与故障恢复已部署

- 用户授权更新 NX。共享上位机及页面清单 51 项与本地 SHA-256 一致，更新其中 26 项；视觉适配器、任务视觉入口、工具、文档和 v0.2.21 固件/源码包另 78 项核验通过。原视觉环境、TensorRT 引擎和现场 `device_links.json`、`auxiliary.json`、`navigation.json` 保留。
- 20:13:22 重启 `h743-web.service`，PID 1746→19715、active。H743 自动重连，未使能、七轴静止；原五段巡航及速度 2500 step/s 保留。H723 当前离线，自动重连等待握手；任务动作默认未配置，视觉标定未确认，未启动任务或相机。
- NX Python 3.8 的 178 项测试和 PTY 三阶段通过。实际视觉虚拟环境完成颜色检测、TensorRT 加载/同步数字推理及切回颜色检查；使用合成图，未打开相机或串口。Edge 检查实际页面任务面板、旧固件复位禁用及 1440/760/390 px 布局通过，无 JavaScript 异常，未发送控制命令。
- **板端仍为 v0.2.18，本次未烧录。** v0.2.21 的 AXF/BIN/HEX/MAP 和源码位于 NX `/home/a/h743/releases/v0.2.21/`。任务视觉微调和故障复位需烧录对应新版固件后使用。任务视觉入口为 `/home/a/h743/tools/nx_mission_vision.py`，使用原 `/home/a/vision-env` 环境，与独立视觉预览择一运行，参见 [MISSION.md](MISSION.md)。
- 备份 `/home/a/h743/backups/nx_update_20261015_200536/`，含更新前程序、参数哈希、状态和上传清单；部署记录见 [validation_nx_update_20261015.json](validation_nx_update_20261015.json)。

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
| 免密 SSH | `ssh nx` 直接登录，无需密码（`~/.ssh/config` 中 `nx` 别名，User=a） |
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
ssh nx
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
# 坐标巡航圆列子任务（2026-10-16）

NX 已增加路线视觉挂起下的三个子任务选择。子任务 1 执行同帧双圆找列、1→2→3→2 校准、可配置外沿补搜（400）与退出右移（800），然后重算剩余巡航；子任务 2、3 使用对应路线段已配置动作，未配置时拒绝启动。现有参数文件不会在部署时覆盖，也不会自动给任何实际站点分配任务。

本扩展依赖 H743 配套固件的 `NAV ALIGN suspend=1`。NX 上传和服务重启不等于 MCU 已烧录；未升级 MCU 时选择子任务 1 会在启动前拒绝。烧录完成后重新连接 H743，以刷新能力和导航状态，重新建立 O 点后再由操作者使能、启动巡航。独立摄像头对准流程仍可使用。

## 中心优先寻路与 NX 执行器部署（2026-10-17）

已通过 USB 更新 NX 上 21 个程序及测试文件，75 个部署清单文件 SHA-256 全部一致。远端 274 项测试通过（1 项 JavaScript 对照测试因无 Node.js 跳过，开发机已验证），PTY 串口三阶段验证通过。服务重新启动后 PID 为 8818，H743 固件 0.2.22 与 H723 均已重连，NX 执行器状态可读取，七轴静止、未使能，转向最低速度读回 200 steps/s。

现有地图 A→B 路径已验证为经过中心 `(14400,14400)` 后到达 B；网页 HTML 与 JavaScript 的 HTTP 响应哈希均与本地一致。USB 地址为 `http://192.168.55.1:8770/`，当前 Wi-Fi 地址为 `http://192.168.1.196:8770/`，两者均已验证可访问。旧 Wi-Fi 地址 `.197` 已失效。

部署前程序、配置和状态备份位于 `/home/a/h743/backups/center_navigation_20261017_170328/`；四份 JSON 配置哈希不变。本次未烧录固件、未进行实车运动测试。现存 C、D 路线段使用圆列子任务 1，但旧配置没有启用 `arrival_enabled`，运行前需在界面启用对应到点调姿并确认朝向；O 点当前尚未建立。验收数据见 `validation_nx_center_navigation_20261017.json`。

架构修正：任务决策和参数全部在 NX；H743 仅增加通用 SUSPEND / POSE / RESTORE，保持原地图航向。早期 ring_task_20261016 固件已撤回，不应烧录。通用接口说明见 PROTOCOL.md。

## 转向最低速度与转前内移部署（2026-10-16）

用户完成 H743 烧录后，NX 更新 6 个上位机文件并重启 `h743-web.service`（PID 6509）。73 个部署清单文件已校验，本次复制文件 SHA-256 全部一致；NX 262 项测试、4 项内移专项测试及 PTY 三阶段验证通过。备份位于 `/home/a/h743/backups/turn_inset_20261016_185542/`，4 个现有 JSON 参数文件哈希保持一致。

重启后 H743 `/dev/ttyUSB0` 已连接，H723 `/dev/ttyACM0` 持续输出，未使能、未启动运动。实际网页只读检查通过，坐标巡航新增“转向前向场地中心内移”开关，默认关闭、1000 脉冲；启用后应用于巡航拐弯及到点调姿。转向最低速度由配套 H743 固件执行；本次部署未执行烧录或实车转向验证。验收记录见 `validation_nx_turn_inset_20261016.json`。


## 2026-10-18：纠正舵机界面与物理引脚映射

用户确认 PH6 为 0–180°，原 PH9 改为 PE4 的舵机为 0–270°。上一轮把界面 S1 标签改成 PE4、S2 改成 PH6，却保留 180°/270° 行程并直用协议通道，导致控制和初始化选错引脚。本次恢复界面 S1=PH6/180°、S2=PE4/270°，统一修正网页、Tk、连接初始化、任务动作及回显的通道映射。固件保持协议通道 1=PE4/270°、2=PH6/180°，无需再次烧录。

NX 已部署并重启服务，五份现场 JSON 配置哈希不变。备份 `/home/a/h743/backups/servo_pin_fix_20261018_155103/`。板端回报 PH6=120°/1833 µs、PE4=0°/500 µs，网页实测两张卡片的引脚、行程及读数与其一致；网页只读验收未发送舵机定位命令。H743 在线，H723 streaming，未使能底盘。66 项本地测试、浏览器模拟端到端测试、7 项 NX 专项测试通过。


## 2026-10-18：PE4 无输出的固件错误与待完成实机诊断

用户随后报告两路舵机均不动作，网页目标变化且指令收到板端 OK。上述回显验收不能证明物理动作。核对 STM32CubeMX 的 STM32H743IITx 芯片引脚数据后确认：PE4=TIM15_CH1N，PE5 才是 TIM15_CH1。先前把 PE4 当作主输出并仅设置 CC1E，无法使能 PE4。v0.2.23 改为 CC1NE+CC1NP、BDTR.MOE，保持正向 500–2500 µs PWM；PH6 的正常 CH1 输出保持不变。新增 `SERVO DIAG 1/2` 只读真实寄存器，避免用缓存目标误判 PWM 正常。

35 项 ARM/上位机测试及前端映射测试通过；NX 引脚标签已补上 CH1N，服务未重启。本地独立固件位于 `firmware/releases/v0.2.23-servo-pe4-ch1n/`，尚待用户烧录及实机验证。H06 不动作原因未查明，不能据此次 PE4 修复宣称两路均恢复。


## 2026-10-18 16:16 日志：纠正 CH1N 独立输出极性，继续诊断 H06

v0.2.23 实机诊断 PE4 为 ccer=c、CCR=500；CNT=102 时 IDR=0，CNT=5101 时 IDR=1。与 RM0433 Rev 6 第 1814 页表 337 一致：当 CC1E=0、CC1NE=1 时 OC1N=OC1REF XOR CC1NP，先前多设的 CC1NP 导致长高电平。v0.2.24 改为 ccer=4。H06 的 PSC=199、ARR=19999、CCR=2000、CCMR=68、CCER=1、AF2/MODER2 正常，CNT=2099/8098 时 IDR=0 均处于正常低电平区；用户确认 H06 仍不动作，不能据此确认其物理输出正常。

v0.2.24 新增 45 ms 只读 GPIO 数字输入采样，报告高脉宽、周期、边沿数、采样间隙和采样有效性；不改变 H06 PWM 配置。ARM 执行测试覆盖两引脚短脉冲、19500 us 反相长脉冲、常低、常高与时钟停滞；尚待新固件烧录后取得实际引脚采样结果。

## 2026-10-20：B点完整抓取独立参数部署

NX已部署7个程序与测试文件并重启 `h743-web.service`（PID 8913）。调试页 `/debug` → 升降与旋转 → 抓取与放置新增“一键测试B点完整抓取”，使用B点独立机构配置完成一件物料的张爪、下降、夹取、提升、入盘和回零。直接调试不移动底盘、不等待颜色；正式全流程仍在下降后等待B点颜色确认。启动B点测试仅保存B点参数草稿，不保存或使用通用抓放草稿。

远端29项专项测试通过，7个文件SHA-256一致，`/`、`/debug`、`/pick_place.js`、`/app.js` 实际响应均与部署文件一致。实际网页16个B点字段、19个通用字段分别匹配现场配置，B点转移位置 `-1750` 与通用抓取旋转 `-1850` 独立显示，418px视口无横向溢出、无浏览器错误或警告。9份现场JSON参数哈希不变。H743在线、H723 streaming，未使能、七轴静止、抓放任务idle。

备份及完整部署记录：`/home/a/h743/backups/b_full_pickup_20261020_135056/`。M6板端仍未建立零位（`referenced=0, configured=0`，当前范围0～0）；保存的范围-2950～0不等同于已生效。实测前需由操作者确认机构位置并建立零位。本次未烧录固件、未执行实物抓取。系统用户journal未提供日志文件，因此日志检查没有可用记录；服务、设备遥测和实际HTTP内容均已独立验证。验收数据见 [validation_nx_b_full_pickup_20261020.json](validation_nx_b_full_pickup_20261020.json)。
