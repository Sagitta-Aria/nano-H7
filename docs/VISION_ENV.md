# NX 视觉运行环境

2026-10-15 通过 `ssh nano` 在 NX 配置并验证。后续按用户要求启动了连续拍摄和 GPU 圆环/数字实物测试，并修复帧率与数字 1 裁剪问题；当前启动方式见末尾“实物测试与优化”。前面的环境验证章节保留当时的测试范围。

## 已确认的平台

- Jetson Xavier NX，ARM64，L4T 35.4.1，Python 3.8.10。
- HDMI-0 屏幕当前为 800×480，X11 桌面 `:0`。
- USB 摄像头为 `USB 2.0 Camera: HD USB Camera`，主采集节点 `/dev/video0`，同设备还有 `/dev/video1`。
- CSI 采集设备尚未注册；GStreamer 的 `nvarguscamerasrc` 插件存在不代表传感器可用。

实际运行计划是连续取帧、识别并显示到 NX 本机屏幕。当前上位机仍使用按需拍照，本次未修改其行为。

## 环境和启动方式

独立虚拟环境：`/home/a/vision-env`。

在 NX 的 Bash 终端执行：

```bash
source /home/a/vision-setup/activate.sh
python -m pip check
```

激活脚本选择该虚拟环境，并在未设置相关变量时配置 `DISPLAY=:0` 和当前可读的 `/run/user/1000/gdm/Xauthority`，供 SSH 启动的程序向本机屏幕显示。桌面会话变化后需重新核对显示编号和授权文件。

| 包 | 已验证版本 |
| --- | --- |
| opencv-python | 4.9.0.80 |
| numpy | 1.24.4 |
| onnxruntime | 1.17.3 |
| rapidocr-onnxruntime | 1.2.3 |
| TensorRT（JetPack 系统绑定） | 8.5.2.2 |
| onnx（引擎构建工具） | 1.16.2 |

完整依赖版本保存在 NX 的 `/home/a/vision-setup/requirements-lock.txt`。引导工具为 pip 25.0.1、setuptools 75.3.2、wheel 0.45.1。系统缺少 ensurepip，因此使用下载的 wheel 引导虚拟环境内的 pip，再以 `venv --without-pip` 补齐标准激活脚本；没有安装或升级系统 Python 包。

OpenCV wheel 支持 Qt5 窗口和 V4L2，没有内置 GStreamer 后端。虚拟环境中的 `gi`、`cairo` 链接至 `/usr/lib/python3/dist-packages/` 对应模块，以复用系统 GStreamer 1.16.3；CSI 接入可以沿用现有 GI/Argus 方式。这里的 GI 链接依赖当前系统 Python 3.8 ABI。

ONNX Runtime 保留 CPU 版作对照，可用 provider 为 `AzureExecutionProvider` 和 `CPUExecutionProvider`。GPU 推理使用下面的原生 TensorRT 接口，不依赖 ONNX Runtime CUDA provider。不要在这个环境中直接安装原目录锁定的 OpenCV 5 依赖覆盖已验证版本。

## GPU 推理入口

按用户后续要求，2026-10-15 接入系统现有 CUDA 11.4、TensorRT 8.5.2.2。GPU 计算能力为 7.2。虚拟环境中的 `tensorrt` 及其版本信息链接到 `/usr/lib/python3.8/dist-packages/`，没有替换系统 CUDA 或 TensorRT。

`tools/nx_vision_trt.py` 已安装为虚拟环境可导入的 `nx_vision_trt` 模块。它替换 RapidOCR 的识别 session，保留原数字预处理、字符表、自定义 CTC 解码和投票；CUDA 运行时传输通过 ctypes 调用，不需要额外创建 PyCUDA 上下文。OpenCV 几何检测仍在 CPU 执行。

原视觉源码后续部署到 NX 后，用下面的入口运行已有脚本，路径替换为实际部署位置：

```bash
source /home/a/vision-setup/activate.sh
python -m nx_vision_trt /实际部署位置/ring_digit.py --camera 0
```

直接运行 `python ring_digit.py` 仍走其原 CPU 工厂。GPU 入口不会静默回退 CPU。编程调用也可以使用 `nx_vision_trt.accelerate(digit_core.make_engine())`，结束后调用 `engine.nx_trt_session.close()` 释放 GPU 缓冲区。

引擎位置：`/home/a/vision-setup/engines/ppocrv3-rec-fp16.plan`，构建脚本：`tools/build_nx_vision_engine.py`，有效构建日志：`/home/a/vision-setup/trt-build-dynamic.log`。它由原 `ch_PP-OCRv3_rec_infer.onnx` 在本机生成，使用 FP16，输入范围为 `1×3×48×8` 至 `1×3×48×320`，优化宽度为 64。仅支持本任务的单图数字识别；不支持批量文字、超出范围的宽度或其他检测模型。引擎与 GPU/TensorRT 版本相关，换设备或升级 TensorRT 后应重新构建。

原 ONNX 的高度和宽度都命名为 `?`。直接用 trtexec 构建会触发 TensorRT 的同名维度相等约束，实际输入被限制成 `48×48`，与原动态宽度预处理不兼容。构建脚本在解析前仅在内存中固定 batch=1、height=48，并把动态宽度命名为 `width`；不修改原 ONNX 文件、权重或预处理。构建结束后会验证实际 profile，GPU 适配器加载时也会拒绝错误 profile。

重建命令：`/home/a/vision-env/bin/python /home/a/vision-setup/build_nx_vision_engine.py`。脚本保存 timing cache，后续在同一硬件/软件版本重建时可复用。

模型 SHA-256：`897a3ededb38fee0dae2c1ccee38241f37df202c9509e3abca02e9217c5ee615`。

系统 `nvjpegdec`、`nvvidconv`、`nvv4l2decoder`、`v4l2src` 和 `appsink` 插件均存在。已用合成 JPEG 验证 `nvjpegdec → videoconvert → BGR appsink` 通路，不代表实际摄像头吞吐。

USB 摄像头声明 MJPEG 模式支持 640×480 约 120 FPS、1280×720 60 FPS；YUYV 640×480 为 30 FPS。NX 当前屏幕为 60 Hz。上述是枚举能力，实际采集、算法处理和显示帧率须分别实测。

## GPU 验证与性能

使用三种 OpenCV 字体生成环内数字 1、2、3，共九张 640×480 合成图。CPU 和 GPU 均识别正确，输出数字一致；FP16 的置信度与 CPU 有小幅数值差异。GPU session 在主线程创建、工作线程调用通过，宽度 8、17、48、96、320 均执行通过，超出 profile 的输入会明确拒绝。

`tools/benchmark_vision_nx.py` 使用原源码在内存中运行，OpenCV 线程数设为 1。以下为热身后每项 30 次计时，包含预处理、CPU/GPU 数据传输及结果解码；没有摄像头读取或屏幕显示：

| 阶段 | 平均耗时 | 中位数 | P95 |
| --- | ---: | ---: | ---: |
| CPU OCR | 56.1 ms | 28.5 ms | 159.9 ms |
| GPU OCR | 13.5 ms | 8.7 ms | 30.9 ms |
| 圆环检测和裁剪（CPU） | 77.0 ms | 74.9 ms | 89.5 ms |
| 圆环检测和 CPU OCR 串行 | 126.4 ms | 103.9 ms | 200.7 ms |
| 圆环检测和 GPU OCR 串行 | 90.0 ms | 84.4 ms | 109.8 ms |

此次 OCR 平均加速约 4.1 倍；完整串行计算吞吐由约 7.9 提升到 11.1 帧/秒。后者仍受到 CPU 圆环检测限制，不能据 OCR 单独耗时宣称整套系统达到 74 FPS。原程序采用后台 OCR，实际连续采集、异步处理和屏幕显示帧率仍需按后续任务测量。这次没有改写圆环算法。

GPU 启动器的 `ring_digit.py` 和 `digit_core.py` 离线图片入口均实际执行通过；使用的原源码临时副本在验证后删除，没有启动摄像头循环。适配器随后使用 TensorRT 8.5 的 tensor I/O 接口消除旧 binding API 的弃用提示，并重新通过动态宽度和工作线程检查。

NX 侧完整记录为 `/home/a/vision-setup/gpu-validation.json`，本地副本为 `docs/validation_nx_vision_gpu_20261015.json`。

## 验证结果和边界

- 虚拟环境内 `pip check` 通过，实际导入的 OpenCV、NumPy 位于 `/home/a/vision-env/`。
- 将新增的五个 Python 源文件通过 SSH 传入内存加载，未启动其 `main()`，未部署视觉程序。
- 合成二维码 `012+009` 的解码和前导零分组通过。
- 合成红色圆块中心定位通过，中心为 `(320, 240)`。
- 使用原有圆环检测、内区提取和数字识别函数，合成环内数字 1、2、3 均通过。该次调用耗时分别约 216、33、32 ms；首个结果包含首次推理开销，不能据此判断持续运行帧率。
- GI/GStreamer 合成视频帧获取通过。
- OpenCV 在 NX 的 `:0` 屏幕创建临时测试窗口、处理窗口事件并关闭，通过。
- 激活脚本已实际执行验证，`python` 指向 `/home/a/vision-env/bin/python`。

NX 侧记录保存在 `/home/a/vision-setup/source-smoke.json`、`display-smoke.json` 和 `README.md`。

以上验证覆盖环境、合成图识别和本机窗口接口，不代表现场摄像头识别率或连续运行性能。此次没有打开摄像头采集、发送机器人控制指令或重启上位机服务；检查结束时 `h743-web.service` 保持 active，PID 1746，与配置前一致。

## 实物测试与优化（2026-10-15 后续）

用户随后要求打开摄像头测试，并反馈帧率低、数字 1 不稳定。原三份圆环/数字源码已部署到 `/home/a/vision-live/ring_digit/`，本地原始算法文件保持原样。新增 `tools/nx_vision_live.py` 和 `tools/nx_vision_fast.py` 负责现场入口与优化处理，GPU 仍使用已验证的原 TensorRT 引擎，没有重建权重或引擎。

启动（在 NX 终端）：

```bash
source /home/a/vision-setup/activate.sh
cd /home/a/vision-live/ring_digit
python -u nx_vision_live.py . --camera 0
```

显示到 `:0` 的 800×480 本机屏幕，画面保持 640×480，使用 V4L2 MJPEG 和单缓冲。相机对 60 FPS 请求实际回报约 120 FPS；这个属性值不是实测处理帧率。窗口中的 FPS 按最近两秒实际完成的处理帧数计算。OCR 在原后台工作线程运行，因此该 FPS 也不等于每秒独立完成的 OCR 次数。Q/Esc 退出，S 将原图和 OCR 内区保存到同目录 `captures/`。

实拍复现与修复：

- 原程序实物处理约 7～9 FPS。失败帧中圆环检测约 86～92 ms，裁剪约 19～21 ms。
- 内层印刷环存在断点，原环带占比校验拒绝内层、选到外层，使 OCR 混入多圈圆环。现在只有在孔洞很干净、原图明暗反差充分时，才允许部分断开的环带。
- 灰纸周围填纯白，以及少量残留环边，会影响 Otsu 前景与数字包围框。现在在内区 ROI 内做拟合椭圆内缩，按本地纸面亮度填边后再校正。
- 用 OpenCV 通道运算取代全图 NumPy 通道归约；轮廓简化并在候选局部区域校验。优先用全局分割，整帧未找到有效目标才使用自适应分割；因此选择的是成功分割掩码内的最小环，不再总是合并两种掩码的候选。
- 低分 OCR 增加灰度/二值留白视图，至少两个视图以普通阈值确认同一个数字才替换结果。现场优化版的数字 1 也至少达到 0.60，取消原版 0.30 的弱证据接受例外；没有把字母强行映射成数字。

优化后同一张数字 1 靶标实测约 25～28 FPS，通常置信度约 0.67～0.75；五张失败/不稳的实拍图片回放分数约 0.71～0.86。原失败样本的圆环检测降至约 15～16 ms，裁剪约 4～5 ms。测量期间保持原分辨率，没有切换功耗模式。置信度是当前模型的输出分数，不是现场正确率。

`tools/validate_nx_vision_live.py` 提供可重复的 GPU 回放入口。三种字体×1/2/3、五个负例（空白、实心圆、空环、4、12）和五张实拍数字 1，共 19 例通过；正例同时要求输出置信度至少 0.60。原版本在两张已记录实拍失败图中均未识别出 1。实拍夹具位于 `docs/vision-live-fixtures/`，采集时靶标最小完整环内的数字为 1；画面右侧截断的 2 不是本测试目标。

在部署目录回放（先将夹具放到该目录的 `captures/`）：

```bash
python validate_nx_vision_live.py --source . --backend fast \
  --real-one captures/1792051945478247274_frame.png \
  captures/1792051946402762714_frame.png captures/1792052637998556742_frame.png \
  captures/1792052638722056295_frame.png captures/1792052639394072657_frame.png \
  --output optimized.json
```

将 `--backend` 改为 `original` 可复现旧流程，失败时非零退出。现场入口设置 `NX_VISION_BACKEND=original` 可回到原算法，GPU 后端和相机参数相同。运行前先关闭占用摄像头的旧实例。

独立测试进程的 PID 写在远端 `live.pid`，本阶段日志为 `live-current.log`，回放和现场证据见本地 `docs/validation_nx_vision_live_20261015.json`。后续中央范围测试日志见下一节。测试程序没有注册为开机服务。没有调用机器人控制接口，`systemctl --user` 中的 `h743-web.service` 保持 active、PID 1746。

这些用例覆盖已复现问题，尚未覆盖所有拍摄距离、运动模糊和倾斜角度。若要进一步提高 OCR 更新频率和困难姿态的识别率，需要针对实物字体的数据验证或训练专用 1/2/3 分类器。

## 中央识别范围（同日后续）

按用户提出的边缘畸变问题，现场入口默认只在中央 **360×360** 像素内分割和检测圆环，随后只对该目标内区做数字识别。新增 `tools/nx_vision_roi.py` 已部署到现场入口同目录。640×480 原图中的处理边界为 `x=[140,500)`、`y=[60,420)`；没有缩放输入，也没有执行镜头畸变校正。裁剪用于避开边缘区域。

全幅预览保留，黄色框标出处理范围，框外变暗。靶标的有效圆环必须完整放在框内，框外及被边界截断的圆环不参与识别。检测中心、轮廓和椭圆中心统一映射回原图坐标，dx/dy 仍相对于原图中心 `(320,240)`。顶部状态栏移到识别区域之外，避免遮住靶标。

默认启动命令不变，可在启动前配置：

```bash
export NX_VISION_ROI_SIZE=360  # 居中正方形边长，至少 80；0 恢复全图
python -u nx_vision_live.py . --camera 0
```

回放脚本增加 `--roi-size 360`。原 19 例以及框外目标、跨框目标、框内外同时存在目标三个场景，共 **22 例通过**，并检查原图坐标映射。回放结果保存在 `docs/validation_nx_vision_roi_20261015.json`。现场屏幕、GPU 标记和连续数字 1 输出已核验；日志为 `/home/a/vision-live/ring_digit/live-roi.log`，显示处理帧率有波动，观察约 22～28 FPS。相机参数、TensorRT 引擎及上位机服务保持原配置。

## 框底上收与数字 1 输入优化（同日后续）

按用户要求，默认框底从 y=420 上移 30 像素至 y=390，上边沿 y=60、左右边沿 x=140/500 保持固定，形成 **360×330** 处理区域。该范围按当前纸张下缘选择，不会自动跟随纸张移动。用 `NX_VISION_ROI_BOTTOM_TRIM` 配置上收像素数，0 恢复正方形；`NX_VISION_ROI_SIZE=0` 仍恢复全图。回放参数对应 `--roi-size 360 --roi-bottom-trim 30`。

针对数字 1 细长、略倾斜时输出不稳定的问题，在宽高比小于 0.60 的数字输入上，增加纸面亮度归一化、基于前景二阶矩的水平剪切校正和固定留白。剪切前扩展画布，避免裁断笔画；过强剪切或无法提取前景时沿用已有 OCR 路径。该处理并不把细长形状直接判成 1，候选仍由 TensorRT 文字模型给出。

强模型输出（至少 0.85）直接使用；较弱输出仍需要两个视图以普通 0.60 阈值确认。模型权重、字符表和 TensorRT 引擎保持原版本，本次优化的是推理输入和候选处理。

新增八张倾斜姿态实拍图，加上前面五张实拍图、三字体×三个数字、六个负例和四个 ROI 边界场景，共 **32 例通过**，包含数字 7 的拒识、上收框底外目标的排除和原图坐标检查。结果保存在 `docs/validation_nx_vision_one_20261015.json`。十三张实拍 1 均正确，输出约 0.71～0.96；当前倾斜姿态的八张约 0.92～0.96。其离线 OCR 中位耗时约 9～13 ms，现场日志包含线程调度和负载影响，不能等同。

当前现场日志为 `/home/a/vision-live/ring_digit/live-one.log`。切换后观测到约 30 FPS，静止时数字 1 多在 0.94 左右；移动姿态下仍有降分，不能以这批图片推断全部距离、光照和角度的正确率。低分难例继续保留原多视图处理。上位机服务没有重启。

## 抓取参考点（同日后续）

按用户指定，抓取参考点相对**原画面中心**为 `dx=0, dy=+30`，即 640×480 原图像素 **(320,270)**。图像坐标向右、向下为正。该参考点不随 ROI 下沿上收而移动；当前检测范围仍为 360×330。

屏幕上紫色 `GRAB` 十字表示抓取点，灰色小十字保留原画面中心。顶部 `dx/dy` 继续显示圆环相对原画面中心的位置；新增 `ex/ey` 表示 `圆环中心−抓取点`，二者为零才表示对准抓取点。例如检测到 `(333,273)` 时，`dx=13,dy=33`，`ex=13,ey=3`。无目标时误差为 `None`，不沿用旧值。

现场入口通过 `NX_VISION_GRASP_DX`、`NX_VISION_GRASP_DY` 配置，默认分别为 0、30。屏幕标记、实时遥测和原周期报告共用 `nx_vision_roi.grasp_metrics()`，日志同时记录 `grasp_point`、`grasp_offset`、`error_dx`、`error_dy`。最新日志为 `/home/a/vision-live/ring_digit/live-grasp.log`。

已检查对准点、原图中心、右下偏移、左上偏移、无目标及另一分辨率的坐标；验证记录为 `docs/validation_nx_vision_grasp_20261015.json`。此设置用于视觉对准参考，未接入电机或夹爪控制。

## 色块与数字同时识别（同日后续）

按用户要求，将原 `色块识别/color_block_center.py` 部署至现场入口同目录，并新增 `tools/nx_vision_color.py` 接入共享摄像头流程。每个原始帧在同一个 360×330 ROI 内分别检测圆环和色块，不额外打开摄像头。数字 OCR 继续使用 TensorRT；颜色使用原六色 HSV 范围和圆形筛选。

支持 `red`、`yellow`、`blue`、`green`、`black`、`light_blue`。保留面积最小的有效**实心圆形色块**；默认最小面积 500 像素平方，可通过 `NX_VISION_COLOR_MIN_AREA` 调整。框外和跨边界的目标不参与选择。新增检查要求轮廓内至少 85% 的像素匹配原始颜色掩码，避免把空心黑色圆环当成实心黑色块；这与原外接圆填充率的几何检查分别执行。

屏幕中的圆环数字继续以绿色轮廓显示，色块使用青色轮廓和下方独立的 `Color` 状态行。两种检测结果相互独立，都将坐标映射回原图，并使用 `(320,270)` 抓取点计算各自的 `ex/ey`。无色块时显示 `Color: --`，日志中 `color_block=None`，不会保留上一帧结果。原周期报告和 JSON 遥测增加嵌套的 `color_block`；其中 `coverage` 是颜色像素覆盖率，不是模型置信度。

`tools/validate_nx_vision_color.py` 的 23 个颜色/边界/实拍负例通过，覆盖红色双区间、蓝色与浅蓝色各区间、最小有效目标、面积过滤、空心环过滤、ROI 外/跨界目标和抓取点坐标。另在同一合成帧同时识别出数字 3 和绿色块，GPU OCR 输出正确，输入图像未被色块绘图修改。实拍数字 1 的两张旧夹具没有误报为黑色块。测试命令：

```bash
python validate_nx_vision_color.py --source . \
  --real-ring captures/1792051945478247274_frame.png captures/1792052638722056295_frame.png \
  --output color-validation.json
```

验证记录为 `docs/validation_nx_vision_color_20261015.json`。当前现场日志为 `/home/a/vision-live/ring_digit/live-color.log`，实际数字靶标画面观察约 23～24 FPS，颜色检测每帧约 7 ms；截图和系统负载可能造成波动。现场屏幕已确认出现颜色状态行，彩色色块在当前光照下的准确性仍需用户拿实物测试。上位机服务保持 active、PID 1746，没有执行运动或抓取命令。

## 数字和色块按键切换（同日后续，当前运行方式）

按用户选择，现场入口改为同一窗口中的两种互斥模式。摄像头保持打开，按 **1** 选择圆环数字识别，按 **2** 选择色块识别；Q/Esc 退出，S 保存当前原图（数字模式同时保存 OCR 内区）。启动默认是数字模式，也可指定初始模式：

```bash
python -u nx_vision_live.py . --camera 0 --mode digit
python -u nx_vision_live.py . --camera 0 --mode color
```

以上命令择一运行，不能同时占用摄像头。数字模式不执行颜色检测；色块模式不执行圆环检测或 OCR。首次直接启动色块模式时不导入和加载 OCR/GPU 模型；切换到数字模式时首次加载 TensorRT，后续缓存引擎以缩短切换时间。已加载的引擎在色块模式中保持空闲。切换前等待唯一的后台 OCR 任务完成并丢弃其结果，同时清空目标、投票、置信度和帧率窗口，避免旧结果串入另一模式。

两种模式沿用中央 360×330 ROI，以及原图中心偏移 `(dx=0,dy=+30)` 的抓取参考点 `(320,270)`。坐标仍使用原图像素，`ex/ey` 统一表示当前目标中心减抓取点。各模式显示自己的结果、FPS 和切换键；现场窗口名改为 `NX Vision`。当前日志是 `/home/a/vision-live/ring_digit/live-modes.log`，包含 `mode`、本模式的 `ring_frames`、`color_frames` 和 `ocr_submitted` 计数，便于确认另一算法没有运行。

`tools/validate_nx_vision_modes.py` 检查色块冷启动不加载 OCR、真实数字 1 的 GPU 推理和投票、切换时等待并清空旧结果、引擎复用、原图不被绘图修改，以及 ROI/抓取点坐标。检查期间将未选中算法替换为一旦调用便抛错的函数，确认两条处理路径隔离。另在 NX 实际窗口注入 1→2→1 按键，验证重复切换和 Q 退出、释放摄像头；最后重新启动数字模式供现场测试。

验证记录为 `docs/validation_nx_vision_modes_20261015.json`。当前静止数字 1 画面中，数字模式两段采样的处理帧率中位数分别为 27.7、27.6 FPS，数字输出约 0.85～0.88；色块模式两段采样分别为 45.2、45.95 FPS。色块采样时画面仍是数字纸靶，无有效色块，因此这些帧率不代表各类实物色块的完整性能测试。离线回放中的帧率包含人为等待或缺少摄像头/显示的影响，不用作现场性能。上位机服务保持 active、PID 1746。

## 侧边触摸切换（同日后续，当前界面）

现场没有连接键盘，新增 `tools/nx_vision_ui.py`，在原 640×480 预览右边追加 160 像素宽的触摸栏，完整界面为 800×480。相机图像不缩放，检测算法继续只接收原始相机帧，ROI 和抓取点保持原图坐标。

右侧大按钮“数字”“色块”直接切换模式，当前模式以绿色和“当前模式”文字标出。按下和松开必须都在同一个按钮内；相机区域、按钮间隙和拖出按钮不会触发切换。GUI 回调只记录请求，由主循环执行已有的模式切换和 OCR 清理，避免在回调中阻塞 GPU。原键盘快捷键继续可用。

中文使用 NX 已安装的 Pillow 和 `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`，两种按钮状态在首次显示时预先绘制并缓存，不在每帧重新排版文字。部署时需将 `nx_vision_ui.py` 和 `nx_vision_live.py` 放在同一目录。

已完成 Python 编译检查，核对 NX 实际截图中的中文、按钮尺寸、全屏比例和模式高亮；程序已收到现场切到色块模式的点击并正常继续处理，日志包含 `touch_mode_requested` 和 `mode_changed`。本次初始自动取样早于 GPU 初始化完成，没有取得统计帧，不作为性能结论；未再自动切换现场正在测试的模式。当前日志为 `/home/a/vision-live/ring_digit/live-touch.log`，验证记录为 `docs/validation_nx_vision_touch_20261015.json`。
