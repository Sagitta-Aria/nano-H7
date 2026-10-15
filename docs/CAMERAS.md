# 摄像头1 / 摄像头2

上位机使用独立栏目、独立采集进程和独立照片。打开、拍照、关闭某一栏目的设备不会操作另一栏目。仍然按需拍照；浏览页面不会打开设备或持续传输画面。

Nano 的设备列表支持 USB V4L2 摄像头和已成功注册的 Jetson CSI 传感器。CSI 通过系统 Python GI 的 GStreamer/Argus 接口采集，OpenCV 只负责照片编码，不要求 pip OpenCV 启用 GStreamer。

在栏目选择设备后点击“记住此摄像头”，绑定保存到 Nano 的 `~/h743/host/cameras.json`。部署工具保留该文件。USB 优先记录 `/dev/v4l/by-id`，其次记录 `/dev/v4l/by-path`；CSI 记录 `csi:` 加传感器设备树路径。CSI 通常没有独立序列号，绑定表示该传感器型号与接口；更换接口后需要重新绑定。USB by-path 同样依赖连接口。

已绑定设备缺失时，栏目不会自动认领另一个设备。重新绑定前必须关闭本栏目设备；同一物理摄像头不能被两个栏目同时打开或绑定。

摄像头1兼容原有 `/api/camera/*` API；摄像头2使用 `/api/camera2/*`。两者均有 GET `devices`、`state`、`frame` 以及 POST `start`、`stop`、`snapshot`、`bind`。写操作遵循上位机控制权规则。

2026-10-12 实机检查：设备为 Jetson Xavier NX，L4T 35.4.1。设备树预设 IMX219 两路：

| 接口配置 | 传感器设备树路径 | I²C 配置地址 |
| --- | --- | --- |
| module0 | `/proc/device-tree/cam_i2cmux/i2c@0/rbpcv2_imx219_a@10` | `9-0010`（0x10） |
| module1 | `/proc/device-tree/cam_i2cmux/i2c@1/rbpcv2_imx219_c@10` | `10-0010`（0x10） |

这只是系统配置，不能作为实际摄像头已识别的证据。检查时两路驱动均报 `i2c read probe (-121)`，未生成 `/dev/video*`，因此摄像头2实际接口绑定与拍照验证仍需等待传感器恢复。

CSI 故障排查：先核对传感器型号是否匹配驱动，再断电检查排线方向、插入深度及卡扣。重新开机后用 `v4l2-ctl --list-devices` 和 `journalctl -k` 确认传感器注册成功，之后保存实际绑定并拍照。
