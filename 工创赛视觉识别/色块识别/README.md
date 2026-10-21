# 六种颜色分别识别与阈值调整

每个程序只检测一种颜色，使用简单的 HSV 分割，不限制色块形状。黑色选择面积最大的有效色块，其余五种颜色选择面积最小的有效色块；目标都须达到面积下限。用轮廓重心计算中心，在原始实时画面顶部显示中心坐标和 `dx、dy`。默认摄像头为 1、面积下限为 500 像素平方。

## 选择程序

| 颜色 | Python 程序 | 双击启动 | 按 S 后生成的独立参数文件 |
| --- | --- | --- | --- |
| 红 | `red_block.py` | `启动红色识别.cmd` | `red_thresholds.json` |
| 黄 | `yellow_block.py` | `启动黄色识别.cmd` | `yellow_thresholds.json` |
| 蓝 | `blue_block.py` | `启动蓝色识别.cmd` | `blue_thresholds.json` |
| 绿 | `green_block.py` | `启动绿色识别.cmd` | `green_thresholds.json` |
| 黑 | `black_block.py` | `启动黑色识别.cmd` | `black_thresholds.json` |
| 浅蓝 | `light_blue_block.py` | `启动浅蓝识别.cmd` | `light_blue_thresholds.json` |

`color_block_center.py` 是六个入口共用的检测和调参模块，需要与六个程序放在一起。每个入口顶部的 `DEFAULT_RANGES` 和 `DEFAULT_MIN_AREA` 也可以直接修改；已有 JSON 时优先加载 JSON，按 R 才会恢复代码默认值。

在本文件夹的 PowerShell 中手动启动（以红色为例）：

```powershell
..\黑色圆环识别与数字识别\.venv\Scripts\python.exe -B red_block.py --camera 1
```

其他颜色只需替换脚本名。同一摄像头通常不能被多个程序同时占用，请退出当前程序后再运行下一种颜色。

## 在每个程序中调阈值

程序打开三个窗口：`Camera` 显示实时画面和所选色块的坐标差（黑色选最大，其他颜色选最小）；`Mask` 显示分割结果（白色为匹配区域，黑色为背景）；`HSV` 提供滑块。识别使用的就是 Mask 窗口中的结果，滑块调整立即生效。

| 滑块 | 范围 | 用途 |
| --- | --- | --- |
| H min / H max | 0–179 | 颜色的色相范围 |
| S min / S max | 0–255 | 饱和度范围，S min 越低越能接受淡色，也越容易匹配灰白背景 |
| V min / V max | 0–255 | 亮度范围，黑色主要调整 V max |
| Min area | 1–20000，已保存值更大时自动扩展 | 最小轮廓面积；小噪声被选中时提高，小目标漏检时降低 |
| Range 0/1 | 0 或 1，仅双区间颜色显示 | 切换正在编辑的 HSV 区间，两组始终取并集 |

红色跨越色相两端，蓝色和浅蓝色为区分深浅也使用两组阈值。切换 Range 不会禁用另一组，也不会丢失已调参数。蓝和浅蓝是独立程序，调参后需自行避免两者范围重叠。下限拖到高于上限时，程序自动将上限抬到下限。

建议先将目标放入画面，调整 H 排除其他颜色，再用 S/V 去掉背景，最后调整 Min area 过滤小噪声。分割图会进行固定的 5×5 开闭运算去噪，过细的目标可能被滤掉。黑色背景和阴影也可能匹配黑色阈值。

| 按键 | 作用 |
| --- | --- |
| S | 保存当前颜色全部区间和面积阈值，下次启动自动加载 |
| R | 将当前颜色恢复为 Python 文件中的默认阈值，需要按 S 才覆盖已保存参数 |
| C | 在摄像头 0/1 之间切换 |
| Q / Esc | 退出，未按 S 的调整不会保存；关闭任意窗口也退出 |

按键时请先点击程序窗口。参数文件保存在脚本目录，六种颜色互不覆盖。保存失败时画面提示 `Save failed`，具体原因显示在终端。

## 坐标与命令行参数

```text
dx = 色块重心 x - 画面宽 // 2
dy = 色块重心 y - 画面高 // 2
```

右、下为正，左、上为负，单位是像素。目标部分超出画面时，中心对应可见部分。没有目标时显示 `No color block`，不会保留上一次坐标。

六个程序都支持 `--camera`、`--min-area` 和 `--config`。例如：

```powershell
..\黑色圆环识别与数字识别\.venv\Scripts\python.exe -B black_block.py --camera 1 --min-area 800
..\黑色圆环识别与数字识别\.venv\Scripts\python.exe -B red_block.py --config .\red_thresholds.json
```

不存在的参数文件使用默认值，按 S 时创建；已存在但格式不正确的参数文件会明确报错。自定义 `--config` 时请使用已存在的父文件夹。使用的依赖仍是 `requirements.txt` 中的 OpenCV 和 NumPy。
