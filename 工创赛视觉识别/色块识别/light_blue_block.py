"""浅蓝色块识别；窗口滑块实时调整阈值，S 保存。"""
from color_block_center import run_color

# 每组格式：((H下限, S下限, V下限), (H上限, S上限, V上限))。
DEFAULT_RANGES = [((85, 50, 46), (99, 255, 255)),
                  ((100, 50, 160), (135, 159, 255))]
DEFAULT_MIN_AREA = 500

if __name__ == "__main__":
    run_color("light_blue", DEFAULT_RANGES, DEFAULT_MIN_AREA)
