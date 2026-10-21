"""蓝色色块识别；窗口滑块实时调整阈值，S 保存。"""
from color_block_center import run_color

# 每组格式：((H下限, S下限, V下限), (H上限, S上限, V上限))。
DEFAULT_RANGES = [((100, 160, 46), (135, 255, 255)),
                  ((100, 50, 46), (135, 159, 159))]
DEFAULT_MIN_AREA = 500

if __name__ == "__main__":
    run_color("blue", DEFAULT_RANGES, DEFAULT_MIN_AREA)
