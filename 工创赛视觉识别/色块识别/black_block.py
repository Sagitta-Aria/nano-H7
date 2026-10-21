"""选择面积最大的黑色色块；窗口滑块实时调整阈值，S 保存。"""
from color_block_center import run_color

# 每组格式：((H下限, S下限, V下限), (H上限, S上限, V上限))。
DEFAULT_RANGES = [((0, 0, 0), (179, 255, 45))]
DEFAULT_MIN_AREA = 500

if __name__ == "__main__":
    run_color("black", DEFAULT_RANGES, DEFAULT_MIN_AREA, largest=True)
