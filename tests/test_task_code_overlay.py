"""Task-code card on the NX screen: state gating and the white full-screen frame.

The card must appear only while the two-round flow is running and must use a
pure white background with one centred black line whose glyphs are at least
12 px tall (the competition code is 15 characters, e.g. 156+123+516+231).
"""

import unittest

import numpy as np

from host import camera_monitor as monitor

CODE = '156+123+516+231'


def dark_bbox(frame):
    """Return the bounding box of drawn (non-white) pixels, or None."""
    gray = frame.min(axis=2)
    rows, columns = np.where(gray < 200)
    if not len(rows):
        return None
    return int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())


class TaskCodeStateTests(unittest.TestCase):
    def test_code_only_while_the_task_is_active(self):
        self.assertEqual(monitor.task_code({'task': {'active': False, 'task_plan': {'code': CODE}}}), '')
        self.assertEqual(monitor.task_code({'task': {'active': True, 'task_plan': {'code': CODE}}}), CODE)

    def test_missing_or_stale_plan_shows_nothing(self):
        self.assertEqual(monitor.task_code({}), '')
        self.assertEqual(monitor.task_code(None), '')
        self.assertEqual(monitor.task_code('nonsense'), '')
        self.assertEqual(monitor.task_code({'task': {'active': True, 'task_plan': None}}), '')
        self.assertEqual(monitor.task_code({'task': {'active': True}}), '')
        self.assertEqual(monitor.task_code({'task': {'active': True, 'task_plan': {'code': '   '}}}), '')
        self.assertEqual(monitor.task_code({'task': {'active': True, 'task_plan': {'code': 42}}}), '')

    def test_code_is_trimmed(self):
        state = {'task': {'active': True, 'task_plan': {'code': '  ' + CODE + '\n'}}}
        self.assertEqual(monitor.task_code(state), CODE)


class TaskCodeFrameTests(unittest.TestCase):
    def test_blank_code_is_a_plain_white_screen(self):
        frame = monitor.task_code_frame('', (480, 800))
        self.assertEqual(frame.shape, (480, 800, 3))
        self.assertTrue((frame == 255).all())
        self.assertIsNone(dark_bbox(monitor.task_code_frame(None, (480, 800))))

    def test_background_is_pure_white_and_text_is_black(self):
        frame = monitor.task_code_frame(CODE, (480, 800))
        # corners and the outer 20-pixel frame stay untouched by the glyphs
        for corner in (frame[:20, :20], frame[:20, -20:], frame[-20:, :20], frame[-20:, -20:]):
            self.assertTrue((corner == 255).all())
        self.assertTrue((frame.min() < 40), 'card text must be near-black')

    def test_fifteen_character_code_fits_with_tall_glyphs(self):
        frame = monitor.task_code_frame(CODE, (480, 800))
        left, top, right, bottom = dark_bbox(frame)
        self.assertGreaterEqual(bottom - top + 1, monitor.TASK_CODE_MIN_PX)
        self.assertLessEqual(right - left + 1, 800 - 2 * monitor.TASK_CODE_MARGIN + 2)
        self.assertGreaterEqual(left, monitor.TASK_CODE_MARGIN - 2)
        # one single line, centred in the 800x480 screen
        self.assertLessEqual(abs((left + right) // 2 - 400), 8)
        self.assertLessEqual(abs((top + bottom) // 2 - 240), 12)

    def test_longer_code_still_fits_on_one_line(self):
        frame = monitor.task_code_frame('1+2+3+4+5+6+7+8+9', (480, 800))
        left, top, right, bottom = dark_bbox(frame)
        self.assertGreaterEqual(bottom - top + 1, monitor.TASK_CODE_MIN_PX)
        self.assertLessEqual(right - left + 1, 800 - 2 * monitor.TASK_CODE_MARGIN + 2)

    def test_impossible_code_raises_instead_of_shrinking_below_the_limit(self):
        with self.assertRaises(ValueError):
            monitor.task_code_frame('7' * 400, (480, 800))


if __name__ == '__main__':
    unittest.main()
