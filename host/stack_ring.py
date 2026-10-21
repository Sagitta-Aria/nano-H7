"""Second-D identity from remembered colors, with ring-centered positioning.

No OCR and no three-ring requirement. Ambiguous colors never become a point ID.
All motion and freshness barriers remain with the existing camera owner.
"""
import copy
import itertools
import math
import time

from .ring_sequence import RingSequence


def select_anchor(frame, rings, settings, layout, yaw, expected=None):
    """Associate interior color with ring geometry before preview annotations.

    Require a dominant calibrated color inside the shrunken hole, excluding the
    black rim. Ordered pairs may disambiguate repeated colors; a single ring
    must have a unique remembered identity. Always return the ring center.
    """
    import cv2
    import numpy as np
    colors = {int(point): item['color'] for point, item in layout.items()}
    observations = []
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for ring in rings:
        mask = np.zeros(frame.shape[:2], np.uint8)
        contour = np.asarray(ring['inner'])
        center = np.asarray(ring['center'])
        interior = np.rint(center + .7*(contour-center)).astype(np.int32)
        cv2.drawContours(mask, [interior], -1, 255, -1)
        pixels = cv2.countNonZero(mask)
        scores = []
        for color in set(colors.values()):
            item = settings['colors'][color]
            low, high = item['ranges'][0]
            selected = cv2.bitwise_and(cv2.inRange(hsv, np.array(low), np.array(high)), mask)
            count = cv2.countNonZero(selected)
            if count >= item['min_area'] and count >= .12*pixels:
                scores.append((count, color))
        scores.sort(reverse=True)
        color = scores[0][1] if scores and (len(scores) == 1 or scores[0][0] > 1.5*scores[1][0]) else None
        observations.append(dict(center=list(ring['center']), color=color, point=None))
    angle = math.radians(yaw)
    forward = lambda obs: math.sin(angle)*obs['center'][0]-math.cos(angle)*obs['center'][1]
    ordered = sorted([i for i, obs in enumerate(observations) if obs['color']],
                     key=lambda i: forward(observations[i]))
    assignments = []
    separated = all(forward(observations[b])-forward(observations[a]) >= 20
                    for a, b in zip(ordered, ordered[1:]))
    if 0 < len(ordered) <= 3 and separated:
        for points in itertools.combinations((1, 2, 3), len(ordered)):
            if all(colors[p] == observations[i]['color'] for i, p in zip(ordered, points)):
                assignments.append(points)
    if assignments:
        for index, i in enumerate(ordered):
            candidates = {points[index] for points in assignments}
            if len(candidates) == 1:
                observations[i]['point'] = candidates.pop()
    candidates = [i for i, obs in enumerate(observations)
                  if obs['point'] is not None and (expected is None or obs['point'] == expected)]
    chosen = min(candidates, key=lambda i: math.dist(observations[i]['center'],
                    [frame.shape[1]/2, frame.shape[0]/2])) if candidates else None
    return (rings[chosen] if chosen is not None else None,
            observations[chosen]['point'] if chosen is not None else None, observations)


class StackRingSequence(RingSequence):
    """Align a pair, anchor an identifiable ring, then visually confirm point 2.

    Use only on the second held D visit with a complete successful first layer
    and the same forward row orientation. Shared offsets are pinned by Mission.
    """
    def __init__(self, owner):
        super().__init__(owner)
        self.offsets = dict(self.task_options['material_offsets'])
        self.confirming_middle = False
        self.candidate = None

    def snapshot(self):
        result = super().snapshot()
        result.update(identity_source='remembered_color', offsets=dict(self.offsets),
                      expected=2 if self.confirming_middle else self.candidate,
                      spacing_steps=None)
        return result

    def switch_digit(self):
        """Start only a nonnumeric stream; the worker must never create OCR."""
        self.owner.end_stream()
        self.owner.begin_stream('stack:2' if self.confirming_middle else 'stack')
        self.observations = self.stable = self.leg = 0
        self.position = 0
        self.candidate = self.previous_center = None
        self.after = self.owner.capture_after
        self.deadline = time.monotonic()+60

    def matches(self, report):
        """Require remembered-color evidence, never a digit-shaped substitute."""
        return (report.get('found') is True and report.get('identity_source') == 'remembered_color'
                and report.get('anchor_point') == self.expected)

    def next_search(self):
        """Bound color-identity search to the existing +/-X sweep, then stop."""
        if self.phase == 'seek' and self.leg >= 2 and self.position == 0:
            self.owner.stop('搜索范围内无法由第一批颜色确认点号，停止第二次D点放置。', True)
            return
        super().next_search()

    def consume(self, report):
        """Keep pair-based heading alignment; seek one stable explicit point ID."""
        if self.phase != 'seek':
            super().consume(report)
            return
        if report['captured_monotonic'] < self.after:
            return
        self.line = copy.deepcopy(report.get('row_line'))
        if self.line:
            error = self.line.get('error_deg')
            if type(error) not in (int, float) or not math.isfinite(error) or abs(error) > 90:
                self.owner.stop('圆列方向无效。', True)
                return
            if abs(error) > self.config['angle_tolerance_deg'] and not self.operation:
                self.phase = 'align'
                self.stable = 0
                self.previous_angle = None
                return
        point = report.get('anchor_point')
        matched = (report.get('found') is True and type(point) is int and point in (1,2,3)
                   and report.get('identity_source') == 'remembered_color'
                   and (not self.confirming_middle or point == 2))
        same = (matched and point == self.candidate and self.previous_center is not None
                and math.dist(report['center'], self.previous_center) <= 30)
        self.stable = self.stable+1 if same else (1 if matched else 0)
        self.candidate = point if matched else None
        self.previous_center = list(report['center']) if matched else None
        if self.operation:
            if self.stable >= 3:
                self.halt_requested = True
            return
        self.observations += 1
        if self.stable >= 3:
            self.expected = point
            # Lock identity during approach, including when another ring enters.
            self.owner.config['stack_expected'] = point
            self.owner.end_stream()
            self.owner.begin_stream('stack')
            self.start_approach(dict(point=point, identity_source='remembered_color',
                center=list(report['center']), frame_id=report['frame_id']))
        elif self.observations >= 5:
            self.observations = 0
            self.next_search()

    def reply(self, purpose, fields):
        """A transfer must complete its own move ID and every commanded pulse."""
        transfer = self.phase == 'transfer' and self.operation is not None
        if transfer and purpose == 'ring_status' and (fields.get('state') == 'stopped' or
                (fields.get('state') == 'done' and fields.get('remaining') != '0')):
            self.owner.stop('回点2的定距移动未完成。', True)
            return
        super().reply(purpose, fields)
        if transfer and self.operation is None and self.owner.state != 'fault':
            self.confirming_middle = True
            self.owner.config['stack_expected'] = 2
            self.owner.state = 'ring_sequence'
            self.phase = 'seek'
            self.switch_digit()

    def finish_approach(self):
        """Accept only the parent's fresh zero-velocity/idle visual barrier."""
        self.owner.record_row_event('approach_complete', target=self.pending_target.copy())
        self.pending_target.update(aligned=True, center=list(self.owner.vision['center']),
            frame_id=self.owner.last_frame_id, positioning='ring_vision')
        point = self.pending_target['point']
        self.recognized.append(self.pending_target)
        self.pending_target = None
        self.position = self.offsets[point]
        self.owner.generation += 1
        self.owner.capturing = False
        self.owner.end_stream()
        self.owner.state = 'ring_sequence'
        self.owner.vx = self.owner.vy = self.owner.yaw = 0
        self.owner.wheels = [0]*4
        if point == 2:
            self.phase = 'complete'
            self.owner.state = 'done'
            return
        self.phase = 'transfer'
        self.move(self.offsets[2]-self.offsets[point])
