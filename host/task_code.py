"""Competition task data only: two batches, three fixed tray slots per batch.

Parsing never opens hardware or schedules motion. Each placement digit belongs
to the pickup digit at the same index; tray retrieval reverses both together.
"""
import re

COLORS = {
    '1': ('red', '红'), '2': ('yellow', '黄'), '3': ('blue', '蓝'),
    '4': ('green', '绿'), '5': ('black', '黑'), '6': ('light_blue', '浅蓝'),
}
SLOT_ANGLES = (0, 135, 270)


def parse_task_code(text):
    """Validate four groups and return a fresh plan; repeated pickup colors are allowed.

Both placement groups must cover markers 1, 2, 3 exactly once. This strict
competition schema is separate from the legacy QR viewer's permissive parser.
"""
    if not isinstance(text, str) or len(text) > 128 or not re.fullmatch(
            r'\s*[0-9]{3}(?:\s*\+\s*[0-9]{3}){3}\s*', text):
        raise ValueError('比赛任务码必须是四组三位数字，例如 156+123+516+231。')
    groups = [group.strip() for group in text.strip().split('+')]
    batches = []
    for batch, offset in enumerate((0, 2), 1):
        pickup, placement = groups[offset:offset+2]
        if any(d not in COLORS for d in pickup):
            raise ValueError('第{}批抓取颜色只能是1～6（红、黄、蓝、绿、黑、浅蓝）。'.format(batch))
        if sorted(placement) != ['1', '2', '3']:
            raise ValueError('第{}批放置位置必须是1、2、3的排列，不能重复。'.format(batch))
        # Pickup fills 270/135/0; retrieval is always 0/135/270. Reverse color
        # and destination together so each physical item keeps its own target.
        items = [dict(slot=i+1, pickup_index=3-i, angle_deg=angle, color_id=int(color),
                      color=COLORS[color][0], color_name=COLORS[color][1], target=int(target))
                 for i, (angle, color, target) in enumerate(zip(SLOT_ANGLES, reversed(pickup), reversed(placement)))]
        batches.append(dict(batch=batch, pickup=pickup, placement=placement, items=items,
                            pickup_items=[dict(item) for item in reversed(items)]))
    return dict(schema=2, code='+'.join(groups), groups=groups, batches=batches)
