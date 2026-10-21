"""Image alignment decisions with no task or hardware owner.

Camera-one direction tracking and existing calibrated tasks supply their own
acquisition and movement executors. Helpers never send I/O or assert position.
"""
import copy
import math

# Camera-one visual servoing only needs direction, speed and a pixel deadzone.
# Keep the existing mission calibration API below for previously saved tasks.
TRACKING_DEFAULT = {'sps': 2000, 'deadzone_px': 5, 'reverse_x': False, 'reverse_y': False,
                    'kp': 10., 'ki': 0., 'kd': .15, 'min_sps': 300, 'accel': 6000,
                    'camera_yaw_deg': 90.}


def validate_tracking(value):
    """Validate camera tracking controls; no distance or calibration is inferred."""
    if not isinstance(value, dict) or set(value) != set(TRACKING_DEFAULT):
        raise ValueError('请提供完整的对准速度、死区和方向设置。')
    for key, low, high in (('sps', 1, 5000), ('deadzone_px', 0, 100), ('min_sps', 0, 5000), ('accel', 1, 20000)):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError('{} 必须是 {}～{} 的整数。'.format(key, low, high))
    if any(type(value[key]) is not bool for key in ('reverse_x', 'reverse_y')):
        raise ValueError('方向反转必须为开关值。')
    if value['min_sps'] > value['sps']:
        raise ValueError('最低靠近速度不能超过速度上限。')
    for key in ('kp', 'ki', 'kd'):
        if type(value[key]) not in (int, float) or not math.isfinite(value[key]) or not 0 <= value[key] <= 100:
            raise ValueError('PID 参数必须是 0～100 的有限数值。')
    angle = value['camera_yaw_deg']
    if type(angle) not in (int, float) or not math.isfinite(angle) or not -180 <= angle <= 180:
        raise ValueError('摄像头安装角必须是 -180～180 度的有限数值，左旋为正。')
    return copy.deepcopy(value)


class VisualPID:
    """NX pixel-error PID producing simultaneous camera speeds, never distances.

    Integrate only new camera samples using capture time. Integral is bounded,
    and deadzone/crossings clear accumulated state. D is filtered for camera
    noise. Camera mounting rotation, motor inversion and yaw mixing belong to
    camera_to_body/mix_wheels/the executor.
    """
    def __init__(self, tracking):
        self.config = validate_tracking(tracking)
        self.integral = [0., 0.]
        self.previous = [None, None]
        self.derivative = [0., 0.]
        self.timestamp = None
        self.previous_output = [0., 0.]
        self.speed_cap = [float(self.config['sps'])]*2
        self.last_side = [0, 0]

    def update(self, dx, dy, captured, latency=0.):
        """Regulate one fresh frame, braking from observed image closing speed.

        Pixel speed predicts a stop horizon; no pixel-to-pulse calibration or
        claim of actual motor position is made. A zero output can mean braking
        outside tolerance: the caller must test pixel error to declare success.
        """
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (dx, dy, captured, latency)) or latency < 0:
            raise ValueError('视觉偏差或采集时间无效。')
        if self.timestamp is not None and captured <= self.timestamp:
            raise ValueError('视觉控制需要采集时间递增的新帧。')
        c = self.config
        dt = min(1., max(.01, captured-self.timestamp)) if self.timestamp is not None else 0.
        self.timestamp = captured
        result = []
        for i, error in enumerate((dx, dy)):
            previous = self.previous[i]
            if abs(error) <= c['deadzone_px']:
                self.integral[i] = self.derivative[i] = 0.
                self.previous[i] = None
                self.previous_output[i] = 0.
                result.append(0)
                continue
            if previous is not None and previous*error <= 0:
                self.integral[i] = self.derivative[i] = 0.
                previous = None
            sign = 1 if error > 0 else -1
            if self.last_side[i] and self.last_side[i] != sign:
                # Repeated overshoots reveal excessive gain/latency. Reduce
                # this approach's ceiling instead of reversing at full speed.
                self.speed_cap[i] = max(1., self.speed_cap[i]*.5)
                self.integral[i] = self.derivative[i] = 0.
                previous = None
            self.last_side[i] = sign
            derivative = (error-previous)/dt if previous is not None and dt else 0.
            self.derivative[i] = .3*derivative+.7*self.derivative[i]
            limit = c['sps']/c['ki'] if c['ki'] else 0.
            previous_integral = self.integral[i]
            self.integral[i] = max(-limit, min(limit, self.integral[i]+error*dt))
            output = c['kp']*error+c['ki']*self.integral[i]+c['kd']*self.derivative[i]
            remaining = abs(error)-c['deadzone_px']
            # The distant approach floor fades near the deadzone. Forcing the
            # same 300 step/s at a one-pixel residual causes repeated crossings.
            slow_zone = max(2., c['deadzone_px']*2., c['min_sps']/max(c['kp'], .01))
            floor = c['min_sps']*min(1., remaining/slow_zone)
            directed = output*sign
            magnitude = min(self.speed_cap[i], max(floor, directed)) if directed > 0 else 0.
            closing = max(0., -self.derivative[i]*sign)
            # Half the deceleration time accounts for average speed while
            # ramping down. Include frame age and the 20 Hz command interval.
            horizon = latency + .05 + abs(self.previous_output[i])/(2*c['accel'])
            if closing*horizon >= remaining:
                magnitude = 0.
                self.integral[i] = 0.
            elif magnitude >= self.speed_cap[i] and dt:
                # Do not build additional integral while the actuator saturates.
                self.integral[i] = previous_integral
            result.append(round(magnitude)*(1 if error > 0 else -1))
            self.previous_output[i] = magnitude
            self.previous[i] = error
        vx = -result[0] if c['reverse_x'] else result[0]
        vy = result[1] if c['reverse_y'] else -result[1]
        return vx, vy


def camera_to_body(right, forward, camera_yaw_deg):
    """Rotate camera translation into body right/forward; left yaw is positive.

    Apply after visual PID and before wheel mixing. A +90 degree mount maps
    camera forward to body left, and camera right to body forward. This fixed
    mounting angle never changes the gyro's body heading target or yaw rate.
    """
    angle = math.radians(camera_yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return round(cosine*right-sine*forward), round(sine*right+cosine*forward)


def mix_wheels(vx, vy, yaw, invert, maximum):
    """Mix body right/forward/CW rates in the firmware's M1..M4 order.

    Rates are relative step/s; no metric wheel geometry or pixel-distance
    conversion is assumed. Uniform scaling preserves direction at saturation.
    Apply the same per-wheel polarity mask as CHASSIS/gyro primitives.
    """
    wheels = [vy-vx+yaw, vy+vx-yaw, vy+vx+yaw, vy-vx-yaw]
    scale = min(1., maximum/max(1., max(map(abs, wheels))))
    return [round(v*scale)*(-1 if invert & (1 << i) else 1) for i, v in enumerate(wheels)]


DEFAULT = {'verified': False, 'matrix': [[0, 0], [0, 0]],
           'tolerance_px': 3, 'stable_frames': 3, 'max_iterations': 12,
           'max_step': 100, 'sps': 100, 'accel': 1000}


def validated(value):
    """Reject unmeasured/degenerate verified matrices and unsafe step limits."""
    if not isinstance(value, dict) or set(value) != set(DEFAULT) or type(value['verified']) is not bool:
        raise ValueError('请提供完整视觉标定。')
    matrix = value['matrix']
    if not isinstance(matrix, list) or len(matrix) != 2 or any(not isinstance(row, list) or len(row) != 2 for row in matrix):
        raise ValueError('标定矩阵必须是 2×2。')
    if any(type(x) not in (int, float) or abs(x) > 1000 or not math.isfinite(x) for row in matrix for x in row):
        raise ValueError('标定系数无效。')
    if value['verified'] and abs(matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]) < 1e-9:
        raise ValueError('已确认的标定矩阵不可退化。')
    for key, low, high in (('tolerance_px',0,100), ('stable_frames',2,20), ('max_iterations',1,16),
                          ('max_step',1,1000), ('sps',1,5000), ('accel',1,20000)):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError('{} 必须是 {}～{} 的整数。'.format(key, low, high))
    return copy.deepcopy(value)


def alignment_step(dx, dy, calibration):
    """Calculate one bounded body-axis correction from validated calibration.

    Caller verifies physical calibration and feedback freshness. Choosing one
    axis at a time allows a new image after every physical displacement.
    """
    if any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 10000 for v in (dx, dy)):
        raise ValueError('视觉偏差无效。')
    if max(abs(dx), abs(dy)) <= calibration['tolerance_px']:
        return {'aligned': True}
    forward, left = [row[0]*dx + row[1]*dy for row in calibration['matrix']]
    amount = forward if abs(forward) >= abs(left) else left
    direction = ('FWD' if amount > 0 else 'BACK') if abs(forward) >= abs(left) else ('LEFT' if amount > 0 else 'RIGHT')
    return {'aligned': False, 'direction': direction,
            'steps': max(1, min(calibration['max_step'], int(abs(amount) + .5))),
            'sps': calibration['sps'], 'accel': calibration['accel']}
