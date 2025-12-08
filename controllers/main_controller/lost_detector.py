from collections import deque
import math

class LostDetector:
    def __init__(self, robot, time_step=64):
        self.robot = robot
        self.time_step = time_step
        self.is_lost = False
        self.confidence = 1.0

        self.conf_history = []
        self.step_index = 0

        self.ps = [self.robot.getDevice(f'ps{i}') for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

        self.left_enc = self.robot.getDevice('left wheel sensor')
        self.right_enc = self.robot.getDevice('right wheel sensor')
        self.left_enc.enable(self.time_step)
        self.right_enc.enable(self.time_step)

        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist = deque(maxlen=10)
        self.ir_hist = deque(maxlen=10)

        self.min_move_rad = 0.01
        self.min_ir_change = 80.0
        self.confidence_drop = 0.05
        self.confidence_gain = 0.02
        self.lost_threshold = 0.4

    def _read_ir(self):
        return [s.getValue() for s in self.ps]

    def _odom_step(self):
        l = self.left_enc.getValue()
        r = self.right_enc.getValue()
        if self.prev_left is None:
            self.prev_left, self.prev_right = l, r
            return 0.0
        dl = l - self.prev_left
        dr = r - self.prev_right
        self.prev_left, self.prev_right = l, r
        return abs(dl) + abs(dr)

    def _ir_change(self, ir):
        if self.prev_ir is None:
            self.prev_ir = ir
            return 0.0
        d = sum(abs(a - b) for a, b in zip(ir, self.prev_ir))
        self.prev_ir = ir
        return d

    def reset(self):
        self.is_lost = False
        self.confidence = 1.0
        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist.clear()
        self.ir_hist.clear()
        self.conf_history.clear()
        self.step_index = 0
        print("[lost_detector] Robot trust level reset to 1")

    def check(self, localiser=None):
        moved = self._odom_step()
        ir = self._read_ir()
        ir_delta = self._ir_change(ir)

        self.delta_hist.append(moved)
        self.ir_hist.append(ir_delta)

        moving_now = moved > self.min_move_rad
        env_static = ir_delta < self.min_ir_change

        not_moving = moved <= self.min_move_rad
        env_spike = ir_delta > (self.min_ir_change * 4)

        suspicious = (moving_now and env_static) or (not_moving and env_spike)

        if suspicious:
            self.confidence = max(0.0, self.confidence - self.confidence_drop)
        else:
            self.confidence = min(1.0, self.confidence + self.confidence_gain)

        self.step_index += 1
        self.conf_history.append((self.step_index, self.confidence))

        self.is_lost = (self.confidence < self.lost_threshold)

        print(
            f"[lost_detector] Odom movement magnitude:{moved:.3f} "
            f"IR change magnitude:{ir_delta:.1f} "
            f"Robot localisation confidence:{self.confidence:.2f} "
            f"Is robot lost:{self.is_lost}"
        )
