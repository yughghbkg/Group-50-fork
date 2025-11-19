from collections import deque
import math

class LostDetector:
    def __init__(self, robot, time_step=64):
        self.robot = robot
        self.time_step = time_step
        self.is_lost = False
        self.confidence = 1.0

        # Proximity sensors (e-puck has ps0..ps7)
        self.ps = [self.robot.getDevice(f'ps{i}') for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

        # Wheel encoders
        self.left_enc = self.robot.getDevice('left wheel sensor')
        self.right_enc = self.robot.getDevice('right wheel sensor')
        self.left_enc.enable(self.time_step)
        self.right_enc.enable(self.time_step)

        # History
        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist = deque(maxlen=10)   # odom movement magnitude history
        self.ir_hist = deque(maxlen=10)      # IR change magnitude history

        # Tunables
        self.min_move_rad = 0.01      # small rad change that counts as “moved”
        self.min_ir_change = 5.0      # raw IR delta that suggests environment change
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
        # simple proxy for “movement magnitude”
        return abs(dl) + abs(dr)

    def _ir_change(self, ir):
        if self.prev_ir is None:
            self.prev_ir = ir
            return 0.0
        d = sum(abs(a - b) for a, b in zip(ir, self.prev_ir))
        self.prev_ir = ir
        return d

    def reset(self):
        """Reset confidence + history after a successful replan/recovery."""
        self.confidence = 1.0
        self.is_lost = False
        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist.clear()
        self.ir_hist.clear()

    def check(self, localiser=None):
        # Read sensors
        moved = self._odom_step()
        ir = self._read_ir()
        ir_delta = self._ir_change(ir)

        self.delta_hist.append(moved)
        self.ir_hist.append(ir_delta)

        # 1) Moving (odom) but environment not changing (IR) → suspicious
        moving_now = moved > self.min_move_rad
        env_static = ir_delta < self.min_ir_change

        # 2) Not moving (odom) and IR spikes → suspicious (e.g. bumped/turned without odom)
        not_moving = moved <= self.min_move_rad
        env_spike = ir_delta > (self.min_ir_change * 4)

        suspicious = (moving_now and env_static) or (not_moving and env_spike)

        # Confidence update
        if suspicious:
            self.confidence = max(0.0, self.confidence - self.confidence_drop)
        else:
            self.confidence = min(1.0, self.confidence + self.confidence_gain)

        self.is_lost = (self.confidence < self.lost_threshold)

        # Debug print
        print(f"[lost] moved={moved:.3f} irΔ={ir_delta:.1f} conf={self.confidence:.2f} lost={self.is_lost}")
        
    def reset(self):
        """
        Reset lost state and confidence back to fully trusted.
        Call this after a successful replan.
        """
        self.is_lost = False
        self.confidence = 1.0
        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist.clear()
        self.ir_hist.clear()
        print("[lost] state reset (confidence=1.0)")