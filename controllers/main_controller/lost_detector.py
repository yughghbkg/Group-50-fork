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

<<<<<<< HEAD
        self.min_move_rad = 0.01
        self.min_ir_change = 80.0
=======
        # Tunables
        self.min_move_rad = 0.01      # small rad change that counts as “moved”
        self.min_ir_change = 5.0      # raw IR delta that suggests environment change
>>>>>>> c29697f7747cecc3e3cabdc835f20bb3cf323706
        self.confidence_drop = 0.05
        self.confidence_gain = 0.02
        self.lost_threshold = 0.6

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
