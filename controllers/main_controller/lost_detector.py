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

        # --------------- IR SENSORS ----------------
        self.ps = [self.robot.getDevice(f'ps{i}') for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

        # --------------- ENCODERS ------------------
        self.left_enc = self.robot.getDevice('left wheel sensor')
        self.right_enc = self.robot.getDevice('right wheel sensor')
        self.left_enc.enable(self.time_step)
        self.right_enc.enable(self.time_step)

        self.prev_left = None
        self.prev_right = None
        self.prev_ir = None
        self.delta_hist = deque(maxlen=10)
        self.ir_hist = deque(maxlen=10)

        # thresholds for your *existing* logic
        self.min_move_rad = 0.01       # in encoder radians (dl+dr)
        self.min_ir_change = 80.0
        self.confidence_drop = 0.05
        self.confidence_gain = 0.02
        self.lost_threshold = 0.4

        # --------------- STUCK DETECTOR -----------
        # These are in METERS, based on odometry pose
        self.STUCK_ODOM_EPS = 0.005    # < 5 mm per step = "not really moving"
        self.STUCK_MIN_STEPS = 30      # how many steps in a row before "stuck"

        self.prev_odom_pos = None      # (x, y) from localiser

    # -------------------------------------------------
    # Low–level helpers
    # -------------------------------------------------
    def _read_ir(self):
        return [s.getValue() for s in self.ps]

    def _odom_step(self):
        """Return |dl|+|dr| in encoder radians (your original metric)."""
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

    # -------------------------------------------------
    # Public API
    # -------------------------------------------------
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
        self.prev_odom_pos = None
        print("[lost_detector] Robot trust level reset to 1")

    def check(self, localiser=None):
        """
        Update lost / confidence estimate.

        If `localiser` is provided, we also do STUCK detection using
        odometry pose in *meters*.
        """
        # ---------- 1) Your existing signals ----------
        moved_rad = self._odom_step()       # encoder-space movement
        ir = self._read_ir()
        ir_delta = self._ir_change(ir)

        self.delta_hist.append(moved_rad)
        self.ir_hist.append(ir_delta)

        moving_now = moved_rad > self.min_move_rad
        env_static = ir_delta < self.min_ir_change

        not_moving = moved_rad <= self.min_move_rad
        env_spike = ir_delta > (self.min_ir_change * 4.0)

        suspicious = (moving_now and env_static) or (not_moving and env_spike)

        if suspicious:
            self.confidence = max(0.0, self.confidence - self.confidence_drop)
        else:
            self.confidence = min(1.0, self.confidence + self.confidence_gain)

        # ---------- 2) STUCK detection using odometry pose ----------
        odom_dist = 0.0
        stuck_now = False

        if localiser is not None:
            # true odometry pose in meters
            ox, oy, _ = localiser.estimate_pose()

            if self.prev_odom_pos is None:
                self.prev_odom_pos = (ox, oy)
            else:
                px, py = self.prev_odom_pos
                odom_dist = math.hypot(ox - px, oy - py)
                self.prev_odom_pos = (ox, oy)

            # "stuck" = encoders say "trying to move" but odom pose not changing
            if moving_now and odom_dist < self.STUCK_ODOM_EPS:
                # accumulate stuck steps
                self.stuck_steps += 1
            else:
                # reset if either not commanded to move or actually moving
                self.stuck_steps = 0

            stuck_now = (self.stuck_steps >= self.STUCK_MIN_STEPS)

            # optionally punish confidence if stuck
            if stuck_now:
                self.confidence = min(self.confidence, 0.3)

        # ---------- 3) Combine into final "is_lost" ----------
        self.step_index += 1
        self.conf_history.append((self.step_index, self.confidence))

        lost_confidence = (self.confidence < self.lost_threshold)
        self.is_lost = lost_confidence or stuck_now

        # ---------- 4) Debug print ----------
        print(
            f"[lost_detector] step={self.step_index} "
            f"enc_move={moved_rad:.3f} "
            f"odom_move={odom_dist:.3f} "
            f"IR_change={ir_delta:.1f} "
            f"conf={self.confidence:.2f} "
            f"stuck_steps={self.stuck_steps} "
            f"is_lost={self.is_lost}"
        )
