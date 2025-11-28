import math

class Localiser:

    def __init__(self, robot,
                 time_step=64,
                 wheel_radius=0.0205,
                 axle_length=0.053,
                 cell_size=0.05):
        self.robot = robot
        self.time_step = time_step
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.cell_size = cell_size

        self.origin_x = -0.5
        self.origin_y = -0.5
        self.x, self.y, self.theta = -0.5, -0.5, -0.2618

        self.left_enc = robot.getDevice("left wheel sensor")
        self.right_enc = robot.getDevice("right wheel sensor")
        self.left_enc.enable(time_step)
        self.right_enc.enable(time_step)
        self._prev_l = None
        self._prev_r = None

        self.ir_sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
        for s in self.ir_sensors:
            s.enable(time_step)
        self.ir_max = 3000.0
        self.sensor_groups = {"front": [0, 7], "left": [1, 2],
                              "back": [3, 4], "right": [5, 6]}

        self.world_map = [
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1],
            [1,0,1,0,1,0,1,1,1,0,1,0,1,1,0,1],
            [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,1],
            [1,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1],
            [1,0,0,0,0,0,1,0,0,0,1,0,0,0,1,1],
            [1,1,1,1,1,0,1,1,1,0,1,1,1,0,0,1],
            [1,0,0,0,1,0,0,0,1,0,0,0,1,1,0,1],
            [1,0,1,0,1,1,1,0,1,0,1,0,0,1,0,1],
            [1,0,1,0,0,0,1,0,1,0,1,1,0,1,0,1],
            [1,0,1,1,1,0,1,0,1,0,0,0,0,1,0,1],
            [1,0,0,0,1,0,0,0,1,1,1,1,0,1,0,1],
            [1,1,1,0,1,1,1,0,1,0,0,0,0,1,0,1],
            [1,0,0,0,0,0,0,0,1,0,1,1,1,1,0,1],
            [1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,1],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        ]
        self.h, self.w = len(self.world_map), len(self.world_map[0])
        self.free = [(iy, ix)
                     for iy in range(self.h)
                     for ix in range(self.w)
                     if self.world_map[iy][ix] == 0]

        self.belief = [[0.0]*self.w for _ in range(self.h)]
        if self.free:
            p = 1.0/len(self.free)
            for iy, ix in self.free:
                self.belief[iy][ix] = p
        print("[init] free cells:", len(self.free))

    def update(self):
        dx, dy, dtheta = self._odometry_delta()
        self.x += dx
        self.y += dy
        self.theta = (self.theta + dtheta + math.pi) % (2*math.pi) - math.pi
        print(f"[update] odom delta: dx={dx:.3f} dy={dy:.3f} dtheta={dtheta:.3f}")

        self._motion_update(dx, dy)
        self._sensor_update()

    def estimate_position(self):
        best = None
        max_p = 0.0
        for iy in range(self.h):
            for ix in range(self.w):
                if self.belief[iy][ix] > max_p:
                    max_p = self.belief[iy][ix]
                    best = (iy, ix)
        if best is None:
            print("[estimate] no belief -> fallback to continuous")
            return self.x, self.y
        wx = self.origin_x + (best[1]+0.5)*self.cell_size
        wy = self.origin_y + (best[0]+0.5)*self.cell_size
        print(f"[estimate] max cell={best}  world=({wx:.3f}, {wy:.3f})  p={max_p:.3f}")
        return wx, wy

    def _odometry_delta(self):
        l = self.left_enc.getValue()
        r = self.right_enc.getValue()
        if self._prev_l is None:
            self._prev_l, self._prev_r = l, r
            return 0.0, 0.0, 0.0
        dl = (l - self._prev_l)*self.wheel_radius
        dr = (r - self._prev_r)*self.wheel_radius
        self._prev_l, self._prev_r = l, r
        dc = (dl+dr)/2.0
        dtheta = (dr-dl)/self.axle_length
        dx = dc*math.cos(self.theta)
        dy = dc*math.sin(self.theta)
        return dx, dy, dtheta

    def _motion_update(self, dx, dy):
        if not self.free:
            return
        shift_x = int(round(dx/self.cell_size))
        shift_y = int(round(dy/self.cell_size))
        new_b = [[0.0]*self.w for _ in range(self.h)]
        for iy in range(self.h):
            for ix in range(self.w):
                p = self.belief[iy][ix]
                if p <= 0.0 or self.world_map[iy][ix] == 1:
                    continue
                ty, tx = iy+shift_y, ix+shift_x
                if 0 <= ty < self.h and 0 <= tx < self.w and self.world_map[ty][tx] == 0:
                    new_b[ty][tx] += p
                else:
                    new_b[iy][ix] += p   
        self.belief = new_b
        self._normalize()
        print(f"[motion] shift=({shift_x}, {shift_y})  sum after norm={sum(sum(row) for row in self.belief):.3f}")


    def _sensor_update(self):
        meas = self._read_ir_group()
        print(f"[sensor] ir measured: front={meas[0]:.0f} left={meas[1]:.0f} back={meas[2]:.0f} right={meas[3]:.0f}")
        new_b = [[0.0]*self.w for _ in range(self.h)]
        total = 0.0
        for iy in range(self.h):
            for ix in range(self.w):
                if self.world_map[iy][ix] == 1:
                    continue
                expect = self._expected_ir(iy, ix)
                like = self._likelihood(meas, expect)
                post = self.belief[iy][ix]*like
                new_b[iy][ix] = post
                total += post
        if total > 0.0:
            for iy in range(self.h):
                for ix in range(self.w):
                    new_b[iy][ix] /= total
            self.belief = new_b
        print(f"[sensor] likelihood total={total:.3f}")


    def _read_ir_group(self):
        raw = [s.getValue() for s in self.ir_sensors]
        front = max(raw[i] for i in self.sensor_groups["front"])
        left  = max(raw[i] for i in self.sensor_groups["left"])
        back  = max(raw[i] for i in self.sensor_groups["back"])
        right = max(raw[i] for i in self.sensor_groups["right"])
        return [front, left, back, right]

    def _expected_ir(self, iy, ix):
        def dist(dy, dx):
            steps = 0
            while True:
                steps += 1
                d = steps*self.cell_size
                if d > 0.25:
                    return 0.25
                ny, nx = iy+dy*steps, ix+dx*steps
                if not (0 <= ny < self.h and 0 <= nx < self.w) or self.world_map[ny][nx] == 1:
                    return d
        d_front = dist(0, 1)
        d_left  = dist(1, 0)
        d_back  = dist(0, -1)
        d_right = dist(-1, 0)
        return [self.ir_max*(1-d/0.25) for d in (d_front, d_left, d_back, d_right)]

    def _likelihood(self, m, e):
        err = 0.0
        for a, b in zip(m, e):
            err += abs(a/self.ir_max - b/self.ir_max)
        err /= 4.0
        return max(1e-3, 1.0 - err)

    def _normalize(self):
        s = sum(sum(row) for row in self.belief)
        if s > 0.0:
            for iy in range(self.h):
                for ix in range(self.w):
                    self.belief[iy][ix] /= s
