# localisation.py
import math

class Localiser:
    """
    Odometry + gyro-fused heading + simplified Markov localisation
    """

    def __init__(
        self,
        robot,
        time_step=64,
        wheel_radius=0.0205,
        axle_length=0.053,
        cell_size=0.05,
        world_map=None,
    ):
        self.robot = robot
        self.time_step = time_step

        # -----------------------------------------------------
        # Continuous pose estimate
        # -----------------------------------------------------
        self.x = -0.325
        self.y = -0.325
        import math
        self.theta = -math.pi / 2

        self.wheel_radius = wheel_radius
        self.axle_length = axle_length

        # Encoders
        self.left_encoder = self.robot.getDevice("left wheel sensor")
        self.right_encoder = self.robot.getDevice("right wheel sensor")
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)

        self._prev_left = None
        self._prev_right = None
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.prev_gyro_time = None
        self.gyro_theta = 0.0

        # IR sensors
        self.ir_names = [f"ps{i}" for i in range(8)]
        self.ir_sensors = []
        for name in self.ir_names:
            s = self.robot.getDevice(name)
            s.enable(self.time_step)
            self.ir_sensors.append(s)

        self.sensor_groups = {
            "front": [0, 7],
            "left": [1, 2],
            "back": [3, 4],
            "right": [5, 6],
        }
        self.ir_max_value = 3000.0
        self.sensor_max_range = 0.25

        # -----------------------------------------------------
        # Markov grid map
        # -----------------------------------------------------
        self.cell_size = cell_size

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

        self.map_h = len(self.world_map)
        self.map_w = len(self.world_map[0])

        self.belief = [[0.0 for _ in range(self.map_w)] for _ in range(self.map_h)]
        self.free_cells = [(iy, ix)
                           for iy in range(self.map_h)
                           for ix in range(self.map_w)
                           if self.world_map[iy][ix] == 0]

        p0 = 1.0 / len(self.free_cells)
        for (iy, ix) in self.free_cells:
            self.belief[iy][ix] = p0

        self.motion_correct_prob = 0.80
        self.motion_noise_prob = 0.20

        print("[localiser] Odometry + gyro + Markov localisation initialised")

    # -----------------------------------------------------
    # Public update
    # -----------------------------------------------------
    def update(self):
        dx, dy, dtheta = self._update_odometry()
        self._update_gyro()

        self.theta = 0.9 * (self.theta + dtheta) + 0.1 * self.gyro_theta

        self._markov_motion_update(dx, dy)
        self._markov_sensor_update()

    def estimate_position(self):
        max_p = 0.0
        best_cell = None
        for iy in range(self.map_h):
            for ix in range(self.map_w):
                p = self.belief[iy][ix]
                if p > max_p:
                    max_p = p
                    best_cell = (iy, ix)

        if best_cell is None:
            return (self.x, self.y)

        wy, wx = best_cell
        return self._world_from_cell(wy, wx)

    def estimate_pose(self):
        return (self.x, self.y, self.theta)

    # -----------------------------------------------------
    # Odometry 
    # -----------------------------------------------------
    def _update_odometry(self):
        left = self.left_encoder.getValue()
        right = self.right_encoder.getValue()

        if self._prev_left is None:
            self._prev_left = left
            self._prev_right = right
            return 0.0, 0.0, 0.0

        d_left = left - self._prev_left
        d_right = right - self._prev_right

        self._prev_left = left
        self._prev_right = right

        dl = d_left * self.wheel_radius
        dr = d_right * self.wheel_radius

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.axle_length

        old_x, old_y = self.x, self.y

        self.theta += dtheta
        self.theta = (self.theta + math.pi) % (2*math.pi) - math.pi

        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        dx = self.x - old_x
        dy = self.y - old_y
        return dx, dy, dtheta

    def _update_gyro(self):
        gx, gy, gz = self.gyro.getValues()  
        dt = self.time_step / 1000  
        self.gyro_theta += gz * dt
        self.gyro_theta = (self.gyro_theta + math.pi) % (2*math.pi) - math.pi


    # ------------------------------------------------------------------
    # 2) Markov motion model (motion update)
    # ------------------------------------------------------------------
    def _markov_motion_update(self, dx, dy):
        """
        Use odometry displacement (dx, dy) to shift the belief and add
        diffusion noise. For simplicity, only position is considered,
        orientation is ignored.
        """
        if not self.free_cells:
            return

        # Convert continuous displacement to grid offset (column=x, row=y)
        # Note: we assume world x corresponds to horizontal grid axis,
        # and world y corresponds to vertical grid axis.
        if self.cell_size <= 0:
            return

        shift_x = int(round(dx / self.cell_size))
        shift_y = int(round(dy / self.cell_size))

        # If displacement is small, shift may be 0; we still apply some noise
        new_belief = [[0.0 for _ in range(self.map_w)] for _ in range(self.map_h)]

        for iy in range(self.map_h):
            for ix in range(self.map_w):
                if self.world_map[iy][ix] == 1:
                    continue
                p = self.belief[iy][ix]
                if p <= 0.0:
                    continue

                # Main target cell after predicted motion
                target_y = iy + shift_y
                target_x = ix + shift_x

                # Check if target cell is valid (in map and free)
                if (0 <= target_y < self.map_h and
                        0 <= target_x < self.map_w and
                        self.world_map[target_y][target_x] == 0):
                    # Most of the probability moves to the target cell
                    main_p = p * self.motion_correct_prob
                    new_belief[target_y][target_x] += main_p

                    # Remaining probability diffuses to the 4-neighborhood
                    noise_p = p * self.motion_noise_prob
                    neighbors = [
                        (target_y + 1, target_x),
                        (target_y - 1, target_x),
                        (target_y, target_x + 1),
                        (target_y, target_x - 1),
                    ]
                    valid_neighbors = [
                        (ny, nx)
                        for (ny, nx) in neighbors
                        if 0 <= ny < self.map_h and
                           0 <= nx < self.map_w and
                           self.world_map[ny][nx] == 0
                    ]
                    if valid_neighbors:
                        share = noise_p / len(valid_neighbors)
                        for (ny, nx) in valid_neighbors:
                            new_belief[ny][nx] += share
                    else:
                        # No valid neighbors, keep noise probability in target cell
                        new_belief[target_y][target_x] += noise_p
                else:
                    # If target cell is invalid (hit wall/out of bounds),
                    # stay in place and slightly diffuse around
                    stay_p = p * (self.motion_correct_prob + self.motion_noise_prob * 0.5)
                    new_belief[iy][ix] += stay_p

                    noise_p = p * self.motion_noise_prob * 0.5
                    neighbors = [
                        (iy + 1, ix),
                        (iy - 1, ix),
                        (iy, ix + 1),
                        (iy, ix - 1),
                    ]
                    valid_neighbors = [
                        (ny, nx)
                        for (ny, nx) in neighbors
                        if 0 <= ny < self.map_h and
                           0 <= nx < self.map_w and
                           self.world_map[ny][nx] == 0
                    ]
                    if valid_neighbors:
                        share = noise_p / len(valid_neighbors)
                        for (ny, nx) in valid_neighbors:
                            new_belief[ny][nx] += share
                    else:
                        new_belief[iy][ix] += noise_p

        # Normalize
        self._normalize_belief(new_belief)
        self.belief = new_belief

    # ------------------------------------------------------------------
    # 3) Markov observation model (sensor update)
    # ------------------------------------------------------------------
    def _markov_sensor_update(self):
        """
        Update belief using IR sensor readings.
        Simplified method:
          - Aggregate 8 sensors into 4 values: front/left/back/right
          - For each grid cell, estimate the "expected sensor pattern"
            from the distance to the nearest obstacle in each of the
            four directions
          - Compare measured vs expected values to obtain a likelihood
        """
        if not self.free_cells:
            return

        measured = self._read_ir_group_values()  # [front, left, back, right]

        new_belief = [[0.0 for _ in range(self.map_w)] for _ in range(self.map_h)]
        total = 0.0

        for iy in range(self.map_h):
            for ix in range(self.map_w):
                if self.world_map[iy][ix] == 1:
                    continue
                prior = self.belief[iy][ix]
                if prior <= 0.0:
                    continue

                expected = self._expected_ir_for_cell(iy, ix)
                likelihood = self._sensor_likelihood(measured, expected)

                posterior = prior * likelihood
                new_belief[iy][ix] = posterior
                total += posterior

        if total > 0.0:
            # Normal normalization
            for iy in range(self.map_h):
                for ix in range(self.map_w):
                    new_belief[iy][ix] /= total
            self.belief = new_belief
        else:
            # In extreme cases where all likelihoods are ~0, fall back to
            # the belief after motion update (i.e. ignore this observation)
            pass

    # ------------------------------------------------------------------
    # 4) Helper functions: read IR, expected values, likelihood, etc.
    # ------------------------------------------------------------------
    def _read_ir_group_values(self):
        """Simplify 8 IR readings into [front, left, back, right] by taking max in each group."""
        raw = [s.getValue() for s in self.ir_sensors]
        # Clamp to [0, ir_max_value]
        raw = [max(0.0, min(v, self.ir_max_value)) for v in raw]

        front = max(raw[i] for i in self.sensor_groups["front"])
        left = max(raw[i] for i in self.sensor_groups["left"])
        back = max(raw[i] for i in self.sensor_groups["back"])
        right = max(raw[i] for i in self.sensor_groups["right"])

        return [front, left, back, right]

    def _expected_ir_for_cell(self, iy, ix):
        """
        For grid cell (iy, ix), estimate the distance to the nearest obstacle
        in the front / left / back / right directions, then convert these
        distances into "expected IR strength".
        """
        # Assume:
        #   front: +x direction (increasing column index)
        #   left:  +y direction (increasing row index)
        #   back:  -x direction
        #   right: -y direction
        dist_front = self._distance_to_wall(iy, ix, dy=0, dx=1)
        dist_left = self._distance_to_wall(iy, ix, dy=1, dx=0)
        dist_back = self._distance_to_wall(iy, ix, dy=0, dx=-1)
        dist_right = self._distance_to_wall(iy, ix, dy=-1, dx=0)

        return [
            self._distance_to_ir_strength(dist_front),
            self._distance_to_ir_strength(dist_left),
            self._distance_to_ir_strength(dist_back),
            self._distance_to_ir_strength(dist_right),
        ]

    def _distance_to_wall(self, iy, ix, dy, dx):
        """
        From cell (iy, ix), search along direction (dy, dx) for the nearest
        obstacle, and return the distance (m). The maximum search distance
        is sensor_max_range.
        """
        steps = 0
        while True:
            steps += 1
            dist = steps * self.cell_size
            if dist > self.sensor_max_range:
                # No wall found within effective range
                return self.sensor_max_range

            ny = iy + dy * steps
            nx = ix + dx * steps
            if not (0 <= ny < self.map_h and 0 <= nx < self.map_w):
                # Out of bounds is treated as hitting a wall
                return dist

            if self.world_map[ny][nx] == 1:
                return dist

    def _distance_to_ir_strength(self, dist):
        """
        Map "distance to wall" to expected IR strength.
        Assume closer distance => larger IR value, linearly decreasing.
        """
        if dist >= self.sensor_max_range:
            return 0.0
        # Linear: dist=0 => ir_max, dist=range => 0
        return self.ir_max_value * (1.0 - dist / self.sensor_max_range)

    def _sensor_likelihood(self, measured, expected):
        """
        Compute likelihood based on the difference between measured and
        expected sensor values. Use a simple model:
        likelihood = 1 - average normalized absolute error, with a positive
        lower bound.
        """
        diffs = []
        for m, e in zip(measured, expected):
            # Normalize to [0, 1]
            m_n = m / self.ir_max_value if self.ir_max_value > 0 else 0.0
            e_n = e / self.ir_max_value if self.ir_max_value > 0 else 0.0
            diffs.append(abs(m_n - e_n))

        avg_diff = sum(diffs) / len(diffs) if diffs else 1.0
        # Larger difference => smaller likelihood
        likelihood = 1.0 - avg_diff
        # Avoid being exactly 0, keep a small probability
        return max(likelihood, 1e-3)

    # ------------------------------------------------------------------
    # 5) Belief normalization + coordinate transform + default map
    # ------------------------------------------------------------------
    def _normalize_belief(self, belief):
        total = 0.0
        for iy in range(self.map_h):
            for ix in range(self.map_w):
                total += belief[iy][ix]

        if total > 0.0:
            for iy in range(self.map_h):
                for ix in range(self.map_w):
                    belief[iy][ix] /= total
        else:
            # If the sum is 0, reset to a uniform distribution over free cells
            if not self.free_cells:
                return
            p0 = 1.0 / len(self.free_cells)
            for iy in range(self.map_h):
                for ix in range(self.map_w):
                    belief[iy][ix] = 0.0
            for (fy, fx) in self.free_cells:
                belief[fy][fx] = p0

    def _world_from_cell(self, iy, ix):
        width_m = self.map_w * self.cell_size   # 0.8
        height_m = self.map_h * self.cell_size  # 0.8

        origin_x = -width_m / 2.0   # -0.4
        origin_y = -height_m / 2.0  # -0.4

        world_x = origin_x + (ix + 0.5) * self.cell_size
        world_y = origin_y + (iy + 0.5) * self.cell_size
        return world_x, world_y

    def _build_default_map(self):
        """
        Build a simple default map:
        - Walls around the border, free space inside
        """
        w = 8  # number of columns
        h = 8  # number of rows

        world_map = [[0 for _ in range(w)] for _ in range(h)]

        # Set outer border as walls
        for iy in range(h):
            for ix in range(w):
                if iy == 0 or iy == h - 1 or ix == 0 or ix == w - 1:
                    world_map[iy][ix] = 1


        print("[localiser] WARNING: using default toy world_map, "
              "please replace with your maze occupancy grid for better results.")
        return world_map
