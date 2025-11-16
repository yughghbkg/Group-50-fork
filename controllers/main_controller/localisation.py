# localisation.py
import math

class Localiser:
    """
    Odometry + simplified Markov localisation

    - Use wheel encoders for odometry to obtain continuous pose (x, y, theta)
    - Build a discrete grid map world_map (0=free, 1=obstacle)
    - belief[y][x] represents the probability that the robot is at this cell
    - On each update():
        1) Use odometry dx, dy to do motion update (translation + noise diffusion)
        2) Use IR sensors to do sensor update (adjust probability according to the
           pattern of "distance to walls")
    """

    def __init__(
        self,
        robot,
        time_step=64,
        wheel_radius=0.0205,      # e-puck wheel radius (m)
        axle_length=0.053,        # distance between the two wheel centers (m)
        cell_size=0.1,            # grid cell size (m) — can be tuned according to your maze size
        world_map=None,           # 0=free, 1=occupied; if None, use a simple default map
    ):
        self.robot = robot
        self.time_step = time_step

        # =========================
        # 1. Odometry part
        # =========================
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length

        self.left_encoder = self.robot.getDevice("left wheel sensor")
        self.right_encoder = self.robot.getDevice("right wheel sensor")
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)

        # Continuous pose estimate (initial position as origin)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading (rad)

        self._prev_left = None
        self._prev_right = None

        # =========================
        # 2. Infrared sensors (for Markov sensor model)
        # =========================
        # e-puck IR names are usually ps0..ps7
        self.ir_names = [f"ps{i}" for i in range(8)]
        self.ir_sensors = []
        for name in self.ir_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.ir_sensors.append(sensor)

        # Simplify 8 IR sensors into 4 directions: front / left / back / right
        # This grouping is only an approximation: roughly split into 4 groups
        # according to the e-puck sensor layout
        self.sensor_groups = {
            "front": [0, 7],   # front-left + front-right
            "left": [1, 2],    # left side
            "back": [3, 4],    # back side
            "right": [5, 6],   # right side
        }
        self.ir_max_value = 3000.0       # for normalization; exact value is not very sensitive
        self.sensor_max_range = 0.25     # effective sensor range (m), can be tuned

        # =========================
        # 3. Markov localisation grid
        # =========================
        self.cell_size = cell_size
        if world_map is None:
            world_map = self._build_default_map()

        self.world_map = world_map
        self.map_height = len(world_map)          # number of rows (y)
        self.map_width = len(world_map[0])        # number of columns (x)

        # belief[y][x] — probability distribution
        self.belief = [[0.0 for _ in range(self.map_width)] for _ in range(self.map_height)]
        self.free_cells = [(iy, ix)
                           for iy in range(self.map_height)
                           for ix in range(self.map_width)
                           if self.world_map[iy][ix] == 0]

        # Initial belief: uniform over all free cells
        if self.free_cells:
            p0 = 1.0 / len(self.free_cells)
            for (iy, ix) in self.free_cells:
                self.belief[iy][ix] = p0

        # Motion model parameters
        self.motion_correct_prob = 0.80  # probability that motion follows odometry prediction
        self.motion_noise_prob = 0.20    # total probability diffused to neighboring cells

        print("[localiser] Odometry + Markov localisation initialised")

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def update(self):
        """
        Call once per simulation step:
        1) Update odometry
        2) Update belief using odometry displacement (motion update)
        3) Update belief using IR sensors (sensor update)
        """
        dx, dy, dtheta = self._update_odometry()
        # Motion update
        self._markov_motion_update(dx, dy)
        # Sensor update
        self._markov_sensor_update()

    def estimate_position(self):
        """
        Return the center coordinates (x, y) of the grid cell with maximum
        probability in the Markov belief as the final position estimate.

        Note: This uses the world mapping of the grid coordinate system, with
        the origin assumed near the map center. If you need to fully align
        with the Webots world, you should fine-tune _world_from_cell()
        according to the actual size and placement of your maze.
        """
        max_p = 0.0
        best_cell = None
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                p = self.belief[iy][ix]
                if p > max_p:
                    max_p = p
                    best_cell = (iy, ix)

        if best_cell is None:
            # If belief is all zeros, fall back to pure odometry estimate
            return (self.x, self.y)

        wy, wx = best_cell
        world_x, world_y = self._world_from_cell(wy, wx)
        return (world_x, world_y)

    def estimate_pose(self):
        """
        Return the continuous pose (x, y, theta) from pure odometry,
        useful as a baseline or for debugging.
        """
        return (self.x, self.y, self.theta)

    # ------------------------------------------------------------------
    # 1) Odometry internal implementation
    # ------------------------------------------------------------------
    def _update_odometry(self):
        left = self.left_encoder.getValue()
        right = self.right_encoder.getValue()

        if self._prev_left is None or self._prev_right is None:
            self._prev_left = left
            self._prev_right = right
            return 0.0, 0.0, 0.0

        d_left = left - self._prev_left
        d_right = right - self._prev_right

        self._prev_left = left
        self._prev_right = right

        # Angle change -> wheel arc length
        dl = d_left * self.wheel_radius
        dr = d_right * self.wheel_radius

        dc = (dl + dr) / 2.0                       # forward distance of midpoint
        dtheta = (dr - dl) / self.axle_length      # heading change

        # Update pose
        old_x, old_y, old_theta = self.x, self.y, self.theta

        self.theta += dtheta
        # Normalize to [-pi, pi)
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        dx = self.x - old_x
        dy = self.y - old_y

        # You can enable the following print for debugging
        # print(f"[odom] x={self.x:.3f} y={self.y:.3f} theta={self.theta:.2f}")

        return dx, dy, dtheta

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
        new_belief = [[0.0 for _ in range(self.map_width)] for _ in range(self.map_height)]

        for iy in range(self.map_height):
            for ix in range(self.map_width):
                if self.world_map[iy][ix] == 1:
                    continue
                p = self.belief[iy][ix]
                if p <= 0.0:
                    continue

                # Main target cell after predicted motion
                target_y = iy + shift_y
                target_x = ix + shift_x

                # Check if target cell is valid (in map and free)
                if (0 <= target_y < self.map_height and
                        0 <= target_x < self.map_width and
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
                        if 0 <= ny < self.map_height and
                           0 <= nx < self.map_width and
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
                        if 0 <= ny < self.map_height and
                           0 <= nx < self.map_width and
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

        new_belief = [[0.0 for _ in range(self.map_width)] for _ in range(self.map_height)]
        total = 0.0

        for iy in range(self.map_height):
            for ix in range(self.map_width):
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
            for iy in range(self.map_height):
                for ix in range(self.map_width):
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
            if not (0 <= ny < self.map_height and 0 <= nx < self.map_width):
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
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                total += belief[iy][ix]

        if total > 0.0:
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    belief[iy][ix] /= total
        else:
            # If the sum is 0, reset to a uniform distribution over free cells
            if not self.free_cells:
                return
            p0 = 1.0 / len(self.free_cells)
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    belief[iy][ix] = 0.0
            for (fy, fx) in self.free_cells:
                belief[fy][fx] = p0

    def _world_from_cell(self, iy, ix):
        """
        Convert grid indices (iy, ix) to world coordinates (x, y).

        Current implementation:
          - Place the whole map centered around (0, 0)
          - cell_size controls the physical size of each grid cell

        If you already have a more precise world <-> grid mapping defined
        in your Planner, it is better to modify this function to match
        the Planner's logic.
        """
        # Map size (meters)
        width_m = self.map_width * self.cell_size
        height_m = self.map_height * self.cell_size

        # Map center as (0, 0)
        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        world_x = origin_x + (ix + 0.5) * self.cell_size
        world_y = origin_y + (iy + 0.5) * self.cell_size

        return world_x, world_y

    def _build_default_map(self):
        """
        Build a simple default map:
        - Walls around the border, free space inside
        - You can replace this with a grid map that matches your maze_world
          one-to-one, for example by copying from the occupancy grid
          provided in the course.
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
