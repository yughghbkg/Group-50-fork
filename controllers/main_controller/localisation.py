# localisation.py
import math


class Localiser:
    """
    Odometry + simplified 2D Markov localisation on a grid.
    """

    def __init__(
        self,
        robot,
        time_step=64,
        wheel_radius=0.0205,      # e-puck wheel radius (meters)
        axle_length=0.053,        # distance between wheels (meters)
        cell_size=0.05,           # grid cell size in meters (smaller = finer resolution)
        world_map=None,           # 2D list: 0 = free, 1 = occupied; see _build_default_map()
    ):
        self.robot = robot
        self.time_step = time_step

        # ------------------------------------------------------------------
        # 1. Odometry setup
        # ------------------------------------------------------------------
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length

        self.left_encoder = self.robot.getDevice("left wheel sensor")
        self.right_encoder = self.robot.getDevice("right wheel sensor")
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)

        # Continuous pose from odometry (origin at initial pose)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading in radians

        self._prev_left = None
        self._prev_right = None

        # ------------------------------------------------------------------
        # 2. IR sensors for the Markov sensor model
        # ------------------------------------------------------------------
        # Typical e-puck IR names: ps0..ps7
        self.ir_names = [f"ps{i}" for i in range(8)]
        self.ir_sensors = []
        for name in self.ir_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.ir_sensors.append(sensor)

        # Group 8 IR sensors into 4 approximate directions
        self.sensor_groups = {
            "front": [0, 7],   # front-left + front-right
            "left": [1, 2],
            "back": [3, 4],
            "right": [5, 6],
        }
        self.ir_max_value = 3000.0       # used for normalisation
        self.sensor_max_range = 0.25     # assumed max useful range (meters)

        # ------------------------------------------------------------------
        # 3. Markov grid and belief
        # ------------------------------------------------------------------
        self.cell_size = cell_size

        # world_map: 2D occupancy grid. If not provided, we build a default maze.
        if world_map is None:
            world_map = self._build_default_map()

        self.world_map = world_map
        self.map_height = len(world_map)          # number of rows (y index)
        self.map_width = len(world_map[0])        # number of columns (x index)

        # belief[y][x] = probability of being in that grid cell
        self.belief = [[0.0 for _ in range(self.map_width)]
                       for _ in range(self.map_height)]

        # list of free cells (y, x) for convenience
        self.free_cells = [
            (iy, ix)
            for iy in range(self.map_height)
            for ix in range(self.map_width)
            if self.world_map[iy][ix] == 0
        ]

        self._init_uniform_belief()

        # Motion model parameters (tune as needed)
        self.motion_correct_prob = 0.80  # probability assigned to the odom-predicted cell
        self.motion_noise_prob = 0.20    # spread to neighbours

        print("[localiser] Odometry + Markov localisation initialised")

    # ======================================================================
    # Public interface
    # ======================================================================
    def update(self):
        """
        Call this once per control loop iteration:
          1) Update odometry pose.
          2) Apply Markov motion update using odometry displacement.
          3) Apply Markov sensor update using IR readings.
        """
        dx, dy, dtheta = self._update_odometry()
        self._markov_motion_update(dx, dy)
        self._markov_sensor_update()

    def estimate_position(self):
        """
        Returns a 2D position estimate (x, y) in world coordinates
        based on the maximum a posteriori (MAP) cell of the Markov belief.
        If belief is degenerate (all zeros), falls back to raw odometry.
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
            # Degenerate case: just return odometry pose.
            return (self.x, self.y)

        cell_y, cell_x = best_cell
        world_x, world_y = self._world_from_cell(cell_y, cell_x)
        return (world_x, world_y)

    def estimate_pose(self):
        """
        Returns the continuous odometry pose (x, y, theta).
        This does NOT include Markov corrections and can be used
        as a baseline for evaluation.
        """
        return (self.x, self.y, self.theta)

    def get_belief(self, copy=True):
        """
        Returns the current belief grid.

        Args:
            copy (bool): if True (default), returns a deep copy so callers
                         cannot accidentally modify internal state.

        Returns:
            2D list of floats with shape [map_height][map_width].
        """
        if not copy:
            return self.belief
        return [row[:] for row in self.belief]

    def reset_belief(self, mode="uniform", center_cell=None, sigma_cells=2.0):
        """
        Reset the belief distribution.

        Args:
            mode (str): "uniform" or "gaussian".
                        - "uniform": equal probability over all free cells.
                        - "gaussian": Gaussian over grid cells (y, x) around
                          `center_cell` with std `sigma_cells`.
            center_cell (tuple|None): (iy, ix) grid indices for Gaussian center.
                                      If None and mode == "gaussian", we use
                                      the current MAP cell.
            sigma_cells (float): standard deviation of the Gaussian in
                                 cell units (only used for "gaussian").
        """
        if mode.lower() == "uniform":
            self._init_uniform_belief()
            return

        if mode.lower() == "gaussian":
            if not self.free_cells:
                return

            if center_cell is None:
                # Use current MAP cell as center if not provided
                max_p = 0.0
                center_cell = self.free_cells[0]
                for iy, ix in self.free_cells:
                    p = self.belief[iy][ix]
                    if p > max_p:
                        max_p = p
                        center_cell = (iy, ix)

            cy, cx = center_cell
            new_belief = [[0.0 for _ in range(self.map_width)]
                          for _ in range(self.map_height)]

            two_sigma_sq = 2.0 * sigma_cells * sigma_cells
            for iy, ix in self.free_cells:
                dy = iy - cy
                dx = ix - cx
                d2 = dx * dx + dy * dy
                new_belief[iy][ix] = math.exp(-d2 / two_sigma_sq)

            self._normalize_belief(new_belief)
            self.belief = new_belief
            return

        # Any other string: fall back to uniform
        self._init_uniform_belief()

    def measure_uncertainty(self, method="entropy"):
        """
        Measure localisation uncertainty based on the current belief.

        Args:
            method (str): "entropy" or "variance".
                - "entropy": returns Shannon entropy H(p) = -sum p log p.
                  Higher entropy means more spread-out belief.
                - "variance": returns a dict with variance over x and y:
                      {"var_x": ..., "var_y": ..., "total_var": ...}
                  Variance is computed in world coordinates (meters^2).

        Returns:
            float or dict depending on method.
        """
        m = method.lower()
        if m == "entropy":
            return self._compute_entropy()
        elif m == "variance":
            return self._compute_position_variance()
        else:
            raise ValueError(f"Unknown uncertainty method: {method}")

    # ======================================================================
    # 1) Odometry internals
    # ======================================================================
    def _update_odometry(self):
        """
        Read wheel encoders and update (x, y, theta) using a standard
        differential drive odometry model.

        Returns:
            (dx, dy, dtheta): displacement since last update in world frame.
        """
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

        # Wheel angle change → arc length
        dl = d_left * self.wheel_radius
        dr = d_right * self.wheel_radius

        dc = (dl + dr) / 2.0                       # forward distance of robot center
        dtheta = (dr - dl) / self.axle_length      # change in heading

        old_x, old_y, old_theta = self.x, self.y, self.theta

        self.theta += dtheta
        # Wrap angle to [-pi, pi)
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        dx = self.x - old_x
        dy = self.y - old_y

        return dx, dy, dtheta

    # ======================================================================
    # 2) Markov motion model
    # ======================================================================
    def _markov_motion_update(self, dx, dy):
        """
        Apply a simple motion update to the belief based on odometry
        displacement (dx, dy).

        The motion model:
            - Converts (dx, dy) into a grid shift (integer number of cells).
            - Moves probability mass accordingly.
            - Spreads some probability to neighbouring cells (motion_noise_prob).
        """
        if not self.free_cells or self.cell_size <= 0.0:
            return

        # Convert continuous displacement to grid cell offsets
        shift_x = int(round(dx / self.cell_size))
        shift_y = int(round(dy / self.cell_size))

        new_belief = [[0.0 for _ in range(self.map_width)]
                      for _ in range(self.map_height)]

        for iy in range(self.map_height):
            for ix in range(self.map_width):
                if self.world_map[iy][ix] == 1:
                    continue
                p = self.belief[iy][ix]
                if p <= 0.0:
                    continue

                target_y = iy + shift_y
                target_x = ix + shift_x

                if (
                    0 <= target_y < self.map_height
                    and 0 <= target_x < self.map_width
                    and self.world_map[target_y][target_x] == 0
                ):
                    # Most probability moves to the odom-predicted cell
                    main_p = p * self.motion_correct_prob
                    new_belief[target_y][target_x] += main_p

                    # Remaining probability spreads to 4-neighbourhood of target
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
                        if (
                            0 <= ny < self.map_height
                            and 0 <= nx < self.map_width
                            and self.world_map[ny][nx] == 0
                        )
                    ]
                    if valid_neighbors:
                        share = noise_p / len(valid_neighbors)
                        for (ny, nx) in valid_neighbors:
                            new_belief[ny][nx] += share
                    else:
                        # No valid neighbours: dump noise into target cell
                        new_belief[target_y][target_x] += noise_p
                else:
                    # Predicted move hits a wall or goes out of map:
                    # stay in place and spread some probability to neighbours.
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
                        if (
                            0 <= ny < self.map_height
                            and 0 <= nx < self.map_width
                            and self.world_map[ny][nx] == 0
                        )
                    ]
                    if valid_neighbors:
                        share = noise_p / len(valid_neighbors)
                        for (ny, nx) in valid_neighbors:
                            new_belief[ny][nx] += share
                    else:
                        new_belief[iy][ix] += noise_p

        self._normalize_belief(new_belief)
        self.belief = new_belief

    # ======================================================================
    # 3) Markov sensor model
    # ======================================================================
    def _markov_sensor_update(self):
        """
        Update the belief using IR readings.

        Steps:
            - Aggregate 8 IR sensors into 4 directional values.
            - For each grid cell, compute expected IR readings based on
              distance to nearest wall in each direction.
            - Compute a simple likelihood comparing measured vs expected.
            - Multiply prior belief by likelihood and normalise.
        """
        if not self.free_cells:
            return

        measured = self._read_ir_group_values()  # [front, left, back, right]

        new_belief = [[0.0 for _ in range(self.map_width)]
                      for _ in range(self.map_height)]
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
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    new_belief[iy][ix] /= total
            self.belief = new_belief
        else:
            # In extreme cases where all likelihoods are ~0 we keep
            # the motion-updated belief (i.e., skip the observation).
            pass

    # ======================================================================
    # 4) Sensor helpers
    # ======================================================================
    def _read_ir_group_values(self):
        """Return aggregated IR values [front, left, back, right]."""
        raw = [s.getValue() for s in self.ir_sensors]
        raw = [max(0.0, min(v, self.ir_max_value)) for v in raw]

        front = max(raw[i] for i in self.sensor_groups["front"])
        left = max(raw[i] for i in self.sensor_groups["left"])
        back = max(raw[i] for i in self.sensor_groups["back"])
        right = max(raw[i] for i in self.sensor_groups["right"])

        return [front, left, back, right]

    def _expected_ir_for_cell(self, iy, ix):
        """
        Compute expected IR readings [front, left, back, right] for a given grid cell.
        Directions are defined in grid coordinates as:
            - front: +x
            - left:  +y
            - back:  -x
            - right: -y
        """
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
        From cell (iy, ix), march along direction (dy, dx) until hitting an occupied
        cell or leaving the map, and return the physical distance in meters.
        """
        steps = 0
        while True:
            steps += 1
            dist = steps * self.cell_size
            if dist > self.sensor_max_range:
                # No wall within sensor range
                return self.sensor_max_range

            ny = iy + dy * steps
            nx = ix + dx * steps
            if not (0 <= ny < self.map_height and 0 <= nx < self.map_width):
                # Leaving map is treated as hitting a wall
                return dist

            if self.world_map[ny][nx] == 1:
                return dist

    def _distance_to_ir_strength(self, dist):
        """
        Map distance to expected IR strength:
            - dist = 0   → ir_max_value
            - dist >= R  → 0
        Using a simple linear model for now.
        """
        if dist >= self.sensor_max_range:
            return 0.0
        return self.ir_max_value * (1.0 - dist / self.sensor_max_range)

    def _sensor_likelihood(self, measured, expected):
        """
        Compute a simple likelihood based on average absolute difference between
        measured and expected (both normalised to [0, 1]).
        """
        diffs = []
        for m, e in zip(measured, expected):
            m_n = m / self.ir_max_value if self.ir_max_value > 0 else 0.0
            e_n = e / self.ir_max_value if self.ir_max_value > 0 else 0.0
            diffs.append(abs(m_n - e_n))

        if not diffs:
            return 1e-3

        avg_diff = sum(diffs) / len(diffs)
        likelihood = 1.0 - avg_diff
        return max(likelihood, 1e-3)  # avoid exactly zero

    # ======================================================================
    # 5) Belief normalisation, uncertainty, coordinate transforms, map build
    # ======================================================================
    def _init_uniform_belief(self):
        """Set a uniform belief over all free cells."""
        if not self.free_cells:
            return

        for iy in range(self.map_height):
            for ix in range(self.map_width):
                self.belief[iy][ix] = 0.0

        p0 = 1.0 / len(self.free_cells)
        for iy, ix in self.free_cells:
            self.belief[iy][ix] = p0

    def _normalize_belief(self, belief):
        """Normalise a 2D belief grid in-place; fall back to uniform if needed."""
        total = 0.0
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                total += belief[iy][ix]

        if total > 0.0:
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    belief[iy][ix] /= total
        else:
            # Degenerate case: reinitialise as uniform over free cells
            if not self.free_cells:
                return
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    belief[iy][ix] = 0.0
            p0 = 1.0 / len(self.free_cells)
            for iy, ix in self.free_cells:
                belief[iy][ix] = p0

    def _compute_entropy(self):
        """Compute Shannon entropy of the current belief (natural logarithm)."""
        H = 0.0
        eps = 1e-12
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                p = self.belief[iy][ix]
                if p > 0.0:
                    H -= p * math.log(p + eps)
        return H

    def _compute_position_variance(self):
        """
        Compute variance of position in world coordinates based on the belief.

        Returns:
            dict with keys:
              - "var_x": variance over x (m^2)
              - "var_y": variance over y (m^2)
              - "total_var": var_x + var_y
        """
        # First compute mean position E[x], E[y]
        mean_x = 0.0
        mean_y = 0.0
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                p = self.belief[iy][ix]
                if p <= 0.0:
                    continue
                wx, wy = self._world_from_cell(iy, ix)
                mean_x += p * wx
                mean_y += p * wy

        # Then compute E[x^2], E[y^2]
        mean_x2 = 0.0
        mean_y2 = 0.0
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                p = self.belief[iy][ix]
                if p <= 0.0:
                    continue
                wx, wy = self._world_from_cell(iy, ix)
                mean_x2 += p * wx * wx
                mean_y2 += p * wy * wy

        var_x = max(0.0, mean_x2 - mean_x * mean_x)
        var_y = max(0.0, mean_y2 - mean_y * mean_y)
        return {"var_x": var_x, "var_y": var_y, "total_var": var_x + var_y}

    def _world_from_cell(self, iy, ix):
        """
        Convert grid indices (iy, ix) to world coordinates (x, y).

        Current convention:
            - The grid is centered around (0, 0) in world coordinates.
            - cell_size defines how large each grid cell is (meters).
        IMPORTANT:
            In your report and planner, you should ensure that this transform
            is consistent with how you discretise the Webots world. If your
            maze is not centered at the origin, you need to adjust origin_x
            and origin_y to match.
        """
        width_m = self.map_width * self.cell_size
        height_m = self.map_height * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        world_x = origin_x + (ix + 0.5) * self.cell_size
        world_y = origin_y + (iy + 0.5) * self.cell_size

        return world_x, world_y

    def _build_default_map(self):
        """
        Build a default occupancy grid roughly matching a simple maze.

        IMPORTANT:
            This is still a placeholder. For best performance and to satisfy
            your project feedback, you should replace this with an occupancy
            grid that matches your actual Webots maze layout (same dimensions
            and wall positions).

        Format:
            - Returns a 2D list world_map[h][w]
            - 0 = free space, 1 = wall/obstacle
        """
        # Example: 12x12 grid with outer walls and some internal barriers.
        # Replace this with your real maze occupancy grid.
        w = 12  # number of columns
        h = 12  # number of rows

        world_map = [[0 for _ in range(w)] for _ in range(h)]

        # Outer walls
        for iy in range(h):
            for ix in range(w):
                if iy == 0 or iy == h - 1 or ix == 0 or ix == w - 1:
                    world_map[iy][ix] = 1

        # Example internal walls (you should adapt to your maze):
        # A vertical wall near the left
        for iy in range(2, 9):
            world_map[iy][3] = 1

        # A horizontal wall near the top
        for ix in range(4, 10):
            world_map[2][ix] = 1

        # A U-shaped structure in the middle
        for ix in range(4, 8):
            world_map[6][ix] = 1
        for iy in range(6, 9):
            world_map[iy][4] = 1
        for iy in range(6, 9):
            world_map[iy][7] = 1

        print(
            "[localiser] WARNING: using a default maze occupancy grid.\n"
            "Replace _build_default_map() with your actual maze layout or pass\n"
            "a world_map into Localiser(...) to match your Webots world."
        )
        return world_map
