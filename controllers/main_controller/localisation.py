import math


class Localiser:
    """
    Odometry + simplified 2D Markov localisation on a grid.
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
    
        # --------------------------------------------------
        # 1. Odometry setup
        # --------------------------------------------------
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
    
        self.left_encoder = self.robot.getDevice("left wheel sensor")
        self.right_encoder = self.robot.getDevice("right wheel sensor")
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)
    
        # --- Initial pose from Webots translation, if Supervisor ---
       
  
    
        
        

        # --- Initial pose (pure odometry frame, internal only) ---
        self.x = -0.14  # or 0.0 – just pick something consistent with your occupancy grid
        self.y = -0.26
        self.theta = 0.0
        
        self._prev_left = None
        self._prev_right = None

        # --------------------------------------------------
        self.ir_names = [f"ps{i}" for i in range(8)]
        self.ir_sensors = []
        for name in self.ir_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.ir_sensors.append(sensor)
    
        self.sensor_groups = {
            "front": [0, 7],
            "left": [1, 2],
            "back": [3, 4],
            "right": [5, 6],
        }
        self.ir_max_value = 3000.0
        self.sensor_max_range = 0.25
    
        # --------------------------------------------------
        # 3. Markov grid and belief
        # --------------------------------------------------
        self.cell_size = cell_size
    
        if world_map is None:
            world_map = self._build_default_map()
    
        self.world_map = world_map
        self.map_height = len(world_map)
        self.map_width = len(world_map[0])
    
        self.belief = [[0.0 for _ in range(self.map_width)]
                       for _ in range(self.map_height)]
    
        self.free_cells = [
            (iy, ix)
            for iy in range(self.map_height)
            for ix in range(self.map_width)
            if self.world_map[iy][ix] == 0
        ]
    
        self._init_uniform_belief()
    
        # Motion model parameters
        self.motion_correct_prob = 0.80
        self.motion_noise_prob = 0.20
    
        # --------------------------------------------------
        # 4. Debug: initial coordinates & map info
        # --------------------------------------------------
        print("=" * 50)
        print("[localiser] Initial coordinates and grid-map parameters:")
        print(f"[localiser] Robot initial world coordinates (odometry origin): "
              f"(x={self.x:.3f}m, y={self.y:.3f}m)")
        print(f"[localiser] Robot initial heading angle: theta={self.theta:.3f}rad "
              f"({math.degrees(self.theta):.1f}°)")
    
        width_m = self.map_width * self.cell_size
        height_m = self.map_height * self.cell_size
        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0
    
        init_ix = int(round((self.x - origin_x) / self.cell_size))
        init_iy = int(round((self.y - origin_y) / self.cell_size))
        init_ix = max(0, min(self.map_width - 1, init_ix))
        init_iy = max(0, min(self.map_height - 1, init_iy))
    
        print(f"\n[localiser] Grid map parameters:")
        print(f"[localiser] Grid map size: {self.map_height} rows × {self.map_width} columns")
        print(f"[localiser] Cell size: {self.cell_size:.3f}m")
        print(f"[localiser] Map physical dimensions: width={width_m:.3f}m, height={height_m:.3f}m")
        print(f"[localiser] Grid-map origin (world coordinates): "
              f"(x={origin_x:.3f}m, y={origin_y:.3f}m)")
    
        print(f"\n[localiser] Grid coordinate corresponding to robot initial world position: "
              f"(iy={init_iy}, ix={init_ix})")
        init_cell_state = "free (traversable)" if self.world_map[init_iy][init_ix] == 0 else "obstacle (blocked)"
        print(f"[localiser] Initial grid cell state: {init_cell_state}")
        print("=" * 50 + "\n")
    
        print("[localiser] init complete")
    
       
    def update(self):
        """
        One localisation update step:
          1) Update odometry pose.
          2) Markov motion update.
          3) Markov sensor update.
        """
        dx, dy, dtheta = self._update_odometry()
        self._markov_motion_update(dx, dy)
        self._markov_sensor_update()
        print("[localiser] belief update complete")

    def estimate_position(self):
        """
        Returns a 2D position estimate (x, y) from the MAP cell of the belief.
        Falls back to odometry if belief is degenerate.
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
            return (self.x, self.y)

        cell_y, cell_x = best_cell
        world_x, world_y = self._world_from_cell(cell_y, cell_x)
        return (world_x, world_y)

    def estimate_pose(self):
        """Return the continuous odometry pose (x, y, theta)."""
        return (self.x, self.y, self.theta)

    def get_belief(self, copy=True):
        """
        Returns the current belief grid.

        Args:
            copy (bool): if True, returns a deep copy.
        """
        if not copy:
            return self.belief
        return [row[:] for row in self.belief]

    def reset_belief(self, mode="uniform", center_cell=None, sigma_cells=2.0):
        """
        Reset the belief distribution.

        Args:
            mode (str): "uniform" or "gaussian".
            center_cell (tuple|None): (iy, ix) center for Gaussian mode.
            sigma_cells (float): std dev in cell units for Gaussian.
        """
        if mode.lower() == "uniform":
            self._init_uniform_belief()
            return

        if mode.lower() == "gaussian":
            if not self.free_cells:
                return

            if center_cell is None:
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

        self._init_uniform_belief()

    def measure_uncertainty(self, method="entropy"):
        """
        Measure localisation uncertainty.

        Args:
            method (str): "entropy" or "variance".
        """
        m = method.lower()
        if m == "entropy":
            return self._compute_entropy()
        elif m == "variance":
            return self._compute_position_variance()
        else:
            raise ValueError(f"Unknown uncertainty method: {method}")

    # ----------------- Odometry internals -----------------
    def _update_odometry(self):
        """
        Read wheel encoders and update (x, y, theta) using a
        differential drive odometry model.

        Returns:
            (dx, dy, dtheta) since last update in world frame.
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

        dl = d_left * self.wheel_radius
        dr = d_right * self.wheel_radius

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.axle_length

        old_x, old_y, old_theta = self.x, self.y, self.theta

        self.theta += dtheta
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        dx = self.x - old_x
        dy = self.y - old_y

        return dx, dy, dtheta

    # ----------------- Markov motion model -----------------
    def _markov_motion_update(self, dx, dy):
        """
        Simple motion update to the belief based on odometry displacement.
        """
        if not self.free_cells or self.cell_size <= 0.0:
            return

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
                    main_p = p * self.motion_correct_prob
                    new_belief[target_y][target_x] += main_p

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
                        new_belief[target_y][target_x] += noise_p
                else:
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

    # ----------------- Markov sensor model -----------------
    def _markov_sensor_update(self):
        """
        Update the belief using IR readings.
        """
        if not self.free_cells:
            return

        measured = self._read_ir_group_values()

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
            # keep motion-updated belief if all likelihoods collapse
            pass

    # ----------------- Sensor helpers -----------------
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
        Compute expected IR readings [front, left, back, right] for a given cell.
        Directions in grid coordinates:
            front: +x, left: +y, back: -x, right: -y
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
        From cell (iy, ix), march along direction (dy, dx) until hitting an
        occupied cell or leaving the map, and return distance in meters.
        """
        steps = 0
        while True:
            steps += 1
            dist = steps * self.cell_size
            if dist > self.sensor_max_range:
                return self.sensor_max_range

            ny = iy + dy * steps
            nx = ix + dx * steps
            if not (0 <= ny < self.map_height and 0 <= nx < self.map_width):
                return dist

            if self.world_map[ny][nx] == 1:
                return dist

    def _distance_to_ir_strength(self, dist):
        """
        Map distance to expected IR strength:
            dist = 0   → ir_max_value
            dist >= R  → 0
        Linear model.
        """
        if dist >= self.sensor_max_range:
            return 0.0
        return self.ir_max_value * (1.0 - dist / self.sensor_max_range)

    def _sensor_likelihood(self, measured, expected):
        """
        Likelihood based on average absolute difference between
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
        return max(likelihood, 1e-3)

    # ----------------- Belief, transforms, map -----------------
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
            if not self.free_cells:
                return
            for iy in range(self.map_height):
                for ix in range(self.map_width):
                    belief[iy][ix] = 0.0
            p0 = 1.0 / len(self.free_cells)
            for iy, ix in self.free_cells:
                belief[iy][ix] = p0

    def _compute_entropy(self):
        """Compute Shannon entropy of the current belief (natural log)."""
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
            dict with var_x, var_y, total_var (m^2).
        """
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
        Grid is centered at (0, 0) with given cell_size.
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
        Build the occupancy grid used for localisation and planning.

        Returns:
            2D list: world_map[h][w], where 0 = free space, 1 = obstacle.
        """
        world_map = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 19 (was 0)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 18 (was 1)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 17 (was 2)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 16 (was 3)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 15 (was 4)
            [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],  # iy = 14 (was 5)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 13 (was 6)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 12 (was 7)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # iy = 11 (was 8)
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],  # iy = 10 (was 9)
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 9  (was 10)
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 8  (was 11)
            [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 7  (was 12)
            [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 6  (was 13)
            [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 5  (was 14)
            [0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 4  (was 15)
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 3  (was 16)
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 2  (was 17)
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 1  (was 18)
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # iy = 0  (was 19)]
        ]
        
        print("[localiser] Using REAL Webots occupancy grid")

       
     
        return world_map

