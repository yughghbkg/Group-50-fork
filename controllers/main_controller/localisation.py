import math


class Localiser:
    """
    GPS + IMU pose (no lidar mapping). Public API stays compatible: update(),
    estimate_position(), estimate_pose(), get_belief(), reset_belief(),
    measure_uncertainty().
    """

    def __init__(self, robot, time_step=64, cell_size=0.05, world_map=None):
        self.robot = robot
        self.time_step = int(time_step)
        self.cell_size = cell_size

        # --- Sensors (match wasd.cpp naming) ---
        self.gps = self.robot.getDevice("gps")
        if self.gps:
            self.gps.enable(self.time_step)
        else:
            print("[localiser][WARN] gps device not found, pose will stay at origin")

        self.imu = self._try_get_imu()
        if self.imu:
            self.imu.enable(self.time_step)
        else:
            print("[localiser][WARN] inertial unit not found, heading will stay at 0")

        # --- Grid map setup ---
        if world_map is None:
            world_map = self._build_default_map()
        self.world_map = world_map
        self.map_height = len(world_map)
        self.map_width = len(world_map[0])

        # Pose state (directly from GPS / IMU)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._pose_valid = False

        # Deterministic belief: 1.0 at the GPS cell, 0 elsewhere
        self.belief = [[0.0 for _ in range(self.map_width)]
                       for _ in range(self.map_height)]

        # Logging cadence
        self._log_tick = 0
        self._log_every = 30  # steps between health logs

        self._update_pose_from_sensors(initial=True)
        print("[localiser] init complete (GPS/IMU only)")

    # ------------------------------------------------------------------ #
    # Public API (same names as before)
    # ------------------------------------------------------------------ #
    def update(self):
        """Refresh pose from GPS/IMU."""
        self._update_pose_from_sensors()
        self._update_belief_from_pose()
        self._log_sensor_status()

    def estimate_position(self):
        """Return current world position (x, y) taken from GPS."""
        return (self.x, self.y)

    def estimate_pose(self):
        """Return current pose (x, y, theta) taken from GPS + IMU."""
        return (self.x, self.y, self.theta)

    def get_belief(self, copy=True):
        if not copy:
            return self.belief
        return [row[:] for row in self.belief]

    def reset_belief(self, *_, **__):
        """Reset to a deterministic belief at the current GPS cell."""
        self._update_belief_from_pose()

    def measure_uncertainty(self, method="entropy"):
        m = method.lower()
        if m == "entropy":
            return self._compute_entropy()
        if m == "variance":
            return self._compute_position_variance()
        raise ValueError(f"Unknown uncertainty method: {method}")

    def has_valid_pose(self):
        return self._pose_valid

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #
    def _try_get_imu(self):
        for name in ("yaw", "inertial unit", "inertial_unit", "imu"):
            dev = self.robot.getDevice(name)
            if dev:
                return dev
        return None

    def _update_pose_from_sensors(self, initial=False):
        if self.gps:
            pos = self.gps.getValues()
            if pos and len(pos) >= 2 and not (math.isnan(pos[0]) or math.isnan(pos[1])):
                self.x, self.y = pos[0], pos[1]
                self._pose_valid = True
            else:
                if not getattr(self, "_gps_warned", False):
                    print("[localiser][WARN] gps returned NaN, keeping previous pose until valid")
                    self._gps_warned = True
                return
        if self.imu:
            rpy = self.imu.getRollPitchYaw()
            yaw = None
            if rpy and len(rpy) >= 3:
                yaw = rpy[2]
            elif rpy and len(rpy) >= 1:
                yaw = rpy[0]
            if yaw is not None and not math.isnan(yaw):
                self.theta = self._wrap_angle(yaw)
            elif not getattr(self, "_imu_warned", False):
                print("[localiser][WARN] imu returned invalid yaw, keeping previous heading")
                self._imu_warned = True
        if initial:
            self._update_belief_from_pose()

    def _update_belief_from_pose(self):
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                self.belief[iy][ix] = 0.0
        cell = self._world_to_cell(self.x, self.y)
        if cell:
            iy, ix = cell
            self.belief[iy][ix] = 1.0

    def _world_to_cell(self, wx, wy):
        if math.isnan(wx) or math.isnan(wy):
            return None
        origin_x = -(self.map_width * self.cell_size) / 2.0
        origin_y = -(self.map_height * self.cell_size) / 2.0

        ix = int((wx - origin_x) / self.cell_size)
        iy = int((wy - origin_y) / self.cell_size)

        if ix < 0 or ix >= self.map_width or iy < 0 or iy >= self.map_height:
            return None
        return (iy, ix)

    def _world_from_cell(self, iy, ix):
        width_m = self.map_width * self.cell_size
        height_m = self.map_height * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        world_x = origin_x + (ix + 0.5) * self.cell_size
        world_y = origin_y + (iy + 0.5) * self.cell_size

        return world_x, world_y

    def _wrap_angle(self, ang):
        while ang > math.pi:
            ang -= 2 * math.pi
        while ang < -math.pi:
            ang += 2 * math.pi
        return ang

    # ------------------------------------------------------------------ #
    # Basic uncertainty utilities (still used by replanner)
    # ------------------------------------------------------------------ #
    def _compute_entropy(self):
        H = 0.0
        eps = 1e-12
        for iy in range(self.map_height):
            for ix in range(self.map_width):
                p = self.belief[iy][ix]
                if p > 0.0:
                    H -= p * math.log(p + eps)
        return H

    def _compute_position_variance(self):
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

    # ------------------------------------------------------------------ #
    # Logging helpers
    # ------------------------------------------------------------------ #
    def _log_sensor_status(self):
        self._log_tick += 1
        if self._log_tick % self._log_every != 0:
            return

        gps_state = "gps=nan"
        if not (math.isnan(self.x) or math.isnan(self.y)):
            gps_state = f"gps=({self.x:.3f},{self.y:.3f})"

        imu_state = "yaw=nan"
        if not math.isnan(self.theta):
            imu_state = f"yaw={self.theta:.3f}"

        print(f"[localiser][health] {gps_state} | {imu_state}")

    # ------------------------------------------------------------------ #
    # Default map (static occupancy grid for planning)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _build_default_map():
        world_map  =   [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]
        return world_map
