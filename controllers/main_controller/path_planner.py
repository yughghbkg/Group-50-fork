import math
import heapq

class Planner:
    def __init__(self, robot, localiser, cell_size=0.05):
        self.robot = robot
        self.localiser = localiser
        self.cell_size = cell_size

        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        self.max_speed = 6.28
        self.turn_speed = 3.5
        self.forward_speed = 4.0
        self.waypoint_tol = 0.06
        self.pos_deadband = 0.015  # distance deadband to avoid twitching
        self.angle_deadband = math.radians(5)  # small angle errors ignored
        self.corr_gain = 1.5  # softer correction gain

        self.world_map = None
        self.path = []           # Final smoothed path used for tracking
        self.current_wp_index = 0

        # B-spline smoothing related parameters
        self.smooth_bspline = True
        self.samples_per_segment = 5  # Number of samples per control-point segment

        self.time_step = int(robot.getBasicTimeStep())
        self.ps = [robot.getDevice(f"ps{i}") for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

    def set_map(self, world_map):
        self.world_map = world_map
        self.map_h = len(world_map)
        self.map_w = len(world_map[0])

    # ------------------------------------------------------------------
    # Planning: A* -> coarse path -> B-spline smoothing -> smooth path
    # ------------------------------------------------------------------
    def plan(self, start_world, goal_world):
        print("=======================================")
        print("I am starting to plan")
        print(f"[planner] Planning path in world coordinates: from {start_world} to {goal_world}")

        start_cell = self.world_to_cell(start_world)
        goal_cell = self.world_to_cell(goal_world)

        print(f"[planner] Planning path in grid coordinates: from {start_cell} to {goal_cell}")

        path_cells = self.a_star(start_cell, goal_cell)
        if path_cells is None:
            print("[planner] No feasible path")
            self.path = []
            return

        # Raw polyline path from A* (world coordinates)
        raw_path = [self.cell_to_world(c) for c in path_cells]

        # Apply B-spline smoothing and sampling to the A* path
        if self.smooth_bspline:
            smooth_path = self._smooth_path_bspline(
                raw_path, samples_per_seg=self.samples_per_segment
            )
            # Ensure the start and end exactly match the original start/goal
            if smooth_path:
                smooth_path[0] = raw_path[0]
                smooth_path[-1] = raw_path[-1]
            self.path = smooth_path
        else:
            self.path = raw_path

        self.current_wp_index = 0

        print(f"\n[planner] Raw path length: {len(raw_path)}")
        print(f"[planner] Smoothed path length: {len(self.path)}")
        for idx, waypoint in enumerate(self.path):
            wx, wy = waypoint
            print(f"[planner] Waypoint {idx:2d}: ({wx:.3f}, {wy:.3f})")

        print("I finished planning")
        print("=======================================")

    # ------------------------------------------------------------------
    # Follow the smoothed path
    # ------------------------------------------------------------------
    def follow_path(self):
        if not self.path or self.current_wp_index >= len(self.path):
            self.stop()
            return
    
        rx, ry = self.localiser.estimate_position()
        _, _, heading = self.localiser.estimate_pose()

        # snap to closest waypoint ahead to reduce oscillation
        self._advance_to_closest(rx, ry)
        tx, ty = self.path[self.current_wp_index]

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        if dist < self.waypoint_tol:
            self.current_wp_index += 1
            return

        target_angle = math.atan2(dy, dx)
        angle_err = self.angle_diff(target_angle, heading)

        # deadband on small angular errors
        if abs(angle_err) < self.angle_deadband:
            angle_err = 0.0

        if abs(angle_err) > math.radians(35):
            turn = self.turn_speed if angle_err > 0 else -self.turn_speed
            self.left_motor.setVelocity(-turn)
            self.right_motor.setVelocity(turn)
            return

        # deadband near waypoint to avoid twitching
        if dist < self.pos_deadband:
            correction = 0.0
        else:
            correction = angle_err * self.corr_gain

        left = self.forward_speed - correction
        right = self.forward_speed + correction

        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))

        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def _advance_to_closest(self, rx, ry):
        """Advance current waypoint to the nearest point ahead to reduce small oscillations."""
        if not self.path:
            return
        best_idx = self.current_wp_index
        best_d2 = float("inf")
        for i in range(self.current_wp_index, len(self.path)):
            px, py = self.path[i]
            d2 = (px - rx) ** 2 + (py - ry) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
            # small early exit if already very close
            if d2 < (self.pos_deadband * 2) ** 2:
                best_idx = i
                break
        self.current_wp_index = best_idx

    # ------------------------------------------------------------------
    # A* search
    # ------------------------------------------------------------------
    def a_star(self, start, goal):
        sy, sx = start
        gy, gx = goal

        open_set = []
        heapq.heappush(open_set, (0, (sy, sx)))

        came_from = {}
        g_score = {(sy, sx): 0}
        dirs = [(1,0), (-1,0), (0,1), (0,-1)]

        while open_set:
            _, (y, x) = heapq.heappop(open_set)

            if (y, x) == (gy, gx):
                return self.reconstruct_path(came_from, start, goal)

            for dy, dx in dirs:
                ny = y + dy
                nx = x + dx

                if not (0 <= ny < self.map_h and 0 <= nx < self.map_w):
                    continue
                if self.world_map[ny][nx] == 1:
                    continue

                tentative = g_score[(y, x)] + 1
                if (ny, nx) not in g_score or tentative < g_score[(ny, nx)]:
                    g_score[(ny, nx)] = tentative
                    f = tentative + abs(ny - gy) + abs(nx - gx)
                    heapq.heappush(open_set, (f, (ny, nx)))
                    came_from[(ny, nx)] = (y, x)

        return None

    def reconstruct_path(self, came_from, start, goal):
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # B-spline smoothing: apply cubic B-spline to the A* world-coordinate path
    # ------------------------------------------------------------------
    def _smooth_path_bspline(self, points, samples_per_seg=5):
        """
        Apply uniform cubic B-spline smoothing to the given path points and
        sample points uniformly along the curve.

        points: [(x, y), ...] polyline path from A* (world coordinates)
        samples_per_seg: number of interpolation samples per B-spline segment
        """
        n = len(points)
        if n < 4:
            # Too few points: skip smoothing and return the original path
            return points[:]

        # Pad endpoints so curve segments can be generated near the start and end
        cps = [points[0]] + points + [points[-1], points[-1]]

        smoothed = []

        # Cubic uniform B-spline basis functions
        def basis(t):
            t2 = t * t
            t3 = t2 * t
            B0 = (-t3 + 3*t2 - 3*t + 1) / 6.0
            B1 = (3*t3 - 6*t2 + 4) / 6.0
            B2 = (-3*t3 + 3*t2 + 3*t + 1) / 6.0
            B3 = t3 / 6.0
            return B0, B1, B2, B3

        # Sample each control-point interval [Pi, Pi+1] using groups of 4 points
        for i in range(len(cps) - 3):
            p0 = cps[i]
            p1 = cps[i+1]
            p2 = cps[i+2]
            p3 = cps[i+3]
            for j in range(samples_per_seg):
                t = j / float(samples_per_seg)
                B0, B1, B2, B3 = basis(t)
                x = (B0 * p0[0] + B1 * p1[0] + B2 * p2[0] + B3 * p3[0])
                y = (B0 * p0[1] + B1 * p1[1] + B2 * p2[1] + B3 * p3[1])
                smoothed.append((x, y))

        # Finally ensure the goal point is included
        smoothed.append(points[-1])

        return smoothed

    # ------------------------------------------------------------------
    # Coordinate transforms, angle utilities & stop
    # ------------------------------------------------------------------
    def world_to_cell(self, pos):
        x, y = pos

        origin_x = -(self.map_w * self.cell_size) / 2.0
        origin_y = -(self.map_h * self.cell_size) / 2.0

        ix = int((x - origin_x) / self.cell_size)
        iy = int((y - origin_y) / self.cell_size)

        ix = max(0, min(self.map_w - 1, ix))
        iy = max(0, min(self.map_h - 1, iy))

        return (iy, ix)

    def cell_to_world(self, cell):
        iy, ix = cell

        origin_x = -(self.map_w * self.cell_size) / 2.0
        origin_y = -(self.map_h * self.cell_size) / 2.0

        wx = origin_x + (ix + 0.5) * self.cell_size
        wy = origin_y + (iy + 0.5) * self.cell_size

        return (wx, wy)

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
