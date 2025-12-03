import math
import heapq

class Planner:
    # Basic initialization: store robot and localiser, and set motor mode
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

        # Common speed parameters for differential drive (adjustable)
        self.max_speed = 6.28
        self.turn_speed = 3.5
        self.forward_speed = 4.0

        self.world_map = None
        self.path = []
        self.current_wp_index = 0
        
        self.time_step = int(robot.getBasicTimeStep())
        self.ps = [robot.getDevice(f"ps{i}") for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

    # Load occupancy grid map (0 = free, 1 = obstacle)
    def set_map(self, world_map):
        self.world_map = world_map
        self.map_h = len(world_map)
        self.map_w = len(world_map[0])

    # Run A* algorithm: input world coordinates, output continuous-space path
    def plan(self, start_world, goal_world):
        print("=======================================")
        print("=======================================")
        print("=======================================")
        print("I am starting to plan")
        print(f"[planner] Planning path in world coordinates: from {start_world} to {goal_world}")

        start_cell = self.world_to_cell(start_world)
        goal_cell = self.world_to_cell(goal_world)
        print(f"[planner] Planning path in simulation grid coordinates: from {start_cell} to {goal_cell}")

        path_cells = self.a_star(start_cell, goal_cell)
        if path_cells is None:
            print("[planner] No feasible path")
            self.path = []
            return

        # Convert grid path to world coordinates
        self.path = [self.cell_to_world(c) for c in path_cells]
        self.current_wp_index = 0
        
        # -------------------------- Key modification: print full path --------------------------
        print(f"\n[planner] Path planning completed!")
        print(f"[planner] Path length (number of waypoints): {len(self.path)}")
        print(f"[planner] Full path (index: world coordinate (x, y), unit: meters):")
        for idx, waypoint in enumerate(self.path):
            wp_x, wp_y = waypoint
            print(f"[planner] Waypoint {idx:2d}: ({wp_x:.3f}, {wp_y:.3f})")
        # ----------------------------------------------------------------------------
        
        print("I finished planning\n")
        print("=======================================")
        print("=======================================")
        print("=======================================")

    # Follow the planned path and drive the robot
    def follow_path(self):
        if not self.path or self.current_wp_index >= len(self.path):
            self.stop()
            return
    
        tx, ty = self.path[self.current_wp_index]
        rx, ry, heading = self.localiser.estimate_pose()
    
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
    
        # Reached waypoint
        if dist < 0.04:
            self.current_wp_index += 1
            return
    
        target_angle = math.atan2(dy, dx)
        angle_err = self.angle_diff(target_angle, heading)
    
        # If heading error is large, rotate in place
        if abs(angle_err) > math.radians(35):
            turn = self.turn_speed if angle_err > 0 else -self.turn_speed
            self.left_motor.setVelocity(-turn)
            self.right_motor.setVelocity(turn)
            return
    
        # Small heading error: move forward while correcting direction
        correction = angle_err * 2.0
        left = self.forward_speed - correction
        right = self.forward_speed + correction
    
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
    
        # === New: Infrared safety override (prevent collision with wall) ===
        # ps0 and ps7 are e-puck’s forward sensors
        front_left = self.ps[7].getValue()
        front_right = self.ps[0].getValue()
        front = max(front_left, front_right)
    
        WALL_THRESHOLD = 80.0  # This threshold can be adjusted
    
        if front > WALL_THRESHOLD:
            # Very close to the wall in front
            # Reverse while turning away from the wall
            if front_left > front_right:
                # Left wall is closer → reverse and turn right
                left = -0.5 * self.max_speed
                right = -self.max_speed
            else:
                # Right wall is closer → reverse and turn left
                left = -self.max_speed
                right = -0.5 * self.max_speed
            # (No return here; only override speeds for this step)
    
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
    

    # A* search on the grid
    def a_star(self, start, goal):
        (sy, sx) = start
        (gy, gx) = goal

        open_set = []
        heapq.heappush(open_set, (0, (sy, sx)))

        came_from = {}
        g_score = { (sy, sx): 0 }

        dirs = [(1,0), (-1,0), (0,1), (0,-1)]  # 4-neighborhood movement

        while open_set:
            _, (y, x) = heapq.heappop(open_set)

            if (y, x) == (gy, gx):
                return self.reconstruct_path(came_from, start, goal)

            for dy, dx in dirs:
                ny = y + dy
                nx = x + dx

                if not (0 <= ny < self.map_h and 0 <= nx < self.map_w):
                    continue
                if self.world_map[ny][nx] == 1:  # Obstacle
                    continue

                tentative = g_score[(y, x)] + 1
                if (ny, nx) not in g_score or tentative < g_score[(ny, nx)]:
                    g_score[(ny, nx)] = tentative
                    f = tentative + abs(ny - gy) + abs(nx - gx)
                    heapq.heappush(open_set, (f, (ny, nx)))
                    came_from[(ny, nx)] = (y, x)

        return None

    # Reconstruct final path by backtracking
    def reconstruct_path(self, came_from, start, goal):
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
        return path

    # World coordinate → grid cell index
    def world_to_cell(self, pos):
        x, y = pos
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")
        # print(f"[path_planner] Received world coordinate: ({x:.3f}, {y:.3f})")

        width_m = self.map_w * self.cell_size
        height_m = self.map_h * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        ix = int((x - origin_x) / self.cell_size)
        iy = int((y - origin_y) / self.cell_size)

        ix = max(0, min(self.map_w - 1, ix))
        iy = max(0, min(self.map_h - 1, iy))
        # print(f"[path_planner] Converted simulation grid coordinate: ({x:.3f}, {y:.3f})")
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")

        return (iy, ix)

    # Grid index → world coordinate (cell center)
    def cell_to_world(self, cell):
        iy, ix = cell

        width_m = self.map_w * self.cell_size
        height_m = self.map_h * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        wx = origin_x + (ix + 0.5) * self.cell_size
        wy = origin_y + (iy + 0.5) * self.cell_size

        return (wx, wy)

    # Normalize angle difference to [-pi, pi]
    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    # Stop robot movement
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)