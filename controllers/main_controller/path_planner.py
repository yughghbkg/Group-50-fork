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

        self.world_map = None
        self.path = []
        self.current_wp_index = 0

        self.time_step = int(robot.getBasicTimeStep())
        self.ps = [robot.getDevice(f"ps{i}") for i in range(8)]
        for s in self.ps:
            s.enable(self.time_step)

    def set_map(self, world_map):
        self.world_map = world_map
        self.map_h = len(world_map)
        self.map_w = len(world_map[0])

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

        self.path = [self.cell_to_world(c) for c in path_cells]
        self.current_wp_index = 0
        
        print(f"\n[planner] Path length: {len(self.path)}")
        for idx, waypoint in enumerate(self.path):
            wx, wy = waypoint
            print(f"[planner] Waypoint {idx:2d}: ({wx:.3f}, {wy:.3f})")

        print("I finished planning")
        print("=======================================")

    def follow_path(self):
        if not self.path or self.current_wp_index >= len(self.path):
            self.stop()
            return
    
        tx, ty = self.path[self.current_wp_index]

        rx, ry = self.localiser.estimate_position()
        _, _, heading = self.localiser.estimate_pose()
    
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
    
        if dist < 0.06:
            self.current_wp_index += 1
            return
    
        target_angle = math.atan2(dy, dx)
        angle_err = self.angle_diff(target_angle, heading)
    
        if abs(angle_err) > math.radians(35):
            turn = self.turn_speed if angle_err > 0 else -self.turn_speed
            self.left_motor.setVelocity(-turn)
            self.right_motor.setVelocity(turn)
            return
    
        correction = angle_err * 2.0
        left = self.forward_speed - correction
        right = self.forward_speed + correction
    
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
    
        front_left = self.ps[7].getValue()
        front_right = self.ps[0].getValue()

        if max(front_left, front_right) > 80.0:
            if front_left > front_right:
                left = -0.5 * self.max_speed
                right = -self.max_speed
            else:
                left = -self.max_speed
                right = -0.5 * self.max_speed
    
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

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
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)