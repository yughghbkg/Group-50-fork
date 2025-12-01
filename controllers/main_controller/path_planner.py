import math
import heapq

class Planner:
    # 基础初始化：存储机器人、定位器，并设置电机模式
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

        # 差分驱动常用的速度参数（可调节）
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

    # 加载占据栅格地图（0=空闲，1=障碍物）
    def set_map(self, world_map):
        self.world_map = world_map
        self.map_h = len(world_map)
        self.map_w = len(world_map[0])

    # 运行A*算法：输入世界坐标，输出连续空间路径
    def plan(self, start_world, goal_world):
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")
        print("我开始规划了")
        print(f"[规划器] 正在规划从 {start_world} 到 {goal_world} 的路径")

        start_cell = self.world_to_cell(start_world)
        goal_cell = self.world_to_cell(goal_world)

        path_cells = self.a_star(start_cell, goal_cell)
        if path_cells is None:
            print("[规划器] 无可行路径")
            self.path = []
            return

        # 将栅格路径转换为世界坐标
        self.path = [self.cell_to_world(c) for c in path_cells]
        self.current_wp_index = 0
        print(f"[规划器] 路径长度 = {len(self.path)}")
        print("我规划完成了")
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")

    # 跟踪规划路径并驱动机器人
    def follow_path(self):
        if not self.path or self.current_wp_index >= len(self.path):
            self.stop()
            return
    
        tx, ty = self.path[self.current_wp_index]
        rx, ry, heading = self.localiser.estimate_pose()
    
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
    
        # 到达路点
        if dist < 0.04:
            self.current_wp_index += 1
            return
    
        target_angle = math.atan2(dy, dx)
        angle_err = self.angle_diff(target_angle, heading)
    
        # 如果航向误差较大，原地旋转
        if abs(angle_err) > math.radians(35):
            turn = self.turn_speed if angle_err > 0 else -self.turn_speed
            self.left_motor.setVelocity(-turn)
            self.right_motor.setVelocity(turn)
            return
    
        # 航向误差较小：前进的同时修正方向
        correction = angle_err * 2.0
        left = self.forward_speed - correction
        right = self.forward_speed + correction
    
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
    
        # === 新增：红外安全覆盖（防止撞墙） ===
        # ps0和ps7是e-puck的前向传感器
        front_left = self.ps[7].getValue()
        front_right = self.ps[0].getValue()
        front = max(front_left, front_right)
    
        WALL_THRESHOLD = 80.0  # 可调节此阈值
    
        if front > WALL_THRESHOLD:
            # 前方非常靠近墙壁
            # 后退的同时转向远离墙壁
            if front_left > front_right:
                # 左侧墙壁更近 → 倒车并右转
                left = -0.5 * self.max_speed
                right = -self.max_speed
            else:
                # 右侧墙壁更近 → 倒车并左转
                left = -self.max_speed
                right = -0.5 * self.max_speed
            # （此处不return，仅覆盖当前步骤的速度）
    
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
    

    # 栅格上的A*搜索
    def a_star(self, start, goal):
        (sy, sx) = start
        (gy, gx) = goal

        open_set = []
        heapq.heappush(open_set, (0, (sy, sx)))

        came_from = {}
        g_score = { (sy, sx): 0 }

        dirs = [(1,0), (-1,0), (0,1), (0,-1)]  # 4邻域移动

        while open_set:
            _, (y, x) = heapq.heappop(open_set)

            if (y, x) == (gy, gx):
                return self.reconstruct_path(came_from, start, goal)

            for dy, dx in dirs:
                ny = y + dy
                nx = x + dx

                if not (0 <= ny < self.map_h and 0 <= nx < self.map_w):
                    continue
                if self.world_map[ny][nx] == 1:  # 障碍物
                    continue

                tentative = g_score[(y, x)] + 1
                if (ny, nx) not in g_score or tentative < g_score[(ny, nx)]:
                    g_score[(ny, nx)] = tentative
                    f = tentative + abs(ny - gy) + abs(nx - gx)
                    heapq.heappush(open_set, (f, (ny, nx)))
                    came_from[(ny, nx)] = (y, x)

        return None

    # 通过回溯重建最终路径
    def reconstruct_path(self, came_from, start, goal):
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
        return path

    # 世界坐标 → 栅格索引
    def world_to_cell(self, pos):
        x, y = pos
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")
        print(f"[path_planner]我得到的世界坐标: ({x:.3f}, {y:.3f})")

        width_m = self.map_w * self.cell_size
        height_m = self.map_h * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        ix = int((x - origin_x) / self.cell_size)
        iy = int((y - origin_y) / self.cell_size)

        ix = max(0, min(self.map_w - 1, ix))
        iy = max(0, min(self.map_h - 1, iy))
        print(f"[path_planner]我得到的仿真世界坐标: ({x:.3f}, {y:.3f})")
        # print("=======================================")
        # print("=======================================")
        # print("=======================================")


        return (iy, ix)

    # 栅格索引 → 世界坐标（单元格中心）
    def cell_to_world(self, cell):
        iy, ix = cell

        width_m = self.map_w * self.cell_size
        height_m = self.map_h * self.cell_size

        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        wx = origin_x + (ix + 0.5) * self.cell_size
        wy = origin_y + (iy + 0.5) * self.cell_size

        return (wx, wy)

    # 将角度差归一化到[-pi, pi]范围
    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    # 停止机器人运动
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
