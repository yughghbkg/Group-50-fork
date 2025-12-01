# replanner.py

class Replanner:
    """
    当机器人被检测为“丢失”状态时处理重规划逻辑。

    职责：
      - 存储世界坐标系下的目标点。
      - 触发时，从当前估计位姿重新调用A*规划器。
      - 重置规划器的路径点索引，使其从新路径开始跟随。
      - 可选地重置丢失检测器。
    """

    def __init__(self, robot, planner, localiser, lost_detector=None):
        self.robot = robot
        self.planner = planner
        self.localiser = localiser
        self.lost_detector = lost_detector

        # 世界坐标系下的(x, y)
        self.goal = None

    # ---------------------------------------------------------
    # 公共方法
    # ---------------------------------------------------------
    def set_goal(self, goal_world):
        """
        设置世界坐标系下的目标点。
        参数:
            goal_world: 元组 (x, y)
        """
        self.goal = goal_world
        # print("##############################")
        print(f"[重规划器] 目标点已设置为 {self.goal}")
        # print("##############################")

    def replan(self, start_world=None):
        """
        从给定的起始位姿触发重规划。

        参数:
            start_world: 世界坐标系下的(x, y)。若为None，则使用定位器的当前位置估计。
        """
        # print("##############################")
        # print("##############################")
        # print("##############################")
        if self.goal is None:
            print("[重规划器] 错误：未设置目标点，无法执行重规划！")
            return

        # 若调用者未指定起始位姿，则向定位器查询
        if start_world is None:
            start_world = self.localiser.estimate_position()

        # 尝试记录定位不确定性（若支持）
        try:
            entropy = self.localiser.measure_uncertainty(method="entropy")
            print(f"[重规划器] 从 {start_world} 开始重规划，熵值={entropy:.3f}")
        except Exception:
            # 即使获取不确定性失败，仍继续执行重规划
            print(f"[重规划器] 从 {start_world} 开始重规划")

        # 调用规划器计算新路径
        self.planner.plan(start_world, self.goal)

        # 从第一个路径点开始跟随新路径
        self.planner.current_wp_index = 0

        # 重规划完成后，若提供了丢失检测器则重置它
        if self.lost_detector is not None:
            try:
                self.lost_detector.reset()
                print("[重规划器] 丢失检测器已重置")
            except AttributeError:
                # 若reset()方法不存在，非致命错误
                print("[重规划器] 警告：丢失检测器无reset()方法")

        print("[重规划器] 重规划执行完成")
        # print("##############################")
        # print("##############################")
        # print("##############################")
