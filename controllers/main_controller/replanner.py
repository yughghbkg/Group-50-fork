class Replanner:
    def __init__(self, robot, planner):
        self.robot = robot
        self.planner = planner
        self.goal = None  # target position not set yet

    def set_goal(self, goal_world):
        self.goal = goal_world  # store the target in world coordinates
        print(f"[replanner] goal set to {self.goal}")

    def replan(self, current_pos):
        # can't replan without a goal
        if self.goal is None:
            print("[replanner] ERROR: goal not set, cannot replan!")
            return

        # robot got lost — restart planning from the new estimated pose
        print(f"[replanner] LOST detected → replanning from {current_pos}")

        # trigger planner with updated start position
        self.planner.plan(current_pos, self.goal)

        # reset waypoint pointer so the robot starts following from the first one again
        self.planner.current_wp_index = 0

        print("[replanner] replanning complete!")
