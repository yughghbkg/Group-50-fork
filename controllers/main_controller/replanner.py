# replanner.py

class Replanner:
    def __init__(self, robot, planner, localiser, lost_detector=None):
        self.robot = robot
        self.planner = planner
        self.localiser = localiser
        self.lost_detector = lost_detector

        # (x, y) in world coordinates
        self.goal = None


    def set_goal(self, goal_world):
        self.goal = goal_world
        print(f"[replanner] goal set to {self.goal}")

    def replan(self, start_world=None):
        if self.goal is None:
            print("[replanner] ERROR: goal not set, cannot replan!")
            return

       
        if start_world is None:
            start_world = self.localiser.estimate_position()

        try:
            entropy = self.localiser.measure_uncertainty(method="entropy")
            print(f"[replanner] Replanning from {start_world}, entropy={entropy:.3f}")
        except Exception:
            print(f"[replanner] Replanning from {start_world}")

        self.planner.plan(start_world, self.goal)

        self.planner.current_wp_index = 0

        if self.lost_detector is not None:
            try:
                self.lost_detector.reset()
                print("[replanner] Lost detector reset")
            except AttributeError:
                # If reset() doesn't exist yet, nothing fatal
                print("[replanner] Warning: lost_detector has no reset() method")

        print("[replanner] Replanning complete")
