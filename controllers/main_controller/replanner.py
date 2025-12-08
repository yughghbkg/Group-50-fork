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
        print(f"[replanner] Goal point set to {self.goal}")

    def replan(self, start_world=None):
       
        if self.goal is None:
            print("[replanner] Error: No goal set, cannot replan!")
            return

        # If no start pose is given, query the localiser
        if start_world is None:
            start_world = self.localiser.estimate_position()

        # Attempt to record uncertainty (if supported)
        try:
            entropy = self.localiser.measure_uncertainty(method="entropy")
            print(f"[replanner] Replanning from {start_world}, entropy={entropy:.3f}")
        except Exception:
            # Even if uncertainty retrieval fails, continue replanning
            print(f"[replanner] Replanning from {start_world}")

        # Call planner to compute a new path
        self.planner.plan(start_world, self.goal)

        # Start following the new path from waypoint 0
        self.planner.current_wp_index = 0

        # After replanning, reset lost detector if available
        if self.lost_detector is not None:
            try:
                self.lost_detector.reset()
                print("[replanner] Lost detector has been reset")
            except AttributeError:
                # Non-fatal if reset() doesn't exist
                print("[replanner] Warning: lost detector has no reset() method")

        print("[replanner] Replanning completed")
       