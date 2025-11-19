# replanner.py

class Replanner:
    """
    Handles replanning when the robot is detected as 'lost'.

    Responsibilities:
      - Store the goal in world coordinates.
      - When triggered, call the A* planner again from the current estimated pose.
      - Reset the planner's waypoint index so it starts following the new path.
      - Optionally reset the lost detector.
    """

    def __init__(self, robot, planner, localiser, lost_detector=None):
        self.robot = robot
        self.planner = planner
        self.localiser = localiser
        self.lost_detector = lost_detector

        # (x, y) in world coordinates
        self.goal = None

    # ---------------------------------------------------------
    # Public methods
    # ---------------------------------------------------------
    def set_goal(self, goal_world):
        """
        Set the target goal in world coordinates.

        Args:
            goal_world: tuple (x, y)
        """
        self.goal = goal_world
        print(f"[replanner] goal set to {self.goal}")

    def replan(self, start_world=None):
        """
        Trigger a replan from the given start pose.

        Args:
            start_world: (x, y) in world coordinates. If None, use the
                         localiser's current position estimate.
        """
        if self.goal is None:
            print("[replanner] ERROR: goal not set, cannot replan!")
            return

        # If the caller doesn't give a start pose, ask the localiser
        if start_world is None:
            start_world = self.localiser.estimate_position()

        # Try to log localisation uncertainty if available
        try:
            entropy = self.localiser.measure_uncertainty(method="entropy")
            print(f"[replanner] Replanning from {start_world}, entropy={entropy:.3f}")
        except Exception:
            # If anything goes wrong, still replan
            print(f"[replanner] Replanning from {start_world}")

        # Call the planner to compute a new path
        self.planner.plan(start_world, self.goal)

        # Start following the new path from the first waypoint
        self.planner.current_wp_index = 0

        # After replanning, we can reset the lost detector if provided
        if self.lost_detector is not None:
            try:
                self.lost_detector.reset()
                print("[replanner] Lost detector reset")
            except AttributeError:
                # If reset() doesn't exist yet, nothing fatal
                print("[replanner] Warning: lost_detector has no reset() method")

        print("[replanner] Replanning complete")
