from controller import Robot
from localisation import Localiser
from path_planner import Planner
from lost_detector import LostDetector
from replanner import Replanner

TIME_STEP = 64
robot = Robot()

print("[main_controller] init…")

localiser = Localiser(robot)
planner = Planner(robot)
lost_detector = LostDetector(robot)
replanner = Replanner(robot, planner)

print("[main_controller] entering loop")
i = 0
while robot.step(TIME_STEP) != -1:
    localiser.update()
    lost_detector.check(localiser)
    if lost_detector.is_lost:
        print("[lost] TRUE — would trigger replanning here")
        replanner.replan(localiser.estimate_position())
    planner.follow_path()
    i += 1
    if i % 30 == 0:
        print(f"[main_controller] tick {i}")
