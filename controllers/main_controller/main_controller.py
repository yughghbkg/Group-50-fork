from controller import Robot
from localisation import Localiser
from path_planner import Planner
from lost_detector import LostDetector
from replanner import Replanner

TIME_STEP = 64
robot = Robot()

print("[main_controller] init…")

localiser = Localiser(robot,  time_step=TIME_STEP)
planner = Planner(robot)
lost_detector = LostDetector(robot)
replanner = Replanner(robot, planner)

print("[main_controller] entering loop")
i = 0
was_lost = False 

while robot.step(TIME_STEP) != -1:
    localiser.update()
    lost_detector.check(localiser)
    if not was_lost and lost_detector.is_lost:
        print("[lost] TRUE — triggering replanning once")
        est_x, est_y = localiser.estimate_position()
        replanner.replan((est_x, est_y))

    was_lost = lost_detector.is_lost
    planner.follow_path()
    i += 1
    if i % 30 == 0:
        print(f"[main_controller] tick {i}")
