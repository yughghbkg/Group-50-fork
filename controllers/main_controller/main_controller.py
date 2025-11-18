# main_controller.py

from controller import Robot
from localisation import Localiser
from path_planner import Planner
from lost_detector import LostDetector
from replanner import Replanner
import sys

REQUIRED_DEVICES = [
    "left wheel motor",
    "right wheel motor",
    "left wheel sensor",
    "right wheel sensor",
    "gyro",
] + [f"ps{i}" for i in range(8)]

TIME_STEP = 64


def sanity_check_devices(robot):
    print("[sanity] checking required devices...")
    missing = []
    for name in REQUIRED_DEVICES:
        dev = robot.getDevice(name)
        if dev is None:
            print(f"[sanity][ERROR] missing device: {name}")
            missing.append(name)
        else:
            print(f"[sanity][OK] {name}")
    if missing:
        print("[sanity] FATAL: some devices are missing, cannot continue!")
        sys.exit(1)
    print("[sanity] all required devices found.")


# ===========================================
# Initialisation
# ===========================================
robot = Robot()

print("=== DEVICE LIST BEGIN ===")
for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    print(dev.getName(), "| type:", dev.getNodeType())
print("=== DEVICE LIST END ===")

print("[main_controller] Init…")

sanity_check_devices(robot)

# 1) Localisation (Markov + odom)
localiser = Localiser(robot)

# 2) Planner (we pass localiser so it can use heading)
planner = Planner(robot, localiser)

# 3) Lost detector
lost_detector = LostDetector(robot)

# 4) Replanner
replanner = Replanner(robot, planner, localiser, lost_detector)
# ============================================================
# Set the world map for A*
# ============================================================
planner.set_map(localiser.world_map)

# ============================================================
# Goal (world coordinates)
# ============================================================
goal = (0.2, 0.2)   # TODO: adjust for your maze
replanner.set_goal(goal)

# ============================================================
# Initial planning (first path)
# ============================================================
start = localiser.estimate_position()
planner.plan(start, goal)

print(f"[main_controller] First plan from {start} → {goal}")

# ===========================================
# Main Loop
# ===========================================
print("[main_controller] Entering main loop…")
step_count = 0

while robot.step(TIME_STEP) != -1:

    # 1) Localisation update
    localiser.update()

    # 2) Lost detection
    lost_detector.check(localiser)

    # 3) Replanning when lost
        if lost_detector.is_lost:
        print("[lost] TRUE → calling replanner")
        # start pose will be taken from localiser inside replanner
        replanner.replan()

    # 4) Path following (movement control)
    planner.follow_path()

    # Debug heartbeat
    step_count += 1
    if step_count % 50 == 0:
        est = localiser.estimate_position()
        print(f"[main_controller] tick {step_count}, est={est}")
