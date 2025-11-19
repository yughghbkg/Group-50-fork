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

# NEW: set up motors here so we can directly control them in BACKOFF mode
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

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

# NEW: simple state machine for backing off when lost
MODE_FOLLOW = 0
MODE_BACKOFF = 1
mode = MODE_FOLLOW
backoff_steps = 0
turn_steps = 0

# reasonable speed for e-puck
BACKOFF_SPEED = 3.0

while robot.step(TIME_STEP) != -1:

    # 1) Localisation update
    localiser.update()

    # 2) Lost detection
    lost_detector.check(localiser)

    # 3) If we just became lost while in FOLLOW mode → enter BACKOFF mode
    if lost_detector.is_lost and mode == MODE_FOLLOW:
        print("[main_controller] lost detected → entering BACKOFF mode")
        mode = MODE_BACKOFF
        backoff_steps = 20   # reverse for 20 ticks
        turn_steps = 20      # then turn for 20 ticks

    # 4) Handle BACKOFF behaviour (reverse + turn), then replan once
    if mode == MODE_BACKOFF:
        est_before = localiser.estimate_position()
        print(f"[BACKOFF] BEFORE: pose={est_before}, backoff_steps={backoff_steps}, turn_steps={turn_steps}")

        if backoff_steps > 0:
            # reverse away from the wall
            left_motor.setVelocity(-BACKOFF_SPEED)
            right_motor.setVelocity(-BACKOFF_SPEED)
            backoff_steps -= 1

        elif turn_steps > 0:
            # rotate in place to face away from the wall
            left_motor.setVelocity(BACKOFF_SPEED)
            right_motor.setVelocity(-BACKOFF_SPEED)
            turn_steps -= 1

        else:
            # finished backing off: now replan once and return to FOLLOW mode
            print("[main_controller] backoff complete → calling replanner")
            replanner.replan()
            mode = MODE_FOLLOW

        # IMPORTANT: skip normal path following while backing off
        step_count += 1
        if step_count % 50 == 0:
            est = localiser.estimate_position()
            print(f"[main_controller] tick {step_count}, est={est}")
        continue

    # 5) Normal path following (only when not backing off)
    planner.follow_path()

    # Debug heartbeat
    step_count += 1
    if step_count % 50 == 0:
        est = localiser.estimate_position()
        print(f"[main_controller] tick {step_count}, est={est}")
