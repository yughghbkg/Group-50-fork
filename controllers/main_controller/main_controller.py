# main_controller.py


from controller import Supervisor

from localisation import Localiser
from path_planner import Planner
from lost_detector import LostDetector
from replanner import Replanner
import sys
import csv
import math


REQUIRED_DEVICES = [
    "left wheel motor",
    "right wheel motor",
    "left wheel sensor",
    "right wheel sensor",
    "gyro",
] + [f"ps{i}" for i in range(8)]

TIME_STEP = 64

# "baseline" = no lost detection / replanning
# "adaptive" = full system with lost detection + replanning
EXPERIMENT_MODE = "adaptive"

# Distance threshold (meters) for "reached goal"
GOAL_TOL = 0.05

# Desired goal in world coordinates (x, y) i.e. (x, z in Webots)
GOAL_WORLD = (0.25, 0.25)

# Hard cap on control steps so the sim always ends
MAX_STEPS = 10000



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


def compute_top_right_(localiser):
    """
    Choose the 'top-right' goal as the free cell whose WORLD coordinates
    (x, y) have the largest x + y.
    """
    world_map = localiser.world_map
    h = len(world_map)
    w = len(world_map[0])
    cell_size = localiser.cell_size

    width_m = w * cell_size
    height_m = h * cell_size
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0

    best_score = -1e9
    best_world = None
    best_cell = None

    for iy in range(h):
        for ix in range(w):
            if world_map[iy][ix] != 0:
                continue
            wx = origin_x + (ix + 0.5) * cell_size
            wy = origin_y + (iy + 0.5) * cell_size
            score = wx + wy
            if score > best_score:
                best_score = score
                best_world = (wx, wy)
                best_cell = (iy, ix)

    if best_world is None:
        print("[main_controller][FATAL] No free cell found for goal!")
        sys.exit(1)

    print(
        f"[main_controller] Goal cell chosen as (iy={best_cell[0]}, ix={best_cell[1]}), "
        f"world coords={best_world}"
    )
    return best_world


robot = Supervisor()

print("=== DEVICE LIST BEGIN ===")
for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    print(dev.getName(), "| type:", dev.getNodeType())
print("=== DEVICE LIST END ===")

print(f"[main_controller] Init… (EXPERIMENT_MODE={EXPERIMENT_MODE})")

sanity_check_devices(robot)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

localiser = Localiser(robot)
planner = Planner(robot, localiser)

if EXPERIMENT_MODE == "adaptive":
    lost_detector = LostDetector(robot)
    replanner = Replanner(robot, planner, localiser, lost_detector)
else:
    lost_detector = None
    replanner = None

planner.set_map(localiser.world_map)

# Use fixed goal at (0.45, 0.45)
goal = GOAL_WORLD

# Optional: print which grid cell this corresponds to
h = len(localiser.world_map)
w = len(localiser.world_map[0])
cell_size = localiser.cell_size
width_m = w * cell_size
height_m = h * cell_size
origin_x = -width_m / 2.0
origin_y = -height_m / 2.0

goal_x, goal_y = goal
goal_ix = int(round((goal_x - origin_x) / cell_size - 0.5))
goal_iy = int(round((goal_y - origin_y) / cell_size - 0.5))


# Region in odometry coordinates that we treat as "goal reached"
GOAL_X_MIN = -0.50
GOAL_X_MAX = -0.35

GOAL_Y_MIN = -0.50
GOAL_Y_MAX = -0.30

print(
    f"[main_controller] Final goal (world coords) = {goal}, "
    f"approx grid cell (iy={goal_iy}, ix={goal_ix})"
)

if replanner is not None:
    replanner.set_goal(goal)


if EXPERIMENT_MODE == "baseline":
    odom_x, odom_y, _ = localiser.estimate_pose()
    start = (odom_x, odom_y)
else:
    start = localiser.estimate_position()

planner.plan(start, goal)
print(f"[main_controller] First plan from {start} → {goal}")

if not planner.path:
    print("[main_controller] No feasible path found to the goal. "
          "Check your occupancy grid versus the Webots maze.")
    with open(
        "experiment_log_adaptive.csv" if EXPERIMENT_MODE == "adaptive"
        else "experiment_log_baseline.csv",
        "w",
        newline=""
    ) as f:
        writer = csv.writer(f)
        writer.writerow([
            "step", "odom_x", "odom_y",
            "map_x", "map_y",
            "dist_to_goal_map", "dist_to_goal_odom",
            "confidence", "is_lost",
        ])
    sys.exit(0)

print("[main_controller] Entering main loop…")
step_count = 0
reached_goal = False

MODE_FOLLOW = 0
MODE_BACKOFF = 1
mode = MODE_FOLLOW
backoff_steps = 0
turn_steps = 0

BACKOFF_SPEED = 3.0

prev_odom_x, prev_odom_y, _ = localiser.estimate_pose()
accum_path_len = 0.0

log_rows = []
csv_header = [
    "step",
    "odom_x", "odom_y",
    "map_x", "map_y",
    "dist_to_goal_map", "dist_to_goal_odom",
    "confidence", "is_lost",
]

while robot.step(TIME_STEP) != -1:
    step_count += 1
    if step_count > MAX_STEPS:
        print(f"[main_controller] Reached MAX_STEPS={MAX_STEPS}, stopping simulation.")
        break

    localiser.update()

    # Estimates
    odom_x, odom_y, _ = localiser.estimate_pose()
    map_x, map_y = localiser.estimate_position()

    # True Webots pose (Supervisor)
    self_node = robot.getSelf()
   
    tx, ty, tz = self_node.getField("translation").getSFVec3f()
    true_x = tx
    true_y = ty



    print(
        f"[DEBUG] WEBOTS (x,y)=({tx:.3f}, {ty:.3f}) | "
        f"ODOM (x,y)=({odom_x:.3f}, {odom_y:.3f}) | "
        f"MARKOV (x,y)=({map_x:.3f}, {map_y:.3f})"
    )

    # Distances to nominal goal (0.25, 0.25) for plotting/analysis
    dist_to_goal_map = math.hypot(map_x - goal[0], map_y - goal[1])
    dist_to_goal_odom = math.hypot(odom_x - goal[0], odom_y - goal[1])

    # Path length accumulation (from odometry)
    step_dist = math.hypot(odom_x - prev_odom_x, odom_y - prev_odom_y)
    accum_path_len += step_dist
    prev_odom_x, prev_odom_y = odom_x, odom_y

    conf = ""
    is_lost = ""

    if EXPERIMENT_MODE == "adaptive":
        lost_detector.check(localiser)
        conf = lost_detector.confidence
        is_lost = int(lost_detector.is_lost)

        if lost_detector.is_lost and mode == MODE_FOLLOW:
            print("[main_controller] Following planned path, but I am lost")
            mode = MODE_BACKOFF
            backoff_steps = 20
            turn_steps = 20

        if mode == MODE_BACKOFF:
            est_before = localiser.estimate_position()
            print(
                f"[main_controller] I am lost, I will back off. BEFORE: "
                f"pose={est_before}, backoff_steps={backoff_steps}, turn_steps={turn_steps}"
            )

            if backoff_steps > 0:
                left_motor.setVelocity(-BACKOFF_SPEED)
                right_motor.setVelocity(-BACKOFF_SPEED)
                backoff_steps -= 1

            elif turn_steps > 0:
                left_motor.setVelocity(BACKOFF_SPEED)
                right_motor.setVelocity(-BACKOFF_SPEED)
                turn_steps -= 1

            else:
                print("[main_controller] Backoff complete, I will replan my route now")
                replanner.replan()
                mode = MODE_FOLLOW

        else:
            planner.follow_path()

    else:
        planner.follow_path()

    log_rows.append([
        step_count,
        odom_x, odom_y,
        map_x, map_y,
        dist_to_goal_map, dist_to_goal_odom,
        conf, is_lost,
    ])

    if step_count % 50 == 0:
        print(
            f"[main_controller] Step {step_count}, "
            f"odom=({odom_x:.3f}, {odom_y:.3f}), "
            f"map=({map_x:.3f}, {map_y:.3f}), "
            f"dist_to_goal_map={dist_to_goal_map:.3f}"
        )

    # --- Manual goal region in odom coordinates (for your experiment) ---
    if (GOAL_X_MIN <= true_x <= GOAL_X_MAX) and (GOAL_Y_MIN <= true_y <= GOAL_Y_MAX):
        print(
            f"[main_controller] Goal region reached at step {step_count}! "
        f"True pose (Webots) = ({true_x:.3f}, {true_y:.3f}), "
        f"Markov_est=({map_x:.3f}, {map_y:.3f}), "
        f"odom=({odom_x:.3f}, {odom_y:.3f}), "
        f"approx path length={accum_path_len:.3f} m"
        )
        reached_goal = True
        break



    
print(f"DEBUG odom at end: ({odom_x:.3f}, {odom_y:.3f})")
   
print("[main_controller] Simulation ended, exporting logs...")

if EXPERIMENT_MODE == "adaptive":
    with open("confidence_history.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "confidence"])
        for step_idx, conf_val in lost_detector.conf_history:
            writer.writerow([step_idx, conf_val])
    print("[main_controller] Saved confidence_history.csv")

log_filename = (
    "experiment_log_adaptive.csv"
    if EXPERIMENT_MODE == "adaptive"
    else "experiment_log_baseline.csv"
)

with open(log_filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(csv_header)
    writer.writerows(log_rows)

print(f"[main_controller] Saved {log_filename}")
print(f"[main_controller] reached_goal={reached_goal}, "
      f"final_path_length={accum_path_len:.3f} m")
print("[main_controller] Done.")
