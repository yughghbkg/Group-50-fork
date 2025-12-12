# main_controller.py

from controller import Robot
from localisation import Localiser
from path_planner import Planner
from lost_detector import LostDetector
from replanner import Replanner
import sys
import csv
import math

# ---------- Display helpers (A* path and current pose) ----------
def world_to_cell_for_display(pos, map_w, map_h, cell_size):
    x, y = pos
    origin_x = -(map_w * cell_size) / 2.0
    origin_y = -(map_h * cell_size) / 2.0
    ix = int((x - origin_x) / cell_size)
    iy = int((y - origin_y) / cell_size)
    ix = max(0, min(map_w - 1, ix))
    iy = max(0, min(map_h - 1, iy))
    return iy, ix


def cell_to_pixel(ix, iy, disp_w, disp_h, map_w, map_h):
    cell_w = disp_w / map_w
    cell_h = disp_h / map_h
    # flip vertically
    px = int(ix * cell_w + cell_w / 2)
    py = int((map_h - 1 - iy) * cell_h + cell_h / 2)
    return px, py


def render_display(display, world_map, path_world, cur_world, cell_size):
    if display is None:
        return
    w = display.getWidth()
    h = display.getHeight()
    map_h = len(world_map)
    map_w = len(world_map[0])
    cell_w = w / map_w
    cell_h = h / map_h

    # clear background
    display.setColor(0x222222)
    display.fillRectangle(0, 0, w, h)

    # draw obstacles (flipped vertically)
    display.setColor(0xAA3333)
    for iy in range(map_h):
        for ix in range(map_w):
            if world_map[iy][ix] == 1:
                x = int(ix * cell_w)
                y = int((map_h - 1 - iy) * cell_h)
                display.fillRectangle(x, y, int(cell_w), int(cell_h))

    # draw path (green)
    if path_world:
        display.setColor(0x00BB66)
        pts = []
        for wx, wy in path_world:
            cy, cx = world_to_cell_for_display((wx, wy), map_w, map_h, cell_size)
            px, py = cell_to_pixel(cx, cy, w, h, map_w, map_h)
            pts.append((px, py))
        for i in range(1, len(pts)):
            display.drawLine(pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1])
        for (px, py) in pts:
            display.drawOval(px - 2, py - 2, 4, 4)

    # draw current position (blue cross)
    if cur_world:
        cy, cx = world_to_cell_for_display(cur_world, map_w, map_h, cell_size)
        px, py = cell_to_pixel(cx, cy, w, h, map_w, map_h)
        display.setColor(0x2288FF)
        display.drawLine(px - 4, py, px + 4, py)
        display.drawLine(px, py - 4, px, py + 4)


def render_confidence(display, conf_history):
    """
    Render confidence history on a dedicated Display.
    Clamps values to [0,1] and uses full display area.
    """
    if display is None:
        return

    w = display.getWidth()
    h = display.getHeight()

    margin_left = 28
    margin_right = 8
    margin_top = 18
    margin_bottom = 14

    plot_w = max(10, w - margin_left - margin_right)
    plot_h = max(10, h - margin_top - margin_bottom)
    x0 = margin_left
    y0 = margin_top
    x1 = x0 + plot_w - 1
    y1 = y0 + plot_h - 1

    # background
    display.setColor(0x101820)
    display.fillRectangle(0, 0, w, h)
    display.setColor(0x2A303A)
    display.fillRectangle(x0, y0, plot_w, plot_h)

    # grid + axis ticks
    display.setColor(0x555555)
    ticks = [0.0, 0.25, 0.5, 0.75, 1.0]
    for t in ticks:
        y = y1 - int(t * (plot_h - 1))
        display.drawLine(x0, y, x1, y)
        display.drawLine(x0 - 4, y, x0, y)
        label = f"{t:.2g}"
        display.drawText(label, 2, y - 5)

    # border
    display.setColor(0x777777)
    display.drawRectangle(x0, y0, plot_w - 1, plot_h - 1)

    # data
    if conf_history:
        display.setColor(0x4DD2FF)
        last = conf_history[-plot_w:]
        n = len(last)
        if n == 1:
            c = max(0.0, min(1.0, last[0][1]))
            y = y1 - int(c * (plot_h - 1))
            display.drawLine(x0, y, x1, y)
        else:
            for i in range(1, n):
                _, c0 = last[i - 1]
                _, c1 = last[i]
                x_prev = x0 + int((i - 1) / (n - 1) * (plot_w - 1))
                x_curr = x0 + int(i / (n - 1) * (plot_w - 1))
                y_prev = y1 - int(max(0.0, min(1.0, c0)) * (plot_h - 1))
                y_curr = y1 - int(max(0.0, min(1.0, c1)) * (plot_h - 1))
                display.drawLine(x_prev, y_prev, x_curr, y_curr)

            # highlight latest point
            _, clast = last[-1]
            x_last = x1
            y_last = y1 - int(max(0.0, min(1.0, clast)) * (plot_h - 1))
            display.setColor(0xFFCC66)
            display.drawLine(x_last - 2, y_last, x_last + 2, y_last)
            display.drawLine(x_last, y_last - 2, x_last, y_last + 2)

    # labels
    display.setColor(0xCCCCCC)
    display.drawText("Confidence 0–1", x0, max(0, margin_top - 14))


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

# Hard cap on control steps so the sim always ends
MAX_STEPS = 50000
REPLAN_INTERVAL = 1000000  # effectively disable auto-replan (no lidar map updates)


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


def compute_top_right_goal(localiser):
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

# Default goal for the new map
def set_goal_world(x=0.52, y=0.27, z=0.0):
    """
    Directly set a goal in world coordinates (default from user data).
    Returns a tuple (x, y).
    """
    return (x, y)


robot = Robot()

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

display = robot.getDevice("display")
if display:
    print("[main_controller] display device found, will render path/pose")
else:
    print("[main_controller][WARN] display device not found, path viz disabled")

confidence_display = robot.getDevice("confidence_display")
if confidence_display:
    print("[main_controller] confidence_display device found, will render confidence curve")
else:
    print("[main_controller][WARN] confidence_display not found, confidence viz disabled")

localiser = Localiser(robot)
planner = Planner(robot, localiser)

if EXPERIMENT_MODE == "adaptive":
    lost_detector = LostDetector(robot)
    replanner = Replanner(robot, planner, localiser, lost_detector)
else:
    lost_detector = None
    replanner = None

planner.set_map(localiser.world_map)

'''
Interface adjustment here
Old map: compute_top_right_goal(localiser)
New map: set_goal_world(0.52, 0.27)
'''

goal = set_goal_world(0.52, 0.27)

print(f"[main_controller] Final goal (world coords) = {goal}")
if replanner is not None:
    replanner.set_goal(goal)

# Wait for a valid GPS pose before first plan (up to 200 cycles)
if not localiser.has_valid_pose():
    print("[main_controller][WARN] Waiting for valid GPS/IMU fix before first plan...")
    warmup_steps = 200
    while warmup_steps > 0 and not localiser.has_valid_pose():
        if robot.step(TIME_STEP) == -1:
            sys.exit(0)
        localiser.update()
        warmup_steps -= 1
    if not localiser.has_valid_pose():
        print("[main_controller][WARN] GPS still invalid after warmup, will fall back to (0,0)")

start = localiser.estimate_position() if localiser.has_valid_pose() else (0.0, 0.0)
print(f"[main_controller] Planning start pose (world): {start} (valid={localiser.has_valid_pose()})")

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

    odom_x, odom_y, _ = localiser.estimate_pose()
    map_x, map_y = localiser.estimate_position()

    dist_to_goal_map = math.hypot(map_x - goal[0], map_y - goal[1])
    dist_to_goal_odom = math.hypot(odom_x - goal[0], odom_y - goal[1])

    step_dist = math.hypot(odom_x - prev_odom_x, odom_y - prev_odom_y)
    accum_path_len += step_dist
    prev_odom_x, prev_odom_y = odom_x, odom_y

    conf = ""
    is_lost = ""

    if not localiser.has_valid_pose():
        if step_count % 20 == 0:
            print("[main_controller][WARN] GPS pose still invalid, skipping control/planning this cycle")
        continue

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

    # Auto-replan disabled (no dynamic map updates)

    # Display rendering of map, path and current position
    render_display(
        display,
        localiser.world_map,
        planner.path,
        (map_x, map_y),
        localiser.cell_size,
    )
    if lost_detector is not None:
        render_confidence(confidence_display, lost_detector.conf_history)

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

    if dist_to_goal_map < GOAL_TOL:
        print(
            f"[main_controller] Goal reached at step {step_count}! "
            f"Markov_est=({map_x:.3f}, {map_y:.3f}), "
            f"odom=({odom_x:.3f}, {odom_y:.3f}), "
            f"approx path length={accum_path_len:.3f} m"
        )
        reached_goal = True
        break

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
