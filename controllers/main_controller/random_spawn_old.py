import argparse
import random
from pathlib import Path

from localisation import Localiser


def build_world_map(cell_size):
    """
    Reuse the static occupancy grid defined in localisation.py.
    The helper is kept here so the script stays decoupled from Webots.
    """
    world_map = Localiser._build_default_map()
    h = len(world_map)
    w = len(world_map[0])
    return world_map, w, h, cell_size


def pick_random_free_cell(world_map, rng):
    free = [
        (iy, ix)
        for iy, row in enumerate(world_map)
        for ix, cell in enumerate(row)
        if cell == 0
    ]
    if not free:
        raise RuntimeError("No free cells found in world_map; cannot place robot.")
    return rng.choice(free)


def cell_to_world(cell, map_w, map_h, cell_size):
    iy, ix = cell
    origin_x = -(map_w * cell_size) / 2.0
    origin_y = -(map_h * cell_size) / 2.0
    wx = origin_x + (ix + 0.5) * cell_size
    wy = origin_y + (iy + 0.5) * cell_size
    return wx, wy


def update_robot_pose_in_wbt(wbt_path, new_xy, random_yaw, rng):
    lines = wbt_path.read_text(encoding="utf-8").splitlines()

    robot_start = None
    translation_idx = None
    rotation_idx = None
    for i, line in enumerate(lines):
        if line.strip().startswith("E-puck"):
            robot_start = i
            continue
        if robot_start is not None and translation_idx is None and line.strip().startswith("translation "):
            translation_idx = i
            continue
        if robot_start is not None and rotation_idx is None and line.strip().startswith("rotation "):
            rotation_idx = i
        if robot_start is not None and translation_idx is not None and rotation_idx is not None:
            break

    if translation_idx is None:
        raise RuntimeError("Could not find E-puck translation field in WBT file.")

    parts = lines[translation_idx].strip().split()
    if len(parts) < 4:
        raise RuntimeError("Unexpected translation line format: " + lines[translation_idx])
    _, old_x, old_y, old_z = parts[:4]
    new_line = f"  translation {new_xy[0]:.6f} {new_xy[1]:.6f} {old_z}"
    lines[translation_idx] = new_line

    if random_yaw:
        if rotation_idx is None:
            raise RuntimeError("Requested random yaw but no rotation field found in WBT file.")
        yaw = rng.uniform(-3.141592653589793, 3.141592653589793)
        lines[rotation_idx] = f"  rotation 0 0 1 {yaw:.6f}"
    else:
        yaw = None

    wbt_path.write_text("\n".join(lines), encoding="utf-8")
    return (float(old_x), float(old_y)), yaw


def main():
    parser = argparse.ArgumentParser(
        description="Place the robot at a random free cell from localisation.world_map."
    )
    parser.add_argument(
        "--wbt",
        # default=Path(__file__).resolve().parents[2] / "worlds" / "maze_world_new.wbt",
        default=Path(__file__).resolve().parents[2] / "worlds" / "maze_world.wbt",
        type=Path,
        help="Path to the .wbt world file to modify.",
    )
    parser.add_argument(
        "--cell-size",
        default=0.05,
        type=float,
        help="Cell size used by the static world_map (meters).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional RNG seed for reproducible placement.",
    )
    parser.add_argument(
        "--random-yaw",
        action="store_true",
        help="Also randomise robot heading around the Z axis.",
    )
    args = parser.parse_args()

    rng = random.Random(args.seed)

    world_map, map_w, map_h, cell_size = build_world_map(args.cell_size)
    cell = pick_random_free_cell(world_map, rng)
    world_xy = cell_to_world(cell, map_w, map_h, cell_size)
    old_xy, yaw = update_robot_pose_in_wbt(args.wbt, world_xy, args.random_yaw, rng)

    print(f"[random_spawn] Selected free cell (iy={cell[0]}, ix={cell[1]})")
    print(f"[random_spawn] World coords: x={world_xy[0]:.3f}, y={world_xy[1]:.3f}")
    print(f"[random_spawn] Updated WBT: {args.wbt}")
    print(f"[random_spawn] Previous pose x={old_xy[0]:.3f}, y={old_xy[1]:.3f}")
    if yaw is not None:
        print(f"[random_spawn] New yaw={yaw:.3f} rad")


if __name__ == "__main__":
    main()
