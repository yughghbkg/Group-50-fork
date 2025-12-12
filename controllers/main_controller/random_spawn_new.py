import argparse
import random
import re
from collections import deque
from pathlib import Path

from localisation import Localiser


def build_world_map(cell_size):
    """
    Reuse the static occupancy grid defined in localisation.py.
    """
    world_map = Localiser._build_default_map()
    h = len(world_map)
    w = len(world_map[0])
    return world_map, w, h, cell_size


def parse_arena_from_wbt(wbt_path: Path):
    """
    Parse ARENA RectangleArena from WBT.
    Returns:
      offset_xy: (ax, ay) or (0,0) if not found
      bounds: (xmin,xmax,ymin,ymax) or None if not found
    """
    text = wbt_path.read_text(encoding="utf-8", errors="ignore")
    m = re.search(
        r"DEF\s+ARENA\s+RectangleArena\s*{.*?"
        r"translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+).*?"
        r"floorSize\s+([-\d.eE]+)\s+([-\d.eE]+)",
        text,
        re.S
    )
    if not m:
        return (0.0, 0.0), None

    ax, ay, az, fw, fh = map(float, m.groups())
    xmin = ax - fw / 2.0
    xmax = ax + fw / 2.0
    ymin = ay - fh / 2.0
    ymax = ay + fh / 2.0
    return (ax, ay), (xmin, xmax, ymin, ymax)


def enclosed_free_mask(world_map):
    """
    Remove outside-free area:
    - Flood-fill all free cells (0) connected to the map boundary.
    - Those are "outside".
    - Enclosed free cells are the remaining 0s not connected to boundary.
    Returns a boolean mask same size: True if cell is enclosed free.
    """
    h = len(world_map)
    w = len(world_map[0])

    outside = [[False] * w for _ in range(h)]
    q = deque()

    def push_if_free(iy, ix):
        if 0 <= iy < h and 0 <= ix < w and world_map[iy][ix] == 0 and not outside[iy][ix]:
            outside[iy][ix] = True
            q.append((iy, ix))

    # seed flood fill from boundary free cells
    for ix in range(w):
        push_if_free(0, ix)
        push_if_free(h - 1, ix)
    for iy in range(h):
        push_if_free(iy, 0)
        push_if_free(iy, w - 1)

    # 4-neighbor flood
    while q:
        iy, ix = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            push_if_free(iy + dy, ix + dx)

    enclosed = [[False] * w for _ in range(h)]
    for iy in range(h):
        for ix in range(w):
            enclosed[iy][ix] = (world_map[iy][ix] == 0 and not outside[iy][ix])
    return enclosed


def is_cell_clear(world_map, iy, ix, clearance_cells=1):
    """
    "Reasonable point": the cell itself must be free (0), and all cells within
    the clearance_cells range must also be free (no obstacles).
    Uses a Chebyshev neighborhood (square window).
    """
    h = len(world_map)
    w = len(world_map[0])
    if world_map[iy][ix] != 0:
        return False

    r = int(clearance_cells)
    for yy in range(max(0, iy - r), min(h, iy + r + 1)):
        for xx in range(max(0, ix - r), min(w, ix + r + 1)):
            if world_map[yy][xx] != 0:
                return False
    return True


def pick_random_free_cell(world_map, rng, require_enclosed=True, clearance_cells=1):
    """
    Pick a random free cell, optionally requiring that it is inside an enclosed region
    (not connected to map boundary), and has obstacle clearance.
    """
    h = len(world_map)
    w = len(world_map[0])

    enclosed = enclosed_free_mask(world_map) if require_enclosed else None

    candidates = []
    for iy in range(h):
        for ix in range(w):
            if world_map[iy][ix] != 0:
                continue
            if require_enclosed and not enclosed[iy][ix]:
                continue
            if not is_cell_clear(world_map, iy, ix, clearance_cells=clearance_cells):
                continue
            candidates.append((iy, ix))

    if not candidates:
        raise RuntimeError(
            "No valid free cells found. "
            "Try lowering clearance_cells or disabling require_enclosed."
        )

    return rng.choice(candidates)


def cell_to_world(cell, map_w, map_h, cell_size, offset_xy=(0.0, 0.0), flip_y=False):
    """
    Convert grid cell (iy, ix) to world coords.
    Base convention: origin at (-W/2, -H/2) in map-local frame.
    Then apply offset_xy (e.g., ARENA translation) to get WBT world coords.
    flip_y=True is available if your map y indexing is inverted relative to world.
    """
    iy, ix = cell
    if flip_y:
        iy = (map_h - 1 - iy)

    origin_x = -(map_w * cell_size) / 2.0
    origin_y = -(map_h * cell_size) / 2.0

    wx = origin_x + (ix + 0.5) * cell_size
    wy = origin_y + (iy + 0.5) * cell_size

    return wx + offset_xy[0], wy + offset_xy[1]


def inside_bounds(xy, bounds):
    if bounds is None:
        return True
    x, y = xy
    xmin, xmax, ymin, ymax = bounds
    return (xmin <= x <= xmax) and (ymin <= y <= ymax)


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
    lines[translation_idx] = f"  translation {new_xy[0]:.6f} {new_xy[1]:.6f} {old_z}"

    if random_yaw:
        if rotation_idx is None:
            raise RuntimeError("Requested random yaw but no rotation field found in WBT file.")
        yaw = rng.uniform(-math.pi, math.pi)
        lines[rotation_idx] = f"  rotation 0 0 1 {yaw:.6f}"
    else:
        yaw = None

    wbt_path.write_text("\n".join(lines), encoding="utf-8")
    return (float(old_x), float(old_y)), yaw


def main():
    parser = argparse.ArgumentParser(
        description="Place the robot at a random valid free cell from localisation.world_map."
    )
    parser.add_argument(
        "--wbt",
        default=Path(__file__).resolve().parents[2] / "worlds" / "maze_world_new.wbt",
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

    # Parse arena: used for offset (convert map-local coords to world coords) + bounds checking
    arena_offset, arena_bounds = parse_arena_from_wbt(args.wbt)

    cell = pick_random_free_cell(
        world_map,
        rng,
        require_enclosed=True,
        clearance_cells=1
    )

    # Normal conversion (do not flip y)
    world_xy = cell_to_world(cell, map_w, map_h, cell_size, offset_xy=arena_offset, flip_y=False)

    # If outside arena bounds, try automatically flipping y (common when index directions differ)
    if not inside_bounds(world_xy, arena_bounds):
        alt_xy = cell_to_world(cell, map_w, map_h, cell_size, offset_xy=arena_offset, flip_y=True)
        if inside_bounds(alt_xy, arena_bounds):
            world_xy = alt_xy
        else:
            # Still outside: indicates a serious mismatch between the map and WBT size/offset
            print("[random_spawn] Warning: sampled point is outside ARENA bounds even after flip_y.")

    old_xy, yaw = update_robot_pose_in_wbt(args.wbt, world_xy, args.random_yaw, rng)

    print(f"[random_spawn] Selected free cell (iy={cell[0]}, ix={cell[1]})")
    print(f"[random_spawn] World coords: x={world_xy[0]:.3f}, y={world_xy[1]:.3f}")
    print(f"[random_spawn] ARENA offset used: ax={arena_offset[0]:.3f}, ay={arena_offset[1]:.3f}")
    if arena_bounds is not None:
        print(f"[random_spawn] ARENA bounds: xmin={arena_bounds[0]:.3f}, xmax={arena_bounds[1]:.3f}, "
              f"ymin={arena_bounds[2]:.3f}, ymax={arena_bounds[3]:.3f}")
    print(f"[random_spawn] Updated WBT: {args.wbt}")
    print(f"[random_spawn] Previous pose x={old_xy[0]:.3f}, y={old_xy[1]:.3f}")
    if yaw is not None:
        print(f"[random_spawn] New yaw={yaw:.3f} rad")


if __name__ == "__main__":
    main()
