import re
import math
from pathlib import Path

# === CONFIG YOU CAN TWEAK IF NEEDED ===
WBT_FILE = "maze_world.wbt"   # <-- change to your .wbt filename
CELL_SIZE = 0.05              # this matches your Localiser / Planner
GRID_W = 20                   # 20 cells * 0.05 = 1.0 m (fits [-0.5, 0.5])
GRID_H = 20

# Only consider walls inside this box (your epuck & goal are inside ~[-0.5, 0.5])
X_MIN, X_MAX = -0.5, 0.5
Y_MIN, Y_MAX = -0.5, 0.5

def parse_walls(text):
    """
    Parse all 'DEF wall Solid' blocks and return a list of axis-aligned
    rectangles (xmin, xmax, ymin, ymax) in world coordinates.
    We assume walls only rotated by 0, pi/2, pi, -pi/2 around z.
    """
    walls = []
    blocks = text.split("DEF wall Solid {")
    for blk in blocks[1:]:
        # translation
        m_t = re.search(r"translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk)
        # rotation: we only care about the angle (4th number)
        m_r = re.search(r"rotation\s+[^\n]*\s([-\d.eE]+)", blk)
        # Box size
        m_s = re.search(r"geometry\s+Box\s*{\s*size\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk, re.S)

        if not (m_t and m_r and m_s):
            continue

        tx, ty, tz = map(float, m_t.groups())
        angle = float(m_r.group(1))
        sx, sy, sz = map(float, m_s.groups())

        # We only want maze walls near the arena, ignore big outer stuff if any
        if tx < X_MIN - 0.3 or tx > X_MAX + 0.3 or ty < Y_MIN - 0.3 or ty > Y_MAX + 0.3:
            continue

        # Normalise angle to [0, 2π)
        a = (angle % (2 * math.pi))

        # Axis-aligned approximation:
        # if angle ≈ 0 or π -> length along y = sy, along x = sx
        # if angle ≈ π/2 or 3π/2 -> swapped
        # (your walls all use only these rotations)
        if abs(a - 0.0) < 1e-2 or abs(a - math.pi) < 1e-2:
            half_x = sx / 2.0
            half_y = sy / 2.0
        elif abs(a - math.pi/2) < 1e-2 or abs(a - 3*math.pi/2) < 1e-2:
            half_x = sy / 2.0
            half_y = sx / 2.0
        else:
            # Fallback: just treat as axis-aligned with sy as vertical extent
            half_x = sx / 2.0
            half_y = sy / 2.0

        # Add a tiny margin so cells touching the wall are marked as occupied
        margin = 0.01
        xmin = tx - half_x - margin
        xmax = tx + half_x + margin
        ymin = ty - half_y - margin
        ymax = ty + half_y + margin

        walls.append((xmin, xmax, ymin, ymax))

    return walls

def build_grid(walls):
    """
    Build occupancy grid: 1 = obstacle, 0 = free
    Coordinate convention must match Localiser / Planner:
      - cell_size = CELL_SIZE
      - origin at (-width/2, -height/2)
    """
    width_m = GRID_W * CELL_SIZE
    height_m = GRID_H * CELL_SIZE
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0

    grid = [[0 for _ in range(GRID_W)] for _ in range(GRID_H)]

    for iy in range(GRID_H):
        for ix in range(GRID_W):
            # centre of this cell in world coords
            wx = origin_x + (ix + 0.5) * CELL_SIZE
            wy = origin_y + (iy + 0.5) * CELL_SIZE

            # mark occupied if inside any wall rectangle
            for (xmin, xmax, ymin, ymax) in walls:
                if xmin <= wx <= xmax and ymin <= wy <= ymax:
                    grid[iy][ix] = 1
                    break

    return grid

def main():
    text = Path(WBT_FILE).read_text()
    walls = parse_walls(text)
    print(f"Parsed {len(walls)} internal walls for grid mapping.")

    grid = build_grid(walls)

    # Print as Python list literal so you can paste into localiser/planner
    print("\n# === GENERATED OCCUPANCY GRID (0=free, 1=wall) ===")
    print("WORLD_MAP = [")
    for row in grid:
        print("    " + str(row) + ",")
    print("]")

if __name__ == "__main__":
    main()
