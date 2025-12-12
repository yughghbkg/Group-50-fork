import re
import math
from pathlib import Path

# --- config ---
SCRIPT_DIR = Path(__file__).resolve().parent
WBT_FILE = SCRIPT_DIR / "maze_world.wbt"  

# grid resolution and size 
CELL_SIZE = 0.05             
GRID_W = 20                  
GRID_H = 20

# only keep walls near the maze area
X_MIN, X_MAX = -0.5, 0.5
Y_MIN, Y_MAX = -0.5, 0.5

def parse_walls(text):

    '''
    Parse wall solids from the .wbt file.
    Walls are approximated as axis-aligned rectangles.
    Only rotations of 0 / 90 / 180 degrees are handled.
    '''

    walls = []
    blocks = text.split("DEF wall Solid {")
    for blk in blocks[1:]:
        m_t = re.search(r"translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk)
        m_r = re.search(r"rotation\s+[^\n]*\s([-\d.eE]+)", blk)
        m_s = re.search(r"geometry\s+Box\s*{\s*size\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk, re.S)

        if not (m_t and m_r and m_s):
            continue

        tx, ty, tz = map(float, m_t.groups())
        angle = float(m_r.group(1))
        sx, sy, sz = map(float, m_s.groups())

        if tx < X_MIN - 0.3 or tx > X_MAX + 0.3 or ty < Y_MIN - 0.3 or ty > Y_MAX + 0.3:
            continue

        a = (angle % (2 * math.pi))

        # axis-aligned approximation based on rotation
        if abs(a - 0.0) < 1e-2 or abs(a - math.pi) < 1e-2:
            half_x = sx / 2.0
            half_y = sy / 2.0
        elif abs(a - math.pi/2) < 1e-2 or abs(a - 3*math.pi/2) < 1e-2:
            half_x = sy / 2.0
            half_y = sx / 2.0
        else:

            # fallback: treat as axis-aligned
            half_x = sx / 2.0
            half_y = sy / 2.0

        # small margin so touching cells count as occupied
        margin = 0.01
        xmin = tx - half_x - margin
        xmax = tx + half_x + margin
        ymin = ty - half_y - margin
        ymax = ty + half_y + margin

        walls.append((xmin, xmax, ymin, ymax))

    return walls

def build_grid(walls):
    width_m = GRID_W * CELL_SIZE
    height_m = GRID_H * CELL_SIZE
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0

    grid = [[0 for _ in range(GRID_W)] for _ in range(GRID_H)]

    for iy in range(GRID_H):
        for ix in range(GRID_W):
            wx = origin_x + (ix + 0.5) * CELL_SIZE
            wy = origin_y + (iy + 0.5) * CELL_SIZE

            for (xmin, xmax, ymin, ymax) in walls:
                if xmin <= wx <= xmax and ymin <= wy <= ymax:
                    grid[iy][ix] = 1
                    break

    return grid

def main():
    wbt_path = Path(WBT_FILE)
    if not wbt_path.exists():
        raise FileNotFoundError(f"Could not find WBT file at {wbt_path}")

    text = wbt_path.read_text()
    walls = parse_walls(text)
    print(f"Parsed {len(walls)} internal walls for grid mapping.")

    grid = build_grid(walls)

    print("\n# === GENERATED OCCUPANCY GRID (0=free, 1=wall) ===")
    print("WORLD_MAP = [")
    for row in grid:
        print("    " + str(row) + ",")
    print("]")

if __name__ == "__main__":
    main()
