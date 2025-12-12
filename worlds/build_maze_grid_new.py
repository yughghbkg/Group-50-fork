import re
import math
from pathlib import Path

# ========= CONFIG =========
SCRIPT_DIR = Path(__file__).resolve().parent
WBT_FILE = SCRIPT_DIR / "maze_world_new.wbt"   

CELL_SIZE = 0.05
GRID_W = 40
GRID_H = 40

# Only consider walls within this range (can be shrunk/expanded as needed)
X_MIN, X_MAX = -3.0, 3.0
Y_MIN, Y_MAX = -3.0, 3.0


EXTRA_MARGIN = 0.009  


# ========= MATH UTILS =========
def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def axis_angle_to_rot(axis, angle):
    """Rodrigues: axis = (x,y,z), angle in rad. Return 3x3 rotation matrix."""
    x, y, z = axis
    n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-12:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    x, y, z = x/n, y/n, z/n
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c

    return (
        (x*x*C + c,     x*y*C - z*s, x*z*C + y*s),
        (y*x*C + z*s,   y*y*C + c,   y*z*C - x*s),
        (z*x*C - y*s,   z*y*C + x*s, z*z*C + c),
    )

def mat3_mul_vec3(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

def project_interval(points, axis2):
    mn = mx = dot(points[0], axis2)
    for p in points[1:]:
        v = dot(p, axis2)
        if v < mn: mn = v
        if v > mx: mx = v
    return mn, mx

def intervals_overlap(a, b, extra=0.0):
    return not (a[1] < b[0] - extra or b[1] < a[0] - extra)

def sat_intersect(polyA, polyB, axes, extra=0.0):
    for ax in axes:
        ia = project_interval(polyA, ax)
        ib = project_interval(polyB, ax)
        if not intervals_overlap(ia, ib, extra=extra):
            return False
    return True


# ========= PARSING =========
def parse_arena(text):
    m = re.search(
        r"DEF\s+ARENA\s+RectangleArena\s*{.*?"
        r"translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+).*?"
        r"floorSize\s+([-\d.eE]+)\s+([-\d.eE]+)",
        text,
        re.S
    )
    if not m:
        return None

    ax, ay, az, fw, fh = map(float, m.groups())
    xmin = ax - fw / 2.0
    xmax = ax + fw / 2.0
    ymin = ay - fh / 2.0
    ymax = ay + fh / 2.0
    return (xmin, xmax, ymin, ymax)

def parse_walls(text):
    walls = []
    blocks = text.split("DEF wall Solid {")
    for blk in blocks[1:]:
        m_t = re.search(r"translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk)
        m_r = re.search(r"rotation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk)
        m_s = re.search(r"geometry\s+Box\s*{\s*size\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", blk, re.S)

        if not (m_t and m_r and m_s):
            continue

        tx, ty, tz = map(float, m_t.groups())
        rx, ry, rz, angle = map(float, m_r.groups())
        sx, sy, sz = map(float, m_s.groups())

        if tx < X_MIN - 0.5 or tx > X_MAX + 0.5 or ty < Y_MIN - 0.5 or ty > Y_MAX + 0.5:
            continue

        hx, hy = sx / 2.0, sy / 2.0
        R = axis_angle_to_rot((rx, ry, rz), angle)

        local = [
            (-hx, -hy, 0.0),
            (-hx,  hy, 0.0),
            ( hx,  hy, 0.0),
            ( hx, -hy, 0.0),
        ]

        world2d = []
        for p in local:
            pr = mat3_mul_vec3(R, p)
            world2d.append((tx + pr[0], ty + pr[1]))

        e0 = (world2d[1][0] - world2d[0][0], world2d[1][1] - world2d[0][1])
        e1 = (world2d[3][0] - world2d[0][0], world2d[3][1] - world2d[0][1])

        def norm2(v):
            return math.sqrt(v[0]*v[0] + v[1]*v[1])

        n0 = norm2(e0)
        n1 = norm2(e1)
        if n0 < 1e-12 or n1 < 1e-12:
            continue
        u = (e0[0] / n0, e0[1] / n0)
        v = (e1[0] / n1, e1[1] / n1)

        xs = [p[0] for p in world2d]
        ys = [p[1] for p in world2d]
        aabb = (min(xs), max(xs), min(ys), max(ys))

        walls.append({
            "center": (tx, ty),
            "corners": world2d,
            "axes": (u, v),
            "aabb": aabb,
        })

    return walls


# ========= GRID BUILDING =========
def build_grid(walls, arena_bounds=None):
    width_m = GRID_W * CELL_SIZE
    height_m = GRID_H * CELL_SIZE
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0

    half = CELL_SIZE / 2.0
    grid = [[0 for _ in range(GRID_W)] for _ in range(GRID_H)]

    # Sample points: center + four directions (slightly inset to reduce edge-touch occupancy)
    s = half * 0.6
    samples = [(0.0, 0.0), (s, 0.0), (-s, 0.0), (0.0, s), (0.0, -s)]

    # Precompute for each wall: center point (more accurate using corners), half-lengths along u and v
    pre = []
    for w in walls:
        c = w["corners"]
        cx = sum(p[0] for p in c) / 4.0
        cy = sum(p[1] for p in c) / 4.0
        u, v = w["axes"]
        hu = 0.5 * math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1])
        hv = 0.5 * math.hypot(c[3][0] - c[0][0], c[3][1] - c[0][1])
        pre.append((w["aabb"], (cx, cy), (u, v), (hu, hv)))

    for iy in range(GRID_H):
        cy = origin_y + (iy + 0.5) * CELL_SIZE
        for ix in range(GRID_W):
            cx = origin_x + (ix + 0.5) * CELL_SIZE
            cell_aabb = (cx - half, cx + half, cy - half, cy + half)

            if arena_bounds is not None:
                axmin, axmax, aymin, aymax = arena_bounds
                on_left   = abs(cx - axmin) <= half and (aymin - half) <= cy <= (aymax + half)
                on_right  = abs(cx - axmax) <= half and (aymin - half) <= cy <= (aymax + half)
                on_bottom = abs(cy - aymin) <= half and (axmin - half) <= cx <= (axmax + half)
                on_top    = abs(cy - aymax) <= half and (axmin - half) <= cx <= (axmax + half)
                if on_left or on_right or on_bottom or on_top:
                    grid[iy][ix] = 1
                    continue

            occupied = False

            for (aabb, wc, axes, halfs) in pre:
                xmin, xmax, ymin, ymax = aabb
                if cell_aabb[1] < xmin or cell_aabb[0] > xmax:
                    continue
                if cell_aabb[3] < ymin or cell_aabb[2] > ymax:
                    continue

                (wx, wy) = wc
                (u, v) = axes
                (hu, hv) = halfs

                for (dx, dy) in samples:
                    px = cx + dx
                    py = cy + dy
                    rx = px - wx
                    ry = py - wy
                    du = rx * u[0] + ry * u[1]
                    dv = rx * v[0] + ry * v[1]
                    if abs(du) <= (hu + EXTRA_MARGIN) and abs(dv) <= (hv + EXTRA_MARGIN):
                        occupied = True
                        break

                if occupied:
                    break

            grid[iy][ix] = 1 if occupied else 0

    return grid


def main():
    wbt_path = Path(WBT_FILE)
    if not wbt_path.exists():
        raise FileNotFoundError(f"Could not find WBT file at {wbt_path}")

    text = wbt_path.read_text(encoding="utf-8", errors="ignore")

    arena_bounds = parse_arena(text)
    if arena_bounds is None:
        print("Warning: ARENA RectangleArena not found; arena boundary will NOT be added.")
    else:
        print(f"ARENA bounds (xmin,xmax,ymin,ymax) = {arena_bounds}")

    walls = parse_walls(text)
    print(f"Parsed {len(walls)} wall solids.")

    grid = build_grid(walls, arena_bounds=arena_bounds)

    print("\n# === GENERATED OCCUPANCY GRID (0=free, 1=wall) ===")
    print("WORLD_MAP = [")
    for row in grid:
        print("    " + str(row) + ",")
    print("]")


if __name__ == "__main__":
    main()
