# map_builder.py
#
# Supervisor controller that scans axis-aligned wall Solids in the world
# and builds an occupancy grid (0 = free, 1 = wall), then prints it
# as Python code to paste into localisation.py.

from controller import Supervisor
import math

# -------------------------------------------------------------
# CONFIGURATION – YOU MAY NEED TO EDIT THESE CONSTANTS
# -------------------------------------------------------------

# Desired grid resolution (rows × cols)
GRID_ROWS = 12
GRID_COLS = 12

# Safety margin around wall boxes when marking cells as occupied (meters)
WALL_MARGIN = 0.01

# Name of the arena node (RectangleArena PROTO) – you MUST set this DEF in your world:
# Example in your .wbt:
#   DEF ARENA RectangleArena { ... }
ARENA_DEF = "ARENA"

# Optional: only treat Solids whose DEF starts with this prefix as walls
# Set to "" to treat ALL Solid nodes as obstacles.
WALL_DEF_PREFIX = "wall"   # change to your prefix, or "" to disable

# -------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------

def get_arena_bounds(supervisor):
    """
    Get (x_min, x_max, z_min, z_max) of the floor from a RectangleArena.
    Assumes the arena is centered at (0, 0) in x–z.
    """
    arena = supervisor.getFromDef(ARENA_DEF)
    if arena is None:
        print(f"[map_builder] ERROR: No node with DEF {ARENA_DEF}. "
              f"Set your RectangleArena to 'DEF {ARENA_DEF} RectangleArena {{ ... }}'")
        return None

    # RectangleArena has a 'floorSize' field: [sizeX, sizeZ]
    floor_field = arena.getField("floorSize")
    if floor_field is None:
        print("[map_builder] ERROR: ARENA node has no 'floorSize' field; "
              "make sure it is a RectangleArena.")
        return None

    size_x, size_z = floor_field.getSFVec2f()
    x_min = -size_x / 2.0
    x_max =  size_x / 2.0
    z_min = -size_z / 2.0
    z_max =  size_z / 2.0

    print(f"[map_builder] Arena floor size: ({size_x:.3f} m, {size_z:.3f} m)")
    print(f"[map_builder] x in [{x_min:.3f}, {x_max:.3f}], z in [{z_min:.3f}, {z_max:.3f}]")

    return x_min, x_max, z_min, z_max


def collect_wall_boxes(supervisor):
    """
    Scan the scene tree, find Solid nodes that represent walls,
    and approximate them as axis-aligned rectangles in x–z.

    Returns a list of dicts:
      { "x": center_x, "z": center_z, "half_x": hx, "half_z": hz }
    """
    walls = []

    root = supervisor.getRoot()
    children_field = root.getField("children")
    count = children_field.getCount()

    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue

        # Only care about Solid nodes
        if node.getTypeName() != "Solid":
            continue

        def_name = node.getDef()
        if def_name is None:
            def_name = ""

        # If a prefix is configured, filter by it
        if WALL_DEF_PREFIX and not def_name.startswith(WALL_DEF_PREFIX):
            continue

        # Get boundingObject node (we assume it's a Box)
        bo_field = node.getField("boundingObject")
        if bo_field is None:
            continue
        bo_node = bo_field.getSFNode()
        if bo_node is None:
            continue
        if bo_node.getTypeName() != "Box":
            # If your walls use a different shape, you need to extend this logic.
            continue

        size_field = bo_node.getField("size")
        if size_field is None:
            continue
        sx, sy, sz = size_field.getSFVec3f()  # full extents

        # Center position of the Solid
        t_field = node.getField("translation")
        tx, ty, tz = t_field.getSFVec3f()

        wall = {
            "x": tx,
            "z": tz,
            "half_x": sx / 2.0 + WALL_MARGIN,
            "half_z": sz / 2.0 + WALL_MARGIN,
            "def": def_name,
        }
        walls.append(wall)

    print(f"[map_builder] Found {len(walls)} wall boxes:")
    for w in walls:
        print(f"  DEF {w['def']}: center=({w['x']:.3f},{w['z']:.3f}), "
              f"half_x={w['half_x']:.3f}, half_z={w['half_z']:.3f}")

    return walls


def build_grid(x_min, x_max, z_min, z_max, walls):
    """
    Build a GRID_ROWS × GRID_COLS occupancy grid.

    Cell (row, col):
      row = 0 → bottom (min z)
      row = GRID_ROWS-1 → top (max z)
      col = 0 → left (min x)
      col = GRID_COLS-1 → right (max x)
    """
    grid = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

    cell_w = (x_max - x_min) / GRID_COLS
    cell_h = (z_max - z_min) / GRID_ROWS

    print(f"[map_builder] cell_w={cell_w:.4f}, cell_h={cell_h:.4f}")

    # For each cell, check if its center lies inside any wall box
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            cx = x_min + (col + 0.5) * cell_w
            cz = z_min + (row + 0.5) * cell_h

            occupied = False
            for w in walls:
                if (abs(cx - w["x"]) <= w["half_x"] and
                        abs(cz - w["z"]) <= w["half_z"]):
                    occupied = True
                    break

            grid[row][col] = 1 if occupied else 0

    return grid


def print_grid_as_python(grid):
    """
    Print the grid as a Python list literal suitable for localisation.py.
    """
    print("\n[map_builder] =================== OCCUPANCY GRID (Python) ===================")
    print("world_map = [")
    for row in grid:
        print("    " + str(row) + ",")
    print("]")
    print("[map_builder] =============================================================\n")

# -------------------------------------------------------------
# Main supervisor logic
# -------------------------------------------------------------

def main():
    sup = Supervisor()
    time_step = int(sup.getBasicTimeStep())

    # Let Webots initialise the world (one step is usually enough)
    sup.step(time_step)

    bounds = get_arena_bounds(sup)
    if bounds is None:
        return
    x_min, x_max, z_min, z_max = bounds

    walls = collect_wall_boxes(sup)
    if not walls:
        print("[map_builder] WARNING: No walls found with current filters.")

    grid = build_grid(x_min, x_max, z_min, z_max, walls)

    print_grid_as_python(grid)

    # Optionally, stop the simulation once done
    print("[map_builder] Map building complete. You can copy the grid above into localisation.py.")
    while sup.step(time_step) != -1:
        pass


if __name__ == "__main__":
    main()
