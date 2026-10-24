"""Find the exact cradle boundaries of MotorConnection."""
import trimesh
import numpy as np

m = trimesh.load('stl/MotorConnection_x4.stl', force='mesh')
lo, hi = m.bounds

# The MotorConnection is a U-shaped cradle. The servo drops in from one side.
# We need to find where the U-opening is - the cutter should only
# cut material within the servo pocket, not the structural arms.

# Scan Y at the slot midpoint (Z=-17) to find the cradle Y extent
print("Y-ray at slot center (Z=-17) for different X:")
for x in np.linspace(lo[0] + 1, hi[0] - 1, 15):
    origin = np.array([[x, lo[1] - 1, -17]])
    direction = np.array([[0, 1, 0]])
    locs = m.ray.intersects_location(origin, direction)[0]
    if len(locs) >= 2:
        y_sorted = np.sort(locs[:, 1])
        print(f"  X={x:5.1f}: Y={[f'{v:.1f}' for v in y_sorted]}  ({len(locs)} hits)")
    else:
        print(f"  X={x:5.1f}: open/no material ({len(locs)} hits)")

print()

# Scan X at slot wall positions to find where the cradle material is
print("X-ray at different Y positions (at Z=-31.3, the low wall):")
for y in np.arange(lo[1] + 2, hi[1] - 1, 4.0):
    origin = np.array([[lo[0] - 1, y, -31.3]])
    direction = np.array([[1, 0, 0]])
    locs = m.ray.intersects_location(origin, direction)[0]
    if len(locs) >= 2:
        x_sorted = np.sort(locs[:, 0])
        print(f"  Y={y:5.1f}: X={[f'{v:.1f}' for v in x_sorted]}")
    else:
        print(f"  Y={y:5.1f}: open ({len(locs)} hits)")

print()
print("X-ray at different Y positions (at Z=-2.7, the high wall):")
for y in np.arange(lo[1] + 2, hi[1] - 1, 4.0):
    origin = np.array([[lo[0] - 1, y, -2.7]])
    direction = np.array([[1, 0, 0]])
    locs = m.ray.intersects_location(origin, direction)[0]
    if len(locs) >= 2:
        x_sorted = np.sort(locs[:, 0])
        print(f"  Y={y:5.1f}: X={[f'{v:.1f}' for v in x_sorted]}")
    else:
        print(f"  Y={y:5.1f}: open ({len(locs)} hits)")

print()

# Find the actual X range of the servo pocket (where the slot exists)
# The slot should be bounded in X by the U-cradle walls
print("Containment test at Z=-31.5 (just outside low wall) at center Y=23:")
for x in np.linspace(lo[0], hi[0], 30):
    inside = m.contains(np.array([[x, 23, -31.5]]))[0]
    if inside:
        print(f"  X={x:5.1f}: SOLID", end="")
    else:
        print(f"  X={x:5.1f}: ----", end="")
    print()
