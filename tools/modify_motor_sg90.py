"""Modify MotorConnection_x4 STL to fit SG90 servo - v3.

Problem: Tab slot 28.6mm (via raycast) needs 32.6mm (SG90 32.2 + 0.4 clearance).
Fix: Use TWO thin slab cutters to shave the inner Z-walls outward,
instead of one large box that removes too much material.
"""

import os, sys
import numpy as np
import trimesh

HERE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR  = os.path.join(HERE, "stl")
INPUT_STL  = os.path.join(STL_DIR, "MotorConnection_x4.stl")
OUTPUT_STL = os.path.join(STL_DIR, "MotorConnection_x4_SG90.stl")

CLEARANCE = 0.4
SG90_TAB_SPAN = 32.2 + CLEARANCE   # 32.6 mm target
SG90_BODY_W   = 11.8 + CLEARANCE   # 12.2 mm target


def find_slot_walls(mesh):
    """Find the Z-direction slot walls using raycasting."""
    lo, hi = mesh.bounds

    z_wall_lo_list = []
    z_wall_hi_list = []
    slot_y_ranges = []

    for y in np.arange(lo[1] + 2, hi[1] - 1, 1.0):
        for x in np.linspace(lo[0] + 3, hi[0] - 3, 15):
            origin = np.array([[x, y, lo[2] - 1]])
            direction = np.array([[0, 0, 1]])
            locs, ray_idx, tri_idx = mesh.ray.intersects_location(origin, direction)
            if len(locs) >= 4:
                z_sorted = np.sort(locs[:, 2])
                # Pattern: outer_lo, inner_lo (slot floor), inner_hi (slot ceil), outer_hi
                gap = z_sorted[2] - z_sorted[1]
                if 20 < gap < 40:  # expect ~28.6mm slot
                    z_wall_lo_list.append(z_sorted[1])
                    z_wall_hi_list.append(z_sorted[2])
                    slot_y_ranges.append(y)

    if not z_wall_lo_list:
        print("ERROR: Could not find slot walls!")
        sys.exit(1)

    z_wall_lo = np.median(z_wall_lo_list)
    z_wall_hi = np.median(z_wall_hi_list)
    y_start = min(slot_y_ranges)
    y_end = max(slot_y_ranges)

    return z_wall_lo, z_wall_hi, y_start, y_end


def main():
    print("=" * 70)
    print("  MotorConnection_x4 -> SG90 Modification (v3 - precision)")
    print("=" * 70)

    mesh = trimesh.load(INPUT_STL, force="mesh")
    lo, hi = mesh.bounds
    ext = mesh.extents
    print(f"\nOriginal: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")
    print(f"  Volume: {mesh.volume:.1f} mm3, Faces: {len(mesh.faces)}, Watertight: {mesh.is_watertight}")

    # Find current slot walls
    print("\n--- Finding slot walls ---")
    z_lo_wall, z_hi_wall, y_start, y_end = find_slot_walls(mesh)
    current_span = z_hi_wall - z_lo_wall
    z_center = (z_lo_wall + z_hi_wall) / 2

    print(f"  Slot Z walls: {z_lo_wall:.1f} .. {z_hi_wall:.1f}  (span={current_span:.1f} mm)")
    print(f"  Slot Y range: {y_start:.1f} .. {y_end:.1f}")
    print(f"  Target span: {SG90_TAB_SPAN:.1f} mm")

    expand_each_side = (SG90_TAB_SPAN - current_span) / 2
    print(f"  Need to expand: {expand_each_side:.1f} mm each side")

    if expand_each_side <= 0:
        print("  Slot already wide enough! No modification needed.")
        return

    # Create two thin slab cutters - one for each Z wall
    # Each slab is positioned right at the wall and extends inward by expand_each_side
    y_center = (y_start + y_end) / 2
    y_depth = y_end - y_start + 4.0

    # Slab extends across full X range of the slot (the open U-cradle)
    x_span = ext[0] + 4.0  # full width + margin
    x_center = (lo[0] + hi[0]) / 2

    # Low-Z wall slab: shave from z_lo_wall down by expand_each_side
    slab_lo_center_z = z_lo_wall - expand_each_side / 2
    slab_lo = trimesh.primitives.Box(
        extents=[x_span, y_depth, expand_each_side + 0.1],
        transform=trimesh.transformations.translation_matrix(
            [x_center, y_center, slab_lo_center_z]
        )
    )

    # High-Z wall slab: shave from z_hi_wall up by expand_each_side
    slab_hi_center_z = z_hi_wall + expand_each_side / 2
    slab_hi = trimesh.primitives.Box(
        extents=[x_span, y_depth, expand_each_side + 0.1],
        transform=trimesh.transformations.translation_matrix(
            [x_center, y_center, slab_hi_center_z]
        )
    )

    print(f"\n--- Cutting ---")
    print(f"  Low-Z slab: center Z={slab_lo_center_z:.1f}, thickness={expand_each_side:.1f} mm")
    print(f"  High-Z slab: center Z={slab_hi_center_z:.1f}, thickness={expand_each_side:.1f} mm")

    # Combine cutters
    cutter_combined = trimesh.util.concatenate([slab_lo.to_mesh(), slab_hi.to_mesh()])

    try:
        result = trimesh.boolean.difference([mesh, cutter_combined], engine="manifold")
        if result is None or len(result.faces) < 100:
            raise RuntimeError("Empty result")
        print(f"  Boolean success! {len(result.faces)} faces, watertight={result.is_watertight}")
        modified = result
    except Exception as e:
        print(f"  Boolean failed ({e}), doing sequential cuts...")
        try:
            temp = trimesh.boolean.difference([mesh, slab_lo.to_mesh()], engine="manifold")
            modified = trimesh.boolean.difference([temp, slab_hi.to_mesh()], engine="manifold")
            print(f"  Sequential cuts success! {len(modified.faces)} faces")
        except Exception as e2:
            print(f"  Sequential also failed ({e2})")
            print("  Using vertex displacement fallback...")
            verts = mesh.vertices.copy()
            count = 0
            for i in range(len(verts)):
                if verts[i, 1] < y_start - 1 or verts[i, 1] > y_end + 1:
                    continue
                if abs(verts[i, 2] - z_lo_wall) < 1.0:
                    verts[i, 2] -= expand_each_side
                    count += 1
                elif abs(verts[i, 2] - z_hi_wall) < 1.0:
                    verts[i, 2] += expand_each_side
                    count += 1
            print(f"  Displaced {count} vertices")
            modified = trimesh.Trimesh(vertices=verts, faces=mesh.faces.copy())

    # Verify
    print(f"\n--- Verification ---")
    print(f"  Volume: {modified.volume:.1f} mm3 (was {mesh.volume:.1f}, removed {mesh.volume - modified.volume:.1f} mm3)")
    pct = (mesh.volume - modified.volume) / mesh.volume * 100
    print(f"  Material removed: {pct:.1f}%")

    # Re-measure slot
    z_lo_new, z_hi_new, _, _ = find_slot_walls(modified)
    new_span = z_hi_new - z_lo_new
    print(f"  New slot span: {new_span:.1f} mm (was {current_span:.1f}, target={SG90_TAB_SPAN:.1f})")

    if new_span >= 32.2:
        print(f"\n  >>> SG90 TABS WILL FIT! (span={new_span:.1f} >= 32.2) <<<")
    else:
        print(f"\n  >>> Still short by {32.2 - new_span:.1f} mm <<<")

    # Save
    modified.export(OUTPUT_STL)
    print(f"\n  Saved: {OUTPUT_STL}")
    print(f"  Original (unchanged): {INPUT_STL}")
    print(f"\n  Print 4 copies of MotorConnection_x4_SG90.stl")


if __name__ == "__main__":
    main()
