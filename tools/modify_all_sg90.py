"""Modify all remaining PhoneBot parts to fit SG90 servo.

For each part:
1. Raycast along all 3 axes to find servo-sized internal slots
2. Compare against SG90 dimensions
3. Widen any too-narrow slots using boolean subtraction
4. Save as PartName_SG90.stl

Already done: MotorConnection_x4 (handled separately)
Parts that PASS (no change): HipBracket_x2, TorsoBase_x1, UpperLegLink_x2
"""

import os, sys
import numpy as np
import trimesh

HERE    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(HERE, "stl")

# SG90 dims + clearance
CLEARANCE      = 0.4
SG90_TAB_SPAN  = 32.2 + CLEARANCE   # 32.6 mm
SG90_BODY_W    = 11.8 + CLEARANCE   # 12.2 mm
SG90_BODY_L    = 22.2 + CLEARANCE   # 22.6 mm

# Parts to modify (those with TIGHT or FAIL from fit check)
PARTS_TO_MODIFY = [
    "PelvisTop_x1.stl",
    "PelvisBottom_x1.stl",
    "IdlerServoBracket_x4.stl",
    "Foot_x2.stl",
]


def find_all_slots(mesh, min_gap=8.0, max_gap=40.0):
    """Raycast along all 3 axes to find internal slots.

    Returns list of dicts with slot info:
      { axis, wall_lo, wall_hi, span, y_start, y_end, ... }
    """
    lo, hi = mesh.bounds
    ext = mesh.extents
    slots = []

    for ax in range(3):
        ax_name = "xyz"[ax]
        others = [i for i in range(3) if i != ax]

        # For each perpendicular axis, cast rays to find internal gaps
        for perp_ax in others:
            perp_name = "xyz"[perp_ax]
            third_ax = [i for i in others if i != perp_ax][0]

            wall_lo_list = []
            wall_hi_list = []
            scan_positions = []  # positions along the scan axis

            # Scan along the remaining axis (the one we're not raycasting through)
            for scan_val in np.arange(lo[ax] + 2, hi[ax] - 1, 2.0):
                for third_val in np.linspace(lo[third_ax] + 2, hi[third_ax] - 2, 8):
                    origin = np.zeros((1, 3))
                    origin[0, ax] = scan_val
                    origin[0, third_ax] = third_val
                    origin[0, perp_ax] = lo[perp_ax] - 1

                    direction = np.zeros((1, 3))
                    direction[0, perp_ax] = 1

                    locs = mesh.ray.intersects_location(origin, direction)[0]
                    if len(locs) >= 4:
                        sorted_hits = np.sort(locs[:, perp_ax])
                        # Look for internal gap (enter-exit-enter-exit pattern)
                        # The gap between hit[1] and hit[2] is the slot
                        gap = sorted_hits[2] - sorted_hits[1]
                        if min_gap < gap < max_gap:
                            wall_lo_list.append(sorted_hits[1])
                            wall_hi_list.append(sorted_hits[2])
                            scan_positions.append(scan_val)

            if len(wall_lo_list) < 3:
                continue

            wall_lo_med = np.median(wall_lo_list)
            wall_hi_med = np.median(wall_hi_list)
            span = wall_hi_med - wall_lo_med
            scan_start = min(scan_positions)
            scan_end = max(scan_positions)

            slots.append({
                "scan_axis": ax,
                "slot_axis": perp_ax,
                "slot_axis_name": perp_name,
                "wall_lo": float(wall_lo_med),
                "wall_hi": float(wall_hi_med),
                "span": float(span),
                "scan_start": float(scan_start),
                "scan_end": float(scan_end),
                "n_hits": len(wall_lo_list),
            })

    return slots


def match_sg90(slot):
    """Check if a slot matches SG90 body or tab dimensions.
    Returns (match_type, target_span, expand_needed) or None.
    """
    span = slot["span"]

    # Tab span match: ~28-34mm range
    if 25 < span < SG90_TAB_SPAN:
        return ("tab", SG90_TAB_SPAN, (SG90_TAB_SPAN - span) / 2)

    # Body length match: ~19-23mm range
    if 18 < span < SG90_BODY_L and span < 25:
        return ("body_L", SG90_BODY_L, (SG90_BODY_L - span) / 2)

    # Body width match: ~9-12mm range
    if 8 < span < SG90_BODY_W and span < 18:
        return ("body_W", SG90_BODY_W, (SG90_BODY_W - span) / 2)

    # Already fits
    if SG90_TAB_SPAN <= span < SG90_TAB_SPAN + 5:
        return ("tab_ok", span, 0)
    if SG90_BODY_W <= span < SG90_BODY_W + 5:
        return ("body_W_ok", span, 0)
    if SG90_BODY_L <= span < SG90_BODY_L + 5:
        return ("body_L_ok", span, 0)

    return None


def widen_slot(mesh, slot, expand):
    """Widen a slot by boolean-subtracting two thin slabs at the walls."""
    lo, hi = mesh.bounds
    ext = mesh.extents
    ax = slot["slot_axis"]        # axis along which the slot spans
    scan_ax = slot["scan_axis"]   # axis along which we scanned

    # The third axis
    third_ax = [i for i in range(3) if i != ax and i != scan_ax][0]

    # Slab dimensions
    # Along slot axis: thin slab of thickness = expand + 0.1
    # Along scan axis: cover the slot's scan range + margin
    # Along third axis: full extent + margin

    scan_span = slot["scan_end"] - slot["scan_start"] + 4.0
    scan_center = (slot["scan_start"] + slot["scan_end"]) / 2
    third_span = ext[third_ax] + 4.0
    third_center = (lo[third_ax] + hi[third_ax]) / 2
    slab_thickness = expand + 0.2

    # Low wall slab
    slab_lo_dims = np.zeros(3)
    slab_lo_dims[ax] = slab_thickness
    slab_lo_dims[scan_ax] = scan_span
    slab_lo_dims[third_ax] = third_span

    slab_lo_center = np.zeros(3)
    slab_lo_center[ax] = slot["wall_lo"] - expand / 2
    slab_lo_center[scan_ax] = scan_center
    slab_lo_center[third_ax] = third_center

    # High wall slab
    slab_hi_center = np.zeros(3)
    slab_hi_center[ax] = slot["wall_hi"] + expand / 2
    slab_hi_center[scan_ax] = scan_center
    slab_hi_center[third_ax] = third_center

    slab_lo_mesh = trimesh.primitives.Box(
        extents=slab_lo_dims,
        transform=trimesh.transformations.translation_matrix(slab_lo_center)
    ).to_mesh()

    slab_hi_mesh = trimesh.primitives.Box(
        extents=slab_lo_dims,  # same dims
        transform=trimesh.transformations.translation_matrix(slab_hi_center)
    ).to_mesh()

    # Combine and subtract
    cutter = trimesh.util.concatenate([slab_lo_mesh, slab_hi_mesh])

    try:
        result = trimesh.boolean.difference([mesh, cutter], engine="manifold")
        if result is not None and len(result.faces) > 50:
            return result
    except Exception:
        pass

    # Sequential fallback
    try:
        temp = trimesh.boolean.difference([mesh, slab_lo_mesh], engine="manifold")
        result = trimesh.boolean.difference([temp, slab_hi_mesh], engine="manifold")
        if result is not None and len(result.faces) > 50:
            return result
    except Exception:
        pass

    # Vertex displacement fallback
    print("      Boolean failed, using vertex displacement...")
    verts = mesh.vertices.copy()
    count = 0
    for i in range(len(verts)):
        v = verts[i]
        if v[scan_ax] < slot["scan_start"] - 1 or v[scan_ax] > slot["scan_end"] + 1:
            continue
        if abs(v[ax] - slot["wall_lo"]) < 1.0:
            verts[i, ax] -= expand
            count += 1
        elif abs(v[ax] - slot["wall_hi"]) < 1.0:
            verts[i, ax] += expand
            count += 1
    print(f"      Displaced {count} vertices")
    return trimesh.Trimesh(vertices=verts, faces=mesh.faces.copy())


def process_part(name):
    """Analyze and modify a single part for SG90 compatibility."""
    input_path = os.path.join(STL_DIR, name)
    output_name = name.replace(".stl", "_SG90.stl")
    output_path = os.path.join(STL_DIR, output_name)

    print("=" * 70)
    print(f"  {name}")
    print("=" * 70)

    mesh = trimesh.load(input_path, force="mesh")
    lo, hi = mesh.bounds
    ext = mesh.extents
    print(f"  Size: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")
    print(f"  Volume: {mesh.volume:.1f} mm3, Faces: {len(mesh.faces)}")

    # Find all internal slots
    print(f"\n  Scanning for servo slots...")
    slots = find_all_slots(mesh)

    if not slots:
        print(f"  No internal slots found - checking with smaller gap threshold...")
        slots = find_all_slots(mesh, min_gap=5.0, max_gap=40.0)

    if not slots:
        print(f"  No servo-sized slots detected. Skipping.")
        return None

    # Match slots to SG90 dimensions
    modifications = []
    for s in slots:
        match = match_sg90(s)
        if match is None:
            continue

        match_type, target, expand = match
        status = "OK" if expand <= 0 else f"NEED +{expand:.1f}mm/side"
        print(f"  Slot [{s['slot_axis_name']}]: {s['span']:.1f}mm "
              f"-> {match_type} ({status})")

        if expand > 0.05:  # only modify if > 0.05mm expansion needed
            modifications.append((s, match_type, target, expand))

    if not modifications:
        print(f"\n  All slots already fit SG90! No modification needed.")
        # Still save a copy for consistency
        mesh.export(output_path)
        print(f"  Saved copy as: {output_name}")
        return {"name": name, "status": "ALREADY_OK"}

    # Apply modifications
    print(f"\n  Applying {len(modifications)} modification(s)...")
    current_mesh = mesh

    for slot, match_type, target, expand in modifications:
        print(f"    Widening {slot['slot_axis_name']}-slot: "
              f"{slot['span']:.1f} -> {target:.1f}mm (+{expand:.1f}mm/side)")
        current_mesh = widen_slot(current_mesh, slot, expand)
        print(f"      Result: {len(current_mesh.faces)} faces, "
              f"watertight={current_mesh.is_watertight}")

    # Verify
    print(f"\n  Verification:")
    vol_removed = mesh.volume - current_mesh.volume
    pct = vol_removed / mesh.volume * 100 if mesh.volume > 0 else 0
    print(f"    Volume: {current_mesh.volume:.1f} mm3 "
          f"(removed {vol_removed:.1f} mm3 = {pct:.1f}%)")
    print(f"    Watertight: {current_mesh.is_watertight}")

    # Re-scan to verify
    new_slots = find_all_slots(current_mesh)
    for s in new_slots:
        match = match_sg90(s)
        if match:
            match_type, target, expand = match
            if expand <= 0:
                print(f"    Slot [{s['slot_axis_name']}]: {s['span']:.1f}mm -> FIT OK")
            else:
                print(f"    Slot [{s['slot_axis_name']}]: {s['span']:.1f}mm -> still needs +{expand:.1f}mm")

    # Save
    current_mesh.export(output_path)
    print(f"\n  Saved: {output_name}")

    return {
        "name": name,
        "output": output_name,
        "status": "MODIFIED",
        "vol_removed_pct": pct,
        "modifications": len(modifications),
    }


def main():
    print()
    print("#" * 70)
    print("#  PhoneBot - SG90 Servo Fit Modification (All Remaining Parts)")
    print("#" * 70)
    print()
    print(f"SG90 target dims: tab_span={SG90_TAB_SPAN:.1f}mm, "
          f"body_W={SG90_BODY_W:.1f}mm, body_L={SG90_BODY_L:.1f}mm")
    print(f"Clearance: {CLEARANCE}mm per side")
    print()

    results = []
    for name in PARTS_TO_MODIFY:
        path = os.path.join(STL_DIR, name)
        if not os.path.isfile(path):
            print(f"  SKIP: {name} not found")
            continue
        r = process_part(name)
        if r:
            results.append(r)
        print()

    # Summary
    print()
    print("#" * 70)
    print("#  SUMMARY")
    print("#" * 70)
    print()

    # Include MotorConnection (already done)
    print(f"  {'Part':<30s} {'Status':<15s} {'Output File'}")
    print(f"  {'-'*30} {'-'*15} {'-'*35}")
    print(f"  {'MotorConnection_x4.stl':<30s} {'DONE (prev)':<15s} MotorConnection_x4_SG90.stl")

    for r in results:
        status = r["status"]
        out = r.get("output", r["name"])
        if status == "MODIFIED":
            status_str = f"MODIFIED (-{r['vol_removed_pct']:.0f}%)"
        else:
            status_str = status
        print(f"  {r['name']:<30s} {status_str:<15s} {out}")

    # Parts that don't need modification
    print(f"  {'HipBracket_x2.stl':<30s} {'PASS (no chg)':<15s} (original fits)")
    print(f"  {'TorsoBase_x1.stl':<30s} {'PASS (no chg)':<15s} (original fits)")
    print(f"  {'UpperLegLink_x2.stl':<30s} {'PASS (no chg)':<15s} (original fits)")
    print(f"  {'LowerLegLink_x2.stl':<30s} {'N/A':<15s} (no servo)")

    print()
    print("  All _SG90.stl files are in: stl/")
    print("  Original files are UNCHANGED.")
    print()


if __name__ == "__main__":
    main()
