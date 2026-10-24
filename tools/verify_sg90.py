"""Full verification of ALL SG90 STL files referenced by assembly_3d.html.

Checks each file for:
1. Watertight mesh (no holes)
2. Servo slot dimensions (should fit SG90: tab_span >= 32.2mm)
3. Structural joint integrity (X-axis joints should be ~28.6mm, NOT widened)
4. Compare with originals to verify only correct cuts were made
"""

import os
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(HERE, "stl")
SG90_DIR = os.path.join(STL_DIR, "SG90")

# All unique STL files referenced in assembly_3d.html
HTML_FILES = [
    "TorsoBase_x1.stl",
    "PelvisTop_x1.stl",
    "PelvisBottom_x1.stl",
    "PhoneMount_x1.stl",
    "ElectronicsFrame_x1.stl",
    "HipBracket_x2.stl",
    "MotorConnection_x4.stl",
    "UpperLegLink_x2.stl",
    "LowerLegLink_x2.stl",
    "Foot_x2.stl",
    "IdlerServoBracket_x4.stl",
]

SG90_TAB_SPAN = 32.2  # minimum tab span needed


def find_slots(mesh, slot_axis, scan_axis, third_axis, min_gap=20, max_gap=40):
    """Find slot walls along a specific axis."""
    lo, hi = mesh.bounds
    wall_lo_list = []
    wall_hi_list = []

    for scan_val in np.arange(lo[scan_axis] + 2, hi[scan_axis] - 1, 2.0):
        for third_val in np.linspace(lo[third_axis] + 2, hi[third_axis] - 2, 5):
            origin = np.zeros((1, 3))
            origin[0, scan_axis] = scan_val
            origin[0, third_axis] = third_val
            origin[0, slot_axis] = lo[slot_axis] - 1
            direction = np.zeros((1, 3))
            direction[0, slot_axis] = 1
            locs = mesh.ray.intersects_location(origin, direction)[0]
            if len(locs) >= 4:
                sorted_hits = np.sort(locs[:, slot_axis])
                gap = sorted_hits[2] - sorted_hits[1]
                if min_gap < gap < max_gap:
                    wall_lo_list.append(sorted_hits[1])
                    wall_hi_list.append(sorted_hits[2])

    if len(wall_lo_list) < 3:
        return None
    return {
        "wall_lo": float(np.median(wall_lo_list)),
        "wall_hi": float(np.median(wall_hi_list)),
        "span": float(np.median(wall_hi_list)) - float(np.median(wall_lo_list)),
        "hits": len(wall_lo_list),
    }


def check_file(name):
    """Full check of one SG90 STL file."""
    sg_path = os.path.join(SG90_DIR, name)
    orig_path = os.path.join(STL_DIR, name)

    if not os.path.isfile(sg_path):
        print(f"  MISSING: {sg_path}")
        return False

    sg = trimesh.load(sg_path, force="mesh")
    orig = trimesh.load(orig_path, force="mesh")

    # Basic checks
    ok = True
    size_diff = os.path.getsize(sg_path) - os.path.getsize(orig_path)
    modified = size_diff != 0

    print(f"  Watertight: {sg.is_watertight}")
    print(f"  Faces: {len(sg.faces)} (orig: {len(orig.faces)})")
    print(f"  Volume: {sg.volume:.0f} mm3 (orig: {orig.volume:.0f})")
    print(f"  File size diff: {size_diff:+d} bytes {'(MODIFIED)' if modified else '(identical)'}")

    if not sg.is_watertight:
        print(f"  *** ERROR: NOT WATERTIGHT! ***")
        ok = False

    # For modified parts, check servo slots
    if name in ["MotorConnection_x4.stl", "PelvisTop_x1.stl", "PelvisBottom_x1.stl"]:
        # Check Z-axis servo tab slot (should be >= 32.2mm)
        z_slot = find_slots(sg, slot_axis=2, scan_axis=0, third_axis=1)
        if z_slot:
            status = "OK" if z_slot["span"] >= SG90_TAB_SPAN else "FAIL"
            print(f"  Z-slot (servo tabs): span={z_slot['span']:.1f}mm [{status}]")
            if z_slot["span"] < SG90_TAB_SPAN:
                ok = False
        else:
            print(f"  Z-slot: not found")

    if name in ["PelvisTop_x1.stl", "PelvisBottom_x1.stl"]:
        # Check X-axis structural joints (should be ~28.6mm, NOT widened)
        x_slot = find_slots(sg, slot_axis=0, scan_axis=2, third_axis=1)
        if x_slot:
            # Should be close to original (~28.6mm), NOT widened to 32.6mm
            if x_slot["span"] > 30.0:
                print(f"  X-joint (structural): span={x_slot['span']:.1f}mm *** BROKEN - WAS WIDENED! ***")
                ok = False
            else:
                print(f"  X-joint (structural): span={x_slot['span']:.1f}mm [PRESERVED]")

    # Volume sanity check
    if modified:
        vol_pct = (orig.volume - sg.volume) / orig.volume * 100
        # MotorConnection flanges are 4mm thick; shaving 2mm from each reduces volume by ~36%
        max_vol_pct = 40 if name == "MotorConnection_x4.stl" else 20
        if vol_pct > max_vol_pct:
            print(f"  Volume loss: {vol_pct:.1f}% *** TOO MUCH MATERIAL REMOVED! ***")
            ok = False
        else:
            print(f"  Volume loss: {vol_pct:.1f}% [reasonable]")

    return ok


def main():
    print()
    print("#" * 70)
    print("#  SG90 STL Verification — All files from assembly_3d.html")
    print("#" * 70)
    print()

    all_ok = True
    results = {}

    for name in HTML_FILES:
        print("=" * 60)
        print(f"  {name}")
        print("=" * 60)
        ok = check_file(name)
        results[name] = ok
        if not ok:
            all_ok = False
        print()

    # Summary
    print("=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    for name, ok in results.items():
        status = "PASS" if ok else "*** FAIL ***"
        print(f"  {name:35s} [{status}]")

    print()
    if all_ok:
        print("  ALL FILES OK!")
    else:
        failed = [n for n, ok in results.items() if not ok]
        print(f"  FAILED: {', '.join(failed)}")
    print()


if __name__ == "__main__":
    main()
