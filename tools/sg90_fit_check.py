"""SG90 Servo Fit Check -- measure every servo-related pocket / cradle in the
PhoneBot STL parts and compare against standard SG90 micro-servo dimensions.

Outputs a per-part report showing pocket dims vs SG90 dims, clearance (gap),
and a PASS / TIGHT / FAIL verdict.

SG90 datasheet dimensions (mm):
  Body:            22.2 (L) x 11.8 (W) x 22.7 (H, excluding output shaft)
  With tabs:       32.2 (L) x 11.8 (W)  -- mounting-ear span
  Tab thickness:   ~2.5 mm
  Output shaft dia: ~4.8 mm  (nub on top)
  Shaft height above body: ~4.5 mm (shaft + gear nub)
  Mounting holes:  M2, ~28 mm apart centre-to-centre
  Cable exit:      bottom centre
"""

import glob, os, sys, json
from collections import defaultdict
import numpy as np
import trimesh
from shapely.geometry import Polygon, MultiPolygon

# --- SG90 Reference Dims (mm) -----------------------------------------------
SG90 = {
    "body_L":       22.2,   # body length (no tabs)
    "body_W":       11.8,   # body width
    "body_H":       22.7,   # body height (bottom to top of case, no shaft)
    "tab_span":     32.2,   # ear-to-ear span
    "tab_thick":    2.5,    # tab (mounting ear) thickness
    "shaft_dia":    4.8,    # output shaft diameter
    "shaft_height": 4.5,    # shaft + gear nub above body top
    "screw_spacing": 28.0,  # M2 hole centre-to-centre
}

# Tolerances for verdict
TOL_PASS  = 0.3   # >= 0.3 mm clearance each side -> PASS
TOL_TIGHT = 0.0   # 0..0.3 mm -> TIGHT (printable but snug)
                   # < 0 -> FAIL (interference)

HERE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR  = os.path.join(HERE, "stl")

# Parts most likely to hold servo pockets / cradles
SERVO_PARTS = [
    "MotorConnection_x4.stl",
    "HipBracket_x2.stl",
    "IdlerServoBracket_x4.stl",
    "TorsoBase_x1.stl",
    "PelvisTop_x1.stl",
    "PelvisBottom_x1.stl",
    "UpperLegLink_x2.stl",
    "LowerLegLink_x2.stl",
    "Foot_x2.stl",
]


# --- helpers -----------------------------------------------------------------

def circularity(p):
    """4*pi*A / P^2 -- 1.0 for a circle, ~0.78 for a square."""
    p = p.buffer(0)
    if p.is_empty or p.area <= 0:
        return 0
    return 4 * np.pi * p.area / p.length ** 2


def find_interior_rings(mesh, ax, coord):
    """Return list of (du, dv, area, circ) for interior rings at given slice."""
    origin = np.zeros(3); origin[ax] = coord
    normal = np.zeros(3); normal[ax] = 1
    try:
        sec = mesh.section(plane_origin=origin, plane_normal=normal)
    except Exception:
        return []
    if sec is None:
        return []
    p2, _ = sec.to_2D()
    rings = []
    for poly in p2.polygons_full:
        for ring in poly.interiors:
            r = Polygon(ring)
            if r.is_empty or r.area <= 0:
                continue
            x0, y0, x1, y1 = r.bounds
            rings.append((x1 - x0, y1 - y0, r.area, circularity(r)))
    return rings


def find_notch_voids(mesh, ax, coord, min_area=50):
    """Return (du, dv, area) for convex-hull-minus-silhouette chunks (open cradles)."""
    origin = np.zeros(3); origin[ax] = coord
    normal = np.zeros(3); normal[ax] = 1
    try:
        sec = mesh.section(plane_origin=origin, plane_normal=normal)
    except Exception:
        return []
    if sec is None:
        return []
    p2, _ = sec.to_2D()
    voids = []
    for poly in p2.polygons_full:
        diff = poly.convex_hull.difference(poly)
        geoms = diff.geoms if isinstance(diff, MultiPolygon) else ([diff] if not diff.is_empty else [])
        for g in geoms:
            if g.is_empty or g.area < min_area:
                continue
            x0, y0, x1, y1 = g.bounds
            voids.append((x1 - x0, y1 - y0, g.area))
    return voids


def classify_pocket(measured_dims, ref_dims):
    """Given (width, height) of measured pocket and reference servo body,
    return (clearance_w, clearance_h, verdict)."""
    cw = measured_dims[0] - ref_dims[0]
    ch = measured_dims[1] - ref_dims[1]
    worst = min(cw, ch)
    if worst >= TOL_PASS:
        verdict = "[PASS]"
    elif worst >= TOL_TIGHT:
        verdict = "[TIGHT]"
    else:
        verdict = "[FAIL]"
    return cw, ch, verdict


# --- per-part scan -----------------------------------------------------------

def scan_part(path, step=1.0):
    """Scan an STL for features that look servo-sized.

    Returns a list of dicts:
        { "type": "pocket" | "cradle",
          "axis": "x"|"y"|"z",
          "dims": (du, dv),          # measured opening in mm
          "depth": float,            # measured depth / span in mm
          "match": "body" | "tab" | None,
          "clearance": (cw, ch) | None,
          "verdict": str | None }
    """
    m = trimesh.load(path, force="mesh")
    lo, hi = m.bounds

    # collect: key → record
    pockets = {}   # interior rings  (closed pockets)
    cradles = {}   # convex-hull diff (open U-cradles)

    for ax in range(3):
        axname = "xyz"[ax]
        for c in np.arange(lo[ax] + step, hi[ax] - step / 2, step):
            # -- interior rings --
            for du, dv, area, cf in find_interior_rings(m, ax, c):
                if min(du, dv) < 4.0:
                    continue
                key = ("pocket", axname, round(du, 0), round(dv, 0))
                rec = pockets.setdefault(key, {"n": 0, "cuts": [], "dims": []})
                rec["n"] += 1
                rec["cuts"].append(c)
                rec["dims"].append((du, dv))

            # -- open cradles --
            for du, dv, area in find_notch_voids(m, ax, c, min_area=50):
                if min(du, dv) < 6.0:
                    continue
                key = ("cradle", axname, round(du, 0), round(dv, 0))
                rec = cradles.setdefault(key, {"n": 0, "cuts": [], "dims": []})
                rec["n"] += 1
                rec["cuts"].append(c)
                rec["dims"].append((du, dv))

    results = []

    def _process(store, ftype):
        for (_, axname, _, _), rec in sorted(store.items(), key=lambda kv: -kv[1]["n"]):
            if rec["n"] < 2:
                continue
            dims_arr = np.array(rec["dims"])
            med_du = float(np.median(dims_arr[:, 0]))
            med_dv = float(np.median(dims_arr[:, 1]))
            depth = max(rec["cuts"]) - min(rec["cuts"]) + step

            # try to match against SG90 dims
            match = None
            clearance = None
            verdict = None

            # body match: ~22.2 × 11.8  (either orientation)
            for ref_label, ref_pair in [("body", (SG90["body_L"], SG90["body_W"])),
                                         ("tab",  (SG90["tab_span"], SG90["body_W"]))]:
                for md, rd in [((med_du, med_dv), ref_pair),
                               ((med_dv, med_du), ref_pair)]:
                    cw, ch, v = classify_pocket(md, rd)
                    # must be roughly close (within ±5 mm) to count as a match
                    if abs(cw) < 5 and abs(ch) < 5:
                        if match is None or min(cw, ch) > (clearance[0] if clearance else -999):
                            match = ref_label
                            clearance = (round(cw, 2), round(ch, 2))
                            verdict = v

            results.append({
                "type": ftype,
                "axis": axname,
                "dims": (round(med_du, 1), round(med_dv, 1)),
                "depth": round(depth, 1),
                "n_slices": rec["n"],
                "match": match,
                "clearance": clearance,
                "verdict": verdict,
            })

    _process(pockets, "pocket")
    _process(cradles, "cradle")
    return results


# --- main --------------------------------------------------------------------

def main():
    print("=" * 78)
    print("  SG90 SERVO FIT CHECK -- PhoneBot Parts")
    print("=" * 78)
    print()
    print("SG90 reference dimensions:")
    for k, v in SG90.items():
        print(f"  {k:>15s}: {v:6.1f} mm")
    print()
    print(f"  Clearance thresholds:  PASS >= {TOL_PASS} mm  |  TIGHT >= {TOL_TIGHT} mm  |  FAIL < {TOL_TIGHT} mm")
    print()

    all_results = {}
    servo_features_found = 0

    for name in SERVO_PARTS:
        path = os.path.join(STL_DIR, name)
        if not os.path.isfile(path):
            continue
        print("-" * 78)
        print(f"  {name}")
        print("-" * 78)

        results = scan_part(path)
        all_results[name] = results

        if not results:
            print("    (no servo-sized features detected)\n")
            continue

        for r in results:
            tag = r["type"].upper()
            print(f"    [{tag:6s}] axis={r['axis']}  measured={r['dims'][0]:.1f} x {r['dims'][1]:.1f} mm"
                  f"  depth={r['depth']:.1f} mm  (slices={r['n_slices']})")
            if r["match"]:
                servo_features_found += 1
                cw, ch = r["clearance"]
                print(f"             -> matches SG90 {r['match']}"
                      f"  clearance: dW={cw:+.1f} mm, dH={ch:+.1f} mm  {r['verdict']}")
            else:
                print(f"             -> no SG90 dimension match (not a servo feature)")
        print()

    # -- summary --
    print("=" * 78)
    print("  SUMMARY")
    print("=" * 78)
    if servo_features_found == 0:
        print("  [!] No features closely matching SG90 body/tab dims were found.")
        print("      The design may use a different servo (e.g. SG92R, MG90S, or custom).")
        print("      Double-check the PhoneBot BOM at https://phonebot.dev/")
    else:
        print(f"  Found {servo_features_found} servo-matching feature(s):\n")
        for name, results in all_results.items():
            for r in results:
                if r["match"]:
                    print(f"    {name:30s}  {r['type']:6s}[{r['axis']}]"
                          f"  {r['dims'][0]:5.1f}x{r['dims'][1]:5.1f}  depth {r['depth']:5.1f}"
                          f"  match={r['match']:4s}  {r['verdict']}")
    print()


if __name__ == "__main__":
    main()
