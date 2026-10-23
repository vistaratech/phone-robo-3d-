"""Detailed measurement of servo-critical parts.

For each part: fine cross-section slices; report every distinct closed inner
contour (interior rings) with bbox, area, circularity, and depth span, plus the
outer silhouette dims at the horn side. Prints a condensed table.
"""
import glob, os
import numpy as np
import trimesh
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(HERE, "stl")
TARGETS = ["MotorConnection_x4.stl", "Foot_x2.stl", "HipBracket_x2.stl", "TorsoBase_x1.stl",
           "UpperLegLink_x2.stl", "LowerLegLink_x2.stl", "IdlerServoBracket_x4.stl",
           "PelvisTop_x1.stl", "PelvisBottom_x1.stl"]


def circ(p):
    p = p.buffer(0)
    if p.is_empty or p.area <= 0:
        return 0
    return 4 * np.pi * p.area / p.length ** 2


def rings_at(m, ax, c, step_orientation=None):
    o = np.zeros(3); o[ax] = c
    normal = np.zeros(3); normal[ax] = 1
    try:
        sec = m.section(plane_origin=o, plane_normal=normal)
    except Exception:
        return []
    if sec is None:
        return []
    p2, T = sec.to_2D()
    res = []
    for poly in p2.polygons_full:
        for ring in poly.interiors:
            r = Polygon(ring)
            if r.is_empty or r.area <= 0:
                continue
            x0, y0, x1, y1 = r.bounds
            res.append((x1 - x0, y1 - y0, r.area, circ(r)))
    return res


for name in TARGETS:
    f = os.path.join(STL_DIR, name)
    m = trimesh.load(f, force="mesh")
    lo, hi = m.bounds
    ext = m.extents
    print("=" * 72)
    print(name, "bbox: %.1f x %.1f x %.1f  (bounds %.1f..%.1f, %.1f..%.1f, %.1f..%.1f)" %
          (ext[0], ext[1], ext[2], lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    # coarse map: for each slice axis, which cuts contain big interior rings
    for ax in range(3):
        axname = "xyz"[ax]
        best = {}
        for c in np.arange(lo[ax] + 1.0, hi[ax] - 0.5, 1.0):
            for du, dv, area, cf in rings_at(m, ax, c):
                if min(du, dv) < 4.0:
                    continue
                key = (round(du, 1), round(dv, 1), round(cf, 2))
                rec = best.setdefault(key, [0, 1e9, -1e9])
                rec[0] += 1
                rec[1] = min(rec[1], c)
                rec[2] = max(rec[2], c)
        for (du, dv, cf), (n, c0, c1) in sorted(best.items(), key=lambda kv: (kv[0][0] * kv[0][1]), reverse=True)[:6]:
            print(f"  ring[{axname}] {du:6.1f} x {dv:6.1f}  circ~{cf:4.2f}  span {c0:7.1f}..{c1:7.1f} (d={c1-c0:5.1f}) n={n}")
