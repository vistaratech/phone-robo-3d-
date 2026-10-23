"""Detect open (U-shaped) servo cradles: deep concave notches in cross-sections
plus large axis-aligned pocket-floor faces."""
import glob, os
import numpy as np
import trimesh
from shapely.geometry import Polygon, MultiPolygon

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(HERE, "stl")


def notch_report(path, step=2.0, min_area=60.0):
    m = trimesh.load(path, force="mesh")
    lo, hi = m.bounds
    ext = m.extents
    out = {}
    for ax in range(3):
        axname = "xyz"[ax]
        normal = np.zeros(3); normal[ax] = 1
        for c in np.arange(lo[ax] + step, hi[ax] - step / 2, step):
            o = np.zeros(3); o[ax] = c
            try:
                sec = m.section(plane_origin=o, plane_normal=normal)
            except Exception:
                continue
            if sec is None:
                continue
            p2, _ = sec.to_2D()
            for poly in p2.polygons_full:
                hull = poly.convex_hull
                diff = hull.difference(poly)
                geoms = diff.geoms if isinstance(diff, MultiPolygon) else [diff]
                for g in geoms:
                    if g.is_empty or g.area < min_area:
                        continue
                    x0, y0, x1, y1 = g.bounds
                    du, dv = x1 - x0, y1 - y0
                    if min(du, dv) < 8.0:
                        continue
                    key = (axname, round(du, 1), round(dv, 1))
                    rec = out.setdefault(key, {"n": 0, "cuts": []})
                    rec["n"] += 1
                    rec["cuts"].append(float(c))
    return ext, out


for f in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
    ext, found = notch_report(f)
    rows = []
    for (axname, du, dv), rec in sorted(found.items(), key=lambda kv: -kv[1]["n"]):
        if rec["n"] < 2:
            continue
        zmin, zmax = min(rec["cuts"]), max(rec["cuts"])
        rows.append(f"  notch [{axname}] ~{du:.1f} x {dv:.1f} mm  span {zmin:.1f}..{zmax:.1f} (depth {zmax-zmin+2.0:.1f})  n={rec['n']}")
    if rows:
        print("=" * 70)
        print(os.path.basename(f), " ext mm: %.1f x %.1f x %.1f" % tuple(ext))
        print("\n".join(rows))
