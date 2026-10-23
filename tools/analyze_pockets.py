"""Detect internal pockets (holes) in STL parts by cross-section analysis.

For each STL in stl/, slice along each axis and collect interior rings
(hole contours inside the outer silhouette) that are big enough to matter
(> 10 mm in both in-plane directions / > 8 mm thickness span).
"""
import glob, os, sys
import numpy as np
import trimesh
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(HERE, "stl")

AXES = {"z": (2, 0, 1), "y": (1, 0, 2), "x": (0, 1, 2)}  # slice-axis, u, v


def analyze(path, step=2.0):
    m = trimesh.load(path, force="mesh")
    ext = m.extents
    lo, hi = m.bounds
    found = {}  # key -> record
    for axname, ax in (("z", 2), ("y", 1), ("x", 0)):
        cuts = np.arange(lo[ax] + step, hi[ax] - step / 2, step)
        normal = np.zeros(3)
        normal[ax] = 1
        for c in cuts:
            try:
                sec = m.section(plane_origin=np.where(np.arange(3) == ax, c, 0.0), plane_normal=normal)
            except Exception:
                continue
            if sec is None:
                continue
            p2, _ = sec.to_2D()
            for poly in p2.polygons_full:
                for ring in poly.interiors:
                    r = Polygon(ring)
                    if r.is_empty or r.area <= 0:
                        continue
                    x0, y0, x1, y1 = r.bounds
                    du, dv = x1 - x0, y1 - y0
                    if min(du, dv) < 10.0:
                        continue
                    # ring centre in slice coords -> back to 3D
                    cen = r.centroid
                    p3 = np.where(np.arange(3) == ax, c, 0.0)
                    p3 = p3.astype(float)
                    # put centroid into the two other axes
                    others = [i for i in range(3) if i != ax]
                    # 2D coords lost orientation; use mesh.projected? approximate via ring 3D loopverts
                    key = (axname, round(du, 1), round(dv, 1))
                    rec = found.setdefault(key, {"n": 0, "cuts": []})
                    rec["n"] += np.int64(1)
                    rec["cuts"].append(float(c))
    return ext, (lo, hi), found


for f in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
    ext, bounds, found = analyze(f)
    print("=" * 70)
    print(os.path.basename(f), " ext mm: %.1f x %.1f x %.1f" % tuple(ext))
    for (axname, du, dv), rec in sorted(found.items(), key=lambda kv: -kv[1]["n"]):
        zmin, zmax = min(rec["cuts"]), max(rec["cuts"])
        print(f"  hole [{axname}] ~{du:.1f} x {dv:.1f} mm  span {zmin:.1f}..{zmax:.1f} (thick {zmax-zmin+2.0:.1f})  nslices={rec['n']}")
