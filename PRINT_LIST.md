# PhoneBot - custom printed parts (extracted from PhoneBot_Asm.step)

Source: the PhoneBot assembly CAD (Onshape export) from phonebot.dev. Each part below was exported from that
assembly as-is (millimetres, part's own CAD frame). Quantities = how many times the part appears in the assembly.
Left/right parts are identical (no mirrored instances), so no mirrored STL is needed.

| File | Qty | Size mm (x, y, z) | Solid volume |
|---|---|---|---|
| PelvisTop_x1.stl | 1 | 135.7 x 17.4 x 63.3 | 20.8 cm3 |
| PelvisBottom_x1.stl | 1 | 135.7 x 14.0 x 54.7 | 15.6 cm3 |
| TorsoBase_x1.stl | 1 | 106.0 x 43.6 x 66.0 | 58.0 cm3 |
| ElectronicsFrame_x1.stl | 1 | 91.0 x 5.0 x 49.8 | 7.6 cm3 |
| PhoneMount_x1.stl | 1 | 106.0 x 19.1 x 112.5 | 65.9 cm3 |
| HipBracket_x2.stl | 2 | 54.0 x 79.2 x 83.1 | 33.7 cm3 |
| UpperLegLink_x2.stl | 2 | 46.0 x 54.2 x 80.6 | 32.7 cm3 |
| LowerLegLink_x2.stl | 2 | 46.0 x 31.0 x 92.5 | 22.6 cm3 |
| Foot_x2.stl | 2 | 60.0 x 35.2 x 110.0 | 59.4 cm3 |
| IdlerServoBracket_x4.stl | 4 | 37.5 x 46.0 x 10.8 | 6.2 cm3 |
| MotorConnection_x4.stl | 4 | 38.1 x 46.2 x 36.6 | 14.6 cm3 |

Total: 20 printed pieces, about 550 cm3 of solid material (real filament use depends on infill/walls).
`step/` holds the same parts as STEP in case your print service prefers it.

## Checked
- Every STL is a closed (watertight) mesh and its volume matches the CAD volume.
- Two parts (Foot, IdlerServoBracket) had tiny zero-volume tessellation fragments; they were removed.

## NOT decided by this file - please check before printing
- **Orientation, material, layer height, infill, tolerances:** the assembly does not say. Parts are in their CAD frame, not laid
  flat on a print bed - orient them in your slicer. The site's own "Printable Parts" package / Assembly Tutorial may carry the
  authors' recommended settings; prefer those where available.
- **Heat-set inserts / screws:** the assembly lists M3 and M2.5 heat inserts and M2/M2.5 screws, so print holes as designed
  (do not scale parts) and buy the matching hardware.
- **"Part 29"** (a thin 42 x 1.7 x 42 mm plate, ~0.5 cm3) is in the assembly but was not exported - it may be a spacer/plate that is
  not 3D-printed. Check the Assembly Tutorial. Parts 1-28 are ~5 mm pieces (not printed parts).
- Fit and strength were not tested.

## License / credit
The design belongs to the PhoneBot authors and is shared under CC BY-NC 4.0 (non-commercial; give credit, link the license,
note changes). Printing your own copy through a print service for personal use is fine; selling the robot or parts needs the
authors' permission. Source: https://phonebot.dev/
