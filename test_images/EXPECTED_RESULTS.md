# Real-photo regression set

These three original photos are the fixed real-image regression set for LeatherCAD V0.19. Do not replace them with synthetic images.

| File | Expected markers | Expected physical parts | Key semantic geometry | Python baseline |
|---|---:|---:|---|---|
| 1000048864_beige.jpg | ArUco IDs 1–16 (16/16) | 2 | Two touching/near-touching physical workpieces must be separated; do not fit their contact as a CAD connector. | V0.18: part1 RMS 2.499 mm, P95 4.914, max 7.375; part2 RMS 2.241, P95 4.184, max 6.589 |
| 1000048862_orange.jpg | 16/16 | 1 | One workpiece; retain the genuine oblong slot; suppress quilting/piping texture. | post-median: LINE 14, ARC 12, SLOT 1; RMS 2.685 mm, P95 5.155, max 7.892 |
| 1000048860_gray.jpg | 16/16 | 1 | One workpiece; expected approximately 1 circular hole + 1 oblong slot; suppress binding/texture burrs. | post-median: LINE 8, ARC 11, CIRCLE 1, SLOT 1; RMS 2.404 mm, P95 4.455, max 7.950 |

## Acceptance intent

1. All three photos must detect all 16 ArUco markers.
2. Part count must be beige=2, orange=1, gray=1.
3. Final CAD should visually follow manufactured/design geometry rather than textile edge noise.
4. Do not judge only by unit tests or numeric RMS/P95. Inspect original -> rectified -> mask -> raw contour -> final CAD overlay.
5. Preserve true LINE/ARC/CIRCLE/SLOT semantics where supported; do not convert everything to a polyline.
6. Current practical target is roughly <=10 mm over ~1 m workpieces, but the existing Python baseline is substantially better (~2.2–2.7 mm RMS on these examples), so deployment changes must not materially regress it.
