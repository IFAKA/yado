# Plan: tadpole EPAC trike, buildable parametric model + web app

Goal: a buildable one-person enclosed pedal-assist recumbent trike model, with BOM, drawings, exploded view, checks, exports, and a web app to play with it.
Not certified or legal. Estimates are flagged. Run from project root with `.venv/bin/python`.

## Criteria (each has a check)
| # | Criterion | Check |
|---|-----------|-------|
| C1 | Model builds from params.yaml | `build_all()` runs, 7 subassemblies |
| C2 | Every part has a fastener, weld or clamp | interference + fastener list review |
| C3 | No unresolved part-part interference | `checks.interference`, each hit triaged (accepted with a reason, or fixed) |
| C4 | Frame FEA at 150 kg x 3 g, ratio < 1, cantilever error reported | `/tmp/t_fea.py` (now 0.935; HAZ 0.6 fails, reported) |
| C5 | Steering sweep + Ackermann, no collision | `checks.steering`, `steer_clearance` (done: 36 deg inner lock, no collision) |
| C6 | Tipping >= 35 deg | `checks.tipping` (18.5 deg: FAILS; report options) |
| C7 | Mass, CG | `checks.mass_cg` (vehicle 57.2 kg) |
| C8 | Brake/chain clearance | `checks.clearances` |
| C9 | BOM csv+xlsx, Compliance sheet, Open issues | files in out/ |
| C10 | PDF drawing per fabricated part | out/drawings/*.pdf, look at samples |
| C11 | Exploded MP4 + stills per subassembly with balloons | out/explode/, look at frames |
| C12 | STEP/STL/DXF/ZIP | out/export/ |
| C13 | Web app: orbit, explode, isolate, part info, rebuild | server runs, screenshot it |

## Gaps (open)
- G1 Interference triage (idler plate vs bellcrank, shock vs bridge, pivot lug vs stay ...). Then fix or accept with reason.
- G2 Tipping 18.5 vs 35 deg. Physics: CG 636 mm high, track 750. Report options with numbers.
- G3 Rear suspension travel check not coded.
- G4 Outputs not written (BOM, assembly sequence, drawings, exports, explode, webapp).

## Steps (atomic)
1. checks.py all functions + run_all -> out/checks.json  [steering, mass, tip, clearance done]
2. Triage interference (G1)
3. Rear travel (G3)
4. outputs: BOM csv/xlsx + Compliance + Open issues + assembly sequence
5. drawings PDF + DXF
6. exports STEP/STL
7. GLB + manifest + webapp + server
8. explode frames + MP4 + stills
9. ZIP, final report

## Learned
- Removing the TR-117/118 stays pushes the frame to ratio 1.24: keep them.
- Stay TR-118 lower end at y=195 hit the steering arm at 3 deg; moved to y=140 (FEA still 0.935).
- Tipping about front-wheel-to-rear-wheel line, track alone cannot reach 35 deg with CG 636 mm.
- Belly pan added (TR-625..633), floor 140 mm; ground_clearance check: 20 mm margin over a 100 mm hump loaded, 78 mm floor height at full bump.
- Legal: see memory spain-epac-legal-constraint; closed body, weight, width are TO CONFIRM.
- Recline sweep (backrest 20..45 deg from vertical, /tmp/t_sweep.py): tipping 19.3..18.9 deg (barely changes), rear-lift decel 1.1 g (safe), roof must reach 1287..1458 mm at 20 deg for 1.90 m rider, overall length set by the rear wheel (tail -1425), not the seat. Seat pan sweep (/tmp/t_pan.py): pan 400/300/200/150 -> tip 18.9/20.1/21.4/22.1 deg. Canopy removal only changes 18.9 -> 18.6. 35 deg needs CG ~307 mm: not reachable with BB 550 + seat. Cornering: 19 deg = 0.34 g sideways = min radius ~14 m at 25 km/h.

## Length rebuild (2026-10-25)
Changes: 16" (ISO 305) rear wheel, wheelbase 1000, boom stroke 100, seat pan 280, frame plane z 255, swing-arm pivot 40 mm behind X2, shock lower mount just ahead of the tyre. Canopy tail/nose, lamps, sensor, floor slot now derive from K (rear axle, nose_x).
Results: length 1906 mm (was 2131), mass 63.2 kg, tipping 18.6 deg (fail vs 35), steering sweep clear to 12.5 deg inner wheel, floor margin 17.9 mm loaded (69 mm at full bump), FEA max ratio 0.736 (HAZ 1.23 fails), wheel travel 129 mm (shock lever 3.4, shock force high: TO CONFIRM), chain 9.6 mm from pivot tube.
Why frame plane went down: 16" axle is 200 high, chain top run then sits at 290, so frame tubes must stay below it or the chain hits X1/X2/pivot.
Floor notch widened to |y|<200 and stringers moved to y=180 so the steered tyre clears the floor.
