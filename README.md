# Tadpole EPAC trike

One-off build model of a tadpole (two wheels front, one rear) pedal-assist trike. Not certified or legal. Many numbers are estimates (see `params.yaml`, `PLAN.md`).

## Run the 3D page (view, drive, test edge cases)

```sh
cd web
npm install                    # first time only: installs three.js
python3 -m http.server 8765    # keep it running
```

Open <http://localhost:8765/index.html> in a browser. Direct links:

- `?pov` starts in drive mode.
- `?pov&cam=1` chase camera, `?pov&cam=2` bystander camera.

### Drive mode

Click **POV: drive it** (or open `?pov`).

| Key | Action |
|-----|--------|
| W | pedal (rider 120 W + 250 W motor, assist stops at 25 km/h) |
| S / Space | brake / hard brake |
| A / D | steer left / right |
| mouse drag, wheel | look, zoom |
| C | camera: rider eyes, chase, bystander |
| T | show/hide the test-conditions panel |
| R | reset the trike |
| Esc | exit drive mode |

The test-conditions panel changes slope, wind, side slope, surface (dry to ice), brakes lost, rider and cargo mass, cargo height, motor on/off, and has events (gust, pothole, launch at speed) and ready scenarios (wet emergency stop, downhill with brakes lost, fast corner, cargo up high, and more). The HUD shows speed, side g, slip, a tip ratio (1 = inner wheel lifts) and warnings (sliding, wheels locked, tipped over).

The driving model is a simple dynamic one: mass and CG from rider and cargo, tyre grip limit, axle load shift, rear-wheel drive, rolling and air drag. CG, grip and drag values are estimates, set at the top of `drive()` in `web/index.html`.

## Rebuild the model

The CAD model is code in `trike/`. Run from the project root with `.venv/bin/python`. `trike/webexport.py` writes `web/model.glb` and `web/manifest.json`. See `PLAN.md`.
