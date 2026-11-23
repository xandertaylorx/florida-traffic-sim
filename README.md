# Florida Traffic Sim

A traffic simulator with a top-down 2D view **and a 3D view** (toggle with the 2D/3D button): a Florida-style grid of signalised intersections with 6-lane roads, car parks, and thousands of cars that all follow the same rules. No sound, no dependencies — open `index.html` in a browser.

## What's in it
- **Shared rules, no scripting:** IDM car-following, traffic lights (protected lefts, actuated), junction path reservation, "don't block the box", right-turn-on-red, lane choice for the next turn, safe merging out of car parks.
- **Trips:** every car has a random car park, parking bay and a planned route; it parks for a while, then picks a new trip.
- **Aggressive drivers:** ~9% of drivers, each with a random aggression level (smaller gaps, pushier lane changes, late yellows).
- **Physics:** oriented-box collisions, crashes, wrecks that block lanes, cars braking and steering around them, tow trucks.
- **Experiment:** click a car to inspect it, swerve it, brake hard, floor it, run lights, change its aggression, or send it on a new trip.

## Structure
- `sim.js` — the simulation core, with no drawing code. Cars are `{segment, s, v}` along lanes/paths, so the same sim could drive a 3D renderer: only `carPose()` (x, y, heading) is needed.
- `index.html` — 2D canvas renderer and UI.
- `view3d.js` — 3D renderer (three.js, vendored in `vendor/`): mast-arm signals, buildings, palms, cars with lights; orbit / pan / zoom, and a chase cam (select a car, press Follow).

## Run
Open `index.html` (or serve the folder, e.g. `python3 -m http.server`).

Controls: 2D — drag to pan, scroll to zoom. 3D — drag to orbit, right-drag/shift-drag to pan, scroll to zoom. Click a car in either view. Arrow keys / WASD control the selected car.
