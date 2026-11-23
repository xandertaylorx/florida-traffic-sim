# Florida Traffic Sim

A top-down 2D traffic simulator: a Florida-style grid of signalised intersections with 6-lane roads, car parks, and thousands of cars that all follow the same rules. No sound, no dependencies — open `index.html` in a browser.

## What's in it
- **Shared rules, no scripting:** IDM car-following, traffic lights (protected lefts, actuated), junction path reservation, "don't block the box", right-turn-on-red, lane choice for the next turn, safe merging out of car parks.
- **Trips:** every car has a random car park, parking bay and a planned route; it parks for a while, then picks a new trip.
- **Aggressive drivers:** ~9% of drivers, each with a random aggression level (smaller gaps, pushier lane changes, late yellows).
- **Physics:** oriented-box collisions, crashes, wrecks that block lanes, cars braking and steering around them, tow trucks.
- **Experiment:** click a car to inspect it, swerve it, brake hard, floor it, run lights, change its aggression, or send it on a new trip.

## Structure
- `sim.js` — the simulation core, with no drawing code. Cars are `{segment, s, v}` along lanes/paths, so the same sim could drive a 3D renderer: only `carPose()` (x, y, heading) is needed.
- `index.html` — canvas renderer and UI.

## Run
Open `index.html` (or serve the folder, e.g. `python3 -m http.server`).

Controls: drag to pan, scroll to zoom, click a car. Arrow keys / WASD control the selected car.
