# AutoSIM — Autonomous Driving Simulator

A 2D autonomous driving simulator with an optional lightweight 3D preview, built with [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine). Drive a car around the screen using the arrow keys or WASD. Configuration (window size, starting position, car sprite, scenario obstacles, sensors, and 3D preview camera) is loaded from YAML files so users can author their own scenarios without recompiling.

![Demo Screenshot](assets/screenshot.png)
<!-- Replace with an actual screenshot of the simulator running -->

---

## Product Focus

Robotics engineers repeatedly report that simulation is useful for reducing slow, risky hardware iteration, but that two issues limit its value: the sim-to-real gap and the difficulty of automating realistic sensor/perception tests. AutoSIM is still a lightweight 2D simulator, so it cannot replace Gazebo, Isaac Sim, MuJoCo, or hardware-in-the-loop validation. It can, however, add practical value as a fast smoke-test sandbox for navigation logic by making obstacle layouts, noisy range readings, and sensor dropouts configurable and repeatable.

## Features

- Arrow key controls (forward, backward, left, right)
- Car sprite rendered with GPU acceleration via olcPixelGameEngine Decals
- Screen wrapping — driving off one edge brings you back on the opposite side
- Configuration loaded from a YAML file (no recompile needed to change settings)
- Built-in logger with INFO / WARNING / ERROR levels
- Robotics scenario overlay with configurable rectangular obstacles
- Deterministic 2D range sensor simulator with field-of-view, ray count, max range, Gaussian noise, dropout rate, and seed controls
- Lightweight 3D preview mode with AutoSIM-owned projection math, wireframe boxes, a procedural 3D vehicle model, movable third-person follow camera, and deterministic 3D range scans
- User-authored YAML scenarios for 2D rectangles, 3D boxes, camera settings, and sensor behavior
- In-window GUI/status overlay with discoverable runtime controls

---

## Requirements

- Linux (tested on Ubuntu)
- CMake ≥ 3.8
- C++17 compiler (GCC or Clang)
- The following system libraries:

```bash
sudo apt-get install libx11-dev libgl1-mesa-dev libpng-dev libyaml-cpp-dev
```

---

## Build

```bash
# Clone the repo (with submodules — olcPixelGameEngine is a submodule)
git clone --recurse-submodules https://github.com/jemishGhoghari/AutoSIM_olcPixel.git
cd AutoSIM_olcPixel

# Configure and build
cmake -S . -B build
cmake --build build
```

The compiled binary will be at `build/autonomous_car`.

---

## Run

```bash
# Run with the default config
./build/autonomous_car

# Or pass a custom config file
./build/autonomous_car path/to/your_config.yaml
```

If no config path is given, it defaults to `../configs/initializeGame.yaml`.

---

## 3D Rendering Support

The upstream olcPixelGameEngine project is suitable as AutoSIM's lightweight window/input/pixel-rendering layer, and its README lists a "3D Software renderer" as an extension example. However, the olcPGE wiki also states that the base engine intentionally does not provide typical game resources such as collision detection, vector math, or asset management. AutoSIM therefore treats olcPGE as the renderer/input shell and owns the 3D simulator math, scenario schema, sensors, and validation code in this repository. See `docs/3d_simulator_plan.md` for the verification notes and roadmap.

## Configuration

Edit `configs/initializeGame.yaml` or pass your own YAML file to change the startup settings:

```yaml
scenario_name: "warehouse-smoke-test"
simulator_mode: "2d"               # "2d" or "3d_preview"; F2 toggles at runtime
car_png: "../assets/car.png"       # Path to the car sprite
window_size: "1980x1080"           # Window width x height
initial_position: "100x100"        # Starting X x Y position of the car

robotics_overlay: true              # F1 toggles obstacles/range rays
gui_overlay: true                   # TAB toggles the status/help panel

obstacles:                          # 2D x/y/width/height rectangles in pixels
  - "460x220x160x90"
  - "820x520x130x220"

obstacles_3d:                       # 3D x/y/z/width/depth/height boxes
  - "-420x0x360x160x90x110"
  - "-40x0x620x130x220x160"

camera_3d:
  position: "0x180x-520"       # Focal/startup fallback; 3D mode follows the car
  yaw_degrees: 0
  pitch_degrees: -12
  focal_length: 520

third_person_camera:
  distance: 360                 # Mouse wheel zooms the 3D follow camera at runtime
  height: 145                   # Left-mouse drag vertically raises/lowers camera
  orbit_degrees: 0              # Left-mouse drag horizontally orbits; R resets
  min_distance: 140
  max_distance: 900
  min_height: 45
  max_height: 420

vehicle_model_3d:                # local_x/local_y/local_z/width/depth/height parts
  - "0x18x0x70x120x26"          # chassis
  - "0x42x-8x46x58x28"         # cabin
  - "-42x12x-38x16x24x24"      # rear-left wheel
  - "42x12x-38x16x24x24"       # rear-right wheel
  - "-42x12x38x16x24x24"       # front-left wheel
  - "42x12x38x16x24x24"        # front-right wheel
  - "0x28x72x30x18x16"         # front marker

range_sensor:
  rays: 41                          # Number of simulated range beams
  fov_degrees: 180                  # Sensor field of view
  max_range: 420                    # Maximum beam distance in pixels/units
  noise_stddev: 2.5                 # Gaussian range noise
  dropout_rate: 0.03                # Probability a reading returns max range
  random_seed: 42                   # Repeatable noise/dropout sequence
```

---

## Controls

| Input      | Action          |
|------------|-----------------|
| ↑ Up / W   | Accelerate      |
| ↓ Down / S | Reverse         |
| ← Left / A | Rotate left     |
| → Right / D| Rotate right    |
| F1         | Toggle robotics/range overlay |
| F2         | Toggle 2D / 3D preview mode |
| Tab        | Toggle GUI/status panel |
| Left mouse drag | Orbit the 3D third-person camera left/right and raise/lower it |
| Mouse wheel | Zoom the 3D third-person camera in/out |
| R          | Reset the 3D third-person camera |

---

## Robotics Engineer Mode

The robotics overlay is designed for early navigation and perception smoke tests:

1. Add obstacle rectangles to `configs/initializeGame.yaml`.
2. Tune the range sensor model to mimic imperfect low-cost lidar, sonar, IR, or depth preprocessing.
3. Add `obstacles_3d` boxes and set `simulator_mode: "3d_preview"` when you want a projected 3D smoke-test view.
4. Tune `third_person_camera` and `vehicle_model_3d` to inspect the procedural vehicle from a movable chase-camera view.
5. Use the deterministic `random_seed` to reproduce a failure case, then increase `noise_stddev` or `dropout_rate` to check robustness.

This feature intentionally solves a small pain point: quick, repeatable testing of obstacle-avoidance assumptions before spending time in a heavier simulator or on physical hardware. The 3D preview is a reliable visualization and range-sensor smoke-test layer, but it does **not** claim physical fidelity, wheel slip, 3D contact dynamics, ROS integration, textured model loading, or camera-realistic synthetic data.

---

## Project Structure

```
AutoSIM_olcPixel/
├── assets/                  # Car sprite (car.png)
├── configs/                 # YAML config file
├── include/                 # Header files
│   ├── auto_vehicle.h       # Main vehicle class
│   ├── logger.h             # Logger
│   ├── simulator_3d.h       # 3D projection, box, and range sensor helpers
│   ├── vehicle.h            # Base vehicle class
│   └── utils.h              # Utility functions
├── src/                     # Source files
│   ├── auto_vehicle.cpp     # Vehicle logic + main()
│   ├── logger.cpp           # Logger implementation
│   └── vehicle.cpp          # Base vehicle implementation
├── libraries/
│   └── olcPixelGameEngine/  # olcPGE submodule
├── docs/                    # Design notes and roadmap
├── tests/                   # Unit tests
└── CMakeLists.txt
```
