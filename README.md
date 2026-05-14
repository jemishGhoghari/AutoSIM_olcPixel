# AutoSIM — Autonomous Driving Simulator

A 2D car simulator built with [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine). Drive a car around the screen using the arrow keys. Configuration (window size, starting position, car sprite) is loaded from a YAML file.

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

## Configuration

Edit `configs/initializeGame.yaml` to change the startup settings:

```yaml
car_png: "../assets/car.png"       # Path to the car sprite
window_size: "1980x1080"           # Window width x height
initial_position: "100x100"        # Starting X x Y position of the car

robotics_overlay: true              # Draw obstacles, range rays, and HUD
obstacles:                          # x/y/width/height rectangles in pixels
  - "460x220x160x90"
  - "820x520x130x220"
range_sensor:
  rays: 41                          # Number of simulated range beams
  fov_degrees: 180                  # Sensor field of view
  max_range: 420                    # Maximum beam distance in pixels
  noise_stddev: 2.5                 # Gaussian range noise in pixels
  dropout_rate: 0.03                # Probability a reading returns max range
  random_seed: 42                   # Repeatable noise/dropout sequence
```

---

## Controls

| Key        | Action          |
|------------|-----------------|
| ↑ Up       | Accelerate      |
| ↓ Down     | Reverse         |
| ← Left     | Rotate left     |
| → Right    | Rotate right    |

---

## Robotics Engineer Mode

The robotics overlay is designed for early navigation and perception smoke tests:

1. Add obstacle rectangles to `configs/initializeGame.yaml`.
2. Tune the range sensor model to mimic imperfect low-cost lidar, sonar, IR, or depth preprocessing.
3. Use the deterministic `random_seed` to reproduce a failure case, then increase `noise_stddev` or `dropout_rate` to check robustness.

This feature intentionally solves a small pain point: quick, repeatable testing of obstacle-avoidance assumptions before spending time in a heavier simulator or on physical hardware. It does **not** claim physical fidelity, wheel slip, 3D contact dynamics, ROS integration, or camera-realistic synthetic data.

---

## Project Structure

```
AutoSIM_olcPixel/
├── assets/                  # Car sprite (car.png)
├── configs/                 # YAML config file
├── include/                 # Header files
│   ├── auto_vehicle.h       # Main vehicle class
│   ├── logger.h             # Logger
│   ├── vehicle.h            # Base vehicle class
│   └── utils.h              # Utility functions
├── src/                     # Source files
│   ├── auto_vehicle.cpp     # Vehicle logic + main()
│   ├── logger.cpp           # Logger implementation
│   └── vehicle.cpp          # Base vehicle implementation
├── libraries/
│   └── olcPixelGameEngine/  # olcPGE submodule
├── tests/                   # Unit tests
└── CMakeLists.txt
```
