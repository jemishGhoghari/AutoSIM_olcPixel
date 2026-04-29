# AutoSIM — Autonomous Driving Simulator

A 2D car simulator built with [olcPixelGameEngine](https://github.com/OneLoneCoder/olcPixelGameEngine). Drive a car around the screen using the arrow keys. Configuration (window size, starting position, car sprite) is loaded from a YAML file.

![Demo Screenshot](assets/screenshot.png)
<!-- Replace with an actual screenshot of the simulator running -->

---

## Features

- Arrow key controls (forward, backward, left, right)
- Car sprite rendered with GPU acceleration via olcPixelGameEngine Decals
- Screen wrapping — driving off one edge brings you back on the opposite side
- Configuration loaded from a YAML file (no recompile needed to change settings)
- Built-in logger with INFO / WARNING / ERROR levels

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
