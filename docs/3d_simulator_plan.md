# AutoSIM 3D Simulator Plan

## olcPixelGameEngine 3D support verification

olcPixelGameEngine is a lightweight pixel/game framework, not a full 3D simulation engine. The upstream project describes the engine as a cross-platform pixel drawing and UI framework, and lists a "3D Software renderer" as an example extension. Its wiki also says the base engine does not provide typical game resources such as asset loading, collision detection, or vector mathematics, and that users are expected to provide that functionality. In practice this means AutoSIM can use olcPGE to draw a 3D projection, but reliable robotics simulation needs AutoSIM-owned math, scenario schema, sensors, collision, and validation layers.

Current repository status: the `libraries/olcPixelGameEngine` submodule directory exists, but `olcPixelGameEngine.h` is not present in this checkout. CMake already skips the runnable simulator executable until the submodule is initialized.

## Implementation strategy

1. Keep olcPGE as the window/input/pixel-rendering layer.
2. Add AutoSIM-owned 3D math and sensor primitives that can be unit-tested without a graphical dependency.
3. Render the first 3D simulator milestone as a wireframe preview using olcPGE line primitives.
4. Let users author scenarios in YAML with stable 2D and 3D obstacle schemas.
5. Add a minimal in-window GUI overlay for discoverable controls before adopting a larger GUI toolkit.

## What is implemented now

- `simulator_mode: "2d" | "3d_preview"` selects the mode; F2 toggles at runtime.
- `obstacles_3d` accepts user-authored boxes as `"x×y×z×width×depth×height"`-style x-delimited values without recompiling.
- `camera_3d` sets the preview camera position, yaw, pitch, and focal length.
- 3D range scans use deterministic noise/dropout behavior through the existing `range_sensor` configuration.
- TAB toggles the in-window GUI panel; F1 toggles robotics/range overlays.

## Recommended roadmap

### Milestone 1: reliable 3D preview

- Continue validating 3D projection, ray/box intersection, and YAML parsing with unit tests.
- Add schema documentation and sample scenario files for common warehouses, parking lots, and intersections.
- Add camera controls for pan/orbit/zoom.

### Milestone 2: scenario authoring UX

- Add mouse-based obstacle placement/editing.
- Save edited scenarios back to YAML.
- Validate scenario files before launch and report actionable errors.

### Milestone 3: robotics fidelity

- Add vehicle footprint collision against 3D boxes.
- Add vertical field-of-view for multi-layer lidar/depth-like scans.
- Add sensor recording/replay files for regression tests.

### Milestone 4: full 3D backend decision

If the simulator must support textured 3D models, lighting, cameras, complex physics, or larger worlds, keep the olcPGE preview as a quick smoke-test mode and add a dedicated 3D backend such as OpenGL/OGRE/Filament or integrate with Gazebo/Isaac/MuJoCo for high-fidelity validation.
