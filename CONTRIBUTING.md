# Contributing to espcon

Thank you for your interest in contributing! This guide covers everything you need to build, wire, and extend the project.

## Hardware Requirements

### Bill of Materials

| Component | Specification | Notes |
|-----------|---------------|-------|
| MCU board | ESP32-S3-DevKitC-1 (or compatible) | 16 MB flash + 8 MB PSRAM variant recommended |
| Display | ST7735 (1.44" 128×128) **or** ILI9341 (2.4" 320×240) | Select at compile time |
| Buttons | Momentary tactile switches × 2 | Any standard 6×6 mm push button |
| Joystick | Analog thumbstick module (KY-023 or equivalent) | Outputs two analog voltages |
| Wires | Breadboard jumper wires | |
| Breadboard | Half-size or larger | |

### Wiring Diagram

```
ESP32-S3 DevKitC-1
+------------------+
|                  |
|  3V3 o-----------+-------> VCC (Display, Joystick)
|  GND o-----------+-------> GND (Display, Joystick, Buttons)
|                  |
|  IO10 o----[CS]-----------> Display CS  (Chip Select)
|  IO9  o----[DC]-----------> Display DC  (Data/Command)
|  IO8  o----[RST]----------> Display RST (Reset)
|  IO11 o----[MOSI]---------> Display SDA/MOSI
|  IO12 o----[SCK]----------> Display SCL/SCK
|                  |
|  IO1  o---[BTN1]---GND     Button 1 (move right)
|  IO2  o---[BTN2]---GND     Button 2 (move left)
|                  |         (pins configured INPUT_PULLDOWN;
|                  |          button connects pin to 3V3 when pressed)
|                  |
|  A3   o----[VRy]----------> Joystick Y-axis
|  A4   o----[VRx]----------> Joystick X-axis
|                  |
+------------------+

Button wiring detail:
  IO1 ---[switch]--- 3V3
  IO2 ---[switch]--- 3V3
  (internal pulldown resistor holds pin LOW when switch is open)

Joystick wiring detail:
  VCC  -> 3V3
  GND  -> GND
  VRx  -> A4  (X-axis, horizontal)
  VRy  -> A3  (Y-axis, vertical)
  SW   -> not connected (joystick button unused)
```

---

## PlatformIO Setup

1. **Install PlatformIO**

   Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) for VS Code, or install the CLI:

   ```sh
   pip install platformio
   ```

2. **Clone the repository**

   ```sh
   git clone https://github.com/mikolajlubiak/espcon
   cd espcon
   ```

3. **Upload the filesystem** (required — ships the default `ship.obj` mesh)

   ```sh
   pio run --target uploadfs
   ```

4. **Build and flash**

   For ST7735 display:
   ```sh
   pio run --environment esp32-s3-devkitc-1 --target upload -- -D BUILD_ST7735
   ```

   For ILI9341 display (default):
   ```sh
   pio run --environment esp32-s3-devkitc-1 --target upload
   ```

5. **Open serial monitor** (optional, 9600 baud)

   ```sh
   pio device monitor --baud 9600
   ```

   Enable the `DEBUG` build flag to print camera position to the serial console:
   ```ini
   build_flags = ... -D DEBUG
   ```

---

## How to Add New Games or Scenes

### Loading a Different Mesh

1. Export your 3D model as a Wavefront `.obj` file from Blender, Maya, or any 3D editor. Use triangulated faces (quads are not supported by the parser).

2. Copy the `.obj` file into `data/littlefs/`:

   ```
   data/
   └── littlefs/
       ├── ship.obj       ← default
       └── mymodel.obj    ← your new mesh
   ```

3. In `include/espcon.h`, update the `loadObj` path in `ESPCon::setup()`:

   ```cpp
   mMesh = loadObj("/littlefs/mymodel.obj");
   ```

4. Re-upload the filesystem:

   ```sh
   pio run --target uploadfs
   ```

5. Rebuild and flash.

### Changing the Camera Starting Position

In `include/espcon.h`, find the `camera` member initialization and adjust the starting position:

```cpp
Camera camera = Camera(vec3(0.0f, 0.0f, 16.0f));
//                          x      y      z (distance from origin)
```

### Adding a New Scene Class

The cleanest way to add a new scene is to follow the same structure as `ESPCon`:

1. Create `include/myscene.h` with a class that exposes `uint8_t setup()` and `void loop()`.
2. In `src/main.cpp`, instantiate your scene and delegate `setup()` / `loop()`:

   ```cpp
   #include "myscene.h"
   MyScene scene{};
   void setup() { scene.setup(); }
   void loop()  { scene.loop();  }
   ```

3. Your scene has access to all the math primitives in `custom_math.h` and the `Camera` class in `camera.h`.

### Adjusting Rendering Parameters

All key parameters are `constexpr` values in `include/espcon.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fps` | `60` | Target frame rate |
| `tftCS` / `tftDC` / `tftRST` | `10 / 9 / 8` | SPI pin assignments |
| `buttonsNum` / `buttons[]` | `2`, `{1, 2}` | Button count and pins |
| `light_direction` | `{-1,-1,-1}` | Directional light vector |
| `color::ambient` | `0.1f` | Ambient lighting floor |
| Camera FOV | `45.0f` (ZOOM) | Field of view in degrees |
| Near/far clip | `0.1f / 100.0f` | Frustum clip planes |

---

## Code Style

- **Language**: C++17 (enforced by PlatformIO / GCC flags)
- **Naming**:
  - Types and classes: `PascalCase` (e.g. `ESPCon`, `Camera`, `vec3`)
  - Functions and methods: `camelCase` or `snake_case` as used in the file (e.g. `loadObj`, `compareTris`, `ProcessKeyboard`)
  - Constants: `camelCase` for `constexpr` values (e.g. `tftCS`, `buttonsNum`, `fps`)
  - Member variables: plain `camelCase` (e.g. `deltaTime`, `numTrisToRaster`)
- **Formatting**: 4-space indentation, K&R-style braces
- **Headers**: Use `#pragma once` (not include guards)
- **Memory**: Prefer `calloc`/`realloc`/`free` for heap-allocated mesh data (matches the embedded C style of the surrounding code). Use RAII constructors/destructors for class-owned resources.
- **Comments**: Match the existing style — short inline comments for non-obvious math, no comments for self-explanatory code
- **Build flags**: Add new compile-time options as `-D FLAG` in `platformio.ini` rather than hardcoded values
