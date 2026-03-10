# Architecture Overview

espcon is a minimal, self-contained 3D game console engine for the ESP32-S3. Everything runs on a single 240 MHz CPU core — mesh loading, vertex transformation, rasterization, and display output.

## System Architecture

```
+------------------+     +-------------------+     +--------------------+
|      INPUT       |     |    GAME LOGIC /   |     |    3D RENDERER     |
|                  |     |   ENGINE LOOP     |     |                    |
|  GPIO Buttons    +---->|                   +---->|  Model Transform   |
|  (pin 1, pin 2)  |     |  Camera::Process  |     |  Back-face Cull    |
|                  |     |  Keyboard()       |     |  View Transform    |
|  ADC Joystick    +---->|                   |     |  Projection        |
|  (A3=Y, A4=X)   |     |  Camera::Process  |     |  Perspective Div.  |
|                  |     |  MouseMovement()  |     |  Viewport Scale    |
+------------------+     +-------------------+     +--------------------+
                                                            |
                                                            v
                                              +-----------------------------+
                                              |       DEPTH SORT            |
                                              |   qsort (painter's algo)    |
                                              +-----------------------------+
                                                            |
                                                            v
                                              +-----------------------------+
                                              |     RASTERIZATION           |
                                              |  Adafruit_GFX fillTriangle  |
                                              +-----------------------------+
                                                            |
                                                            v
                                              +-----------------------------+
                                              |     DISPLAY OUTPUT          |
                                              |    SPI (up to ~80 MHz)      |
                                              |  ST7735  or  ILI9341        |
                                              +-----------------------------+
```

---

## Hardware Abstraction

### Buttons (GPIO + INPUT\_PULLDOWN)

Both buttons are connected to GPIO pins 1 and 2 and configured as `INPUT_PULLDOWN` — meaning they read `LOW` at rest and `HIGH` when pressed. The engine polls both pins each frame, packs their states into a single byte with bitwise OR, and maps each bit to a camera movement direction:

```cpp
constexpr uint8_t buttons[buttonsNum] = {1, 2};

// In loop():
buttonsState = buttonsState | (digitalRead(buttons[i]) << i);

if ((buttonsState >> 0) == HIGH) camera.ProcessKeyboard(RIGHT, deltaTime);
if ((buttonsState >> 1) == HIGH) camera.ProcessKeyboard(LEFT, deltaTime);
```

### Joystick (ADC)

The analog joystick outputs two independent voltages on pins A3 (Y-axis) and A4 (X-axis). The ESP32-S3 ADC reads these as 12-bit integers (0–4095). The engine centers the reading around the midpoint (≈2000) and divides to produce a floating-point delta, then applies a deadzone of ±5 units to suppress noise when the stick is at rest:

```
Raw ADC (0–4095)
    → subtract midpoint (4000/2 = 2000)
    → divide by sensitivity (50 for Y, 100 for X)
    → apply deadzone (|value| < 5 → 0)
    → camera.ProcessMouseMovement(x_axis, y_axis)
```

### Display (SPI)

The display is wired on the hardware SPI bus:

| Signal | Pin |
|--------|-----|
| CS     | 10  |
| DC     |  9  |
| RST    |  8  |

The correct driver is selected at compile time:

```cpp
#ifdef BUILD_ST7735
    Adafruit_ST7735 tft = Adafruit_ST7735(tftCS, tftDC, tftRST);
#elif BUILD_ILI9341
    Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCS, tftDC);
#endif
```

ST7735 uses `initR(INITR_144GREENTAB)` and runs the SPI bus at 79.999 MHz (80 MHz produces display artifacts on tested hardware). ILI9341 uses `begin()` with `setRotation(3)` and uses a 1:1 aspect-ratio workaround (the physical display clips output beyond the square region).

---

## Rendering Pipeline

### OBJ Loading from LittleFS

At startup, `loadObj()` opens the mesh file from the LittleFS partition and parses it character-by-character from the `File` stream. The parser is a minimal state machine:

```
file.read() char-by-char
    → accumulate into line buffer (dynamic realloc on overflow)
    → on '\n':
        if line[0] == 'v'  → sscanf → push to verts[]
        if line[0] == 'f'  → sscanf → index into verts[] → push triangle
    → clear line buffer, repeat
```

Both the vertex array and triangle array use `calloc` + `realloc` with doubling growth, so they can handle arbitrary mesh sizes without a compile-time cap. Face indices in OBJ are 1-based; the loader subtracts 1 when indexing into the vertex array.

### Vertex Transformation

Each frame, every triangle in the mesh is transformed through three matrices in sequence:

```
Object Space                  World Space                    View Space
(OBJ coordinates)     *model      (identity)      *view      (camera-relative)
      v  --------->  triTransformed  --------->  triViewed
```

The **model matrix** is currently an identity matrix (`mat4(1.0f)`), so world space equals object space. The **view matrix** is built each frame from the camera's current position and orientation using `point_at()`:

```
point_at(position, position + front, up)
```

This constructs a row-major look-at matrix from the camera's right, up, and forward vectors.

After the view transform, the **projection matrix** is applied:

```
triProjected = triViewed * perspective(fov, aspectRatio, 0.1f, 100.0f)
```

`perspective()` builds a standard frustum projection matrix that maps the view frustum to NDC (Normalized Device Coordinates) in [-1, +1].

### Back-face Culling

Before any expensive transformation, the engine checks whether the triangle faces the camera using a surface normal and a camera-ray dot product:

```cpp
line1  = p[1] - p[0]
line2  = p[2] - p[0]
normal = normalize(cross(line1, line2))
cameraRay = p[0] - cameraPosition

if dot(normal, cameraRay) < 0  →  visible, proceed
                               →  discard (back-facing)
```

This eliminates roughly half of all triangles before they reach the projection stage.

### Lighting

A single directional light points in the direction `{-1, -1, -1}` (normalized). The dot product of the light direction with the triangle surface normal gives a luminance scalar in [0, 1]. This is combined with a configurable ambient floor (0.1) and multiplied against the RGB565 color channels:

```cpp
dp = dot(light_direction, normal);   // [0..1]
c  = col.getColor(dp);               // applies lum to r/g/b with ambient floor
```

Colors themselves cycle each frame using three independent sinusoidal waves with different frequencies, producing a smooth color animation on the mesh.

### Perspective Division and Viewport Transform

After projection, the w component contains the original view-space z. Dividing all components by w performs the perspective divide that creates correct 3D foreshortening:

```cpp
triProjected.p[i] = triProjected.p[i] / triProjected.p[i].w;
```

The resulting NDC coordinates ([-1,+1] in x and y) are then mapped to pixel coordinates:

```
x_pixel = (x_ndc + 1.0) * 0.5 * screenWidth
y_pixel = (y_ndc + 1.0) * 0.5 * screenHeight
```

### Depth Sorting (Painter's Algorithm)

The engine uses the painter's algorithm: all visible triangles are collected into a buffer (`trisToRaster`), sorted back-to-front by their average Z coordinate in view space, and then drawn in that order so that nearer triangles paint over farther ones:

```cpp
qsort(trisToRaster, numTrisToRaster, sizeof(triangle), compareTris);
```

`compareTris` compares `(z[0] + z[1] + z[2]) / 3` for each pair of triangles.

### Rasterization and Frame Buffer → SPI Transfer

There is no off-screen frame buffer. Each triangle is sent directly to the display by calling Adafruit's `fillTriangle()`, which internally generates SPI pixel writes for every pixel inside the triangle boundary. This means each frame is a sequence of SPI transactions that directly stream pixels to the display hardware.

---

## FreeRTOS Usage

The project uses the Arduino framework on top of ESP-IDF, which itself wraps FreeRTOS. The rendering loop runs entirely on the Arduino main task (the `loop()` function). FreeRTOS is leveraged in two ways:

1. **`yield()`** — called after the screen clear and after rasterization when built for ILI9341. This yields the CPU back to the FreeRTOS scheduler, allowing background tasks (Wi-Fi, Bluetooth, watchdog) to run between heavy SPI bursts.

2. **`delay()`** — if the frame finishes faster than the target frame time (60 fps = ~16.7 ms), `delay()` suspends the task for the remaining time, preventing busy-looping.

```cpp
if (frame_delay > deltaTime) {
    delay(frame_delay - deltaTime);
}
```

---

## Display Driver

The display abstraction is fully compile-time. A single preprocessor flag selects the driver:

| Flag            | Driver            | Init call                    | Notes                          |
|-----------------|-------------------|------------------------------|--------------------------------|
| `BUILD_ST7735`  | `Adafruit_ST7735` | `initR(INITR_144GREENTAB)`   | 79.999 MHz SPI (80 MHz = artifacts) |
| `BUILD_ILI9341` | `Adafruit_ILI9341`| `begin()` + `setRotation(3)` | 1:1 aspect ratio workaround    |

Both drivers share the `Adafruit_GFX` interface, so the rendering code only calls `fillTriangle()` and `fillScreen()` — both are available on either driver without `#ifdef` guards in the render loop.

The color format is **RGB565**: 5 bits red, 6 bits green, 5 bits blue, packed into a `uint16_t`. The `color` struct encodes this directly in bitfields:

```cpp
struct color {
    uint8_t r : 5;
    uint8_t g : 6;
    uint8_t b : 5;
    float   ambient = 0.1f;

    uint16_t load() const { return ((r << 11) | (g << 5) | b); }
    uint16_t getColor(const float lum) const { /* applies lighting */ }
};
```

---

## Component Descriptions

| File                    | Role |
|-------------------------|------|
| `src/main.cpp`          | Arduino entry point. Instantiates `ESPCon` and delegates to `setup()` / `loop()`. |
| `include/espcon.h`      | Core engine class. Owns the display, mesh, camera, and rendering loop. Also contains `mesh`, `triangle`, `color` structs; `allocMesh`, `freeMesh`, `initMeshCube`, `loadObj`, `listDir`, `compareTris` free functions. |
| `include/camera.h`      | First-person camera with Euler-angle orientation. Processes keyboard (FORWARD/BACKWARD/LEFT/RIGHT) and mouse-style (yaw/pitch) input. Produces a view matrix via `GetViewMatrix()`. |
| `include/custom_math.h` | 3D math library: `vec3`, `vec4`, `mat4`; operators for addition, subtraction, scalar multiply/divide, matrix-vector multiply; `normalize`, `dot`, `cross`, `point_at`, `perspective`, `deg_to_rad`, `rad_to_deg`, `map_floats`. |
| `src/custom_math.cpp`   | Implementation of the math library. |
| `data/littlefs/ship.obj`| Default Wavefront OBJ mesh (a spaceship). Loaded at runtime from the LittleFS partition. |
| `platformio.ini`        | PlatformIO build configuration. Defines two board environments, LittleFS filesystem target, `-Ofast` optimization, and library dependencies. |
| `boards/*.json`         | Custom board definition for ESP32-S3-DevKitC-1-N16R8V (16 MB flash, 8 MB PSRAM). |
