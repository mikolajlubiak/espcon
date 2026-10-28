# espcon

A real-time 3D renderer for the ESP32-S3, a microcontroller with no GPU. It loads an OBJ mesh from flash and draws it lit and shaded on an SPI TFT display, with an analog joystick and two buttons driving the camera.

## Project showcase

Playlist with demos: https://video.infosec.exchange/w/p/oQ5QCWFV5TxV9F3JkxzUYx

## How it works

Every frame runs entirely on the CPU:

1. **Transform:** every triangle of the mesh goes through my own vector and matrix library (`vec3`, `vec4`, `mat4`, point-at and perspective matrices), which replaced glm in the build.
2. **Back-face culling:** triangles facing away from the camera are dropped using the cross-product face normal.
3. **Flat shading:** one directional light gives each face a Lambert term with an ambient floor, applied to an RGB565 base color that cycles over time.
4. **Projection:** view and perspective transforms, perspective divide and viewport mapping.
5. **Depth sort:** visible triangles are sorted back to front by average depth (painter's algorithm).
6. **Fill:** each triangle is filled in a single color with Adafruit GFX `fillTriangle`, drawn straight to the display over SPI; there is no framebuffer.

Around the pipeline:

- **Mesh loading:** OBJ meshes (`v` and `f` lines) are streamed from a LittleFS partition in flash into growable heap buffers at startup.
- **Displays:** ST7735 and ILI9341 controllers, selected at compile time. SPI runs at 40 MHz on both; 80 MHz was tried on the ST7735 and produced artifacts.
- **Input:** an analog joystick read through the 12-bit ADC, with a software deadzone, controls yaw and pitch; two GPIO buttons strafe the camera.
- **Board:** a custom PlatformIO board definition for the ESP32-S3-DevKitC-1 N16R8V (16 MB flash, 8 MB PSRAM).

## Tech Stack

`C++` `ESP32-S3` `Arduino` `PlatformIO` `SPI` `LittleFS` `Adafruit GFX` `ST7735` `ILI9341`

## Hardware

- ESP32-S3 development board (`esp32-s3-devkitc-1`, or the N16R8V variant with the bundled board definition)
- ST7735 or ILI9341 display on SPI
  - CS → GPIO 10, DC → GPIO 9, RST → GPIO 8 (RST is used by the ST7735 build only)
- 2 buttons from GPIO 1 and GPIO 2 to 3.3 V (the pins use internal pull-downs)
- Analog joystick on A3 (Y axis) and A4 (X axis)

## Build

- Install [PlatformIO](https://platformio.org/)
- `git clone https://github.com/mikolajlubiak/espcon`
- `cd espcon`
- Pick the display in `platformio.ini`: `-D BUILD_ILI9341` (default) or `-D BUILD_ST7735` in `build_flags`
- Connect your board
- Upload the filesystem with the default `ship.obj` mesh:
  - `pio run -e esp32-s3-devkitc-1-n16r8v -t uploadfs`
- Build and flash:
  - `pio run -e esp32-s3-devkitc-1-n16r8v -t upload`
- For a standard ESP32-S3-DevKitC-1, use `-e esp32-s3-devkitc-1` in both commands.

## Usage

- Button on GPIO 1: move the camera right
- Button on GPIO 2: move the camera left
- Joystick: look around
- To render your own mesh, put a triangulated `.obj` with vertex positions only (`v` and `f` lines) in `data/littlefs/` and change the path passed to `loadObj` in `include/espcon.h`.
