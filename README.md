# espcon

**espcon** is a from-scratch embedded game console engine running on an ESP32-S3 microcontroller. It features a fully CPU-based software 3D rendering engine — no GPU, no accelerator — that loads Wavefront OBJ meshes from an on-chip LittleFS filesystem, transforms vertices through a complete model-view-projection pipeline, and pushes rasterized triangles to a color TFT display over SPI. Input is handled via GPIO buttons and an analog joystick, all orchestrated by FreeRTOS.

## Features

- **Software 3D rasterizer** — complete model → view → projection pipeline running entirely on the CPU
- **OBJ mesh loading** — Wavefront `.obj` files streamed and parsed from LittleFS at runtime
- **FreeRTOS task management** — cooperative yielding and frame-rate control via the Arduino/ESP-IDF FreeRTOS layer
- **SPI-driven display output** — rasterized triangles pushed to a TFT display at up to ~80 MHz SPI
- **Dual display support** — ST7735 and ILI9341 selected at compile time with a single build flag
- **Button and analog joystick input** — GPIO (INPUT\_PULLDOWN) buttons for movement, ADC joystick for look-around with hardware deadzone filtering
- **Depth sorting** — painter's algorithm (back-to-front qsort) for correct triangle overdraw
- **Dynamic lighting** — directional light with dot-product shading and configurable ambient floor
- **Dynamic color animation** — RGB565 color cycling via sinusoidal waves per frame

## Tech Stack

`C++` `ESP32-S3` `PlatformIO` `FreeRTOS` `SPI` `LittleFS` `Adafruit GFX` `ST7735` `ILI9341`

## Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Rendering Engine Deep Dive](docs/RENDERING_ENGINE.md)
- [Contributing Guide](CONTRIBUTING.md)

## Hardware

- ESP32-S3 development board (e.g. `esp32-s3-devkitc-1`)
- ST7735 or ILI9341 display connected via SPI
  - CS → pin 10, DC → pin 9, RST → pin 8
- 2 buttons connected to pins 1 and 2 (configured as `INPUT_PULLDOWN`)
- Analog joystick connected to pins A3 (Y-axis) and A4 (X-axis)

## Build

- Install [PlatformIO](https://platformio.org/)
- `git clone https://github.com/mikolajlubiak/espcon`
- `cd espcon`
- Connect your board
- Upload the filesystem (ships the default `ship.obj` mesh):
  - `pio run --target uploadfs`
- Build and flash:
  - ST7735: `pio run --environment esp32-s3-devkitc-1 --target upload -- -D BUILD_ST7735`
  - ILI9341 (default): `pio run --environment esp32-s3-devkitc-1 --target upload`

## Usage

- Button on pin 1 – move camera right
- Button on pin 2 – move camera left
- Joystick (A3/A4) – look around
- Place your own `.obj` mesh files in `data/littlefs/` and update the path in `espcon.h`
