# espcon
Game console engine for ESP32-S3 MCU and ST7735, ILI9341 displays.

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
