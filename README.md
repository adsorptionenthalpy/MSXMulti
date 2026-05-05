# MSXMulti - MSX1 Emulator

MSX1 emulator for Raspberry Pi Pico (RP2040) on PICO-56 or similar PCB.

## Requirements

- Raspberry Pi Pico SDK (1.5.1 or later)
- CMake 3.13+
- ARM GCC toolchain (arm-none-eabi-gcc)

## Build Instructions

```bash
# Set PICO_SDK_PATH environment variable
export PICO_SDK_PATH=/path/to/pico-sdk

# Create build directory
mkdir build
cd build

# Configure
cmake ..

# Build
make -j4
```

The output `MSXMulti.uf2` will be in the build directory.

## Hardware

Designed for PICO-56 board:
- GP0-13: VGA R2R DAC (4-4-4 RGB)
- GP14/15: PS/2 Keyboard
- GP16-19: SD Card (SPI0)
- GP20/21: PWM Stereo Audio
- GP22/26-28: NES Controller Pads

## SD Card Layout

```
/MSX/
  BIOS/
    BIOS.ROM      (32KB MSX1 BIOS - You must include your own)
    DISK.ROM      (Must include)
  ROMS/
    *.rom, *.bin  (Game cartridges)
  DISKS/
    *.dsk         (Disk images)
```

## Credits

- Written by serpentchain (2026)
- Based on msxemulator by shippoiincho (https://github.com/shippoiincho/msxemulator)
- PICO-56 hardware by visrealm (https://github.com/visrealm/pico-56)
