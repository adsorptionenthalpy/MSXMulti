MSX1 emulator for Raspberry Pi Pico (RP2040) on PICO-56 or similar PCB.

<img width="1247" height="785" alt="image" src="https://github.com/user-attachments/assets/e1f4b4ff-484d-4910-8d5b-a5ebdf028cb6" />

<img width="1280" height="1252" alt="image" src="https://github.com/user-attachments/assets/ea603dc1-6a25-40a1-8512-837a834c33f3" />

<img width="895" height="578" alt="image" src="https://github.com/user-attachments/assets/59225edc-74cd-4a40-a422-c8fff099d1c6" />

<img width="1280" height="960" alt="image" src="https://github.com/user-attachments/assets/9511d4b5-7638-45ba-8eaa-990b354b2851" />

<img width="1280" height="744" alt="image" src="https://github.com/user-attachments/assets/77960576-d311-44bd-9e5b-bc48af270255" />

<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/de901b91-2624-4e00-a0de-ec521bca4e78" />



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
