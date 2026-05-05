cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico CACHE STRING "Board type")
set(PICO_PLATFORM rp2040 CACHE STRING "Platform")

# Use prebuilt pioasm and picotool from the InfoNES build
set(pioasm_DIR "${CMAKE_CURRENT_LIST_DIR}/../pico-infonesPlus/build/pioasm" CACHE PATH "")
set(picotool_DIR "${CMAKE_CURRENT_LIST_DIR}/../pico-infonesPlus/build/_deps/picotool" CACHE PATH "")

include(pico_sdk_import.cmake)

project(MSXMulti C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ----------------------------------------------------------------
# PICO-56 drivers (lifted from pico-infonesPlus)
# ----------------------------------------------------------------
set(PWM_AUDIO_PIN_L 20)
set(PWM_AUDIO_PIN_R 21)

add_subdirectory(drivers/pico_vga)
add_subdirectory(drivers/pico_audio_pwm)
add_subdirectory(drivers/pico_ps2_kbd)
add_subdirectory(drivers/pico_fatfs)
add_subdirectory(drivers/pico_nespad)

# ----------------------------------------------------------------
# MSX emulator core libraries
# ----------------------------------------------------------------

# vrEmuTms9918 VDP
add_library(vr_tms9918 STATIC
    msx/tms9918/vrEmuTms9918.c
    msx/tms9918/vrEmuTms9918Util.c
)
target_include_directories(vr_tms9918 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx/tms9918
)

# Zeta Z80 CPU
add_library(zeta_z80 STATIC
    msx/Z80.c
)
target_include_directories(zeta_z80 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx
)
target_compile_definitions(zeta_z80 PUBLIC
    Z80_STATIC
    Z80_WITH_LOCAL_HEADER
    Z80_WITH_Q
    Z80_WITH_ZILOG_NMOS_LD_A_IR_BUG
)

# emu2149 PSG (AY-3-8910)
add_library(emu2149 STATIC
    msx/emu2149/emu2149.c
)
target_include_directories(emu2149 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx/emu2149
)

# emu2212 SCC
add_library(emu2212 STATIC
    msx/emu2212/emu2212.c
)
target_include_directories(emu2212 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx/emu2212
)

# emu2413 OPLL
add_library(emu2413 STATIC
    msx/emu2413/emu2413.c
)
target_include_directories(emu2413 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx/emu2413
)
target_link_libraries(emu2413 PUBLIC
    pico_stdlib
)

# FDC (FatFS version)
add_library(msx_fdc STATIC
    msx/fdc_fatfs.c
)
target_include_directories(msx_fdc PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/msx
)
target_link_libraries(msx_fdc PUBLIC
    pico_fatfs
)

# ----------------------------------------------------------------
# Main executable
# ----------------------------------------------------------------
add_executable(MSXMulti
    main.c
)

target_include_directories(MSXMulti PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/msx
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_vga
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_audio_pwm
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_ps2_kbd
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_nespad
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_fatfs
    ${CMAKE_CURRENT_LIST_DIR}/drivers/pico_fatfs/fatfs
)

target_compile_definitions(MSXMulti PRIVATE
    USE_VGA=1
    USE_PS2_KBD=1
    HW_CONFIG=15
    PICO_FLASH_SIZE_BYTES=2097152
    USE_FDC=1
)

target_link_libraries(MSXMulti PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_timer
    hardware_dma
    hardware_flash
    hardware_pwm
    hardware_vreg
    hardware_spi
    pico_vga
    pico_audio_pwm
    pico_ps2_kbd
    pico_fatfs
    pico_nespad
    zeta_z80
    vr_tms9918
    emu2149
    emu2212
    emu2413
    msx_fdc
)

pico_enable_stdio_usb(MSXMulti 0)
pico_enable_stdio_uart(MSXMulti 0)

pico_add_extra_outputs(MSXMulti)
