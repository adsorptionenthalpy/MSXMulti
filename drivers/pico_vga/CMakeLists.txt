# pico_vga - VGA driver for the PICO-56 board
#
# Lifted from https://github.com/visrealm/pico-56 (MIT) and adapted so
# multicore_launch_core1() is the caller's responsibility (the InfoNES
# emulator owns core1). vga_frens.[ch] is original glue for this project.
#
# Pin layout (hardcoded in vga.c, matches PICO-56 schematic):
#   HSYNC = GPIO 0, VSYNC = GPIO 1, RGB = GPIO 2..13 (R[0..3]=2..5,
#   G[0..3]=6..9, B[0..3]=10..13). Uses PIO0 (sync_sm=0, rgb_sm=1) and
#   two DMA channels.

if (NOT TARGET pico_vga)
    add_library(pico_vga STATIC
        vga.c
        vga-modes.c
        pio_utils.c
        vga_frens.c
    )

    target_include_directories(pico_vga PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pico_vga PUBLIC
        pico_stdlib
        pico_multicore
        pico_divider
        hardware_pio
        hardware_dma
        hardware_clocks
    )

    pico_generate_pio_header(pico_vga ${CMAKE_CURRENT_LIST_DIR}/vga.pio)
endif ()
