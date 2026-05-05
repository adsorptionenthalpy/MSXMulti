# pico_ps2_kbd - PS/2 keyboard host driver
#
# Lifted from https://github.com/visrealm/pico-56 (MIT). The HBC-56-specific
# raiseInterrupt/releaseInterrupt() hooks are stubbed out at build time
# (see ps2-kbd.c). PIO instance and pins are configurable via -D, default
# to PICO-56 values (DATA=14, CLK=15, PIO=pio1).

if (NOT TARGET pico_ps2_kbd)
    add_library(pico_ps2_kbd STATIC
        ps2-kbd.c
        virtual-ps2-kbd.c
    )

    target_include_directories(pico_ps2_kbd PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pico_ps2_kbd PUBLIC
        pico_stdlib
        hardware_pio
        hardware_dma
    )

    pico_generate_pio_header(pico_ps2_kbd ${CMAKE_CURRENT_LIST_DIR}/ps2-kbd.pio)

    if (DEFINED PS2_KBD_DATA_PIN)
        target_compile_definitions(pico_ps2_kbd PUBLIC
            PS2_DATA_GPIO=${PS2_KBD_DATA_PIN}
            PS2_CLOCK_GPIO=${PS2_KBD_CLOCK_PIN}
        )
    endif()
endif ()
