# pico_nespad - lifted from pico-infonesPlus/pico_shared/nespad.{cpp,h}
# Custom NES-pad PIO reader that supports non-adjacent CLK / LATCH /
# DATA pins (the stock pico-nespad library requires CLK+ENABLE adjacent,
# which PICO-56 isn't). HW_CONFIG=15 selects the PICO-56 PIO variant.

if (NOT TARGET pico_nespad)
    add_library(pico_nespad STATIC
        nespad.cpp
    )

    target_include_directories(pico_nespad PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_compile_definitions(pico_nespad PUBLIC
        HW_CONFIG=15
    )

    target_link_libraries(pico_nespad PUBLIC
        pico_stdlib
        hardware_pio
    )
endif()
