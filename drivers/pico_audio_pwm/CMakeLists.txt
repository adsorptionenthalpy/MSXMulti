# pico_audio_pwm - stereo PWM audio driver for PICO-56 (or any RP2040
# board feeding two audio pins through an RC low-pass).

if (NOT TARGET pico_audio_pwm)
    add_library(pico_audio_pwm STATIC
        audio_pwm.c
    )

    target_include_directories(pico_audio_pwm PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pico_audio_pwm PUBLIC
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_timer
        hardware_irq
    )

    if (DEFINED PWM_AUDIO_PIN_L)
        target_compile_definitions(pico_audio_pwm PUBLIC
            PWM_AUDIO_PIN_L=${PWM_AUDIO_PIN_L}
            PWM_AUDIO_PIN_R=${PWM_AUDIO_PIN_R}
        )
    endif()
    if (DEFINED PWM_AUDIO_LR_SWAPPED)
        target_compile_definitions(pico_audio_pwm PUBLIC
            PWM_AUDIO_LR_SWAPPED=${PWM_AUDIO_LR_SWAPPED}
        )
    endif()
endif ()
