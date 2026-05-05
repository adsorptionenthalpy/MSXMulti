#pragma once

#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t nespad_states[2];
extern bool nespad_begin(uint8_t padnum, uint32_t cpu_khz, uint8_t clkPin, uint8_t dataPin,
                         uint8_t latPin, PIO _pio);
extern void nespad_read_start(void);
extern void nespad_read_finish(void);

/* Non-blocking: returns true and writes the latest sample byte to *out
 * if the PIO RX FIFO has data, false otherwise. Used for diagnostics. */
extern bool nespad_read_try(uint8_t pad, uint8_t *out);

#ifdef __cplusplus
}
#endif
