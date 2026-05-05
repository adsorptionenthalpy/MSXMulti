/*
 * pico-infonesPlus VGA glue (PICO-56)
 *
 * Bridges the visrealm vga driver (vga.h) into the InfoNES port:
 *   - The caller (FrensHelpers.cpp) owns the 320x240 16-bit framebuffer
 *     (Frens::framebuffer). We take a pointer to it at init time. NES
 *     content lands at columns 32..287 (centered horizontally), borders
 *     to either side, exactly as the existing RP2350 framebuffer path
 *     does.
 *   - We provide a per-scanline callback that converts each row of the
 *     framebuffer to 12-bit RGB-444 packed for the PICO-56 R2R DAC
 *     (R[0..3]=GPIO 2..5, G=6..9, B=10..13) and writes it into the line
 *     buffer the visrealm driver gives us.
 *   - Mode is 640x480p60 with hPixelScale=vPixelScale=2 (320x240 virtual).
 *
 * Bit packing produced by vga_frens for one pixel:
 *   bit  0..3  = R nibble (low bit -> GPIO 2)
 *   bit  4..7  = G nibble
 *   bit  8..11 = B nibble
 *   bit 12..15 = unused (only 12 RGB GPIOs wired; PIO discards top 4)
 *
 * If channels appear swapped on hardware, edit nes_word_to_vga() in
 * vga_frens.c -- the InfoNES NES palette is the source of truth and
 * this is the single place we re-pack it.
 */

#ifndef PICO_VGA_FRENS_H
#define PICO_VGA_FRENS_H

#include <stdbool.h>
#include <inttypes.h>

#include "vga.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Framebuffer geometry. Matches Frens::SCREENWIDTH/SCREENHEIGHT under
   USE_VGA: 256 native NES pixels, 240 lines. The VGA scanout adds 32
   black pixels of pillarbox each side to reach 320 virtual pixels. */
#define VGA_FRENS_FB_WIDTH  256
#define VGA_FRENS_FB_HEIGHT 240

/* Initialize visrealm VGA at 800x600p60, 3x/2x pixel scale (266x300
   virtual). The framebuffer is 8bpp paletted (256x240 bytes); each byte
   is an index 0..63 into the palette installed via
   vga_frens_set_palette(). After this returns, caller must run
   vgaScanloop() on core1. */
void vga_frens_init(uint8_t *framebuffer);

/* Install the 64-entry palette used at scan-out. Each entry is a
   PICO-56 R2R packed RGB-444 word (R nibble in bits 0..3, G in 4..7, B
   in 8..11). Call before vga_frens_init() or any time after - palette
   updates take effect on the next scanline. */
void vga_frens_set_palette(const uint16_t *pal64);

/* Spin until the next end-of-frame. Used as a vsync barrier. */
void vga_frens_wait_for_vsync(void);

/* Returns frame counter incremented at end-of-frame. */
uint64_t vga_frens_frame_count(void);

#ifdef __cplusplus
}
#endif

#endif /* PICO_VGA_FRENS_H */
