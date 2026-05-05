/*
 * pico-infonesPlus VGA glue (PICO-56). See vga_frens.h.
 *
 * Memory note: the framebuffer is owned by FrensHelpers.cpp
 * (Frens::framebuffer[]). We just keep a pointer to it.
 *
 * Pixel format: 8bpp paletted. Each framebuffer byte is an index 0..63
 * into s_palette[], which holds the actual PICO-56 R2R bit-packed RGB
 * values. This is the RAM-saving format chosen for RP2040 (8bpp 256x240
 * = 60 KB vs 16bpp 256x240 = 122 KB) so that the InfoNES emulator
 * buffers (RAM/SRAM/PPURAM/SPRRAM/ChrBuf = 64 KB) fit in BSS.
 */

#include "vga_frens.h"
#include "vga.h"
#include "vga-modes.h"

#include <string.h>
#include "pico/stdlib.h"

static const uint8_t *s_framebuffer = NULL;
static volatile uint64_t s_frame_count = 0;

/* Diagnostic counters used during Mapper 30 (NESmaker) bring-up. The
 * top-of-screen visualisation is gone, but the symbols are still
 * referenced from InfoNES.cpp / K6502.cpp / Map30 instrumentation,
 * so keep the storage. Compiles to a few bytes of BSS. */
volatile uint32_t g_dbg_vblank_count = 0;
volatile uint32_t g_dbg_nmi_count    = 0;
volatile uint32_t g_dbg_render_count = 0;
volatile uint32_t g_dbg_chr_write    = 0;
volatile uint32_t g_dbg_mapper30_w   = 0;
volatile uint8_t  g_dbg_last_mapper_byte = 0;
volatile uint8_t  g_dbg_ppu_r1       = 0;
volatile uint16_t g_dbg_nmi_pc       = 0;
volatile uint16_t g_dbg_first_brk_pc = 0;
volatile uint8_t  g_dbg_first_brk_captured = 0;
volatile uint16_t g_dbg_last_indjmp_tgt = 0;
volatile uint16_t g_dbg_last_indjmp_ptr = 0;

/* 64-entry palette of PICO-56 R2R packed RGB-444 words. Initialized to
 * the standard NES palette (RGB565 source -> RGB-444 with channel order
 * R-low,G-mid,B-high to match the PICO-56 R2R DAC) on first init. The
 * caller can replace it via vga_frens_set_palette() if desired. */
static uint16_t s_palette[64];

/* RGB565 -> PICO-56 RGB-444 word.
 *
 * Output bit layout (matches PICO-56 R2R DAC wiring on GP2..13):
 *   bit  0..3  = R nibble (low bit -> GPIO 2)
 *   bit  4..7  = G nibble
 *   bit  8..11 = B nibble
 *
 * Input is RGB565 (R in bits 11..15, G in 5..10, B in 0..4). We take
 * the top 4 bits of each channel and pack them in PICO-56 order. This
 * subsumes what nes_word_to_vga() used to do (R<->B swap relative to
 * PicoDVI's bit ordering) directly into the palette table. */
#define VGAF_CC(x) ((((x) >> 11) & 15) | ((((x) >> 6) & 15) << 4) | ((((x) >> 1) & 15) << 8))

static const uint16_t s_default_palette[64] = {
    VGAF_CC(0x39ce), VGAF_CC(0x1071), VGAF_CC(0x0015), VGAF_CC(0x2013),
    VGAF_CC(0x440e), VGAF_CC(0x5402), VGAF_CC(0x5000), VGAF_CC(0x3c20),
    VGAF_CC(0x20a0), VGAF_CC(0x0100), VGAF_CC(0x0140), VGAF_CC(0x00e2),
    VGAF_CC(0x0ceb), VGAF_CC(0x0000), VGAF_CC(0x0000), VGAF_CC(0x0000),
    VGAF_CC(0x5ef7), VGAF_CC(0x01dd), VGAF_CC(0x10fd), VGAF_CC(0x401e),
    VGAF_CC(0x5c17), VGAF_CC(0x700b), VGAF_CC(0x6ca0), VGAF_CC(0x6521),
    VGAF_CC(0x45c0), VGAF_CC(0x0240), VGAF_CC(0x02a0), VGAF_CC(0x0247),
    VGAF_CC(0x0211), VGAF_CC(0x0000), VGAF_CC(0x0000), VGAF_CC(0x0000),
    VGAF_CC(0x7fff), VGAF_CC(0x1eff), VGAF_CC(0x2e5f), VGAF_CC(0x223f),
    VGAF_CC(0x79ff), VGAF_CC(0x7dd6), VGAF_CC(0x7dcc), VGAF_CC(0x7e67),
    VGAF_CC(0x7ae7), VGAF_CC(0x4342), VGAF_CC(0x2769), VGAF_CC(0x2ff3),
    VGAF_CC(0x03bb), VGAF_CC(0x0000), VGAF_CC(0x0000), VGAF_CC(0x0000),
    VGAF_CC(0x7fff), VGAF_CC(0x579f), VGAF_CC(0x635f), VGAF_CC(0x6b3f),
    VGAF_CC(0x7f1f), VGAF_CC(0x7f1b), VGAF_CC(0x7ef6), VGAF_CC(0x7f75),
    VGAF_CC(0x7f94), VGAF_CC(0x73f4), VGAF_CC(0x57d7), VGAF_CC(0x5bf9),
    VGAF_CC(0x4ffe), VGAF_CC(0x0000), VGAF_CC(0x0000), VGAF_CC(0x0000)
};

uint64_t vga_frens_frame_count(void) { return s_frame_count; }

void vga_frens_wait_for_vsync(void)
{
    uint64_t f = s_frame_count;
    while (s_frame_count == f) {
        tight_loop_contents();
    }
}

void vga_frens_set_palette(const uint16_t *pal64)
{
    for (int i = 0; i < 64; ++i) s_palette[i] = pal64[i];
}

/* Mode: 800x600@60Hz with hPixelScale=3, vPixelScale=2 -> 266x300 virtual.
 *
 * Why this configuration:
 *   - visrealm's HBC-56 firmware uses VGA_800_600_60HZ + pixelScale=3 and
 *     it locks reliably on the same PICO-56 monitor we're targeting. The
 *     pixelScale=3 path is also the one their algorithm guarantees an
 *     INTEGER pioClocksPerPixel for, which means rgb pixels land on
 *     exact PIO-tick boundaries with no round-off.
 *   - Visrealm uses vPixelScale=3 too (giving 200 line virtual), but that
 *     crops the NES image (240 lines). We override vPixelScale=2 -> 300
 *     line virtual, which has room for 240 + 30 letterbox each side.
 *   - We also bump sysclk to 266 MHz (matches visrealm's algorithm output
 *     of 5 * 53.33 MHz min PIO clock = 266.6 MHz), so the rgb sm runs at
 *     pioClocksPerPixel = 53.2/40 = 1.33 (essentially 4/3), giving exact
 *     4 PIO-ticks per scaled pixel.
 *
 * Source NES framebuffer is 256x240. Centred in the 266x300 surface:
 *   - horizontal pillarbox: (266 - 256) / 2 = 5 black pixels each side
 *   - vertical letterbox:   (300 - 240) / 2 = 30 black lines top + bottom
 */
#define VGA_FRENS_VIRT_W  266
#define VGA_FRENS_VIRT_H  300
#define VGA_FRENS_HBORDER ((VGA_FRENS_VIRT_W - VGA_FRENS_FB_WIDTH)  / 2)  /* 5 */
#define VGA_FRENS_VBORDER ((VGA_FRENS_VIRT_H - VGA_FRENS_FB_HEIGHT) / 2)  /* 30 */

__attribute__((unused, section(".time_critical.scanline_cb")))
static void scanline_cb(uint16_t y, VgaParams *params, uint16_t *pixels)
{
    /* IMPORTANT: All zero-loops below use a `volatile uint16_t *` write
     * to prevent gcc -O2 from recognising the pattern and emitting a
     * memset call. memset / __wrap_memset live in flash (~0x10004df0)
     * and this function runs in IRQ context on core 1 - calling out to
     * a flash-resident helper while core 0 is mid flash_range_erase
     * would hard-fault core 1 and kill VGA. The volatile cast keeps the
     * stores as individual h-word writes inside this RAM function. */
    volatile uint16_t *vp = pixels;
    const uint32_t n = params->hVirtualPixels; /* 400 */

    /* Vertical letterbox (top 30 / bottom 30 lines) -> black row */
    if (!s_framebuffer || y < VGA_FRENS_VBORDER || y >= VGA_FRENS_VBORDER + VGA_FRENS_FB_HEIGHT) {
        for (uint32_t x = 0; x < n; ++x) vp[x] = 0;
        return;
    }

    const uint32_t fb_y = y - VGA_FRENS_VBORDER;
    const uint8_t *src = &s_framebuffer[fb_y * VGA_FRENS_FB_WIDTH];

    /* Left pillarbox: black (matches monitor overscan if it crops). */
    for (uint32_t x = 0; x < VGA_FRENS_HBORDER; ++x) vp[x] = 0;

    /* Diagnostic: top 6 rows show three flashing bars. Each bar
     * pulses ON (bright color) or OFF (black) based on bit 5 of its
     * counter. If counter is stuck at 0, bar stays black permanently.
     *
     *   rows 0-1: red    -> vblank counter (CPU is reaching SCAN_VBLANK_START)
     *   rows 2-3: green  -> NMI counter   (PPUCTRL bit 7 is set when vblank fires)
     *   rows 4-5: yellow -> render counter (PPUMASK has BG or SP rendering enabled)
     *
     * If grey-screen + all three flashing -> game's CPU running, NMI
     * working, rendering enabled, but tile/palette data not loaded.
     * If grey + only red/green flashing -> game stuck before enabling
     * rendering (rendering bit never set in PPUMASK). */
    /* Diagnostic-stack overlay was here during Mapper 30 (NESmaker)
     * bring-up. Removed - the NES content area is full size again. */

    /* NES content (256 px): each src byte is a palette index (0..63).
     * Mask with 0x3F to be safe against stray high bits. */
    for (uint32_t x = 0; x < VGA_FRENS_FB_WIDTH; ++x) {
        vp[VGA_FRENS_HBORDER + x] = s_palette[src[x] & 0x3F];
    }

    /* Right pillarbox */
    for (uint32_t x = VGA_FRENS_HBORDER + VGA_FRENS_FB_WIDTH; x < n; ++x) {
        vp[x] = 0;
    }
}

__attribute__((section(".time_critical.end_of_frame_cb")))
static void end_of_frame_cb(uint64_t frameNumber)
{
    s_frame_count = frameNumber + 1;
}

void vga_frens_init(uint8_t *framebuffer)
{
    s_framebuffer = framebuffer;

    /* Install default NES palette so the caller doesn't need to set one
     * explicitly. vga_frens_set_palette() can override afterwards. */
    for (int i = 0; i < 64; ++i) s_palette[i] = s_default_palette[i];

    VgaInitParams init;
    init.params         = vgaGetParams(VGA_800_600_60HZ, /*pixelScale=*/3);
    /* Override vertical scale to 2 so we get 300 visible source rows
       (room for 240 NES lines + 30-line letterbox each side), while
       keeping horizontal scale=3 so visrealm's exact-integer PIO timing
       still holds. */
    init.params.vPixelScale     = 2;
    init.params.vVirtualPixels  = init.params.vSyncParams.displayPixels / init.params.vPixelScale; /* 300 */
    init.scanlineFn     = scanline_cb;
    init.endOfFrameFn   = end_of_frame_cb;
    init.endOfScanlineFn= NULL;

    vgaInit(init);
}
