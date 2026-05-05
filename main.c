/*
 * MSXMulti - MSX1 emulator for Raspberry Pi Pico 2040

 * Written by serpentchain
 * Based on msxemulator by shippoiincho
 * Ported to PICO-56 (visrealm) hardware:
 *   - GP0-13:  VGA R2R DAC (4-4-4 RGB)
 *   - GP14/15: PS/2 Keyboard
 *   - GP16-19: SD Card (SPI0)
 *   - GP20/21: PWM Stereo Audio
 *   - GP22/26-28: NES Controller Pads
 *
 * Core 0: Z80 CPU, I/O, menu, audio mixing
 * Core 1: VGA scanline rendering (via pico_vga driver)
 */

#include "msx/msxemulator.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"

#include "vga_frens.h"
#include "audio_pwm.h"
#include "ps2-kbd.h"
#include "nespad.h"
#include "tf_card.h"
#include "ff.h"

#include "msx/tms9918/vrEmuTms9918.h"
#include "msx/tms9918/vrEmuTms9918Util.h"
#include "msx/msxkeymap.h"
#include "msx/Z80.h"
#include "msx/msxmisc.h"
#include "msx/font_jp.h"
#include "msx/msxrom.h"
#include "msx/emu2149/emu2149.h"
#include "msx/emu2413/emu2413.h"
#include "msx/emu2212/emu2212.h"
#include "msx/fdc.h"

/* ------------------------------------------------------------------ */
/*  Display configuration                                             */
/* ------------------------------------------------------------------ */

/* Display dimensions */
#define MSX_WIDTH       256
#define MSX_HEIGHT      192
#define FB_WIDTH        256
#define FB_HEIGHT       240
#define FB_VBORDER      ((FB_HEIGHT - MSX_HEIGHT) / 2)

/* Z80 timing constants (NTSC) */
#define Z80_CYCLES_PER_SCANLINE  228    /* 3.58MHz / 15.7kHz */
#define SCANLINES_PER_FRAME      262    /* NTSC: 262 lines/frame */

/* Joystick default state (active-low: 1 = not pressed) */
#define JOY_IDLE    0x3F

/* Keyboard matrix size */
#define KEYMAP_ROWS 11

static uint8_t framebuffer[FB_WIDTH * FB_HEIGHT];

/* TMS9918 color palette -> PICO-56 RGB-444.
 * PICO-56 DAC: R in bits 0..3, G in 4..7, B in 8..11.
 * Macro converts 8-bit R,G,B to packed RGB-444. */
#define RGB444(r, g, b) (((r) >> 4) | (((g) >> 4) << 4) | (((b) >> 4) << 8))

static const uint16_t tms_palette[64] = {
    RGB444(0x00, 0x00, 0x00),  /*  0: Transparent (black) */
    RGB444(0x00, 0x00, 0x00),  /*  1: Black */
    RGB444(0x21, 0xC8, 0x42),  /*  2: Medium green */
    RGB444(0x5E, 0xDC, 0x78),  /*  3: Light green */
    RGB444(0x54, 0x55, 0xED),  /*  4: Dark blue */
    RGB444(0x7D, 0x76, 0xFC),  /*  5: Light blue */
    RGB444(0xD4, 0x52, 0x4D),  /*  6: Dark red */
    RGB444(0x42, 0xEB, 0xF5),  /*  7: Cyan */
    RGB444(0xFC, 0x55, 0x54),  /*  8: Medium red */
    RGB444(0xFF, 0x79, 0x78),  /*  9: Light red */
    RGB444(0xD4, 0xC1, 0x54),  /* 10: Dark yellow */
    RGB444(0xE6, 0xCE, 0x80),  /* 11: Light yellow */
    RGB444(0x21, 0xB0, 0x3B),  /* 12: Dark green */
    RGB444(0xC9, 0x5B, 0xBA),  /* 13: Magenta */
    RGB444(0xCC, 0xCC, 0xCC),  /* 14: Gray */
    RGB444(0xFF, 0xFF, 0xFF),  /* 15: White */
};

/* ------------------------------------------------------------------ */
/*  VGA hsync flag for Z80 timing                                     */
/* ------------------------------------------------------------------ */

volatile uint32_t video_vsync = 0;

/* ------------------------------------------------------------------ */
/*  MSX emulator state (from msxemulator.c)                           */
/* ------------------------------------------------------------------ */

static Z80 cpu;
uint32_t cpu_cycles = 0;
uint32_t cpu_hsync = 0;

uint8_t extslot[4];
uint8_t megarom[8];
uint8_t memmap[4];

/* MSX memory pool - 112KB total for RAM + optional ROM buffers
 * Layout (from base):
 *   0x00000-0x0FFFF: 64KB base RAM (always available)
 *   0x10000-0x17FFF: 32KB extended RAM / Cart ROM buffer
 *   0x18000-0x1BFFF: 16KB extended RAM / Disk ROM buffer
 * When cart/disk disabled, their regions become extra RAM banks. */
uint8_t msx_memory[0x1C000];  /* 112KB */
uint8_t ioport[0x100];

/* Convenience aliases */
#define mainram msx_memory
#define cartrom_buf (&msx_memory[0x10000])
#define diskrom_ram (&msx_memory[0x18000])

/* BIOS ROM - always separate (slot 0) */
uint8_t bios_ram[0x8000];      /* 32KB BIOS */

/* ROM pointers */
uint8_t *basicrom = bios_ram;
uint8_t *extrom1 = NULL;       /* Points to diskrom_ram when enabled */
uint8_t *extrom2 = NULL;       /* Sound ROM not used */
uint8_t *cartrom1 = NULL;      /* Points to cartrom_buf when enabled */
uint8_t *cartrom2 = NULL;      /* Only one cart slot supported */

uint32_t carttype[2] = {0, 0};
uint32_t cart_enable[2] = {0, 0};
uint32_t cart_loaded[2] = {0, 0};

/* SRAM support - separate 2KB buffer for battery-backed saves */
static uint8_t cart_sram_buf[0x800];  /* 2KB SRAM */
#define cart_sram cart_sram_buf
#define CART_SRAM_SIZE 0x800  /* 2KB */
bool sram_dirty = false;        /* SRAM was modified */
bool sram_enabled[2] = {false, false};  /* SRAM enabled for each slot */
char sram_filename[64];         /* Current SRAM save filename */

/* Large cart support (>32KB ROMs)
 * Memory layout:
 *   Banks 0-3  (0x00000-0x07FFF): cartrom_buf[32KB] in RAM
 *   Banks 4+   (0x08000+):        on-demand from SD with 8KB cache in mainram
 */
static bool large_cart_mode = false;
static char large_cart_path[128];  /* Path for on-demand loading */
static uint32_t large_cart_size = 0;
static uint8_t cached_bank_num = 0xFF;  /* 0xFF = no bank cached */
/* Bank cache uses last 8KB of diskrom_ram area (0x1A000-0x1BFFF).
 * This is safe because large cart games don't use disk simultaneously,
 * and when disk IS used, we're not in large_cart_mode. */
#define bank_cache (&msx_memory[0x1A000])

VrEmuTms9918 *mainscreen;
#define menuscreen mainscreen  /* Share single VDP to save 16KB RAM */
uint8_t scandata[256];

/* VDP state save to SD card (RAM too constrained for 16KB buffer) */
#define VDP_SAVE_FILE "/MSX/.vdpstate.tmp"
static bool vdp_state_saved = false;

uint8_t keymap[11];
volatile uint8_t keypressed = 0;
uint32_t key_caps = 0;
uint32_t key_kana = 0;
uint32_t key_kana_jis = 1;

uint32_t beep_enable = 0;
volatile uint32_t sound_tick = 0;

/* FM synthesis (OPLL / YM2413 / MSX-MUSIC) */
static OPLL *opll = NULL;
static uint8_t opll_reg = 0;

/* Master volume for all audio (0-255, default 100%) */
static uint8_t master_volume = 255;

/* Audio mode setting */
typedef enum {
    AUDIO_MODE_PSG = 0,
    AUDIO_MODE_FM = 1,
    AUDIO_MODE_PSG_FM = 2
} audio_mode_t;

static audio_mode_t audio_mode = AUDIO_MODE_PSG;  /* Default to PSG only */

/* Input configuration */
typedef enum {
    INPUT_MODE_JOYSTICK = 0,  /* Gamepad acts as MSX joystick */
    INPUT_MODE_KEYBOARD = 1,  /* Gamepad simulates MSX keyboard */
    INPUT_MODE_MOUSE = 2      /* Gamepad emulates MSX mouse */
} input_mode_t;

static input_mode_t input_mode = INPUT_MODE_JOYSTICK;
static uint8_t input_joy_port = 0;  /* Which joystick port (0 or 1) */

/* MSX Mouse emulation state */
static int8_t mouse_dx = 0;           /* Accumulated X movement */
static int8_t mouse_dy = 0;           /* Accumulated Y movement */
static uint8_t mouse_buttons = 0;     /* Button state: bit0=left, bit1=right */
static uint8_t mouse_strobe = 0;      /* Last strobe state */
static uint8_t mouse_nibble = 0;      /* Current nibble (0-3) */
static uint8_t mouse_speed = 3;       /* Cursor speed multiplier */

/* MSX keyboard mapping for gamepad buttons (row, bitmask) */
/* Default: A=Space, B=Z, Select=Tab(Graph), Start=Enter, D-pad=Arrows */
typedef struct {
    uint8_t row;
    uint8_t mask;
} msx_key_t;

static msx_key_t key_map_a      = {8, 0x01};  /* Space */
static msx_key_t key_map_b      = {5, 0x80};  /* Z */
static msx_key_t key_map_select = {6, 0x04};  /* Graph (Alt) */
static msx_key_t key_map_start  = {7, 0x80};  /* Enter */
static msx_key_t key_map_up     = {8, 0x20};  /* Up arrow */
static msx_key_t key_map_down   = {8, 0x40};  /* Down arrow */
static msx_key_t key_map_left   = {8, 0x10};  /* Left arrow */
static msx_key_t key_map_right  = {8, 0x80};  /* Right arrow */

/* On-screen keyboard overlay */
static bool osk_active = false;
static int osk_cursor_x = 0;
static int osk_cursor_y = 0;
static uint8_t osk_prev_pad = 0;
static int osk_key_timer = 0;  /* Timer for key press duration */

/* OSK layout: 4 rows of characters */
#define OSK_COLS 13
#define OSK_ROWS 4
static const char osk_layout[OSK_ROWS][OSK_COLS + 1] = {
    "1234567890-=",
    "QWERTYUIOP[]",
    "ASDFGHJKL;' ",  /* Space at end represented by ' ' */
    "ZXCVBNM,./<E"   /* < = Backspace, E = Enter (shown as [E]) */
};

/* MSX key codes for OSK (row, mask) - matches osk_layout
 * From msxkeymap.h: format is (mask, row) but we store as (row, mask) */
static const msx_key_t osk_keys[OSK_ROWS][OSK_COLS] = {
    /* Row 0: 1234567890-= (12 chars, pad with dummy) */
    {{0,0x02},{0,0x04},{0,0x08},{0,0x10},{0,0x20},{0,0x40},{0,0x80},{1,0x01},{1,0x02},{0,0x01},{1,0x04},{1,0x08},{0,0}},
    /* Row 1: QWERTYUIOP[] */
    {{4,0x40},{5,0x10},{3,0x04},{4,0x80},{5,0x02},{5,0x40},{5,0x04},{3,0x40},{4,0x10},{4,0x20},{1,0x20},{1,0x40},{0,0}},
    /* Row 2: ASDFGHJKL;'<Space> */
    {{2,0x40},{5,0x01},{3,0x02},{3,0x08},{3,0x10},{3,0x20},{3,0x80},{4,0x01},{4,0x02},{1,0x80},{2,0x01},{8,0x01},{0,0}},
    /* Row 3: ZXCVBNM,./<BS><Enter> */
    {{5,0x80},{5,0x20},{3,0x01},{5,0x08},{2,0x80},{4,0x08},{4,0x04},{2,0x04},{2,0x08},{2,0x10},{7,0x20},{7,0x80},{0,0}}
};

uint8_t psg_register_number = 0;
uint8_t psg_register[16];
uint32_t psg_osc_interval[4];
uint32_t psg_osc_counter[4];
uint32_t psg_noise_interval;
uint32_t psg_noise_counter;
uint8_t psg_noise_output;
uint32_t psg_noise_seed;
uint32_t psg_envelope_interval;
uint32_t psg_envelope_counter;
uint32_t psg_master_clock = 3579545 / 2;
uint16_t psg_master_volume = 0;
uint8_t psg_tone_on[4], psg_noise_on[4];

static const uint16_t psg_volume[] = {
    0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10,
    0x13, 0x17, 0x1b, 0x20, 0x26, 0x2d, 0x36, 0x40,
    0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff
};

#define SAMPLING_FREQ 22050
#define TIME_UNIT 100000000
#define SAMPLING_INTERVAL (TIME_UNIT / SAMPLING_FREQ)

/* Tape (minimal support) */
uint32_t tape_ready = 0;
uint32_t tape_phase = 0;

/* Menu */
uint32_t menumode = 0;

/* Color themes */
typedef enum {
    THEME_DEFAULT = 0,   /* White on dark blue */
    THEME_TERMINAL = 1,  /* Lime green on black */
    THEME_COUNT
} color_theme_t;

static color_theme_t current_theme = THEME_DEFAULT;

/* Theme definitions: {foreground, background} using TMS9918 colors */
static const uint8_t theme_colors[THEME_COUNT][2] = {
    {15, 4},  /* DEFAULT: White(15) on Blue(4) */
    {3, 1},   /* TERMINAL: Light Green(3) on Black(1) */
};

/* ROM settings - per-game configuration */
#define MAX_ROM_SETTINGS 32
#define ROM_NAME_LEN 24

typedef struct {
    char name[ROM_NAME_LEN];     /* ROM filename (truncated) */
    uint8_t header[4];           /* First 4 bytes for identification */
    uint8_t mapper_type;         /* 0=auto, 1=ASCII8, 2=ASCII16, etc */
    uint8_t ram_size;            /* 0=64KB, 1=16KB, 2=32KB */
    uint8_t flags;               /* Reserved for future use */
    uint8_t padding;
} rom_setting_t;

static rom_setting_t rom_settings[MAX_ROM_SETTINGS];
static int rom_settings_count = 0;

/* Config file path */
#define CONFIG_PATH "/MSX/config.dat"
#define CONFIG_MAGIC 0x4D53584D  /* "MSXM" */

typedef struct {
    uint32_t magic;
    uint8_t theme;
    uint8_t rom_count;
    uint8_t disk_rom_enabled;  /* 0=disabled, 1=enabled */
    uint8_t ram_size;          /* 0=32KB, 1=64KB, 2=96KB, 3=112KB */
    rom_setting_t roms[MAX_ROM_SETTINGS];
} config_data_t;

/* RAM settings */
static uint8_t disk_rom_enabled = 1;  /* Default enabled */
static uint8_t cart_rom_enabled = 1;  /* Default enabled */
static uint8_t ram_size = 1;          /* Default 64KB */

/* RAM size options: 0=32KB, 1=64KB, 2=80KB, 3=96KB, 4=112KB */
static const uint8_t ram_size_kb[] = {32, 64, 80, 96, 112};
/* Max RAM banks for each size (16KB per bank) */
static const uint8_t ram_max_banks[] = {2, 4, 5, 6, 7};

/* Gamepad (NES pad) */
uint32_t gamepad_select = 0;
volatile uint8_t gamepad_info = JOY_IDLE;
volatile uint8_t gamepad_info2 = JOY_IDLE;

/* PS/2 keyboard state */
static uint8_t ps2_key_state[256];
static bool ps2_extended = false;
static bool ps2_release = false;

/* SD card / FatFS */
static FATFS fatfs;
unsigned char filename[256];
unsigned char fd_filename[256];
unsigned char cart1_filename[256];
unsigned char cart2_filename[256];

/* Menu text dimensions */
#define VGA_CHARS_X 32
#define VGA_CHARS_Y 24

/* ------------------------------------------------------------------ */
/*  PS/2 scancode -> MSX keyboard matrix mapping                      */
/* ------------------------------------------------------------------ */

/* MSX keyboard matrix: 11 rows, each row is 8 bits.
 * keymap[row] has bit cleared when key is pressed.
 * PS/2 Set 2 scancode -> (row, bitmask) mapping. */

typedef struct {
    uint8_t scancode;
    uint8_t row;
    uint8_t bitmask;
} Ps2MsxKey;

static const Ps2MsxKey ps2_msx_map[] = {
    /* Row 0: 0-9 and related */
    {0x45, 0x00, 0x01},  /* 0 */
    {0x16, 0x00, 0x02},  /* 1 */
    {0x1E, 0x00, 0x04},  /* 2 */
    {0x26, 0x00, 0x08},  /* 3 */
    {0x25, 0x00, 0x10},  /* 4 */
    {0x2E, 0x00, 0x20},  /* 5 */
    {0x36, 0x00, 0x40},  /* 6 */
    {0x3D, 0x00, 0x80},  /* 7 */

    /* Row 1: 8-9, -, =, [, ], ;, ' */
    {0x3E, 0x01, 0x01},  /* 8 */
    {0x46, 0x01, 0x02},  /* 9 */
    {0x4E, 0x01, 0x04},  /* - */
    {0x55, 0x01, 0x08},  /* = (^) */
    {0x5D, 0x01, 0x10},  /* \ (*) */
    {0x54, 0x01, 0x20},  /* [ (@) */
    {0x5B, 0x01, 0x40},  /* ] ([) */
    {0x4C, 0x01, 0x80},  /* ; */

    /* Row 2: :, ,, ., /, dead, A, B, C */
    {0x52, 0x02, 0x01},  /* ' (:) */
    {0x41, 0x02, 0x04},  /* , */
    {0x49, 0x02, 0x08},  /* . */
    {0x4A, 0x02, 0x10},  /* / */
    {0x61, 0x02, 0x20},  /* \ (ro) - non-US */

    /* Row 2 continued: A, B */
    {0x1C, 0x02, 0x40},  /* A */
    {0x32, 0x02, 0x80},  /* B */

    /* Row 3: C-J */
    {0x21, 0x03, 0x01},  /* C */
    {0x23, 0x03, 0x02},  /* D */
    {0x24, 0x03, 0x04},  /* E */
    {0x2B, 0x03, 0x08},  /* F */
    {0x34, 0x03, 0x10},  /* G */
    {0x33, 0x03, 0x20},  /* H */
    {0x43, 0x03, 0x40},  /* I */
    {0x3B, 0x03, 0x80},  /* J */

    /* Row 4: K-R */
    {0x42, 0x04, 0x01},  /* K */
    {0x4B, 0x04, 0x02},  /* L */
    {0x3A, 0x04, 0x04},  /* M */
    {0x31, 0x04, 0x08},  /* N */
    {0x44, 0x04, 0x10},  /* O */
    {0x4D, 0x04, 0x20},  /* P */
    {0x15, 0x04, 0x40},  /* Q */
    {0x2D, 0x04, 0x80},  /* R */

    /* Row 5: S-Z */
    {0x1B, 0x05, 0x01},  /* S */
    {0x2C, 0x05, 0x02},  /* T */
    {0x3C, 0x05, 0x04},  /* U */
    {0x2A, 0x05, 0x08},  /* V */
    {0x1D, 0x05, 0x10},  /* W */
    {0x22, 0x05, 0x20},  /* X */
    {0x35, 0x05, 0x40},  /* Y */
    {0x1A, 0x05, 0x80},  /* Z */

    /* Row 6: SHIFT, CTRL, GRAPH, CAPS, CODE, F1-F5 */
    {0x58, 0x06, 0x08},  /* Caps Lock */
    {0x05, 0x06, 0x20},  /* F1 */
    {0x06, 0x06, 0x40},  /* F2 */
    {0x04, 0x06, 0x80},  /* F3 */

    /* Row 7: F4, F5, ESC, TAB, STOP, BS, SELECT, RET */
    {0x0C, 0x07, 0x01},  /* F4 */
    {0x03, 0x07, 0x02},  /* F5 */
    {0x76, 0x07, 0x04},  /* ESC */
    {0x0D, 0x07, 0x08},  /* TAB */
    {0x66, 0x07, 0x20},  /* Backspace */
    {0x5A, 0x07, 0x80},  /* Enter (Return) */

    /* Row 8: SPACE, HOME, INS, DEL, arrows */
    {0x29, 0x08, 0x01},  /* Space */
    {0x0E, 0x08, 0x02},  /* ` -> HOME */
    {0x70, 0x08, 0x04},  /* Insert (extended) */
    {0x71, 0x08, 0x08},  /* Delete (extended) */

    /* Extended arrows are handled separately with E0 prefix */
};

#define PS2_MSX_MAP_SIZE (sizeof(ps2_msx_map) / sizeof(ps2_msx_map[0]))

/* Extended keys (E0 prefix) */
static const Ps2MsxKey ps2_msx_extended[] = {
    {0x75, 0x08, 0x20},  /* Up */
    {0x72, 0x08, 0x40},  /* Down */
    {0x6B, 0x08, 0x10},  /* Left */
    {0x74, 0x08, 0x80},  /* Right */
    {0x70, 0x08, 0x04},  /* Insert */
    {0x71, 0x08, 0x08},  /* Delete */
    {0x6C, 0x08, 0x02},  /* Home */
};

#define PS2_MSX_EXT_SIZE (sizeof(ps2_msx_extended) / sizeof(ps2_msx_extended[0]))

/* ------------------------------------------------------------------ */
/*  PSG emulation (inline, non-I2S mode)                              */
/* ------------------------------------------------------------------ */

static void psg_write(uint32_t data) {
    uint32_t freq;
    psg_register_number = ioport[0xa0];
    if (psg_register_number > 15) return;
    psg_register[psg_register_number] = data;

    switch (psg_register_number & 0xf) {
        case 0: case 1:
            if (psg_register[0] == 0 && psg_register[1] == 0) {
                psg_osc_interval[0] = UINT32_MAX; break;
            }
            freq = psg_master_clock / (psg_register[0] + ((psg_register[1] & 0x0f) << 8));
            freq >>= 4;
            if (freq) { psg_osc_interval[0] = TIME_UNIT / freq; psg_osc_counter[0] = 0; }
            else psg_osc_interval[0] = UINT32_MAX;
            break;
        case 2: case 3:
            if (psg_register[2] == 0 && psg_register[3] == 0) {
                psg_osc_interval[1] = UINT32_MAX; break;
            }
            freq = psg_master_clock / (psg_register[2] + ((psg_register[3] & 0x0f) << 8));
            freq >>= 4;
            if (freq) { psg_osc_interval[1] = TIME_UNIT / freq; psg_osc_counter[1] = 0; }
            else psg_osc_interval[1] = UINT32_MAX;
            break;
        case 4: case 5:
            if (psg_register[4] == 0 && psg_register[5] == 0) {
                psg_osc_interval[2] = UINT32_MAX; break;
            }
            freq = psg_master_clock / (psg_register[4] + ((psg_register[5] & 0x0f) << 8));
            freq >>= 4;
            if (freq) { psg_osc_interval[2] = TIME_UNIT / freq; psg_osc_counter[2] = 0; }
            else psg_osc_interval[2] = UINT32_MAX;
            break;
        case 6:
            if (psg_register[6] == 0) { psg_noise_interval = UINT32_MAX; break; }
            freq = psg_master_clock / (psg_register[6] & 0x1f);
            freq >>= 4;
            if (freq) { psg_noise_interval = TIME_UNIT / freq; psg_noise_counter = 0; }
            else psg_noise_interval = UINT32_MAX;
            break;
        case 7:
            psg_tone_on[0] = ((psg_register[7] & 1) == 0 ? 1 : 0);
            psg_tone_on[1] = ((psg_register[7] & 2) == 0 ? 1 : 0);
            psg_tone_on[2] = ((psg_register[7] & 4) == 0 ? 1 : 0);
            psg_noise_on[0] = ((psg_register[7] & 8) == 0 ? 1 : 0);
            psg_noise_on[1] = ((psg_register[7] & 16) == 0 ? 1 : 0);
            psg_noise_on[2] = ((psg_register[7] & 32) == 0 ? 1 : 0);
            break;
        case 0xb: case 0xc:
            freq = psg_master_clock / (psg_register[0xb] + (psg_register[0xc] << 8));
            if (freq) { psg_envelope_interval = TIME_UNIT / freq; psg_envelope_interval <<= 5; }
            else psg_envelope_interval = UINT32_MAX / 2 - 1;
            break;
        case 0xd:
            psg_envelope_counter = 0;
            break;
    }
}

static void psg_reset(int flag) {
    psg_noise_seed = 12345;
    for (int i = 0; i < (flag ? 15 : 16); i++) psg_register[i] = 0;
    psg_register[7] = 0xff;
    psg_noise_interval = UINT32_MAX;
    psg_envelope_interval = UINT32_MAX / 2 - 1;
    for (int i = 0; i < 3; i++) {
        psg_osc_interval[i] = UINT32_MAX;
        psg_tone_on[i] = 0;
        psg_noise_on[i] = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Audio timer callback - generates PSG samples at SAMPLING_FREQ     */
/* ------------------------------------------------------------------ */

static bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {
    uint8_t tone_output[3], noise_output[3], envelope_volume;
    int32_t psg_mix = 0;
    int32_t fm_mix = 0;
    uint32_t pon_count;

    /* Noise generator */
    psg_noise_counter += SAMPLING_INTERVAL;
    if (psg_noise_counter > psg_noise_interval) {
        psg_noise_seed = (psg_noise_seed >> 1)
            | (((psg_noise_seed << 14) ^ (psg_noise_seed << 16)) & 0x10000);
        psg_noise_output = psg_noise_seed & 1;
        psg_noise_counter -= psg_noise_interval;
    }
    for (int i = 0; i < 3; i++)
        noise_output[i] = psg_noise_output ? psg_noise_on[i] : 0;

    /* Envelope */
    envelope_volume = 0;
    switch (psg_register[13] & 0xf) {
        case 0: case 1: case 2: case 3: case 9:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31 - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            }
            break;
        case 4: case 5: case 6: case 7: case 15:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            }
            break;
        case 8:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31 - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 31;
            }
            break;
        case 10:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31 - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter < psg_envelope_interval * 64) {
                envelope_volume = psg_envelope_counter / psg_envelope_interval - 32;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 31;
            }
            break;
        case 11:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31 - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else envelope_volume = 31;
            break;
        case 12:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
            }
            break;
        case 13:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else envelope_volume = 31;
            break;
        case 14:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter < psg_envelope_interval * 64) {
                envelope_volume = 63 - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
            }
            break;
    }

    /* Oscillators */
    for (int i = 0; i < 3; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) tone_output[i] = psg_tone_on[i];
        else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
            tone_output[i] = psg_tone_on[i];
        } else tone_output[i] = 0;
    }

    /* PSG Mixer */
    for (int j = 0; j < 3; j++) {
        if ((tone_output[j] + noise_output[j]) > 0) {
            if ((psg_register[j + 8] & 0x10) == 0)
                psg_mix += psg_volume[(psg_register[j + 8] & 0xf) * 2 + 1];
            else
                psg_mix += psg_volume[envelope_volume];
        }
    }
    psg_mix = psg_mix / 4 + beep_enable * 63;
    if (psg_mix > 255) psg_mix = 255;

    /* FM (OPLL) - generate one sample */
    if (opll && (audio_mode == AUDIO_MODE_FM || audio_mode == AUDIO_MODE_PSG_FM)) {
        fm_mix = OPLL_calc(opll);
        fm_mix = fm_mix >> 4;  /* Scale down OPLL output */
    }

    /* Final mix: PSG + FM, apply master volume */
    int32_t mixed = 0;
    if (audio_mode == AUDIO_MODE_PSG || audio_mode == AUDIO_MODE_PSG_FM) {
        /* PSG: scale up for audible output (0-255 centered around 128, boosted to 16-bit range) */
        mixed = (psg_mix - 128) * 64;
    }
    mixed += fm_mix;
    mixed = (mixed * master_volume) >> 8;

    /* Clamp to 16-bit range */
    if (mixed > 32767) mixed = 32767;
    if (mixed < -32768) mixed = -32768;

    /* Output to PICO-56 stereo PWM */
    int16_t sample = (int16_t)mixed;
    uint32_t packed = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
    audio_pwm_enqueue_sample(packed);

    return true;
}

/* ------------------------------------------------------------------ */
/*  PS/2 keyboard processing                                          */
/* ------------------------------------------------------------------ */

static void ps2_poll(void) {
    for (int i = 0; i < 32; i++) {
        uint8_t sc = ps2kbd_read();
        if (sc == 0) continue;

        if (sc == 0xE0) {
            ps2_extended = true;
            continue;
        }
        if (sc == 0xF0) {
            ps2_release = true;
            continue;
        }

        bool release = ps2_release;
        bool extended = ps2_extended;
        ps2_release = false;
        ps2_extended = false;

        if (extended) {
            for (int k = 0; k < PS2_MSX_EXT_SIZE; k++) {
                if (ps2_msx_extended[k].scancode == sc) {
                    if (release)
                        keymap[ps2_msx_extended[k].row] |= ps2_msx_extended[k].bitmask;
                    else
                        keymap[ps2_msx_extended[k].row] &= ~ps2_msx_extended[k].bitmask;
                    break;
                }
            }
            if (!release) {
                /* Map extended keys for menu navigation */
                if (sc == 0x75) keypressed = 0x52;       /* Up */
                else if (sc == 0x72) keypressed = 0x51;  /* Down */
                else if (sc == 0x7D) keypressed = 0x4b;  /* Page Up */
                else if (sc == 0x7A) keypressed = 0x4e;  /* Page Down */
            }
        } else {
            /* Left Shift (0x12) or Right Shift (0x59) */
            if (sc == 0x12 || sc == 0x59) {
                if (release) keymap[6] |= 0x01;
                else keymap[6] &= ~0x01;
                continue;
            }
            /* Left Ctrl (0x14) */
            if (sc == 0x14) {
                if (release) keymap[6] |= 0x02;
                else keymap[6] &= ~0x02;
                continue;
            }
            /* Left Alt (0x11) -> GRAPH */
            if (sc == 0x11) {
                if (release) keymap[6] |= 0x04;
                else keymap[6] &= ~0x04;
                continue;
            }

            /* Numlock -> enter menu mode */
            if (sc == 0x77 && !release) {
                menumode = 1;
                keypressed = 0;
                continue;
            }

            /* Map to MSX matrix */
            for (int k = 0; k < PS2_MSX_MAP_SIZE; k++) {
                if (ps2_msx_map[k].scancode == sc) {
                    if (release)
                        keymap[ps2_msx_map[k].row] |= ps2_msx_map[k].bitmask;
                    else
                        keymap[ps2_msx_map[k].row] &= ~ps2_msx_map[k].bitmask;
                    break;
                }
            }

            /* For menu: map Enter, Escape, Backspace */
            if (!release) {
                if (sc == 0x5A) keypressed = 0x28;       /* Enter */
                else if (sc == 0x76) keypressed = 0x29;  /* Escape */
                else if (sc == 0x66) keypressed = 0x2a;  /* Backspace */
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  NES controller -> MSX joystick port                               */
/* ------------------------------------------------------------------ */

/* NES pad bits (CONFIRMED from pico-gb-pico56):
 * 0x01=A, 0x02=B, 0x04=Select, 0x08=Start,
 * 0x10=Up, 0x20=Down, 0x40=Left, 0x80=Right
 *
 * MSX joystick port A register (PSG reg 14):
 * Bit 0: Up, 1: Down, 2: Left, 3: Right, 4: TrigA, 5: TrigB
 * (active low in register read, but gamepad_info is active-high mask) */

#define NES_A      0x01
#define NES_B      0x02
#define NES_SELECT 0x04
#define NES_START  0x08
#define NES_UP     0x10
#define NES_DOWN   0x20
#define NES_LEFT   0x40
#define NES_RIGHT  0x80

static uint8_t prev_pad_state = 0;

static void nespad_update(void) {
    nespad_read_start();
}

/* Helper to press/release an MSX key */
static inline void msx_key_set(msx_key_t key, bool pressed) {
    if (pressed)
        keymap[key.row] &= ~key.mask;
    else
        keymap[key.row] |= key.mask;
}

/* Forward declaration for OSK */
static bool osk_handle_input(uint8_t pad);

static void nespad_finish_update(void) {
    nespad_read_finish();
    uint8_t state = nespad_states[0];
    uint8_t state2 = nespad_states[1];
    uint8_t newly = state & ~prev_pad_state;

    /* START+SELECT combo -> toggle menu (always active) */
    if ((state & (NES_START | NES_SELECT)) == (NES_START | NES_SELECT)) {
        if (newly & (NES_START | NES_SELECT)) {
            menumode = 1;
            keypressed = 0;
            osk_active = false;
        }
        prev_pad_state = state;
        return;
    }

    /* SELECT+RIGHT -> open on-screen keyboard */
    if ((state & (NES_SELECT | NES_RIGHT)) == (NES_SELECT | NES_RIGHT)) {
        if (newly & NES_RIGHT) {
            osk_active = true;
            osk_cursor_x = 0;
            osk_cursor_y = 0;
            osk_prev_pad = state;
        }
        prev_pad_state = state;
        return;
    }

    /* If OSK is active, handle OSK input exclusively */
    if (osk_active) {
        osk_handle_input(state);
        prev_pad_state = state;
        return;
    }

    if (input_mode == INPUT_MODE_JOYSTICK) {
        /* Joystick mode: gamepads act as MSX joysticks */
        uint8_t joy1 = JOY_IDLE;
        if (state & NES_UP)    joy1 &= ~0x01;
        if (state & NES_DOWN)  joy1 &= ~0x02;
        if (state & NES_LEFT)  joy1 &= ~0x04;
        if (state & NES_RIGHT) joy1 &= ~0x08;
        if (state & NES_A)     joy1 &= ~0x10;
        if (state & NES_B)     joy1 &= ~0x20;

        uint8_t joy2 = JOY_IDLE;
        if (state2 & NES_UP)    joy2 &= ~0x01;
        if (state2 & NES_DOWN)  joy2 &= ~0x02;
        if (state2 & NES_LEFT)  joy2 &= ~0x04;
        if (state2 & NES_RIGHT) joy2 &= ~0x08;
        if (state2 & NES_A)     joy2 &= ~0x10;
        if (state2 & NES_B)     joy2 &= ~0x20;

        gamepad_info = joy1;
        gamepad_info2 = joy2;

        /* In joystick mode, still allow SELECT+UP/DOWN for volume */
        if (state & NES_SELECT) {
            if (newly & NES_UP) {
                if (master_volume <= 240) master_volume += 16;
                else master_volume = 255;
            }
            if (newly & NES_DOWN) {
                if (master_volume >= 16) master_volume -= 16;
                else master_volume = 0;
            }
        }
    }
    else if (input_mode == INPUT_MODE_MOUSE) {
        /* Mouse mode: D-pad controls cursor, A/B are mouse buttons */
        /* Accumulate movement - speed scales with how long held */
        if (state & NES_UP)    mouse_dy -= mouse_speed;
        if (state & NES_DOWN)  mouse_dy += mouse_speed;
        if (state & NES_LEFT)  mouse_dx -= mouse_speed;
        if (state & NES_RIGHT) mouse_dx += mouse_speed;

        /* Clamp to signed 8-bit range */
        if (mouse_dx > 127) mouse_dx = 127;
        if (mouse_dx < -128) mouse_dx = -128;
        if (mouse_dy > 127) mouse_dy = 127;
        if (mouse_dy < -128) mouse_dy = -128;

        /* Button state: A=left, B=right */
        mouse_buttons = 0;
        if (state & NES_A) mouse_buttons |= 1;
        if (state & NES_B) mouse_buttons |= 2;

        /* Joystick port returns mouse data (handled in io_read) */
        gamepad_info = JOY_IDLE;
        gamepad_info2 = JOY_IDLE;

        /* SELECT+UP/DOWN for volume */
        if (state & NES_SELECT) {
            if (newly & NES_UP) {
                if (master_volume <= 240) master_volume += 16;
                else master_volume = 255;
            }
            if (newly & NES_DOWN) {
                if (master_volume >= 16) master_volume -= 16;
                else master_volume = 0;
            }
        }
    }
    else {
        /* Keyboard mode: gamepad simulates MSX keyboard */
        /* D-pad -> Arrow keys */
        msx_key_set(key_map_up,    state & NES_UP);
        msx_key_set(key_map_down,  state & NES_DOWN);
        msx_key_set(key_map_left,  state & NES_LEFT);
        msx_key_set(key_map_right, state & NES_RIGHT);

        /* Buttons -> Configured keys */
        msx_key_set(key_map_a,      state & NES_A);
        msx_key_set(key_map_b,      state & NES_B);
        msx_key_set(key_map_start,  state & NES_START);

        /* SELECT: if held with UP/DOWN, adjust volume; otherwise map to key */
        if (state & NES_SELECT) {
            if (newly & NES_UP) {
                if (master_volume <= 240) master_volume += 16;
                else master_volume = 255;
            }
            if (newly & NES_DOWN) {
                if (master_volume >= 16) master_volume -= 16;
                else master_volume = 0;
            }
            /* Only press the select-mapped key if not doing volume */
            if (!(state & (NES_UP | NES_DOWN))) {
                msx_key_set(key_map_select, true);
            } else {
                msx_key_set(key_map_select, false);
            }
        } else {
            msx_key_set(key_map_select, false);
        }

        /* Joystick stays neutral in keyboard mode */
        gamepad_info = JOY_IDLE;
        gamepad_info2 = JOY_IDLE;
    }

    prev_pad_state = state;
}

/* ------------------------------------------------------------------ */
/*  Z80 memory read/write (from msxemulator.c, unchanged logic)       */
/* ------------------------------------------------------------------ */

/* Forward declarations for ROM loading functions used in memory handlers */
static bool disk_rom_valid(void);

/* Load a single 8KB bank from SD into bank_cache */
static void load_bank_from_sd(uint8_t bank_num) {
    if (cached_bank_num == bank_num) return;  /* Already cached */
    if (!large_cart_mode || large_cart_path[0] == 0) return;

    FIL f;
    if (f_open(&f, large_cart_path, FA_READ) != FR_OK) return;

    uint32_t offset = (uint32_t)bank_num * 0x2000;
    if (offset >= large_cart_size) {
        f_close(&f);
        memset(bank_cache, 0xFF, 0x2000);
        cached_bank_num = bank_num;
        return;
    }

    f_lseek(&f, offset);
    UINT br;
    f_read(&f, bank_cache, 0x2000, &br);
    f_close(&f);
    cached_bank_num = bank_num;
}

/* Read a byte from large cart ROM (handles multi-region layout) */
static uint8_t read_large_cart_byte(uint32_t offset) {
    /* Bounds check - return 0xFF for reads beyond ROM size */
    if (offset >= large_cart_size) return 0xFF;

    if (offset < 0x8000) {
        /* Banks 0-3: cartrom_buf (32KB) */
        return cartrom_buf[offset];
    } else {
        /* Banks 4+: on-demand from SD */
        uint8_t bank_num = offset >> 13;
        if (cached_bank_num != bank_num) {
            load_bank_from_sd(bank_num);
        }
        return bank_cache[offset & 0x1FFF];
    }
}

static uint8_t mem_read(void *context, uint16_t address) {
    uint8_t slot, extslotno, bank;
    slot = ioport[0xa8];
    bank = (address >> 14);
    slot >>= bank * 2;
    slot &= 3;

    if (address == 0xffff) {
        /* Only slot 3 is expanded - return complement of subslot register */
        if (slot == 3) {
            return ~extslot[3];
        }
        return 0xff;
    }

    switch (slot) {
        case 0:
            return basicrom[address];
        case 1:
            if (cart_enable[0] && address >= 0x4000 && address < 0xC000) {
                /* Cartridge mapped at 0x4000-0xBFFF
                 * Bank index: 0=4000-5FFF, 1=6000-7FFF, 2=8000-9FFF, 3=A000-BFFF */
                uint8_t bank_idx = ((address - 0x4000) >> 13) & 3;
                uint32_t rom_offset;
                switch (carttype[0]) {
                    case 0: /* Plain ROM - mirror within 32KB window like reference */
                        rom_offset = (address - 0x4000) & 0x7FFF;  /* Mask to 32KB */
                        if (cart_loaded[0] > 0 && cart_loaded[0] < 0x8000) {
                            rom_offset = rom_offset % cart_loaded[0];  /* Mirror smaller ROMs */
                        }
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 1: /* ASCII8: 8KB banks */
                        rom_offset = megarom[bank_idx] * 0x2000 + (address & 0x1fff);
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 2: /* ASCII16: 16KB banks */
                        rom_offset = megarom[(address < 0x8000) ? 0 : 1] * 0x4000 + (address & 0x3fff);
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 3: /* Konami without SCC: 8KB banks, bank 0 fixed */
                        if (bank_idx == 0) {
                            rom_offset = address & 0x1fff;
                        } else {
                            rom_offset = megarom[bank_idx] * 0x2000 + (address & 0x1fff);
                        }
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 4: /* Konami SCC: 8KB banks */
                        rom_offset = megarom[bank_idx] * 0x2000 + (address & 0x1fff);
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 5: /* ASCII8 with SRAM: SRAM at 0x8000-0xBFFF when bank has bit 7 set */
                        if (sram_enabled[0] && address >= 0x8000 && address < 0xC000) {
                            return cart_sram[address & 0x7ff];
                        }
                        rom_offset = megarom[bank_idx] * 0x2000 + (address & 0x1fff);
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                    case 6: /* ASCII16 with SRAM: SRAM at 0x8000-0xBFFF when enabled */
                        if (sram_enabled[0] && address >= 0x8000 && address < 0xC000) {
                            return cart_sram[address & 0x7ff];
                        }
                        rom_offset = megarom[(address < 0x8000) ? 0 : 1] * 0x4000 + (address & 0x3fff);
                        return large_cart_mode ? read_large_cart_byte(rom_offset) : cartrom1[rom_offset];
                }
            }
            return 0xff;
        case 2:
            if (cart_enable[1] && address >= 0x4000 && address < 0xC000) {
                uint8_t bank_idx = ((address - 0x4000) >> 13) & 3;
                switch (carttype[1]) {
                    case 0: return cartrom2[address - 0x4000];
                    case 1: return cartrom2[megarom[bank_idx + 4] * 0x2000 + (address & 0x1fff)];
                    case 2: return cartrom2[megarom[((address < 0x8000) ? 0 : 1) + 4] * 0x4000 + (address & 0x3fff)];
                    case 3:
                        if (bank_idx == 0) return cartrom2[address & 0x1fff];
                        return cartrom2[megarom[bank_idx + 4] * 0x2000 + (address & 0x1fff)];
                    case 4: return cartrom2[megarom[bank_idx + 4] * 0x2000 + (address & 0x1fff)];
                }
            }
            return 0xff;
        case 3:
            extslotno = extslot[3];
            extslotno >>= bank * 2;
            extslotno &= 3;
            switch (extslotno) {
                case 0: return mainram[(address & 0x3fff) + memmap[bank] * 0x4000];
                case 1:
                    /* Disk ROM (slot 3-1) - only valid in pages 1-2 (0x4000-0xBFFF) */
                    if (disk_rom_valid() && address >= 0x4000 && address < 0x8000) {
                        /* FDC registers at 0x7FF8-0x7FFF */
                        if (address >= 0x7FF8) {
                            switch (address & 7) {
                                case 0: return fdc_read_status();
                                case 1: return fdc_read_track();
                                case 2: return fdc_read_sector();
                                case 3: return fdc_read();
                                case 4: return fdc_read_control1();
                                case 5: return fdc_read_control2();
                                case 6:
                                case 7: return fdc_read_status_flag();
                            }
                        }
                        return diskrom_ram[address & 0x3FFF];
                    }
                    return 0xff;
                default: return 0xff;
            }
        default: return 0xff;
    }
}

static void mem_write(void *context, uint16_t address, uint8_t data) {
    uint8_t slot, extslotno, bank;
    slot = ioport[0xa8];
    bank = (address >> 14);
    slot >>= bank * 2;
    slot &= 3;

    if (address == 0xffff) {
        extslot[slot] = data;
        return;
    }

    switch (slot) {
        case 0: return;
        case 1:
            if (cart_enable[0]) {
                switch (carttype[0]) {
                    case 1:
                        if (address >= 0x6000 && address < 0x6800) megarom[0] = data;
                        else if (address >= 0x6800 && address < 0x7000) megarom[1] = data;
                        else if (address >= 0x7000 && address < 0x7800) megarom[2] = data;
                        else if (address >= 0x7800 && address < 0x8000) megarom[3] = data;
                        return;
                    case 2:
                        if (address >= 0x6000 && address < 0x6800) megarom[0] = data;
                        else if (address >= 0x7000 && address < 0x7800) megarom[1] = data;
                        return;
                    case 3:
                        if (address >= 0x6000 && address < 0x8000) megarom[1] = data & 0x3f;
                        else if (address >= 0x8000 && address < 0xa000) megarom[2] = data & 0x3f;
                        else if (address >= 0xa000 && address < 0xc000) megarom[3] = data & 0x3f;
                        return;
                    case 4:
                        if (address >= 0x5000 && address < 0x5800) megarom[0] = data & 0x3f;
                        else if (address >= 0x7000 && address < 0x7800) megarom[1] = data & 0x3f;
                        else if (address >= 0x9000 && address < 0x9800) megarom[2] = data & 0x3f;
                        else if (address >= 0xb000 && address < 0xb800) megarom[3] = data & 0x3f;
                        return;
                    case 5: /* ASCII8 with SRAM */
                        if (address >= 0x6000 && address < 0x6800) { megarom[0] = data; sram_enabled[0] = (data & 0x80) != 0; }
                        else if (address >= 0x6800 && address < 0x7000) { megarom[1] = data; }
                        else if (address >= 0x7000 && address < 0x7800) { megarom[2] = data; sram_enabled[0] = (data & 0x80) != 0; }
                        else if (address >= 0x7800 && address < 0x8000) { megarom[3] = data; }
                        else if (sram_enabled[0] && address >= 0x8000 && address < 0xC000) {
                            cart_sram[address & 0x7ff] = data;
                            sram_dirty = true;
                        }
                        return;
                    case 6: /* ASCII16 with SRAM */
                        if (address >= 0x6000 && address < 0x6800) { megarom[0] = data; }
                        else if (address >= 0x7000 && address < 0x7800) { megarom[1] = data; sram_enabled[0] = (data & 0x10) != 0; }
                        else if (sram_enabled[0] && address >= 0x8000 && address < 0xC000) {
                            cart_sram[address & 0x7ff] = data;
                            sram_dirty = true;
                        }
                        return;
                    default: break;
                }
            }
            return;
        case 2:
            if (cart_enable[1]) {
                switch (carttype[1]) {
                    case 1:
                        if (address >= 0x6000 && address < 0x6800) megarom[4] = data;
                        else if (address >= 0x6800 && address < 0x7000) megarom[5] = data;
                        else if (address >= 0x7000 && address < 0x7800) megarom[6] = data;
                        else if (address >= 0x7800 && address < 0x8000) megarom[7] = data;
                        return;
                    case 2:
                        if (address >= 0x6000 && address < 0x6800) megarom[4] = data;
                        else if (address >= 0x7000 && address < 0x7800) megarom[5] = data;
                        return;
                    case 3:
                        if (address >= 0x6000 && address < 0x8000) megarom[5] = data & 0x3f;
                        else if (address >= 0x8000 && address < 0xa000) megarom[6] = data & 0x3f;
                        else if (address >= 0xa000 && address < 0xc000) megarom[7] = data & 0x3f;
                        return;
                    case 4:
                        if (address >= 0x5000 && address < 0x5800) megarom[4] = data & 0x3f;
                        else if (address >= 0x7000 && address < 0x7800) megarom[5] = data & 0x3f;
                        else if (address >= 0x9000 && address < 0x9800) megarom[6] = data & 0x3f;
                        else if (address >= 0xb000 && address < 0xb800) megarom[7] = data & 0x3f;
                        return;
                    default: break;
                }
            }
            return;
        case 3:
            extslotno = extslot[3];
            extslotno >>= bank * 2;
            extslotno &= 3;
            switch (extslotno) {
                case 0:
                    mainram[(address & 0x3fff) + memmap[bank] * 0x4000] = data;
                    return;
                case 1:
                    if (address == 0x7ff8) { fdc_command_write(data); return; }
                    if (address == 0x7ff9) { fdc_write_track(data); return; }
                    if (address == 0x7ffa) { fdc_write_sector(data); return; }
                    if (address == 0x7ffb) { fdc_write(data); return; }
                    if (address == 0x7ffc) { fdc_write_control1(data); return; }
                    if (address == 0x7ffd) { fdc_write_control2(data); return; }
                    return;
                default: return;
            }
        default: break;
    }
}

/* ------------------------------------------------------------------ */
/*  MSX Mouse emulation                                               */
/* ------------------------------------------------------------------ */

/* Read mouse data nibble based on strobe state
 * MSX mouse protocol:
 * - Strobe toggles advance through 4 nibbles
 * - Nibble 0: X high (bits 7-4 of delta)
 * - Nibble 1: X low (bits 3-0 of delta)
 * - Nibble 2: Y high (bits 7-4 of delta)
 * - Nibble 3: Y low (bits 3-0 of delta)
 * - Buttons on bits 4-5 (active low)
 */
static uint8_t mouse_read_port(void) {
    uint8_t result = JOY_IDLE;  /* Default: no buttons, no movement */

    /* Get current nibble data */
    switch (mouse_nibble) {
        case 0: result = (mouse_dx >> 4) & 0x0F; break;
        case 1: result = mouse_dx & 0x0F; break;
        case 2: result = (mouse_dy >> 4) & 0x0F; break;
        case 3: result = mouse_dy & 0x0F; break;
    }

    /* Add button state (active low on bits 4-5) */
    if (!(mouse_buttons & 1)) result |= 0x10;  /* Left button not pressed */
    if (!(mouse_buttons & 2)) result |= 0x20;  /* Right button not pressed */

    return result;
}

/* Handle strobe signal change from PSG register 15 */
static void mouse_strobe_update(uint8_t new_strobe) {
    if (new_strobe != mouse_strobe) {
        mouse_strobe = new_strobe;
        /* Advance to next nibble on strobe change */
        mouse_nibble = (mouse_nibble + 1) & 3;
        /* Clear deltas after all 4 nibbles read */
        if (mouse_nibble == 0) {
            mouse_dx = 0;
            mouse_dy = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Z80 I/O read/write                                                */
/* ------------------------------------------------------------------ */

static uint8_t io_read(void *context, uint16_t address) {
    uint8_t data, b;
    address &= 0xff;

    switch (address) {
        case 0x98:
            return vrEmuTms9918ReadData(mainscreen);
        case 0x99:
            /* VDP status register:
             * Bit 7: Frame flag (vsync) - set at line 192, cleared on read
             * Bit 6: 5th sprite flag
             * Bit 5: Sprite collision flag
             * Bits 4-0: 5th sprite number
             * We get sprite info from library, add our own vsync flag */
            b = vrEmuTms9918ReadStatus(mainscreen) & 0x7F;  /* Clear bit 7 from library */
            if (video_vsync) { b |= 0x80; video_vsync = 0; }  /* Add our vsync flag */
            return b;
        case 0xa2:
            if (ioport[0xa0] < 0x0e) {
                return psg_register[ioport[0xa0] & 0xf];
            } else if (ioport[0xa0] == 0xe) {
                b = 0;
                if (key_kana_jis) b |= 0x40;
                /* Check if mouse mode is active */
                if (input_mode == INPUT_MODE_MOUSE && !gamepad_select) {
                    b |= mouse_read_port();
                } else if (!gamepad_select) {
                    b |= gamepad_info;
                } else {
                    b |= 0x3f;
                }
                return b;
            } else if (ioport[0xa0] == 0xf) {
                b = 0x7f;
                if (key_kana == 0) b |= 0x80;
                return b;
            }
            return 0xff;
        case 0xa3:
            return 0xff;
        case 0xa9:
            return keymap[ioport[0xaa] & 0xf];
        default:
            break;
    }
    return ioport[address & 0xff];
}

static uint32_t vdp_write_count = 0;

static void io_write(void *context, uint16_t address, uint8_t data) {
    uint8_t b;
    address &= 0xff;

    switch (address) {
        case 0x98:
            vrEmuTms9918WriteData(mainscreen, data);
            vdp_write_count++;
            return;
        case 0x99:
            vrEmuTms9918WriteAddr(mainscreen, data);
            vdp_write_count++;
            return;
        case 0xa0:
            ioport[0xa0] = data;
            return;
        case 0xa1:
            psg_write(data);
            if (ioport[0xa0] == 0xf) {
                key_kana = (data & 0x80) ? 0 : 1;
                gamepad_select = (data & 0x40) ? 1 : 0;
                /* Mouse strobe: bit 4 for port 1, bit 5 for port 2 */
                if (input_mode == INPUT_MODE_MOUSE) {
                    mouse_strobe_update((data >> 4) & 1);
                }
            }
            return;
        case 0xa8:
            /* Slot select register - critical for cartridge scanning! */
            ioport[0xa8] = data;
            return;
        case 0xaa:
            if (data & 0x10) tape_ready = 0;
            else {
                tape_ready = 1;
                if (ioport[0xaa] & 0x10) tape_phase = 0;
            }
            key_caps = (data & 0x40) ? 0 : 1;
            beep_enable = (data & 0x80) ? 1 : 0;
            ioport[0xaa] = data;
            return;
        case 0xab:
            if ((data & 0x80) == 0) {
                b = (data & 0x0e) >> 1;
                if (b == 4) { tape_ready = (data & 1) ? 0 : 1; if (!(data & 1)) tape_phase = 0; }
                if (b == 6) { key_caps = (data & 1) ? 0 : 1; }
                if (b == 7) { beep_enable = (data & 1) ? 1 : 0; }
                if (data & 1) ioport[0xaa] |= 1 << b;
                else ioport[0xaa] &= ~(1 << b);
            }
            return;
        case 0x7c:
            /* OPLL (MSX-MUSIC / YM2413) register select */
            opll_reg = data;
            return;
        case 0x7d:
            /* OPLL (MSX-MUSIC / YM2413) data write */
            if (opll) OPLL_writeReg(opll, opll_reg, data);
            return;
        case 0xfc: case 0xfd: case 0xfe: case 0xff:
            ioport[address & 0xff] = data;
            /* Use modulo for correct wrapping with non-power-of-2 bank counts */
            memmap[address & 3] = data % ram_max_banks[ram_size];
            return;
        default:
            break;
    }
    ioport[address & 0xff] = data;
}

static uint8_t ird_read(void *context, uint16_t address) {
    z80_int(&cpu, FALSE);
    return 0xff;
}

static void reti_callback(void *context) {
    /* Nothing needed for MSX IM1 */
}

/* ------------------------------------------------------------------ */
/*  VDP -> Framebuffer rendering (called during hsync-like timing)    */
/* ------------------------------------------------------------------ */

static void render_vdp_scanline(uint32_t line) {
    uint8_t bgcolor;
    VrEmuTms9918 *screen = menumode ? menuscreen : mainscreen;

    bgcolor = vrEmuTms9918RegValue(screen, TMS_REG_FG_BG_COLOR) & 0x0f;
    vrEmuTms9918ScanLine(screen, line, scandata);

    uint8_t *dst = &framebuffer[(line + FB_VBORDER) * FB_WIDTH];

    /* If OSK is active, skip rendering over the OSK area */
    if (osk_active) {
        uint32_t fb_row = line + FB_VBORDER;
        /* OSK covers rows 140-187, columns 64-191 */
        if (fb_row >= 140 && fb_row < 188) {
            for (int x = 0; x < 64; x++)
                dst[x] = scandata[x] & 0x0f;
            for (int x = 192; x < 256; x++)
                dst[x] = scandata[x] & 0x0f;
            return;
        }
    }

    for (int x = 0; x < 256; x++) {
        dst[x] = scandata[x] & 0x0f;
    }
}

/* Fill top and bottom borders with background color */
static void render_vdp_borders(void) {
    uint8_t bgcolor;
    VrEmuTms9918 *screen = menumode ? menuscreen : mainscreen;
    bgcolor = vrEmuTms9918RegValue(screen, TMS_REG_FG_BG_COLOR) & 0x0f;

    memset(framebuffer, bgcolor, FB_WIDTH * FB_VBORDER);
    memset(framebuffer + (FB_VBORDER + MSX_HEIGHT) * FB_WIDTH, bgcolor, FB_WIDTH * FB_VBORDER);
}

/* ------------------------------------------------------------------ */
/*  On-screen keyboard overlay                                        */
/* ------------------------------------------------------------------ */

/* Draw a single character to framebuffer using font data */
static void osk_draw_char(int x, int y, char c, uint8_t fg, uint8_t bg) {
    if (c < 0) c = ' ';
    const uint8_t *glyph = &font[(unsigned char)c * 8];
    for (int row = 0; row < 8; row++) {
        uint8_t *dst = &framebuffer[(y + row) * FB_WIDTH + x];
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            dst[col] = (bits & 0x80) ? fg : bg;
            bits <<= 1;
        }
    }
}

/* Draw the OSK overlay on the framebuffer */
static void osk_draw_overlay(void) {
    if (!osk_active) return;

    /* OSK position: centered, lower portion of screen */
    int osk_x = 64;   /* Left edge */
    int osk_y = 140;  /* Top edge */
    int box_w = 128;  /* Width in pixels */
    int box_h = 48;   /* Height in pixels */

    /* Draw semi-transparent background box */
    for (int y = osk_y; y < osk_y + box_h; y++) {
        uint8_t *dst = &framebuffer[y * FB_WIDTH + osk_x];
        for (int x = 0; x < box_w; x++) {
            dst[x] = 1;  /* Dark blue background */
        }
    }

    /* Draw border */
    for (int x = osk_x; x < osk_x + box_w; x++) {
        framebuffer[osk_y * FB_WIDTH + x] = 15;
        framebuffer[(osk_y + box_h - 1) * FB_WIDTH + x] = 15;
    }
    for (int y = osk_y; y < osk_y + box_h; y++) {
        framebuffer[y * FB_WIDTH + osk_x] = 15;
        framebuffer[y * FB_WIDTH + osk_x + box_w - 1] = 15;
    }

    /* Draw keyboard characters */
    for (int row = 0; row < OSK_ROWS; row++) {
        for (int col = 0; col < OSK_COLS && osk_layout[row][col]; col++) {
            int cx = osk_x + 8 + col * 9;
            int cy = osk_y + 6 + row * 10;
            char ch = osk_layout[row][col];

            /* Special display for space key */
            if (row == 2 && col == 11) ch = '_';  /* Space shown as underscore */

            uint8_t fg = 15, bg = 1;
            if (row == osk_cursor_y && col == osk_cursor_x) {
                fg = 1; bg = 15;  /* Inverted for cursor */
            }

            /* Draw Enter as [E] */
            if (row == 3 && col == 11) {
                osk_draw_char(cx - 3, cy, '[', fg, bg);
                osk_draw_char(cx, cy, 'E', fg, bg);
                osk_draw_char(cx + 6, cy, ']', fg, bg);
            } else {
                osk_draw_char(cx, cy, ch, fg, bg);
            }
        }
    }
}

/* Handle OSK input, returns true if OSK consumed the input */
static bool osk_handle_input(uint8_t pad) {
    if (!osk_active) return false;

    uint8_t pressed = pad & ~osk_prev_pad;
    osk_prev_pad = pad;

    /* Handle key press timer (auto-release after typing) */
    if (osk_key_timer > 0) {
        osk_key_timer--;
        if (osk_key_timer == 0) {
            /* Release all keys */
            for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xFF;
        }
    }

    /* D-pad navigation */
    if (pressed & NES_UP) {
        if (osk_cursor_y > 0) osk_cursor_y--;
    }
    if (pressed & NES_DOWN) {
        if (osk_cursor_y < OSK_ROWS - 1) osk_cursor_y++;
    }
    if (pressed & NES_LEFT) {
        if (osk_cursor_x > 0) osk_cursor_x--;
    }
    if (pressed & NES_RIGHT) {
        int max_col = strlen(osk_layout[osk_cursor_y]) - 1;
        if (osk_cursor_x < max_col) osk_cursor_x++;
    }

    /* A button: type the selected character */
    if (pressed & NES_A) {
        msx_key_t key = osk_keys[osk_cursor_y][osk_cursor_x];
        if (key.row < 11 && key.mask != 0) {
            keymap[key.row] &= ~key.mask;  /* Press key */
            osk_key_timer = 5;  /* Hold for a few frames */
        }
    }

    /* B button: close OSK */
    if (pressed & NES_B) {
        osk_active = false;
        osk_prev_pad = 0;
        /* Release all keys */
        for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xFF;
        return true;
    }

    return true;  /* OSK consumed input */
}

/* ------------------------------------------------------------------ */
/*  Menu system (simplified for PICO-56 with FatFS)                   */
/* ------------------------------------------------------------------ */

/* Forward declaration for SRAM save */
static void save_sram(void);

/* Save VDP state to SD card before entering menu */
static void vdp_save_state(void) {
    FIL f;
    UINT bw;
    if (f_open(&f, VDP_SAVE_FILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        vdp_state_saved = false;
        return;
    }

    /* Save 8 registers */
    uint8_t regs[8];
    for (int i = 0; i < 8; i++) {
        regs[i] = vrEmuTms9918RegValue(mainscreen, (vrEmuTms9918Register)i);
    }
    f_write(&f, regs, 8, &bw);

    /* Save full 16KB VRAM in chunks */
    uint8_t chunk[256];
    for (int addr = 0; addr < 16384; addr += 256) {
        for (int i = 0; i < 256; i++) {
            chunk[i] = vrEmuTms9918VramValue(mainscreen, addr + i);
        }
        f_write(&f, chunk, 256, &bw);
    }

    f_close(&f);
    vdp_state_saved = true;

    /* Also save SRAM if dirty */
    save_sram();
}

/* Restore VDP state from SD card after leaving menu */
static void vdp_restore_state(void) {
    if (!vdp_state_saved) return;

    FIL f;
    UINT br;
    if (f_open(&f, VDP_SAVE_FILE, FA_READ) != FR_OK) {
        vdp_state_saved = false;
        return;
    }

    /* Restore 8 registers */
    uint8_t regs[8];
    if (f_read(&f, regs, 8, &br) != FR_OK || br != 8) {
        f_close(&f);
        f_unlink(VDP_SAVE_FILE);
        vdp_state_saved = false;
        return;
    }
    for (int i = 0; i < 8; i++) {
        vrEmuTms9918WriteRegValue(mainscreen, (vrEmuTms9918Register)i, regs[i]);
    }

    /* Restore full 16KB VRAM in chunks */
    uint8_t chunk[256];
    for (int addr = 0; addr < 16384; addr += 256) {
        if (f_read(&f, chunk, 256, &br) != FR_OK || br != 256) {
            f_close(&f);
            f_unlink(VDP_SAVE_FILE);
            vdp_state_saved = false;
            return;
        }
        vrEmuTms9918SetAddressWrite(mainscreen, addr);
        for (int i = 0; i < 256; i++) {
            vrEmuTms9918WriteData(mainscreen, chunk[i]);
        }
    }

    f_close(&f);
    f_unlink(VDP_SAVE_FILE);  /* Clean up temp file */
}

/* ------------------------------------------------------------------ */
/*  SRAM Save/Load to SD Card                                         */
/* ------------------------------------------------------------------ */

static void set_sram_filename(const char *rompath) {
    /* Extract ROM name and create SRAM filename */
    const char *name = strrchr(rompath, '/');
    name = name ? name + 1 : rompath;

    f_mkdir("/MSX/SRAM");
    snprintf(sram_filename, sizeof(sram_filename), "/MSX/SRAM/%.48s", name);

    /* Replace extension with .srm */
    char *dot = strrchr(sram_filename, '.');
    if (dot) {
        strcpy(dot, ".srm");
    } else {
        strcat(sram_filename, ".srm");
    }
}

static void save_sram(void) {
    if (!sram_dirty) return;
    if (!sram_enabled[0] && !sram_enabled[1]) return;
    if (sram_filename[0] == 0) return;

    FIL f;
    UINT bw;
    if (f_open(&f, sram_filename, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&f, cart_sram, CART_SRAM_SIZE, &bw);
        f_close(&f);
        sram_dirty = false;
    }
}

static void load_sram(void) {
    if (sram_filename[0] == 0) return;

    memset(cart_sram, 0xFF, CART_SRAM_SIZE);  /* Default to 0xFF */

    FIL f;
    UINT br;
    if (f_open(&f, sram_filename, FA_READ) == FR_OK) {
        f_read(&f, cart_sram, CART_SRAM_SIZE, &br);
        f_close(&f);
    }
    sram_dirty = false;
}

/* ------------------------------------------------------------------ */
/*  Config Save/Load                                                   */
/* ------------------------------------------------------------------ */

static void save_config(void) {
    f_mkdir("/MSX");
    FIL f;
    UINT bw;
    if (f_open(&f, CONFIG_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;

    config_data_t cfg;
    cfg.magic = CONFIG_MAGIC;
    cfg.theme = current_theme;
    cfg.rom_count = rom_settings_count;
    cfg.disk_rom_enabled = disk_rom_enabled;
    cfg.ram_size = ram_size;
    memcpy(cfg.roms, rom_settings, sizeof(rom_settings));

    f_write(&f, &cfg, sizeof(cfg), &bw);
    f_close(&f);
}

static void load_config(void) {
    FIL f;
    UINT br;
    if (f_open(&f, CONFIG_PATH, FA_READ) != FR_OK) return;

    config_data_t cfg;
    if (f_read(&f, &cfg, sizeof(cfg), &br) == FR_OK && br == sizeof(cfg)) {
        if (cfg.magic == CONFIG_MAGIC) {
            current_theme = cfg.theme < THEME_COUNT ? cfg.theme : THEME_DEFAULT;
            rom_settings_count = cfg.rom_count;
            if (rom_settings_count > MAX_ROM_SETTINGS) rom_settings_count = MAX_ROM_SETTINGS;
            memcpy(rom_settings, cfg.roms, sizeof(rom_settings));
            disk_rom_enabled = cfg.disk_rom_enabled > 1 ? 1 : cfg.disk_rom_enabled;
            ram_size = cfg.ram_size > 4 ? 1 : cfg.ram_size;
        }
    }
    f_close(&f);
}

static void apply_color_theme(void) {
    uint8_t fg = theme_colors[current_theme][0];
    uint8_t bg = theme_colors[current_theme][1];
    uint8_t color_byte = (fg << 4) | bg;

    /* Set VDP register 7 for border/background (menu only) */
    vrEmuTms9918SetFgBgColor(menuscreen, (vrEmuTms9918Color)fg, (vrEmuTms9918Color)bg);

    /* Update color table for menu character colors - first 16 bytes for normal text */
    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_COLOR_ADDRESS);
    for (int i = 0; i < 16; i++)
        vrEmuTms9918WriteData(menuscreen, color_byte);
    /* Next 16 bytes for highlighted text - swap fg/bg for visibility */
    uint8_t hl_byte = (bg << 4) | fg;
    for (int i = 0; i < 16; i++)
        vrEmuTms9918WriteData(menuscreen, hl_byte);
}

/* Find or create ROM setting entry */
static rom_setting_t* find_rom_setting(const char *name, const uint8_t *header) {
    /* Search by header match first */
    for (int i = 0; i < rom_settings_count; i++) {
        if (memcmp(rom_settings[i].header, header, 4) == 0) {
            return &rom_settings[i];
        }
    }
    return NULL;
}

static rom_setting_t* add_rom_setting(const char *name, const uint8_t *header) {
    /* Check if already exists */
    rom_setting_t *existing = find_rom_setting(name, header);
    if (existing) return existing;

    /* Add new entry if space available */
    if (rom_settings_count >= MAX_ROM_SETTINGS) return NULL;

    rom_setting_t *entry = &rom_settings[rom_settings_count++];
    memset(entry, 0, sizeof(*entry));
    strncpy(entry->name, name, ROM_NAME_LEN - 1);
    memcpy(entry->header, header, 4);
    entry->mapper_type = 0;  /* Auto */
    entry->ram_size = 0;     /* 64KB default */

    save_config();
    return entry;
}

/* ------------------------------------------------------------------ */
/*  Save State to SD Card                                             */
/* ------------------------------------------------------------------ */

#define SAVESTATE_MAGIC 0x4D535831  /* "MSX1" */
#define SAVESTATE_VERSION 1

static bool save_state_to_slot(int slot) {
    if (slot < 0 || slot > 9) return false;
    if (!cart_loaded[0] && !cart_loaded[1]) return false;

    char path[32];
    snprintf(path, sizeof(path), "/MSX/SAVES/slot_%d.sav", slot);
    f_mkdir("/MSX/SAVES");

    FIL f;
    UINT bw;
    if (f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        return false;
    }

    /* Header */
    uint32_t magic = SAVESTATE_MAGIC;
    uint32_t version = SAVESTATE_VERSION;
    f_write(&f, &magic, 4, &bw);
    f_write(&f, &version, 4, &bw);

    /* Z80 CPU state - save key registers */
    f_write(&f, &cpu.cycles, sizeof(cpu.cycles), &bw);
    f_write(&f, &cpu.ix_iy, sizeof(cpu.ix_iy), &bw);
    f_write(&f, &cpu.pc, sizeof(cpu.pc), &bw);
    f_write(&f, &cpu.sp, sizeof(cpu.sp), &bw);
    f_write(&f, &cpu.xy, sizeof(cpu.xy), &bw);
    f_write(&f, &cpu.memptr, sizeof(cpu.memptr), &bw);
    f_write(&f, &cpu.af, sizeof(cpu.af), &bw);
    f_write(&f, &cpu.bc, sizeof(cpu.bc), &bw);
    f_write(&f, &cpu.de, sizeof(cpu.de), &bw);
    f_write(&f, &cpu.hl, sizeof(cpu.hl), &bw);
    f_write(&f, &cpu.af_, sizeof(cpu.af_), &bw);
    f_write(&f, &cpu.bc_, sizeof(cpu.bc_), &bw);
    f_write(&f, &cpu.de_, sizeof(cpu.de_), &bw);
    f_write(&f, &cpu.hl_, sizeof(cpu.hl_), &bw);
    f_write(&f, &cpu.r, sizeof(cpu.r), &bw);
    f_write(&f, &cpu.i, sizeof(cpu.i), &bw);
    f_write(&f, &cpu.r7, sizeof(cpu.r7), &bw);
    f_write(&f, &cpu.im, sizeof(cpu.im), &bw);
    f_write(&f, &cpu.request, sizeof(cpu.request), &bw);
    f_write(&f, &cpu.resume, sizeof(cpu.resume), &bw);
    f_write(&f, &cpu.iff1, sizeof(cpu.iff1), &bw);
    f_write(&f, &cpu.iff2, sizeof(cpu.iff2), &bw);
    f_write(&f, &cpu.q, sizeof(cpu.q), &bw);
    f_write(&f, &cpu.int_line, sizeof(cpu.int_line), &bw);
    f_write(&f, &cpu.halt_line, sizeof(cpu.halt_line), &bw);

    /* Emulator state */
    f_write(&f, &cpu_cycles, sizeof(cpu_cycles), &bw);
    f_write(&f, &cpu_hsync, sizeof(cpu_hsync), &bw);
    f_write(&f, extslot, sizeof(extslot), &bw);
    f_write(&f, megarom, sizeof(megarom), &bw);
    f_write(&f, memmap, sizeof(memmap), &bw);
    f_write(&f, ioport, sizeof(ioport), &bw);
    f_write(&f, carttype, sizeof(carttype), &bw);
    f_write(&f, cart_enable, sizeof(cart_enable), &bw);

    /* Main RAM (always save full 112KB = 0x1C000 for state compatibility) */
    for (int i = 0; i < 0x1C000; i += 512) {
        f_write(&f, &mainram[i], 512, &bw);
    }

    /* VDP state */
    uint8_t vdp_regs[8];
    for (int i = 0; i < 8; i++) {
        vdp_regs[i] = vrEmuTms9918RegValue(mainscreen, (vrEmuTms9918Register)i);
    }
    f_write(&f, vdp_regs, 8, &bw);

    /* VDP VRAM (16KB) - write in chunks */
    uint8_t vram_chunk[256];
    for (int addr = 0; addr < 16384; addr += 256) {
        for (int i = 0; i < 256; i++) {
            vram_chunk[i] = vrEmuTms9918VramValue(mainscreen, addr + i);
        }
        f_write(&f, vram_chunk, 256, &bw);
    }

    f_close(&f);
    return true;
}

static bool load_state_from_slot(int slot) {
    if (slot < 0 || slot > 9) return false;

    char path[32];
    snprintf(path, sizeof(path), "/MSX/SAVES/slot_%d.sav", slot);

    FIL f;
    UINT br;
    if (f_open(&f, path, FA_READ) != FR_OK) {
        return false;
    }

    /* Verify header */
    uint32_t magic, version;
    f_read(&f, &magic, 4, &br);
    f_read(&f, &version, 4, &br);
    if (magic != SAVESTATE_MAGIC || version != SAVESTATE_VERSION) {
        f_close(&f);
        return false;
    }

    /* Z80 CPU state */
    f_read(&f, &cpu.cycles, sizeof(cpu.cycles), &br);
    f_read(&f, &cpu.ix_iy, sizeof(cpu.ix_iy), &br);
    f_read(&f, &cpu.pc, sizeof(cpu.pc), &br);
    f_read(&f, &cpu.sp, sizeof(cpu.sp), &br);
    f_read(&f, &cpu.xy, sizeof(cpu.xy), &br);
    f_read(&f, &cpu.memptr, sizeof(cpu.memptr), &br);
    f_read(&f, &cpu.af, sizeof(cpu.af), &br);
    f_read(&f, &cpu.bc, sizeof(cpu.bc), &br);
    f_read(&f, &cpu.de, sizeof(cpu.de), &br);
    f_read(&f, &cpu.hl, sizeof(cpu.hl), &br);
    f_read(&f, &cpu.af_, sizeof(cpu.af_), &br);
    f_read(&f, &cpu.bc_, sizeof(cpu.bc_), &br);
    f_read(&f, &cpu.de_, sizeof(cpu.de_), &br);
    f_read(&f, &cpu.hl_, sizeof(cpu.hl_), &br);
    f_read(&f, &cpu.r, sizeof(cpu.r), &br);
    f_read(&f, &cpu.i, sizeof(cpu.i), &br);
    f_read(&f, &cpu.r7, sizeof(cpu.r7), &br);
    f_read(&f, &cpu.im, sizeof(cpu.im), &br);
    f_read(&f, &cpu.request, sizeof(cpu.request), &br);
    f_read(&f, &cpu.resume, sizeof(cpu.resume), &br);
    f_read(&f, &cpu.iff1, sizeof(cpu.iff1), &br);
    f_read(&f, &cpu.iff2, sizeof(cpu.iff2), &br);
    f_read(&f, &cpu.q, sizeof(cpu.q), &br);
    f_read(&f, &cpu.int_line, sizeof(cpu.int_line), &br);
    f_read(&f, &cpu.halt_line, sizeof(cpu.halt_line), &br);

    /* Emulator state */
    f_read(&f, &cpu_cycles, sizeof(cpu_cycles), &br);
    f_read(&f, &cpu_hsync, sizeof(cpu_hsync), &br);
    f_read(&f, extslot, sizeof(extslot), &br);
    f_read(&f, megarom, sizeof(megarom), &br);
    f_read(&f, memmap, sizeof(memmap), &br);
    f_read(&f, ioport, sizeof(ioport), &br);
    f_read(&f, carttype, sizeof(carttype), &br);
    f_read(&f, cart_enable, sizeof(cart_enable), &br);

    /* Main RAM (up to 112KB = 0x1C000) */
    for (int i = 0; i < 0x1C000; i += 512) {
        f_read(&f, &mainram[i], 512, &br);
    }

    /* VDP state */
    uint8_t vdp_regs[8];
    f_read(&f, vdp_regs, 8, &br);
    for (int i = 0; i < 8; i++) {
        vrEmuTms9918WriteRegValue(mainscreen, (vrEmuTms9918Register)i, vdp_regs[i]);
    }

    /* VDP VRAM (16KB) */
    uint8_t vram_chunk[256];
    for (int addr = 0; addr < 16384; addr += 256) {
        f_read(&f, vram_chunk, 256, &br);
        vrEmuTms9918SetAddressWrite(mainscreen, addr);
        for (int i = 0; i < 256; i++) {
            vrEmuTms9918WriteData(mainscreen, vram_chunk[i]);
        }
    }

    f_close(&f);
    return true;
}

static void menuinit(void) {
    vrEmuTms9918InitialiseGfxI(menuscreen);
    vrEmuTms9918SetFgBgColor(menuscreen, TMS_WHITE, TMS_DK_BLUE);

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_PATT_ADDRESS);
    vrEmuTms9918WriteBytes(menuscreen, font, 1024);
    vrEmuTms9918WriteBytes(menuscreen, font, 1024);

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_COLOR_ADDRESS);
    for (int i = 0; i < 16; i++)
        vrEmuTms9918WriteData(menuscreen, 0xF4);
    for (int i = 0; i < 16; i++)
        vrEmuTms9918WriteData(menuscreen, 0xF2);

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen, ' ', VGA_CHARS_X * VGA_CHARS_Y);
}

static void video_cls(void) {
    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen, ' ', VGA_CHARS_X * VGA_CHARS_Y);
}

static void menu_print_at(int x, int y, const char *str) {
    vrEmuTms9918SetAddressWrite(menuscreen,
        TMS_DEFAULT_VRAM_NAME_ADDRESS + x + y * VGA_CHARS_X);
    while (*str && x < VGA_CHARS_X) {
        vrEmuTms9918WriteData(menuscreen, *str++);
        x++;
    }
}

static void menu_print_hl(int x, int y, const char *str) {
    vrEmuTms9918SetAddressWrite(menuscreen,
        TMS_DEFAULT_VRAM_NAME_ADDRESS + x + y * VGA_CHARS_X);
    while (*str && x < VGA_CHARS_X) {
        vrEmuTms9918WriteData(menuscreen, (uint8_t)*str++ + 128);
        x++;
    }
}

static void menu_fill_line(int y, uint8_t ch) {
    vrEmuTms9918SetAddressWrite(menuscreen,
        TMS_DEFAULT_VRAM_NAME_ADDRESS + y * VGA_CHARS_X);
    vrEmuTms9918WriteByteRpt(menuscreen, ch, VGA_CHARS_X);
}

static void menu_center(int y, const char *str) {
    int len = strlen(str);
    int x = (VGA_CHARS_X - len) / 2;
    if (x < 0) x = 0;
    menu_print_at(x, y, str);
}

static void menu_render(void) {
    for (int y = 0; y < MSX_HEIGHT; y++) render_vdp_scanline(y);
    render_vdp_borders();
}

/* ------------------------------------------------------------------ */
/*  NES pad + keyboard unified input with debounce                    */
/* ------------------------------------------------------------------ */

static uint8_t menu_prev_pad = 0;
static uint16_t menu_pad_hold = 0;

static void menu_poll_input(void) {
    ps2_poll();
    nespad_update();
    sleep_us(100);
    nespad_finish_update();

    uint8_t pad = nespad_states[0];
    uint8_t newly = pad & ~menu_prev_pad;
    bool repeat = false;

    if (pad != 0 && pad == menu_prev_pad) {
        menu_pad_hold++;
        if (menu_pad_hold > 20 && (menu_pad_hold % 4 == 0))
            repeat = true;
    } else {
        menu_pad_hold = 0;
    }
    menu_prev_pad = pad;

    if (newly || repeat) {
        if (pad & NES_UP)    keypressed = 0x52;  /* Up */
        if (pad & NES_DOWN)  keypressed = 0x51;  /* Down */
        if (pad & NES_LEFT)  keypressed = 0x4b;  /* Left -> Page Up */
        if (pad & NES_RIGHT) keypressed = 0x4e;  /* Right -> Page Down */
        if (pad & NES_A)     keypressed = 0x28;  /* A -> Enter */
        if (pad & NES_B)     keypressed = 0x29;  /* B -> Escape/Back */
        if (pad & NES_START) keypressed = 0x28;  /* Start -> Enter */
    }
}

/* ------------------------------------------------------------------ */
/*  Save/Load State Menus                                             */
/* ------------------------------------------------------------------ */

static bool slot_exists(int slot) {
    char path[32];
    snprintf(path, sizeof(path), "/MSX/SAVES/slot_%d.sav", slot);
    FILINFO fno;
    return (f_stat(path, &fno) == FR_OK);
}

static void run_save_state_menu(void) {
    int selected = 0;
    char buf[40];
    bool slot_cache[10];

    /* Cache slot existence once at menu entry */
    for (int i = 0; i < 10; i++) {
        slot_cache[i] = slot_exists(i);
    }

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (1) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, "Save State");
        menu_fill_line(2, '=');

        for (int i = 0; i < 10; i++) {
            int row = i + 4;
            snprintf(buf, sizeof(buf), "   Slot %d %s", i, slot_cache[i] ? "[Used]" : "[Empty]");
            if (i == selected) {
                buf[1] = '>';
                menu_print_hl(1, row, buf);
            } else {
                menu_print_at(1, row, buf);
            }
        }

        menu_fill_line(15, '-');
        menu_print_at(0, 16, " A: Save to slot");
        menu_print_at(0, 17, " B: Cancel");
        menu_render();

        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < 9) selected++; }

        if (keypressed == 0x28) {
            keypressed = 0;
            if (save_state_to_slot(selected)) {
                slot_cache[selected] = true;  /* Update cache */
                menu_center(19, "Saved!");
                menu_render();
                sleep_ms(500);
            } else {
                menu_center(19, "Save failed!");
                menu_render();
                sleep_ms(500);
            }
            return;
        }

        if (keypressed == 0x29) {
            keypressed = 0;
            return;
        }
    }
}

static void run_load_state_menu(void) {
    int selected = 0;
    char buf[40];
    bool slot_cache[10];

    /* Cache slot existence once at menu entry */
    for (int i = 0; i < 10; i++) {
        slot_cache[i] = slot_exists(i);
    }

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (1) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, "Load State");
        menu_fill_line(2, '=');

        for (int i = 0; i < 10; i++) {
            int row = i + 4;
            snprintf(buf, sizeof(buf), "   Slot %d %s", i, slot_cache[i] ? "[Used]" : "[Empty]");
            if (i == selected) {
                buf[1] = '>';
                menu_print_hl(1, row, buf);
            } else {
                menu_print_at(1, row, buf);
            }
        }

        menu_fill_line(15, '-');
        menu_print_at(0, 16, " A: Load from slot");
        menu_print_at(0, 17, " B: Cancel");
        menu_render();

        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < 9) selected++; }

        if (keypressed == 0x28) {
            keypressed = 0;
            if (slot_cache[selected]) {
                if (load_state_from_slot(selected)) {
                    menu_center(19, "Loaded!");
                    menu_render();
                    sleep_ms(500);
                    menumode = 0;  /* Exit menu after loading */
                    return;
                } else {
                    menu_center(19, "Load failed!");
                    menu_render();
                    sleep_ms(500);
                }
            } else {
                menu_center(19, "Slot is empty!");
                menu_render();
                sleep_ms(500);
            }
            return;
        }

        if (keypressed == 0x29) {
            keypressed = 0;
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  File browser using FatFS on SD card                               */
/* ------------------------------------------------------------------ */

#define FILES_PER_PAGE 16
#define MAX_NAME  48

typedef struct {
    char name[MAX_NAME];
    uint8_t is_dir;
} FileEntry;

/* Lazy loading: only one page of entries in RAM at a time */
static FileEntry page_entries[FILES_PER_PAGE];
static int page_entry_count = 0;  /* entries loaded on current page */
static int total_dirs = 0;        /* directories (excluding ..) */
static int total_files = 0;       /* matching files */
static int total_entries = 0;     /* dirs + files + maybe ".." */
static bool has_parent_dir = false;
static char browser_path[256];
static int current_page = -1;     /* cached page number */

static bool ext_match(const char *fname, const char *exts) {
    const char *dot = strrchr(fname, '.');
    if (!dot) return false;
    const char *p = exts;
    while (*p) {
        while (*p == ' ') p++;
        if (*p == 0) break;
        const char *next = strchr(p, ' ');
        int len = next ? (int)(next - p) : (int)strlen(p);
        if ((int)strlen(dot) == len &&
            strncasecmp(dot, p, len) == 0)
            return true;
        if (!next) break;
        p = next + 1;
    }
    return false;
}

/* Count all entries without storing names - for lazy pagination */
static void count_directory(const char *path, const char *exts) {
    DIR dir;
    FILINFO fno;
    total_dirs = 0;
    total_files = 0;
    current_page = -1;

    has_parent_dir = (strcmp(path, "/") != 0);

    if (f_opendir(&dir, path) != FR_OK) {
        total_entries = has_parent_dir ? 1 : 0;
        return;
    }

    while (1) {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0) break;
        if (fno.fattrib & AM_HID) continue;
        if (fno.fname[0] == '.') continue;

        if (fno.fattrib & AM_DIR) {
            total_dirs++;
        } else {
            if (exts && !ext_match(fno.fname, exts)) continue;
            total_files++;
        }
    }
    f_closedir(&dir);

    total_entries = (has_parent_dir ? 1 : 0) + total_dirs + total_files;
}

/* Load entries for a specific page (0-indexed) */
static int load_page(const char *path, const char *exts, int page) {
    if (page == current_page) return page_entry_count;

    int start_idx = page * FILES_PER_PAGE;
    int end_idx = start_idx + FILES_PER_PAGE;
    page_entry_count = 0;
    int current_idx = 0;

    /* Handle ".." as virtual first entry */
    if (has_parent_dir) {
        if (current_idx >= start_idx && current_idx < end_idx) {
            strcpy(page_entries[page_entry_count].name, "..");
            page_entries[page_entry_count].is_dir = 1;
            page_entry_count++;
        }
        current_idx++;
    }

    DIR dir;
    FILINFO fno;

    /* First pass: directories */
    if (f_opendir(&dir, path) == FR_OK) {
        while (page_entry_count < FILES_PER_PAGE) {
            if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_HID) continue;
            if (fno.fname[0] == '.') continue;
            if (!(fno.fattrib & AM_DIR)) continue;

            if (current_idx >= start_idx && current_idx < end_idx) {
                strncpy(page_entries[page_entry_count].name, fno.fname, MAX_NAME - 1);
                page_entries[page_entry_count].name[MAX_NAME - 1] = 0;
                page_entries[page_entry_count].is_dir = 1;
                page_entry_count++;
            }
            current_idx++;
        }
        f_closedir(&dir);
    }

    /* Adjust index: we've processed all dirs, now at file section */
    current_idx = (has_parent_dir ? 1 : 0) + total_dirs;

    /* Second pass: files */
    if (f_opendir(&dir, path) == FR_OK) {
        while (page_entry_count < FILES_PER_PAGE) {
            if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_HID) continue;
            if (fno.fname[0] == '.') continue;
            if (fno.fattrib & AM_DIR) continue;
            if (exts && !ext_match(fno.fname, exts)) continue;

            if (current_idx >= start_idx && current_idx < end_idx) {
                strncpy(page_entries[page_entry_count].name, fno.fname, MAX_NAME - 1);
                page_entries[page_entry_count].name[MAX_NAME - 1] = 0;
                page_entries[page_entry_count].is_dir = 0;
                page_entry_count++;
            }
            current_idx++;
        }
        f_closedir(&dir);
    }

    current_page = page;
    return page_entry_count;
}

static int file_selector(const char *title, const char *initial_path,
                          const char *exts) {
    int selected = 0;
    int last_selected = -1;  /* Track selection changes for header cache */
    uint8_t cached_header[8];
    bool header_valid = false;
    char buf[40];
    int scroll_offset = 0;   /* For scrolling long filenames */
    int scroll_timer = 0;    /* Delay before scrolling starts */

    strncpy(browser_path, initial_path, sizeof(browser_path) - 1);
    browser_path[sizeof(browser_path) - 1] = 0;
    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

rescan:
    selected = 0;
    last_selected = -1;
    header_valid = false;
    count_directory(browser_path, exts);

    if (total_entries == 0) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, title);
        menu_fill_line(2, '=');
        menu_center(10, "No files found");
        menu_print_at(0, 23, " Esc/B: Back");
        menu_render();
        while (1) {
            vga_frens_wait_for_vsync();
            menu_poll_input();
            if (keypressed == 0x29 || keypressed == 0x28) {
                keypressed = 0;
                return -1;
            }
        }
    }

    while (1) {
        int page = selected / FILES_PER_PAGE;
        int total_pages = (total_entries + FILES_PER_PAGE - 1) / FILES_PER_PAGE;
        int start = page * FILES_PER_PAGE;

        /* Load current page entries from SD */
        load_page(browser_path, exts, page);

        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, title);
        menu_fill_line(2, '=');

        for (int i = 0; i < page_entry_count; i++) {
            int row = i + 4;
            int global_idx = start + i;
            char nd[64];
            const char *name = page_entries[i].name;
            int namelen = strlen(name);

            if (page_entries[i].is_dir) {
                if (strcmp(name, "..") == 0)
                    strcpy(nd, "[..]");
                else
                    snprintf(nd, sizeof(nd), "[%s]", name);
            } else {
                strcpy(nd, name);
            }

            int ndlen = strlen(nd);
            const int MAX_DISPLAY = 27;

            if (global_idx == selected && ndlen > MAX_DISPLAY) {
                /* Scroll long filename for selected item */
                int max_scroll = ndlen - MAX_DISPLAY;
                int ofs = scroll_offset % (max_scroll + 30);
                if (ofs > max_scroll) ofs = 0;  /* Pause at end, then restart */
                snprintf(buf, sizeof(buf), ">%-27.27s", nd + ofs);
                menu_print_hl(1, row, buf);
            } else {
                snprintf(buf, sizeof(buf), " %-27.27s", nd);
                if (global_idx == selected) {
                    buf[0] = '>';
                    menu_print_hl(1, row, buf);
                } else {
                    menu_print_at(1, row, buf);
                }
            }
        }

        /* Show first 8 bytes of selected file (for BIOS identification) */
        int local_sel = selected - start;
        if (local_sel >= 0 && local_sel < page_entry_count && !page_entries[local_sel].is_dir) {
            /* Only read header when selection changes */
            if (selected != last_selected) {
                header_valid = false;
                char filepath[128];
                snprintf(filepath, sizeof(filepath), "%s/%s",
                         browser_path, page_entries[local_sel].name);
                FIL f;
                if (f_open(&f, filepath, FA_READ) == FR_OK) {
                    UINT br;
                    f_read(&f, cached_header, 8, &br);
                    f_close(&f);
                    header_valid = true;
                }
                last_selected = selected;
            }
            if (header_valid) {
                snprintf(buf, sizeof(buf), " Bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                         cached_header[0], cached_header[1], cached_header[2], cached_header[3],
                         cached_header[4], cached_header[5], cached_header[6], cached_header[7]);
                menu_print_at(0, 20, buf);
            }
        } else {
            last_selected = selected;  /* Update even for directories */
        }

        menu_fill_line(21, '-');
        snprintf(buf, sizeof(buf), " Page %d/%d  (%d items)",
                 page + 1, total_pages, total_entries);
        menu_print_at(0, 22, buf);
        menu_print_at(0, 23, " D-pad:Move A/Ent:OK B/Esc:Back");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        /* Increment scroll timer for long filename scrolling */
        scroll_timer++;
        if (scroll_timer >= 10) {  /* Scroll every 10 frames (~6 chars/sec at 60fps) */
            scroll_timer = 0;
            scroll_offset++;
        }

        int old_selected = selected;
        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < total_entries - 1) selected++; }
        if (keypressed == 0x4b) { keypressed = 0; selected -= FILES_PER_PAGE; if (selected < 0) selected = 0; }
        if (keypressed == 0x4e) { keypressed = 0; selected += FILES_PER_PAGE; if (selected >= total_entries) selected = total_entries - 1; }

        /* Reset scroll when selection changes */
        if (selected != old_selected) {
            scroll_offset = 0;
            scroll_timer = 0;
        }

        if (keypressed == 0x28) {
            keypressed = 0;
            local_sel = selected - start;
            if (local_sel >= 0 && local_sel < page_entry_count && page_entries[local_sel].is_dir) {
                if (strcmp(page_entries[local_sel].name, "..") == 0) {
                    char *last = strrchr(browser_path, '/');
                    if (last && last != browser_path)
                        *last = 0;
                    else
                        strcpy(browser_path, "/");
                } else {
                    int plen = strlen(browser_path);
                    if (plen > 0 && browser_path[plen - 1] != '/')
                        snprintf(browser_path + plen,
                                 sizeof(browser_path) - plen,
                                 "/%s", page_entries[local_sel].name);
                    else
                        snprintf(browser_path + plen,
                                 sizeof(browser_path) - plen,
                                 "%s", page_entries[local_sel].name);
                }
                goto rescan;
            }
            if (local_sel >= 0 && local_sel < page_entry_count) {
                snprintf((char *)filename, sizeof(filename),
                         "%s/%s", browser_path, page_entries[local_sel].name);
                return 0;
            }
        }
        if (keypressed == 0x29) {
            keypressed = 0;
            return -1;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Cart type detection (from msxemulator.c)                          */
/* ------------------------------------------------------------------ */

static void cart_type_checker(uint8_t cartno, FIL *f) {
    uint32_t cartsize = f_size(f);
    uint8_t buf[4];
    UINT br;

    /* Small ROMs (<=32KB) don't need a mapper */
    if (cartsize <= 0x8000) {
        carttype[cartno] = 0;
        return;
    }

    /* Count writes to mapper-specific address ranges
     * Look for LD (nnnn),A (0x32 nn nn) and LD (nnnn),HL (0x22 nn nn) */
    uint32_t ascii8_score = 0;   /* 6000-67FF, 6800-6FFF, 7000-77FF, 7800-7FFF */
    uint32_t ascii16_score = 0;  /* 6000-6FFF, 7000-7FFF (16KB pages) */
    uint32_t konami_score = 0;   /* 6000-7FFF, 8000-9FFF, A000-BFFF */
    uint32_t konami_scc_score = 0; /* 5000-57FF, 7000-77FF, 9000-97FF, B000-B7FF */

    f_lseek(f, 0);
    while (f_read(f, buf, 3, &br) == FR_OK && br >= 3) {
        if (buf[0] == 0x32) {  /* LD (nnnn),A */
            uint16_t addr = buf[1] | ((uint16_t)buf[2] << 8);

            /* ASCII8 detection: writes to 6000-7FFF in 0x800 increments */
            if (addr >= 0x6000 && addr < 0x8000) {
                ascii8_score++;
                if ((addr & 0x0800) == 0) ascii16_score++;  /* 6000 or 7000 */
            }

            /* Konami (no SCC): writes to 6000-BFFF */
            if (addr >= 0x6000 && addr < 0xC000) konami_score++;

            /* Konami SCC: writes to 5000, 7000, 9000, B000 ranges */
            if ((addr >= 0x5000 && addr < 0x5800) ||
                (addr >= 0x7000 && addr < 0x7800) ||
                (addr >= 0x9000 && addr < 0x9800) ||
                (addr >= 0xB000 && addr < 0xB800)) {
                konami_scc_score++;
            }
        }
        /* Seek back 2 bytes to catch overlapping patterns */
        f_lseek(f, f_tell(f) - 2);
    }

    /* Determine mapper type based on scores */
    if (konami_scc_score > 2) {
        carttype[cartno] = 4;  /* Konami SCC */
    } else if (konami_score > ascii8_score && konami_score > 3) {
        carttype[cartno] = 3;  /* Konami (no SCC) */
    } else if (ascii16_score > 0 && ascii8_score <= ascii16_score * 2) {
        carttype[cartno] = 2;  /* ASCII16 */
    } else {
        carttype[cartno] = 1;  /* ASCII8 (default for large ROMs) */
    }
}

/* ------------------------------------------------------------------ */
/*  ROM loading from SD card into RAM                                 */
/* ------------------------------------------------------------------ */

/* Load BIOS ROM from SD directly into RAM buffer */
static bool load_bios_file(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;

    uint32_t size = f_size(&f);
    if (size > 0x8000) size = 0x8000;  /* BIOS max 32KB */

    /* Load directly into bios_ram buffer */
    UINT br;
    if (f_read(&f, bios_ram, size, &br) != FR_OK || br != size) {
        f_close(&f);
        return false;
    }
    f_close(&f);
    return true;
}

static bool load_bios_from_sd(void) {
    /* Always load fresh from SD - no flash caching */
    memset(bios_ram, 0xFF, sizeof(bios_ram));
    if (load_bios_file("/MSX/BIOS/BIOS.ROM")) return true;
    if (load_bios_file("/MSX/BIOS.ROM")) return true;
    if (load_bios_file("/BIOS.ROM")) return true;
    return false;
}

/* Load Disk BIOS ROM from SD directly into RAM buffer */
static bool load_disk_rom_file(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;

    uint32_t size = f_size(&f);
    if (size > 0x4000) size = 0x4000;  /* Disk ROM max 16KB */

    /* Load directly into diskrom_ram buffer */
    UINT br;
    if (f_read(&f, diskrom_ram, size, &br) != FR_OK || br != size) {
        f_close(&f);
        return false;
    }
    f_close(&f);
    return true;
}

/* Check if disk ROM is valid and enabled */
static bool disk_rom_valid(void) {
    if (!disk_rom_enabled) return false;
    /* Check for MSX ROM signature "AB" at offset 0 */
    return (diskrom_ram[0] == 'A' && diskrom_ram[1] == 'B');
}

static bool load_disk_rom_from_sd(void) {
    /* Clear diskrom buffer and try to load from SD */
    memset(diskrom_ram, 0xFF, sizeof(diskrom_ram));
    if (load_disk_rom_file("/MSX/BIOS/DISK.ROM")) return true;
    if (load_disk_rom_file("/MSX/BIOS/DISKBIOS.ROM")) return true;
    if (load_disk_rom_file("/MSX/DISK.ROM")) return true;
    if (load_disk_rom_file("/DISK.ROM")) return true;
    return false;
}

/* Detect mapper type based on ROM content */
static void detect_mapper(uint8_t slot, uint32_t size) {
    if (slot != 0) return;

    /* For small ROMs (<=32KB), use plain ROM mode */
    if (size <= 0x8000) {
        carttype[0] = 0;
        return;
    }

    /* For larger ROMs, detect mapper by scanning for bank switch patterns */
    int ascii8_score = 0, ascii16_score = 0, konami_score = 0;
    uint32_t scan_size = (size > 0x8000) ? 0x8000 : size;

    for (uint32_t i = 0; i < scan_size - 2; i++) {
        if (cartrom_buf[i] == 0x32) {  /* LD (nnnn),A */
            uint16_t addr = cartrom_buf[i+1] | (cartrom_buf[i+2] << 8);
            if (addr >= 0x6000 && addr < 0x8000) ascii8_score++;
            if (addr >= 0x6000 && addr < 0x7000) ascii16_score++;
            if (addr >= 0x5000 && addr < 0x5800) konami_score++;
            if (addr >= 0x7000 && addr < 0x7800) konami_score++;
            if (addr >= 0x9000 && addr < 0x9800) konami_score++;
            if (addr >= 0xB000 && addr < 0xB800) konami_score++;
        }
    }

    if (konami_score > ascii8_score && konami_score > ascii16_score) {
        carttype[0] = 4;  /* Konami SCC */
    } else if (ascii16_score > 0 && ascii8_score <= ascii16_score * 2) {
        carttype[0] = 2;  /* ASCII16 */
    } else if (ascii8_score > 0) {
        carttype[0] = 1;  /* ASCII8 */
    } else {
        carttype[0] = 0;  /* Plain ROM */
    }
}

/* Load cartridge ROM from SD into RAM */
static bool load_cart_from_sd(const char *path, uint8_t slot) {
    if (slot != 0) return false;  /* Only slot 0 supported */

    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;

    uint32_t size = f_size(&f);
    UINT br;

    /* Reset large cart state */
    large_cart_mode = false;
    large_cart_size = 0;
    large_cart_path[0] = 0;
    cached_bank_num = 0xFF;

    /* Reset SRAM state for new ROM */
    sram_enabled[0] = false;
    sram_dirty = false;

    /* Load first 32KB into cartrom_buf */
    uint32_t load1 = (size > 0x8000) ? 0x8000 : size;
    if (f_read(&f, cartrom_buf, load1, &br) != FR_OK || br != load1) {
        f_close(&f);
        return false;
    }

    /* For ROMs > 32KB, enable large cart mode (on-demand loading).
     * Large cart mode uses bank_cache at 0x1A000-0x1BFFF which overlaps
     * with diskrom_ram, so disk features are disabled for large carts. */
    if (size > 0x8000) {
        large_cart_mode = true;
        large_cart_size = size;
        strncpy(large_cart_path, path, sizeof(large_cart_path) - 1);
        large_cart_path[sizeof(large_cart_path) - 1] = 0;
        /* Disable disk ROM since bank_cache uses part of that memory */
        extrom1 = NULL;
    }

    f_close(&f);

    cart_loaded[0] = size;
    cart_enable[0] = 1;
    detect_mapper(0, size);

    /* Initialize mapper banks */
    for (int i = 0; i < 4; i++) megarom[i] = i;

    return true;
}

/* ------------------------------------------------------------------ */
/*  Emulator init                                                     */
/* ------------------------------------------------------------------ */

static void init_emulator(void) {
    /* Clear all I/O ports and RAM (only clear configured RAM size) */
    memset(ioport, 0, sizeof(ioport));
    memset(mainram, 0, ram_size_kb[ram_size] * 1024);

    /* Set ROM pointers based on enabled settings */
    extrom1 = disk_rom_enabled ? diskrom_ram : NULL;
    cartrom1 = cart_rom_enabled ? cartrom_buf : NULL;

    /* Reset cartridge enable flags - only set when a cart is explicitly loaded */
    cart_enable[0] = 0;
    cart_enable[1] = 0;
    cart_loaded[0] = 0;
    cart_loaded[1] = 0;
    carttype[0] = 0;
    carttype[1] = 0;
    sram_enabled[0] = false;
    sram_enabled[1] = false;
    sram_dirty = false;

    /* Reset large cart state */
    large_cart_mode = false;
    large_cart_size = 0;
    large_cart_path[0] = 0;
    cached_bank_num = 0xFF;

    /* Slot select: pages 0-1 = slot 0 (BIOS), pages 2-3 = slot 3 (RAM)
     * Bits: 7-6=page3, 5-4=page2, 3-2=page1, 1-0=page0
     * 0xF0 = 11110000 = slot3,slot3,slot0,slot0 */
    ioport[0xa8] = 0xF0;
    for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xff;
    key_kana = 0;
    key_caps = 0;
    for (int i = 0; i < 8; i++) megarom[i] = 0;
    for (int i = 0; i < 4; i++) extslot[i] = 0;
    psg_reset(0);
    tape_ready = 0;
    fdc_init();
    gamepad_info = JOY_IDLE;
    memmap[0] = 3;
    memmap[1] = 2;
    memmap[2] = 1;
    memmap[3] = 0;
    /* Reset VDP to ensure clean state after menu */
    vrEmuTms9918Reset(mainscreen);
    /* Clear framebuffer to black - removes any leftover menu content */
    memset(framebuffer, 0, sizeof(framebuffer));
    /* Reset VDP write counter for debug */
    vdp_write_count = 0;
}

/* ------------------------------------------------------------------ */
/*  SD card + hardware init                                           */
/* ------------------------------------------------------------------ */

/* PICO-56 SD card on SPI0: MISO=GP16, CS=GP17, SCK=GP18, MOSI=GP19 */
static void init_sd(void) {
    pico_fatfs_spi_config_t cfg = {
        .spi_inst = spi0,
        .clk_slow = 100 * 1000,
        .clk_fast = 32 * 1000 * 1000,
        .pin_miso = 16,
        .pin_cs   = 17,
        .pin_sck  = 18,
        .pin_mosi = 19,
        .pullup   = true,
    };
    pico_fatfs_set_config(&cfg);
    f_mount(&fatfs, "", 1);
}

/* ------------------------------------------------------------------ */
/*  Core 1 entry - VGA scanloop                                       */
/* ------------------------------------------------------------------ */

static void core1_main(void) {
    /* Enable lockout so core0 can pause us during flash programming */
    multicore_lockout_victim_init();
    vgaScanloop();
}

/* ------------------------------------------------------------------ */
/*  Input configuration submenu                                       */
/* ------------------------------------------------------------------ */

static const char *key_names[] = {
    "Space", "Z", "X", "A", "S", "Enter", "Graph", "Ctrl", "Shift",
    "Up", "Down", "Left", "Right", "F1", "F2", "F3", "F4", "F5", "Esc"
};
static const msx_key_t key_values[] = {
    {8, 0x01}, /* Space */
    {5, 0x80}, /* Z */
    {5, 0x20}, /* X */
    {2, 0x40}, /* A */
    {5, 0x01}, /* S */
    {7, 0x80}, /* Enter */
    {6, 0x04}, /* Graph */
    {6, 0x02}, /* Ctrl */
    {6, 0x01}, /* Shift */
    {8, 0x20}, /* Up */
    {8, 0x40}, /* Down */
    {8, 0x10}, /* Left */
    {8, 0x80}, /* Right */
    {6, 0x20}, /* F1 */
    {6, 0x40}, /* F2 */
    {6, 0x80}, /* F3 */
    {7, 0x01}, /* F4 */
    {7, 0x02}, /* F5 */
    {7, 0x04}, /* Esc */
};
#define NUM_KEY_OPTIONS 19

static const char *get_key_name(msx_key_t k) {
    for (int i = 0; i < NUM_KEY_OPTIONS; i++) {
        if (key_values[i].row == k.row && key_values[i].mask == k.mask)
            return key_names[i];
    }
    return "???";
}

static void run_input_config(void) {
    char buf[40];
    int selected = 0;
    const int num_items = 7;

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (menumode) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, "Input Configuration");
        menu_fill_line(2, '=');

        /* Mode selection */
        static const char *mode_names[] = {"Joystick", "Keyboard", "Mouse"};
        snprintf(buf, sizeof(buf), "   Mode: %-18s", mode_names[input_mode]);
        if (selected == 0) { buf[1] = '>'; menu_print_hl(1, 4, buf); }
        else menu_print_at(1, 4, buf);

        /* Button mappings (only shown in keyboard mode) */
        if (input_mode == INPUT_MODE_KEYBOARD) {
            snprintf(buf, sizeof(buf), "   A Button: %-14s", get_key_name(key_map_a));
            if (selected == 1) { buf[1] = '>'; menu_print_hl(1, 6, buf); }
            else menu_print_at(1, 6, buf);

            snprintf(buf, sizeof(buf), "   B Button: %-14s", get_key_name(key_map_b));
            if (selected == 2) { buf[1] = '>'; menu_print_hl(1, 7, buf); }
            else menu_print_at(1, 7, buf);

            snprintf(buf, sizeof(buf), "   SELECT: %-16s", get_key_name(key_map_select));
            if (selected == 3) { buf[1] = '>'; menu_print_hl(1, 8, buf); }
            else menu_print_at(1, 8, buf);

            snprintf(buf, sizeof(buf), "   START: %-17s", get_key_name(key_map_start));
            if (selected == 4) { buf[1] = '>'; menu_print_hl(1, 9, buf); }
            else menu_print_at(1, 9, buf);

            menu_print_at(1, 11, " D-Pad always maps to arrows");
        } else if (input_mode == INPUT_MODE_MOUSE) {
            snprintf(buf, sizeof(buf), "   Speed: %-17d", mouse_speed);
            if (selected == 1) { buf[1] = '>'; menu_print_hl(1, 6, buf); }
            else menu_print_at(1, 6, buf);

            menu_print_at(1, 8, " D-Pad: Move cursor");
            menu_print_at(1, 9, " A: Left button");
            menu_print_at(1, 10, " B: Right button");
        } else {
            menu_print_at(1, 6, " Both gamepads act as");
            menu_print_at(1, 7, " MSX joysticks (2 button)");
        }

        snprintf(buf, sizeof(buf), "   Back");
        int back_sel = (input_mode == INPUT_MODE_KEYBOARD) ? 5 :
                       (input_mode == INPUT_MODE_MOUSE) ? 2 : 1;
        if (selected == back_sel) {
            buf[1] = '>';
            menu_print_hl(1, 14, buf);
        } else {
            menu_print_at(1, 14, buf);
        }

        menu_fill_line(17, '-');
        menu_print_at(0, 18, " A: Change   B: Back");
        menu_print_at(0, 19, " L/R: Adjust value");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        /* max_sel: Keyboard=5 (mode,4 buttons,back), Mouse=2 (mode,speed,back), Joy=1 (mode,back) */
        int max_sel = (input_mode == INPUT_MODE_KEYBOARD) ? 5 :
                      (input_mode == INPUT_MODE_MOUSE) ? 2 : 1;
        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < max_sel) selected++; }

        if (keypressed == 0x29) { keypressed = 0; return; }  /* B = back */

        if (keypressed == 0x28) {  /* A = change */
            keypressed = 0;
            if (selected == 0) {
                /* Cycle through: Joystick -> Keyboard -> Mouse -> Joystick */
                input_mode = (input_mode_t)((input_mode + 1) % 3);
                selected = 0;
            } else if (selected == max_sel) {
                keypressed = 0;
                return;  /* Back */
            }
        }

        /* Left/Right to cycle through options */
        uint8_t pad = nespad_states[0];
        static uint8_t prev = 0;
        uint8_t pressed = pad & ~prev;
        prev = pad;

        msx_key_t *target = NULL;
        if (input_mode == INPUT_MODE_KEYBOARD) {
            if (selected == 1) target = &key_map_a;
            else if (selected == 2) target = &key_map_b;
            else if (selected == 3) target = &key_map_select;
            else if (selected == 4) target = &key_map_start;
        }

        if (target && (pressed & (NES_LEFT | NES_RIGHT))) {
            int idx = 0;
            for (int i = 0; i < NUM_KEY_OPTIONS; i++) {
                if (key_values[i].row == target->row && key_values[i].mask == target->mask) {
                    idx = i;
                    break;
                }
            }
            if (pressed & NES_RIGHT) idx = (idx + 1) % NUM_KEY_OPTIONS;
            if (pressed & NES_LEFT) idx = (idx + NUM_KEY_OPTIONS - 1) % NUM_KEY_OPTIONS;
            *target = key_values[idx];
        }

        /* Mouse speed adjustment */
        if (input_mode == INPUT_MODE_MOUSE && selected == 1) {
            if (pressed & NES_RIGHT) {
                if (mouse_speed < 10) mouse_speed++;
            }
            if (pressed & NES_LEFT) {
                if (mouse_speed > 1) mouse_speed--;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Colors submenu                                                    */
/* ------------------------------------------------------------------ */

static void run_colors_menu(void) {
    char buf[40];
    int selected = current_theme;
    static const char *theme_names[] = {
        "Default (White/Blue)",
        "Terminal (Green/Black)",
    };

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (1) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, "Color Theme");
        menu_fill_line(2, '=');

        for (int i = 0; i < THEME_COUNT; i++) {
            snprintf(buf, sizeof(buf), "   %-24s", theme_names[i]);
            if (i == selected) {
                buf[1] = '>';
                if (i == current_theme) buf[2] = '*';
                menu_print_hl(2, i + 4, buf);
            } else {
                if (i == current_theme) buf[2] = '*';
                menu_print_at(2, i + 4, buf);
            }
        }

        menu_fill_line(10, '-');
        menu_print_at(0, 11, " * = Current theme");
        menu_print_at(0, 13, " A: Apply & Save");
        menu_print_at(0, 14, " B: Cancel");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < THEME_COUNT - 1) selected++; }

        if (keypressed == 0x29) { keypressed = 0; return; }  /* B = Cancel */

        if (keypressed == 0x28) {  /* A = Apply */
            keypressed = 0;
            current_theme = selected;
            apply_color_theme();
            save_config();
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  ROM Settings submenu                                              */
/* ------------------------------------------------------------------ */

static void run_rom_settings_menu(void) {
    char buf[40];
    int selected = 0;
    static const char *mapper_names[] = {"Auto", "ASCII8", "ASCII16", "Konami", "KonamiSCC", "ASCII8+SRAM", "ASCII16+SRAM"};
    static const char *ram_names[] = {"64KB", "16KB", "32KB"};

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (1) {
        video_cls();
        menu_fill_line(0, '=');
        menu_center(1, "ROM Settings");
        menu_fill_line(2, '=');

        if (rom_settings_count == 0) {
            menu_print_at(2, 5, "No ROMs loaded yet.");
            menu_print_at(2, 7, "Load a ROM first, then");
            menu_print_at(2, 8, "return here to adjust");
            menu_print_at(2, 9, "its settings.");
        } else {
            /* Show list of ROMs with settings */
            int visible = 6;
            int scroll = 0;
            if (selected >= visible) scroll = selected - visible + 1;
            if (scroll > rom_settings_count - visible) scroll = rom_settings_count - visible;
            if (scroll < 0) scroll = 0;

            for (int i = 0; i < visible && (i + scroll) < rom_settings_count; i++) {
                int idx = i + scroll;
                rom_setting_t *rs = &rom_settings[idx];
                snprintf(buf, sizeof(buf), " %-20.20s", rs->name);
                if (idx == selected) {
                    buf[0] = '>';
                    menu_print_hl(1, i + 4, buf);
                } else {
                    menu_print_at(1, i + 4, buf);
                }
            }

            /* Show selected ROM details */
            if (selected < rom_settings_count) {
                rom_setting_t *rs = &rom_settings[selected];
                menu_fill_line(11, '-');
                snprintf(buf, sizeof(buf), " Hdr: %02X %02X %02X %02X",
                         rs->header[0], rs->header[1], rs->header[2], rs->header[3]);
                menu_print_at(0, 12, buf);
                snprintf(buf, sizeof(buf), " Mapper: %s", mapper_names[rs->mapper_type % 7]);
                menu_print_at(0, 13, buf);
                snprintf(buf, sizeof(buf), " RAM: %s", ram_names[rs->ram_size % 3]);
                menu_print_at(0, 14, buf);

                menu_print_at(0, 16, " LEFT/RIGHT: Change");
            }
        }

        menu_fill_line(18, '-');
        snprintf(buf, sizeof(buf), " %d ROM(s) configured", rom_settings_count);
        menu_print_at(0, 19, buf);
        menu_print_at(0, 21, " Up/Dn: Select ROM");
        menu_print_at(0, 22, " L/R: Change setting");
        menu_print_at(0, 23, " B: Back");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < rom_settings_count - 1) selected++; }

        /* Left/Right to change mapper for selected ROM */
        if (rom_settings_count > 0 && selected < rom_settings_count) {
            rom_setting_t *rs = &rom_settings[selected];
            if (keypressed == 0x4b) {  /* Left (same as Page Up in menu) */
                keypressed = 0;
                if (rs->mapper_type > 0) rs->mapper_type--;
                save_config();
            }
            if (keypressed == 0x4e) {  /* Right (same as Page Down in menu) */
                keypressed = 0;
                if (rs->mapper_type < 6) rs->mapper_type++;
                save_config();
            }
        }

        if (keypressed == 0x29) { keypressed = 0; return; }  /* B = Back */
    }
}

/* Get max RAM option based on what ROMs are disabled.
 * Memory layout: cartrom_buf at 64KB, diskrom_ram at 96KB.
 * RAM cannot overlap with enabled ROM buffers. */
static uint8_t get_max_ram_option(void) {
    if (!cart_rom_enabled && !disk_rom_enabled) return 4;  /* 112KB - no ROM buffers used */
    if (!cart_rom_enabled) return 3;  /* 96KB - cart not used, disk at 96KB */
    /* cart_rom_enabled is true - cartrom_buf at 64KB is in use */
    return 1;  /* 64KB max when cart enabled (regardless of disk) */
}

/* Auto-adjust RAM settings when enabling a ROM feature */
static void adjust_ram_for_cart(void) {
    cart_rom_enabled = 1;
    uint8_t max_ram = get_max_ram_option();
    if (ram_size > max_ram) ram_size = max_ram;
    save_config();
}

static void adjust_ram_for_disk(void) {
    disk_rom_enabled = 1;
    uint8_t max_ram = get_max_ram_option();
    if (ram_size > max_ram) ram_size = max_ram;
    save_config();
}

static void run_ram_settings_menu(void) {
    char buf[40];
    int selected = 0;
    const int num_options = 3;

    while (1) {
        uint8_t max_ram = get_max_ram_option();
        if (ram_size > max_ram) {
            ram_size = max_ram;
            save_config();
        }

        video_cls();
        menu_fill_line(0, '*');
        menu_center(1, "RAM Settings");
        menu_fill_line(2, '*');

        /* RAM Size option */
        snprintf(buf, sizeof(buf), "MSX RAM:      %dKB", ram_size_kb[ram_size]);
        if (selected == 0) menu_print_hl(1, 4, buf);
        else menu_print_at(1, 4, buf);

        /* Cart ROM toggle */
        snprintf(buf, sizeof(buf), "Cart ROM:     %s", cart_rom_enabled ? "32KB" : "OFF");
        if (selected == 1) menu_print_hl(1, 5, buf);
        else menu_print_at(1, 5, buf);

        /* Disk ROM toggle */
        snprintf(buf, sizeof(buf), "Disk ROM:     %s", disk_rom_enabled ? "16KB" : "OFF");
        if (selected == 2) menu_print_hl(1, 6, buf);
        else menu_print_at(1, 6, buf);

        menu_fill_line(8, '-');
        menu_print_at(1, 9, "Fixed allocations:");
        menu_print_at(1, 10, "BIOS ROM:     32KB");
        menu_print_at(1, 11, "Framebuffer:  60KB");
        menu_print_at(1, 12, "VDP VRAM:     16KB");
        menu_fill_line(13, '-');

        int total = ram_size_kb[ram_size] + 32 + 60 + 16 +
                    (cart_rom_enabled ? 32 : 0) +
                    (disk_rom_enabled ? 16 : 0);
        snprintf(buf, sizeof(buf), "Total:       %dKB / 264KB", total);
        menu_print_at(1, 14, buf);

        menu_fill_line(16, '-');
        if (selected == 0) {
            snprintf(buf, sizeof(buf), "Max: %dKB (disable ROMs for more)", ram_size_kb[max_ram]);
            menu_print_at(1, 17, buf);
        } else if (selected == 1 && cart_rom_enabled) {
            menu_print_at(1, 17, "Disable to use up to 96KB RAM");
        } else if (selected == 2 && disk_rom_enabled) {
            menu_print_at(1, 17, "Disable to use up to 112KB RAM");
        } else {
            menu_print_at(1, 17, "");
        }

        menu_print_at(1, 19, "Up/Down: Select  L/R: Change");
        menu_print_at(1, 20, "B: Back (reset to apply)");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < num_options - 1) selected++; }

        if (keypressed == 0x4b) {  /* Left */
            keypressed = 0;
            if (selected == 0 && ram_size > 0) { ram_size--; save_config(); }
            if (selected == 1) { cart_rom_enabled = 0; save_config(); }
            if (selected == 2) { disk_rom_enabled = 0; save_config(); }
        }
        if (keypressed == 0x4e) {  /* Right */
            keypressed = 0;
            if (selected == 0 && ram_size < max_ram) { ram_size++; save_config(); }
            if (selected == 1) { cart_rom_enabled = 1; save_config(); }
            if (selected == 2) { disk_rom_enabled = 1; save_config(); }
        }

        if (keypressed == 0x29) { keypressed = 0; return; }  /* B = Back */
    }
}

static void run_default_settings_menu(void) {
    int confirmed = 0;

    while (1) {
        video_cls();
        menu_fill_line(0, '*');
        menu_center(1, "Default Settings");
        menu_fill_line(2, '*');

        menu_print_at(1, 5, "This will delete your config file");
        menu_print_at(1, 6, "and restore all default settings.");

        menu_fill_line(8, '-');

        if (!confirmed) {
            menu_print_at(1, 10, "Are you sure?");
            menu_print_hl(1, 12, "  No  ");
            menu_print_at(8, 12, "  Yes  ");
        } else {
            menu_print_at(1, 10, "Are you sure?");
            menu_print_at(1, 12, "  No  ");
            menu_print_hl(8, 12, "  Yes  ");
        }

        menu_fill_line(14, '-');
        menu_print_at(1, 15, "Left/Right: Select");
        menu_print_at(1, 16, "A: Confirm  B: Cancel");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x4b) { keypressed = 0; confirmed = 0; }  /* Left = No */
        if (keypressed == 0x4e) { keypressed = 0; confirmed = 1; }  /* Right = Yes */

        if (keypressed == 0x28) {  /* A = Confirm */
            keypressed = 0;
            if (confirmed) {
                /* Delete config file */
                f_unlink(CONFIG_PATH);
                /* Reset to defaults */
                current_theme = THEME_DEFAULT;
                disk_rom_enabled = 1;
                cart_rom_enabled = 1;
                ram_size = 1;  /* 64KB */
                rom_settings_count = 0;
                /* Show confirmation */
                video_cls();
                menu_center(10, "Settings reset to defaults.");
                menu_center(12, "Please restart the emulator.");
                menu_render();
                sleep_ms(2000);
            }
            return;
        }

        if (keypressed == 0x29) { keypressed = 0; return; }  /* B = Cancel */
    }
}

/* ------------------------------------------------------------------ */
/*  Main menu handler                                                 */
/* ------------------------------------------------------------------ */

static char bios_filename[32] = "BIOS.ROM";

static void run_menu(void) {
    char buf[40];
    int selected = 0;
    static const char *items[] = {
        "Load ROM",
        "Load Disk",
        "Change Disk",
        "Select BIOS",
        "Start BASIC",
        "Save State",
        "Load State",
        "Input Config",
        "Audio Mode",
        "Colors",
        "ROM Settings",
        "RAM Settings",
        "Default Settings",
        "Reset",
        "Return to emulator",
    };
    static const int num_items = 15;

    /* Save VDP state to SD before menu corrupts it */
    vdp_save_state();

    /* Reinitialize VDP for menu (shared with emulation) */
    menuinit();
    apply_color_theme();  /* Apply saved color theme after menu init */

    menu_prev_pad = 0xff;
    menu_pad_hold = 0;

    while (menumode) {
        video_cls();

        menu_fill_line(0, '*');
        menu_center(1, "MSXMulti");
        menu_center(2, "2026 serpentchain");
        menu_fill_line(3, '*');

        /* Scrolling menu - 8 visible items at a time */
        #define MENU_VISIBLE 8
        #define MENU_START_ROW 4
        int scroll_offset = 0;
        if (selected >= MENU_VISIBLE) {
            scroll_offset = selected - MENU_VISIBLE + 1;
        }
        if (scroll_offset > num_items - MENU_VISIBLE) {
            scroll_offset = num_items - MENU_VISIBLE;
        }
        if (scroll_offset < 0) scroll_offset = 0;

        for (int i = 0; i < MENU_VISIBLE && (i + scroll_offset) < num_items; i++) {
            int item_idx = i + scroll_offset;
            int row = i + MENU_START_ROW;
            snprintf(buf, sizeof(buf), "   %-26s", items[item_idx]);
            if (item_idx == selected) {
                buf[1] = '>';
                menu_print_hl(1, row, buf);
            } else {
                menu_print_at(1, row, buf);
            }
        }

        /* Show scroll indicators */
        if (scroll_offset > 0) {
            menu_print_at(28, MENU_START_ROW, "^^^");
        }
        if (scroll_offset + MENU_VISIBLE < num_items) {
            menu_print_at(28, MENU_START_ROW + MENU_VISIBLE - 1, "vvv");
        }

        menu_fill_line(12, '-');
        snprintf(buf, sizeof(buf), " BIOS: %.20s", bios_filename);
        menu_print_at(0, 13, buf);
        if (cart_loaded[0]) {
            snprintf(buf, sizeof(buf), " Cart: %.20s", cart1_filename);
            menu_print_at(0, 14, buf);
        }
        if (fd_drive_status[0]) {
            snprintf(buf, sizeof(buf), " Disk: %.20s", fd_filename);
            menu_print_at(0, 15, buf);
        }
        menu_fill_line(16, '-');

        /* Show volume and audio mode */
        static const char *audio_mode_names[] = {"PSG", "FM", "PSG+FM"};
        snprintf(buf, sizeof(buf), " Vol: %d%%  Audio: %s",
                 (master_volume * 100) / 255, audio_mode_names[audio_mode]);
        menu_print_at(0, 17, buf);

        menu_fill_line(19, '-');
        menu_print_at(0, 20, " START+SELECT: Menu");
        menu_print_at(0, 21, " SELECT+UP/DN: Volume");
        menu_print_at(0, 22, " Up/Dn:Move  A:Select");
        menu_print_at(0, 23, " B: Resume emulator");

        menu_render();
        vga_frens_wait_for_vsync();
        menu_poll_input();

        if (keypressed == 0x52) { keypressed = 0; if (selected > 0) selected--; }
        if (keypressed == 0x51) { keypressed = 0; if (selected < num_items - 1) selected++; }

        if (keypressed == 0x29) {
            keypressed = 0;
            /* Only return to game if VDP state was saved (game was running) */
            if (vdp_state_saved) {
                vdp_restore_state();
                /* Set menumode=0 BEFORE rendering so render_vdp_scanline uses mainscreen */
                menumode = 0;
                /* Re-render framebuffer from restored VDP state */
                for (int y = 0; y < MSX_HEIGHT; y++) render_vdp_scanline(y);
                render_vdp_borders();
                /* Reset keyboard state to prevent phantom key presses */
                for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xFF;
                break;
            }
        }

        if (keypressed == 0x28) {
            keypressed = 0;
            switch (selected) {
                case 0:
                    /* Load ROM - only start emulator if file was selected */
                    if (file_selector("Select ROM:",
                                      "/MSX/ROMS", ".rom .bin") == 0) {
                        adjust_ram_for_cart();  /* Auto-enable cart ROM and adjust RAM */
                        init_emulator();  /* Reset first */
                        load_cart_from_sd((const char *)filename, 0);  /* Then load (sets cart_enable) */
                        char *sl = strrchr((char *)filename, '/');
                        strncpy((char *)cart1_filename,
                                sl ? sl + 1 : (char *)filename,
                                sizeof(cart1_filename) - 1);
                        cart1_filename[sizeof(cart1_filename) - 1] = 0;

                        /* Add to ROM settings or get existing (uses first 4 bytes as header) */
                        rom_setting_t *rs = add_rom_setting((const char *)cart1_filename, cartrom_buf);

                        /* Apply saved mapper setting if not Auto */
                        if (rs && rs->mapper_type > 0) {
                            carttype[0] = rs->mapper_type;
                        }

                        /* Load SRAM for battery-backed mappers */
                        if (carttype[0] == 5 || carttype[0] == 6) {
                            set_sram_filename((const char *)filename);
                            load_sram();
                        }

                        /* Apply color theme after loading */
                        apply_color_theme();

                        z80_instant_reset(&cpu);
                        cpu_cycles = 0;
                        cpu_hsync = 0;
                        menumode = 0;
                    }
                    break;
                case 1:
                    /* Load Disk - only start emulator if file was selected */
                    if (file_selector("Select Disk:",
                                      "/MSX/DISKS", ".dsk") == 0) {
                        /* Close any existing disk first */
                        if (fd_drive_status[0]) f_close(&fd_drive[0]);
                        /* Auto-enable disk ROM and adjust RAM */
                        adjust_ram_for_disk();
                        /* Reset emulator BEFORE opening disk (fdc_init clears fd_drive_status) */
                        init_emulator();
                        /* Now open disk and set status */
                        /* Try read/write first, fall back to read-only */
                        FRESULT res = f_open(&fd_drive[0], (const char *)filename,
                                             FA_READ | FA_WRITE);
                        if (res != FR_OK) {
                            res = f_open(&fd_drive[0], (const char *)filename, FA_READ);
                        }
                        if (res == FR_OK) {
                            fd_drive_status[0] = 1;
                            fdc_check(0);
                            char *sl = strrchr((char *)filename, '/');
                            strncpy((char *)fd_filename,
                                    sl ? sl + 1 : (char *)filename,
                                    sizeof(fd_filename) - 1);
                            fd_filename[sizeof(fd_filename) - 1] = 0;
                        }
                        z80_instant_reset(&cpu);
                        cpu_cycles = 0;
                        cpu_hsync = 0;
                        menumode = 0;
                    }
                    break;
                case 2:
                    /* Change Disk - mount without reset, return to emulator */
                    if (file_selector("Change Disk:",
                                      "/MSX/DISKS", ".dsk") == 0) {
                        if (fd_drive_status[0]) f_close(&fd_drive[0]);
                        FRESULT res = f_open(&fd_drive[0], (const char *)filename,
                                             FA_READ | FA_WRITE);
                        if (res != FR_OK) {
                            res = f_open(&fd_drive[0], (const char *)filename, FA_READ);
                        }
                        if (res == FR_OK) {
                            fd_drive_status[0] = 1;
                            fdc_check(0);
                            char *sl = strrchr((char *)filename, '/');
                            strncpy((char *)fd_filename,
                                    sl ? sl + 1 : (char *)filename,
                                    sizeof(fd_filename) - 1);
                            fd_filename[sizeof(fd_filename) - 1] = 0;
                        }
                        /* Return to emulator without reset */
                        if (vdp_state_saved) {
                            vdp_restore_state();
                            menumode = 0;
                            for (int y = 0; y < MSX_HEIGHT; y++) render_vdp_scanline(y);
                            render_vdp_borders();
                            for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xFF;
                        }
                    }
                    break;
                case 3:
                    /* Select BIOS */
                    if (file_selector("Select BIOS:",
                                      "/MSX/BIOS", ".rom .bin") == 0) {
                        if (load_bios_file((const char *)filename)) {
                            char *sl = strrchr((char *)filename, '/');
                            strncpy(bios_filename,
                                    sl ? sl + 1 : (char *)filename,
                                    sizeof(bios_filename) - 1);
                            bios_filename[sizeof(bios_filename) - 1] = 0;
                            init_emulator();
                            z80_instant_reset(&cpu);
                            cpu_cycles = 0;
                            cpu_hsync = 0;
                        }
                    }
                    break;
                case 4:
                    /* Start BASIC */
                    menumode = 0;
                    break;
                case 5:
                    run_save_state_menu();
                    break;
                case 6:
                    run_load_state_menu();
                    break;
                case 7:
                    run_input_config();
                    /* Reset to current pad state so held buttons don't re-trigger */
                    nespad_update();
                    sleep_us(100);
                    nespad_finish_update();
                    menu_prev_pad = nespad_states[0];
                    menu_pad_hold = 0;
                    keypressed = 0;
                    break;
                case 8:
                    /* Audio Mode: cycle through PSG -> FM -> PSG+FM */
                    audio_mode = (audio_mode + 1) % 3;
                    break;
                case 9:
                    /* Colors */
                    run_colors_menu();
                    break;
                case 10:
                    /* ROM Settings */
                    run_rom_settings_menu();
                    break;
                case 11:
                    /* RAM Settings */
                    run_ram_settings_menu();
                    break;
                case 12:
                    /* Default Settings */
                    run_default_settings_menu();
                    break;
                case 13:
                    /* Reset */
                    init_emulator();
                    z80_instant_reset(&cpu);
                    cpu_cycles = 0;
                    cpu_hsync = 0;
                    menumode = 0;
                    break;
                case 14:
                    /* Same as B button - only return if game was running */
                    if (vdp_state_saved) {
                        vdp_restore_state();
                        /* Set menumode=0 BEFORE rendering so render_vdp_scanline uses mainscreen */
                        menumode = 0;
                        /* Re-render framebuffer from restored VDP state */
                        for (int y = 0; y < MSX_HEIGHT; y++) render_vdp_scanline(y);
                        render_vdp_borders();
                        /* Reset keyboard state to prevent phantom key presses */
                        for (int i = 0; i < KEYMAP_ROWS; i++) keymap[i] = 0xFF;
                        break;
                    }
                    /* Otherwise do nothing - stay in menu */
                    break;
            }
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  main()                                                            */
/* ------------------------------------------------------------------ */

int main() {
    /* Clock and voltage setup */
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(266000, true);
    sleep_ms(10);

    stdio_init_all();

    /* VGA init */
    memset(framebuffer, 0, sizeof(framebuffer));
    vga_frens_init(framebuffer);
    multicore_launch_core1(core1_main);
    sleep_ms(10);
    vga_frens_set_palette(tms_palette);

    /* Initialize peripherals */
    ps2kbd_begin();
    nespad_begin(0, clock_get_hz(clk_sys) / 1000, 22, 27, 26, pio1);
    nespad_begin(1, clock_get_hz(clk_sys) / 1000, 22, 28, 26, pio1);
    init_sd();
    f_mkdir("/MSX");
    audio_pwm_setup(SAMPLING_FREQ);

    /* Load config from SD (theme, ROM settings) */
    load_config();

    /* Validate RAM size against enabled ROMs */
    uint8_t max_ram = get_max_ram_option();
    if (ram_size > max_ram) {
        ram_size = max_ram;
        save_config();
    }

    /* Create VDP instance */
    mainscreen = vrEmuTms9918New();
    if (!mainscreen) {
        memset(framebuffer, 6, sizeof(framebuffer));  /* Red = VDP failed */
        while(1) sleep_ms(1000);
    }

    /* Initialize menu system and apply color theme */
    menuinit();
    apply_color_theme();

    /* Load BIOS from SD into RAM */
    if (!load_bios_from_sd()) {
        memset(framebuffer, 6, sizeof(framebuffer));  /* Red = no BIOS */
        while(1) sleep_ms(1000);
    }

    /* Load Disk ROM from SD into RAM (optional - disk support) */
    if (!load_disk_rom_from_sd()) {
        /* Yellow flash = no disk ROM found (non-fatal, disk support disabled) */
        memset(framebuffer, 10, sizeof(framebuffer));
        vga_frens_wait_for_vsync();
        sleep_ms(500);
        memset(framebuffer, 0, sizeof(framebuffer));
    }

    /* Start PSG audio timer */
    opll = NULL;
    static struct repeating_timer sound_timer;
    add_repeating_timer_us(1000000 / SAMPLING_FREQ, sound_handler, NULL, &sound_timer);

    /* Initialize emulator */
    init_emulator();

    /* Set up Z80 */
    cpu.read = mem_read;
    cpu.write = mem_write;
    cpu.in = io_read;
    cpu.out = io_write;
    cpu.fetch = mem_read;
    cpu.fetch_opcode = mem_read;
    cpu.reti = reti_callback;
    cpu.inta = ird_read;

    z80_power(&cpu, true);
    z80_instant_reset(&cpu);
    cpu_hsync = 0;
    cpu_cycles = 0;

    /* Start in menu mode to let user pick a ROM */
    menumode = 1;

    /* Main emulation loop */
    uint32_t vdp_line = 0;
    uint32_t frame_count = 0;

    while (1) {
        if (menumode) {
            run_menu();
            vdp_line = 0;
            cpu_cycles = 0;
            cpu_hsync = 0;
            continue;
        }

        /* Run Z80 for one instruction at a time for accurate timing */
        zusize cycles = z80_run(&cpu, 1);
        if (cycles == 0) cycles = 4;
        cpu_cycles += cycles;

        while (cpu_cycles >= cpu_hsync + Z80_CYCLES_PER_SCANLINE) {
            cpu_hsync += Z80_CYCLES_PER_SCANLINE;

            /* Render VDP scanline */
            if (vdp_line < MSX_HEIGHT) {
                render_vdp_scanline(vdp_line);
            }

            /* Line 192: set vsync flag for BIOS to poll */
            if (vdp_line == MSX_HEIGHT) {
                video_vsync = 1;
            }

            vdp_line++;

            if (vdp_line >= SCANLINES_PER_FRAME) {
                vdp_line = 0;
                frame_count++;

                /* Reset cycle counters each frame to prevent uint32 overflow.
                 * Without this, counters overflow after ~20 mins causing freeze. */
                cpu_cycles -= cpu_hsync;
                cpu_hsync = 0;

                render_vdp_borders();

                /* Draw on-screen keyboard overlay if active */
                osk_draw_overlay();

                /* Poll input once per frame */
                ps2_poll();
                nespad_update();
                nespad_finish_update();

                /* Wait for VGA vsync to maintain 60fps */
                vga_frens_wait_for_vsync();
            }
        }

        /* VDP interrupt: if enabled, trigger on vsync */
        if (video_vsync && cpu.iff1) {
            uint8_t vdp_r1 = vrEmuTms9918RegValue(mainscreen, TMS_REG_1);
            if (vdp_r1 & 0x20) {  /* Interrupt enable bit */
                z80_int(&cpu, true);
            }
        }
    }

    return 0;
}
