/*
 * Project: pico-56 - vga
 *
 * Copyright (c) 2023 Troy Schrapel
 *
 * This code is licensed under the MIT license
 *
 * https://github.com/visrealm/pico-56
 *
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t displayPixels;
  uint32_t frontPorchPixels;
  uint32_t syncPixels;
  uint32_t backPorchPixels;
  uint32_t totalPixels;
  float freqHz;
  bool syncHigh;
} VgaSyncParams;

typedef struct
{
  uint32_t pixelClockKHz;
  VgaSyncParams hSyncParams;
  VgaSyncParams vSyncParams;
  uint32_t hPixelScale;
  uint32_t vPixelScale;
  uint32_t hVirtualPixels;
  uint32_t vVirtualPixels;
  float pioDivider;
  float pioFreqKHz;
  float pioClocksPerPixel;
  float pioClocksPerScaledPixel;
} VgaParams;


typedef void (*vgaScanlineRgbFn)(uint16_t y, VgaParams* params, uint16_t* pixels);
typedef void (*vgaEndOfFrameFn)(uint64_t frameNumber);
typedef void (*vgaEndOfScanlineFn)();

extern uint32_t vgaMinimumPioClockKHz(VgaParams* params);

typedef struct
{
  VgaParams params;
  vgaScanlineRgbFn scanlineFn;
  vgaEndOfFrameFn endOfFrameFn;
  vgaEndOfScanlineFn endOfScanlineFn;
} VgaInitParams;


/* Initialize VGA pio/dma. Caller must invoke vgaScanloop() afterwards on
   whatever core should service scanline callbacks (typically core1). */
void vgaInit(VgaInitParams params);

/* Run the per-scanline service loop. Never returns. Pops multicore FIFO
   messages from the DMA IRQ on the other core and dispatches scanline /
   end-of-line / end-of-frame callbacks. */
void vgaScanloop(void);

VgaInitParams vgaCurrentParams();

#ifdef __cplusplus
}
#endif