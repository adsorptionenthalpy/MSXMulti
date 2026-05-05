/*
 * FDC (MB8877) header - adapted for FatFS (PICO-56 SD card)
 * instead of LittleFS on internal flash.
 */
#ifndef MSX_FDC_H
#define MSX_FDC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FatFS file handle for each drive (up to 4). Defined in fdc_fatfs.c. */
#include "ff.h"

void fdc_init(void);
void fdc_command_write(uint8_t data);

uint8_t fdc_read_status(void);
uint8_t fdc_read_track(void);
uint8_t fdc_read_sector(void);
uint8_t fdc_read_control1(void);
uint8_t fdc_read_control2(void);
uint8_t fdc_read_status_flag(void);
uint8_t fdc_read(void);

void fdc_write_track(uint8_t data);
void fdc_write_sector(uint8_t data);
void fdc_write_control1(uint8_t data);
void fdc_write_control2(uint8_t data);
void fdc_write(uint8_t data);

void fdc_check(uint8_t driveno);
uint8_t fdc_find_sector(void);

extern FIL fd_drive[4];
extern uint8_t fd_drive_status[4];
extern uint32_t fdc_dma_datasize;

#ifdef __cplusplus
}
#endif

#endif /* MSX_FDC_H */
