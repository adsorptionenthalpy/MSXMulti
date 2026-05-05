/*
 * MB8877 FDC Emulation - adapted for FatFS (PICO-56 SD card)
 * Original by shippoiincho, modified to use FatFS instead of LittleFS.
 */

#include "fdc.h"
#include <string.h>

uint8_t fdc_status;
uint8_t fd_track[4];
uint8_t fdc_command;

uint8_t fdc_track_register;
uint8_t fdc_sector_register;
uint8_t fdc_data_register;
uint8_t fdc_control1;
uint8_t fdc_control2;
uint8_t fdc_status_flag;

uint8_t fd_seek_dir;
uint32_t fd_sector_size;
uint32_t fd_sector_bytes;

FIL fd_drive[4];
uint8_t fd_drive_status[4];
uint8_t fd_image_status[4];
uint32_t fdc_dma_datasize;

void fdc_command_write(uint8_t param) {
    uint8_t command, res, driveno;
    fdc_command = param;
    command = param >> 4;
    driveno = fdc_control2 & 1;

    if (fd_drive_status[driveno] == 0) {
        fdc_status = 0x80;
        fdc_status_flag = 0x40;
        return;
    }
    if (driveno > 2) {
        fdc_status = 0x80;
        fdc_status_flag = 0x40;
        return;
    }

    switch (command) {
        case 0:
            fdc_track_register = 0;
            fdc_data_register = 0;
            fdc_status_flag = 0x40;
            fd_track[driveno] = 0;
            fdc_status = 6;
            return;
        case 1:
            fdc_status_flag = 0x40;
            fdc_track_register = fdc_data_register;
            fd_track[driveno] = fdc_track_register;
            fdc_status = (fdc_track_register == 0) ? 6 : 0;
            return;
        case 2:
        case 4:
        case 6:
            fdc_status_flag = 0x40;
            fdc_status = (fdc_track_register == 0) ? 6 : 0;
            fd_track[driveno] = fdc_track_register;
            return;
        case 3:
            fdc_status_flag = 0x40;
            if (fd_seek_dir == 0) {
                res = fdc_track_register;
                fdc_status = (res == 80) ? 0x10 : 0;
                fdc_track_register = res + 1;
            } else {
                res = fdc_track_register;
                if (res == 0) fdc_status = 0x16;
                else if (res == 1) fdc_status = 6;
                else fdc_status = 0;
                fdc_track_register = res - 1;
            }
            fd_track[driveno] = fdc_track_register;
            return;
        case 5:
            fdc_status_flag = 0x40;
            fd_seek_dir = 0;
            res = fdc_track_register;
            fdc_status = (res == 80) ? 0x10 : 0;
            fdc_track_register = res + 1;
            fd_track[driveno] = fdc_track_register;
            return;
        case 7:
            fdc_status_flag = 0x40;
            fd_seek_dir = 1;
            res = fdc_track_register;
            if (res == 0) fdc_status = 0x16;
            else if (res == 1) fdc_status = 6;
            else fdc_status = 0;
            fdc_track_register = res - 1;
            fd_track[driveno] = fdc_track_register;
            return;
        case 8:
        case 9:
            fdc_status_flag = 0x80;
            fdc_find_sector();
            fdc_status = 3;
            return;
        case 0xa:
        case 0xb:
            fdc_status_flag = 0x80;
            fdc_find_sector();
            fdc_status = 3;
            return;
        case 0xc:
            fdc_status_flag = 0x40;
            fdc_status = 0x80;
            return;
        case 0xd:
            fdc_status_flag = (fdc_command & 0xf) ? 0x40 : 0;
            fdc_status = 2;
            return;
        case 0xe:
        case 0xf:
            fdc_status_flag = 0x40;
            fdc_status = 0x80;
            return;
        default:
            return;
    }
}

void fdc_check(uint8_t driveno) {
    FSIZE_t imagesize = f_size(&fd_drive[driveno]);

    switch (imagesize) {
        case 737280:  fd_image_status[driveno] = 0; break;  /* 2DD 720KB */
        case 368640:  fd_image_status[driveno] = 1; break;  /* 1DD 360KB */
        case 655360:  fd_image_status[driveno] = 2; break;  /* 2DD 640KB */
        case 327680:  fd_image_status[driveno] = 3; break;  /* 1DD 320KB */
        default:      fd_image_status[driveno] = 0; break;
    }

    /* Reset file position to start */
    f_lseek(&fd_drive[driveno], 0);
    fdc_status_flag = 0;
    fd_drive_status[driveno] = 3;
}

uint8_t fdc_find_sector(void) {
    uint8_t driveno, track, sector, head;
    uint32_t sector_ptr;

    driveno = fdc_control2 & 1;
    track = fd_track[driveno];
    sector = fdc_sector_register;
    head = fdc_control1 & 1;

    fd_sector_size = 512;

    switch (fd_image_status[driveno]) {
        case 0: sector_ptr = (track * 18 + head * 9 + sector - 1) * 512; break;
        case 1: sector_ptr = (track * 9 + sector - 1) * 512; break;
        case 2: sector_ptr = (track * 16 + head * 8 + sector - 1) * 512; break;
        case 3: sector_ptr = (track * 8 + sector - 1) * 512; break;
        default: sector_ptr = 0; break;
    }

    f_lseek(&fd_drive[driveno], sector_ptr);
    fd_sector_bytes = 0;
    return 0;
}

uint8_t fdc_read(void) {
    uint8_t data = 0xff;
    uint8_t driveno;
    UINT br;

    driveno = fdc_control2 & 1;
    if (fd_drive_status[driveno] == 0) return 0xff;

    if ((fdc_command & 0xe0) != 0x80) {
        return fd_track[driveno];
    }

    if (fd_sector_bytes == fd_sector_size) {
        fdc_status_flag = 0x40;
        fdc_status = 0;
    }

    if (f_read(&fd_drive[driveno], &data, 1, &br) != FR_OK || br != 1) {
        fdc_status = 0x10;  /* Record not found */
        return 0xff;
    }
    fd_sector_bytes++;
    fdc_status_flag = 0x80;

    if (fd_sector_bytes == fd_sector_size) {
        if (fdc_command & 0x10) {
            fdc_sector_register++;
            fdc_find_sector();
            fdc_status_flag = 0x80;
        } else {
            fdc_status_flag = 0x40;
            fdc_status = 0;
        }
    }
    return data;
}

void fdc_write(uint8_t data) {
    uint8_t driveno;
    UINT bw;

    driveno = fdc_control2 & 1;

    if ((fdc_command & 0xe0) != 0xa0) {
        fdc_data_register = data;
        return;
    }

    if (fd_drive_status[driveno] != 1) {
        fdc_status_flag = 0x80;
        fdc_status = 0x43;
        fd_sector_bytes++;
        if (fd_sector_bytes == fd_sector_size) {
            fdc_status_flag = 0x40;
            fdc_status = 0x40;
        }
        return;
    }

    if (fd_sector_bytes == fd_sector_size) {
        fdc_status_flag = 0x40;
        fdc_status = 0;
    }

    f_write(&fd_drive[driveno], &data, 1, &bw);
    fd_sector_bytes++;
    fdc_status_flag = 0x80;

    if (fd_sector_bytes == fd_sector_size) {
        f_sync(&fd_drive[driveno]);
        if (fdc_command & 0x10) {
            fdc_sector_register++;
            fdc_find_sector();
            fdc_status_flag = 0x80;
        } else {
            fdc_status_flag = 0x40;
            fdc_status = 0;
            fdc_command = 0;
        }
    }
}

void fdc_init(void) {
    fdc_status = 0;
    fdc_status_flag = 0;
    fdc_command = 0;
    fdc_track_register = 0;
    fdc_sector_register = 0;
    fdc_data_register = 0;
    fdc_control1 = 0;
    fdc_control2 = 0;
    fd_seek_dir = 0;
    fd_sector_size = 512;
    fd_sector_bytes = 0;
    memset(fd_track, 0, sizeof(fd_track));
    memset(fd_drive_status, 0, sizeof(fd_drive_status));
    memset(fd_image_status, 0, sizeof(fd_image_status));
}

uint8_t fdc_read_status(void) {
    uint8_t driveno = fdc_control2 & 1;
    if (fd_drive_status[driveno] == 0) return fdc_status | 0x80;  /* Not ready */
    if (fd_drive_status[driveno] == 3) return fdc_status | 0x40;  /* Write protected */
    return fdc_status;
}

uint8_t fdc_read_track(void)       { return fdc_track_register; }
uint8_t fdc_read_sector(void)      { return fdc_sector_register; }
uint8_t fdc_read_control1(void)    { return fdc_control1; }
uint8_t fdc_read_control2(void)    { return fdc_control2 & 0xfb; }
uint8_t fdc_read_status_flag(void) { return (~fdc_status_flag) & 0xc0; }

void fdc_write_track(uint8_t data)    { fdc_track_register = data; }
void fdc_write_sector(uint8_t data)   { fdc_sector_register = data; }
void fdc_write_control1(uint8_t data) { fdc_control1 = data; }
void fdc_write_control2(uint8_t data) { fdc_control2 = data; }
