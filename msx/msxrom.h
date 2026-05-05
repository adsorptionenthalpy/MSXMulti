/* ROM areas - loaded from SD card into RAM at startup.
 * This avoids all flash programming issues and makes the emulator
 * purely SD-card based. RP2040 has 264KB SRAM; we allocate:
 *   - 32KB for BIOS ROM
 *   - 16KB for Disk ROM
 *   - 32KB for cartridge ROM
 */

/* ROM buffers in RAM */
extern uint8_t bios_ram[0x8000];     /* 32KB BIOS */
extern uint8_t diskrom_ram[0x4000];  /* 16KB Disk ROM */
extern uint8_t cartrom_buf[0x8000];  /* 32KB cart buffer */

/* ROM pointers - point to RAM buffers */
extern uint8_t *basicrom;
extern uint8_t *extrom1;
extern uint8_t *extrom2;
extern uint8_t *cartrom1;
extern uint8_t *cartrom2;
