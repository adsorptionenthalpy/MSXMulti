// Configuration
#define HW_FLASH_STORAGE_MEGABYTES 2     // Define the pico's FLASH size in Megabytes (1MB - 16MB)
//#define USE_I2S     // Enable I2S DAC Output and SCC emulation
//#define USE_OPLL    // Enable OPLL emulation. need FM-PAC BIOS.
#define USE_FDC     // Enable SONY HBD-F1 emulation. need DISKBIOS.
//#define USE_MORE_OVERCLOCK    // Change System Core clock from 225MHz to 250MHz
//#define USE_CORE_VOLTAGE12    // Change Core voltage to 1.2Volt


// Dependency
#ifdef USE_OPLL
// OPLL emulation require I2S DAC output and more CPU power
#define USE_I2S
#define USE_MORE_OVERCLOCK      
#endif
