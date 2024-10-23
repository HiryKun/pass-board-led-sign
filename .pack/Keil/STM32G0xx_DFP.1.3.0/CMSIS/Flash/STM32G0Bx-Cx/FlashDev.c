/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2015 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        21. July 2015
 * $Revision:    V1.00
 *
 * Project:      Flash Device Description for ST STM32G0xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */

#include "..\FlashOS.H"        // FlashOS Structures

#ifdef FLASH_MEM
#ifdef STM32G0x_128
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0Bx_128",      // Device Name (128kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x0020000,                 // Device Size in Bytes (128kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

// Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB (64 Sectors)
   SECTOR_END
};
#endif

#ifdef STM32G0Bx_256
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0Bx_256",           // Device Name (256kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x0040000,                 // Device Size in Bytes (256kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

// Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB (128 Sectors)
   SECTOR_END
};
#endif
#ifdef STM32G0x_512
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0Bx_512",      // Device Name (128kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x0080000,                 // Device Size in Bytes (512kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

// Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB (256 Sectors)
   SECTOR_END
};
#endif

#endif // FLASH_MEM
