/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2019 ARM Ltd.
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
 * $Date:        15. July 2019
 * $Revision:    V1.0.0
 *
 * Project:      Flash Device Description for ST STM32G0xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "..\FlashOS.h"        // FlashOS Structures

#ifdef FLASH_MEM

#ifdef STM32G0x_16
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0xx 16 KB Flash",    // Device Name (64kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00004000,                 // Device Size in Bytes (16kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

   // Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB
   SECTOR_END
};
#endif

#ifdef STM32G0x_32
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0xx 32 KB Flash",    // Device Name (64kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00008000,                 // Device Size in Bytes (32kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

   // Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB
   SECTOR_END
};
#endif

#ifdef STM32G0x_64
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0xx 64 KB Flash",    // Device Name (64kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00010000,                 // Device Size in Bytes (64kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

   // Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB
   SECTOR_END
};
#endif

#ifdef STM32G0x_128
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "STM32G0xx 128 KB Flash",   // Device Name (128kB)
   ONCHIP,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00020000,                 // Device Size in Bytes (128kB)
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   400,                        // Program Page Timeout 400 mSec
   400,                        // Erase Sector Timeout 400 mSec

	 // Specify Size and Address of Sectors
   0x800, 0x000000,            // Sector Size  2kB
   SECTOR_END
};
#endif

#endif // FLASH_MEM


#if defined FLASH_OTP

  struct FlashDevice const FlashDevice  =  {
      FLASH_DRV_VERS,                     // Driver Version, do not modify!
      "STM32G0xx Flash OTP",              // Device Name
      ONCHIP,                             // Device Type
      0x1FFF7000,                         // Device Start Address
      0x00000400,                         // Device Size in Bytes (1024)
      1024,                               // Programming Page Size
      0,                                  // Reserved, must be 0
      0xFF,                               // Initial Content of Erased Memory
      3000,                               // Program Page Timeout 3 Sec
      3000,                               // Erase Sector Timeout 3 Sec

      // Specify Size and Address of Sectors
      0x0400, 0x000000,                   // Sector Size 1024B
      SECTOR_END
  };

#endif // FLASH_OTP


#if defined FLASH_OPT

#if defined STM32G0x0
  struct FlashDevice const FlashDevice  =  {
      FLASH_DRV_VERS,                     // Driver Version, do not modify!
      "STM32G0x0 Flash Options",          // Device Name
      ONCHIP,                             // Device Type
      0x1FFF7800,                         // Device Start Address
      0x0000000C,                         // Device Size in Bytes (12)
      12,                                 // Programming Page Size
      0,                                  // Reserved, must be 0
      0xFF,                               // Initial Content of Erased Memory
      3000,                               // Program Page Timeout 3 Sec
      3000,                               // Erase Sector Timeout 3 Sec

      // Specify Size and Address of Sectors
      0x000C, 0x000000,                   // Sector Size 12B
      SECTOR_END
  };
#endif // STM32G0x0

#if defined STM32G0x1
  struct FlashDevice const FlashDevice  =  {
      FLASH_DRV_VERS,                     // Driver Version, do not modify!
      "STM32G0x1 Flash Options",          // Device Name
      ONCHIP,                             // Device Type
      0x1FFF7800,                         // Device Start Address
      0x00000020,                         // Device Size in Bytes (32)
      32,                                 // Programming Page Size
      0,                                  // Reserved, must be 0
      0xFF,                               // Initial Content of Erased Memory
      3000,                               // Program Page Timeout 3 Sec
      3000,                               // Erase Sector Timeout 3 Sec

      // Specify Size and Address of Sectors
      0x0020, 0x000000,                   // Sector Size 32B
      SECTOR_END
  };
#endif // STM32G0x1

#endif // FLASH_OPT
