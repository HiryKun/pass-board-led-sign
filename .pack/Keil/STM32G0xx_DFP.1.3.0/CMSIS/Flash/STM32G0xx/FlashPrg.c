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
 * Project:      Flash Programming Functions for ST STM32G0xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "..\FlashOS.h"        // FlashOS Structures

typedef volatile unsigned long    vu32;
typedef          unsigned long     u32;

#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define FLASH_BASE        0x40022000

#define FLASH           ((FLASH_TypeDef  *) FLASH_BASE)

// Flash Registers
typedef struct {
  vu32 ACR;              /* Offset: 0x00  Access Control Register */
  vu32 RESERVED0;        /* Offset: 0x04  Reserved */
  vu32 KEYR;             /* Offset: 0x08  Key Register */
  vu32 OPTKEYR;          /* Offset: 0x0C  Option Key Register */
  vu32 SR;               /* Offset: 0x10  Status Register */
  vu32 CR;               /* Offset: 0x14  Control Register */
  vu32 ECCR;             /* Offset: 0x18  ECC Register */
  vu32 RESERVED1;        /* Offset: 0x1C  Reserved */
  vu32 OPTR;             /* Offset: 0x20  Option Register */
  vu32 PCROP1ASR;        /* Offset: 0x24  Bank PCROP area A Start address Register */
  vu32 PCROP1AER;        /* Offset: 0x28  Bank PCROP area A End address Register */
  vu32 WRP1AR;           /* Offset: 0x2C  Bank WRP area A address Register */
  vu32 WRP1BR;           /* Offset: 0x30  Bank WRP area B address Register */
  vu32 PCROP1BSR;        /* Offset: 0x34  Bank PCROP area B Start address Register */
  vu32 PCROP1BER;        /* Offset: 0x38  Bank PCROP area B End address Register */
  vu32 RESERVED3[17];    /* Offset: 0x3C  Reserved */
  vu32 SECR;             /* Offset: 0x80  Security Register */
} FLASH_TypeDef;


// Flash Keys
//#define RDPRT_KEY                0x00A5
#define FLASH_KEY1               0x45670123
#define FLASH_KEY2               0xCDEF89AB
#define FLASH_OPTKEY1            0x08192A3B
#define FLASH_OPTKEY2            0x4C5D6E7F

// Flash Control Register definitions
#define FLASH_PG                ((unsigned int)(   1U <<  0))
#define FLASH_PER               ((unsigned int)(   1U <<  1))
#define FLASH_MER1              ((unsigned int)(   1U <<  2))
#define FLASH_PNB_MSK           ((unsigned int)(0x3FU <<  3))
#define FLASH_STRT              ((unsigned int)(   1U << 16))
#define FLASH_OPTSTRT           ((unsigned int)(   1U << 17))
#define FLASH_OBL_LAUNCH        ((unsigned int)(   1U << 27))
#define FLASH_OPTLOCK           ((unsigned int)(   1U << 30))
#define FLASH_LOCK              ((unsigned int)(   1U << 31))

// Flash Status Register definitions
#define FLASH_EOP               ((unsigned int)(   1U <<  0))
#define FLASH_OPERR             ((unsigned int)(   1U <<  1))
#define FLASH_PROGERR           ((unsigned int)(   1U <<  3))
#define FLASH_WRPERR            ((unsigned int)(   1U <<  4))
#define FLASH_PGAERR            ((unsigned int)(   1U <<  5))
#define FLASH_SIZERR            ((unsigned int)(   1U <<  6))
#define FLASH_PGSERR            ((unsigned int)(   1U <<  7))
#define FLASH_MISSERR           ((unsigned int)(   1U <<  8))
#define FLASH_FASTERR           ((unsigned int)(   1U <<  9))
#define FLASH_RDERR             ((unsigned int)(   1U << 14))
#define FLASH_OPTVERR           ((unsigned int)(   1U << 15))
#define FLASH_BSY               ((unsigned int)(   1U << 16))

#define FLASH_PGERR             (FLASH_OPERR  | FLASH_PROGERR | FLASH_PROGERR | FLASH_WRPERR  | FLASH_PGAERR | \
                                 FLASH_SIZERR | FLASH_PGSERR  | FLASH_MISSERR | FLASH_FASTERR | FLASH_RDERR  | \
                                 FLASH_OPTVERR                                                                )

void DSB(void) {
    __asm("DSB");
}


/*
 * Get Sector Number
 *    Parameter:      adr:  Sector Address
 *    Return Value:   Sector Number
 */

#if defined FLASH_MEM
unsigned long GetSecNum (unsigned long adr) {
  unsigned long secNum;

  /* Single-Bank Flash */
  /* sector number 0..63 */
  secNum = ( (adr & 0x001FFFF) >> 11);

  return (secNum);                                      // Sector Number
}
#endif // FLASH_MEM


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

#ifdef FLASH_OPT
  FLASH->OPTKEYR  = FLASH_OPTKEY1;                      // Unlock Option Bytes
  FLASH->OPTKEYR  = FLASH_OPTKEY2;
#endif // FLASH_OPT

//FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Cache, no Prefetch
  FLASH->SR   = FLASH_PGERR;                            // Reset Error Flags

  return (0);                                           // Done
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

  FLASH->CR  = FLASH_LOCK;                              // Lock Flash

#ifdef FLASH_OPT
  FLASH->CR = FLASH_OPTLOCK;                            // Lock option bytes
#endif // FLASH_OPT

  return (0);                                           // Done
}


/*
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat) {

  /* force erase even if the content is 'Initial Content of Erased Memory'.
     Only a erased sector can be programmed. I think this is because of ECC */

  return (1);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined FLASH_MEM
int EraseChip (void) {

  FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags

  FLASH->CR  = (FLASH_MER1             );               // Bank     Mass Erase Enabled
  FLASH->CR |=  FLASH_STRT;                             // Start Erase
  DSB();

  while (FLASH->SR & FLASH_BSY) {
  }

  FLASH->CR  =  0;                                      // Reset CR

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR |= FLASH_PGERR;                           // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}
#endif // FLASH_MEM

#ifdef FLASH_OPT
int EraseChip (void) {

  FLASH->SR        = FLASH_PGERR;                       // Reset Error Flags

  FLASH->OPTR      = 0x384FFEAA;                        // Write OPTR reset value
  FLASH->WRP1AR    = 0x0000003F;                        // Write WRP1AR reset value
  FLASH->WRP1BR    = 0x0000003F;                        // Write WRP1BR reset value
#if defined STM32G0x1
  FLASH->PCROP1ASR = 0x000000FF;                        // Write PCROP1ASR reset value
  FLASH->PCROP1AER = 0x00000000;                        // Write PCROP1AER reset value
  FLASH->PCROP1BSR = 0x000000FF;                        // Write PCROP1BSR reset value
  FLASH->PCROP1BER = 0x00000000;                        // Write PCROP1BER reset value
  FLASH->SECR      = 0x00000000;                        // Write SECR reset value
#endif

  FLASH->CR       = FLASH_OPTSTRT;                      // Program values
  DSB();

  while (FLASH->SR & FLASH_BSY) {
  }

  FLASH->CR  =  0;                                      // Reset CR

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR |= FLASH_PGERR;                           // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}
#endif // FLASH_OPT


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseSector (unsigned long adr) {
  u32 n;

  n = GetSecNum(adr);                                   // Get Sector Number 0..255 or 0..511

  FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags

  FLASH->CR  = (FLASH_PER |                             // Page Erase Enabled
                (n <<  3)  );                           // Sector Number. 0 to 255 for each bank
  FLASH->CR |=  FLASH_STRT;                             // Start Erase
  DSB();

  while (FLASH->SR & FLASH_BSY) {
  }

  FLASH->CR  =  0;                                      // Reset CR

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR  = FLASH_PGERR;                           // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}
#endif // FLASH_MEM

#if defined FLASH_OPT || defined FLASH_OTP
int EraseSector (unsigned long adr) {

  /* erase sector is not needed for
     - Flash Option bytes
     - Flash One Time Programmable bytes
  */

  return (0);                                           // Done
}
#endif


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined FLASH_MEM || defined FLASH_OTP
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = (sz + 7) & ~7;                                   // Adjust size for two words

  FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags

  while (sz) {
    FLASH->CR = FLASH_PG ;                              // Programming Enabled

    M32(adr    ) = *((u32 *)(buf + 0));                 // Program the first word of the Double Word
    M32(adr + 4) = *((u32 *)(buf + 4));                 // Program the second word of the Double Word
    DSB();

    while (FLASH->SR & FLASH_BSY) {
    }

    FLASH->CR  =  0;                                    // Reset CR

    if (FLASH->SR & FLASH_PGERR) {                      // Check for Error
      FLASH->SR  = FLASH_PGERR;                         // Reset Error Flags
      return (1);                                       // Failed
    }

    adr += 8;                                           // Go to next DoubleWord
    buf += 8;
    sz  -= 8;
  }

  return (0);                                           // Done
}
#endif // FLASH_MEM || FLASH_OTP

#ifdef FLASH_OPT
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  u32 optr;
  u32 wrp1ar;
  u32 wrp1br;
#if defined STM32G0x1
  u32 pcrop1asr;
  u32 pcrop1aer;
  u32 pcrop1bsr;
  u32 pcrop1ber;
  u32 secr;
#endif

  optr      = *((u32 *)(buf +  0));
  wrp1ar    = *((u32 *)(buf +  4));
  wrp1br    = *((u32 *)(buf +  8));
#if defined STM32G0x1
  pcrop1asr = *((u32 *)(buf + 12));
  pcrop1aer = *((u32 *)(buf + 16));
  pcrop1bsr = *((u32 *)(buf + 20));
  pcrop1ber = *((u32 *)(buf + 24));
  secr      = *((u32 *)(buf + 28));
#endif

  FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags

  FLASH->OPTR      = (optr      & 0x3F4FFFFF);          // Write OPTR values
  FLASH->WRP1AR    = (wrp1ar    & 0x003F003F);          // Write WRP1AR values
  FLASH->WRP1BR    = (wrp1br    & 0x003F003F);          // Write WRP1BR values
#if defined STM32G0x1
  FLASH->PCROP1ASR = (pcrop1asr & 0x000000FF);          // Write PCROP1ASR values
  FLASH->PCROP1AER = (pcrop1aer & 0x800000FF);          // Write PCROP1AER values
  FLASH->PCROP1BSR = (pcrop1bsr & 0x000000FF);          // Write PCROP1BSR values
  FLASH->PCROP1BER = (pcrop1ber & 0x000000FF);          // Write PCROP1BER values
  FLASH->SECR      = (secr      & 0x000100FF);          // Write SECR values
#endif

  FLASH->CR  = FLASH_OPTSTRT;                           // Program values
  DSB();

  while (FLASH->SR & FLASH_BSY) {
  }

  if (FLASH->SR & FLASH_PGERR) {                        // Check for Error
    FLASH->SR |= FLASH_PGERR;                           // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}
#endif //  FLASH_OPT


/*
 *  Verify Flash Contents
 *    Parameter:      adr:  Start Address
 *                    sz:   Size (in bytes)
 *                    buf:  Data
 *    Return Value:   (adr+sz) - OK, Failed Address
 */

#ifdef FLASH_OPT
unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf) {
  u32 optr;
  u32 wrp1ar;
  u32 wrp1br;
#if defined STM32G0x1
  u32 pcrop1asr;
  u32 pcrop1aer;
  u32 pcrop1bsr;
  u32 pcrop1ber;
  u32 secr;
#endif

  optr      = *((u32 *)(buf +  0));
  wrp1ar    = *((u32 *)(buf +  4));
  wrp1br    = *((u32 *)(buf +  8));
#if defined STM32G0x1
  pcrop1asr = *((u32 *)(buf + 12));
  pcrop1aer = *((u32 *)(buf + 16));
  pcrop1bsr = *((u32 *)(buf + 20));
  pcrop1ber = *((u32 *)(buf + 24));
  secr      = *((u32 *)(buf + 28));
#endif

#if 0 /* verify does not work because I do not get the registers updated without a reset! */
  FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags
  FLASH->CR  = FLASH_OBL_LAUNCH;                        // Load Option Bytes -> generates a reset!
  DSB();

  if ((FLASH->OPTR      & 0x3F4FFFFF) != (optr      & 0x3F4FFFFF)) {    // Check OPTR values
    return (adr + 0);
  }

  if ((FLASH->WRP1AR    & 0x003F003F) != (wrp1ar    & 0x003F003F)) {    // Check WRP1AR values
    return (adr + 1);
  }

  if ((FLASH->WRP1BR    & 0x003F003F) != (wrp1br    & 0x003F003F)) {    // Check WRP1BR values
    return (adr + 2);
  }

#if defined STM32G0x1
  if ((FLASH->PCROP1ASR & 0x000000FF) != (pcrop1asr & 0x000000FF)) {    // Check PCROP1ASR values
    return (adr + 3);
  }

  if ((FLASH->PCROP1AER & 0x800000FF) != (pcrop1aer & 0x800000FF)) {    // Check PCROP1AER values
    return (adr + 4);
  }

  if ((FLASH->PCROP1BSR & 0x000000FF) != (pcrop1bsr & 0x000000FF)) {    // Check PCROP1BSR values
    return (adr + 5);
  }

  if ((FLASH->PCROP1BER & 0x000000FF) != (pcrop1ber & 0x000000FF)) {    // Check PCROP1BER values
    return (adr + 6);
  }

  if ((FLASH->SECR      & 0x000100FF) != (secr      & 0x000100FF)) {    // Check SECR values
    return (adr + 7);
  }
#endif

#endif
  return (adr + sz);
}
#endif // FLASH_OPT
