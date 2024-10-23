/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2018 ARM Ltd.
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
 * $Date:        03. December 2018
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
  vu32 PRIVCR;           /* Offset: 0x04  Power Down Key Register */
  vu32 KEYR;             /* Offset: 0x08  Key Register */
  vu32 OPTKEYR;          /* Offset: 0x0C  Option Key Register */
  vu32 SR;               /* Offset: 0x10  Status Register */
  vu32 CR;               /* Offset: 0x14  Control Register */
  vu32 ECCR;             /* Offset: 0x18  ECC Register */
  vu32 RESERVED0;
  vu32 OPTR;             /* Offset: 0x20  Option Register */
} FLASH_TypeDef;


// Flash Keys
#define RDPRT_KEY                0x00A5
#define FLASH_KEY1               0x45670123
#define FLASH_KEY2               0xCDEF89AB
#define FLASH_OPTKEY1            0x08192A3B
#define FLASH_OPTKEY2            0x4C5D6E7F

// Flash Control Register definitions
#define FLASH_PG                ((unsigned int)(   1U <<  0))
#define FLASH_PER               ((unsigned int)(   1U <<  1))
#define FLASH_MER1              ((unsigned int)(   1U <<  2))
#define FLASH_PNB_MSK           ((unsigned int)(0xFFU <<  3))
#define FLASH_BKER              ((unsigned int)(   1U << 13))
#define FLASH_MER2              ((unsigned int)(   1U << 15))
#define FLASH_STRT              ((unsigned int)(   1U << 16))
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
#define FLASH_OPTR_DUALBANK1   ((u32)0x00100000)

#define FLASH_BSY               ((unsigned int)(   1U << 16))
#define FLASH_BSY2               ((unsigned int)(   1U << 17))
#define FLASH_PGERR             (FLASH_OPERR  | FLASH_PROGERR | FLASH_PROGERR | FLASH_WRPERR  | FLASH_PGAERR | \
                                 FLASH_SIZERR | FLASH_PGSERR  | FLASH_MISSERR | FLASH_FASTERR | FLASH_RDERR   )



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

  return (0);                                           // Done
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

  FLASH->CR |= FLASH_LOCK;                              // Lock Flash

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

int EraseChip (void) {

  while (FLASH->SR & FLASH_BSY) {
  }


  
  FLASH->CR |=  FLASH_MER1 ; 
 FLASH->CR |=  FLASH_MER2 ; 	
  FLASH->CR |=  FLASH_STRT;                               // Start Erase
  
  while (FLASH->SR & FLASH_BSY) {
  }



  if (FLASH->SR & FLASH_PGERR) {                          // Check for Error
    FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags
    return (1);                                           // Failed
  }
	
		  FLASH->CR &= (~FLASH_MER1);


  return (0);                                           // Done
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */


int EraseSector (unsigned long adr) {

  unsigned long page;
unsigned long bank1_addr;

	/*Flash Bank address*/
  bank1_addr = (*(unsigned long *)0x1FFF75E0);
   bank1_addr&=0x00000FFF;
   bank1_addr=((bank1_addr*0x400)/2)+0x8000000;

  /*Clear the error status*/
  FLASH->SR = 0x000001FF;	
	 
  /*Clear page and block bits*/
  FLASH->CR &= (~FLASH_PNB_MSK);

  FLASH->CR &= (~FLASH_BKER);	

  /*Calculate the page and set
    the block bit if needed*/
  if(adr<bank1_addr){
    page = ((adr-0x8000000) >>8) ;
 
	}
	else{
		 page = ((adr-0x8000000) >>8) ;
	FLASH->CR |= FLASH_BKER;//(1<<13);
	}
  /*Set the page and the Page erase bit*/
  FLASH->CR |= (FLASH_PER | page);
  /*Start the erase operation*/  
  FLASH->CR |= FLASH_STRT;	

  /*wait until the operation ends*/
  while (FLASH->SR & FLASH_BSY);
 while (FLASH->SR & FLASH_BSY2);
	
	if (FLASH->SR & FLASH_PGERR) {                          // Check for Error
    FLASH->SR  = FLASH_PGERR;                             // Reset Error Flags
    return (1);                                           // Failed
  }
	
	 FLASH->CR &= (~FLASH_PER);
	FLASH->CR&= (~FLASH_BKER);
	FLASH->CR &= ~(FLASH_PNB_MSK);
	 if ((* (unsigned long *)0x08000000) != 0xFFFFFFFF)
  {
    FLASH->ACR &= 0xFFFEFFFF;
  }
  return (0);                                           // Done
}





/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
 
	sz = (sz + 7) & ~7;  
unsigned long tab[8];
	int i;
	
    while (FLASH->SR & FLASH_BSY) {};
    FLASH->SR = 0x000001FF;                               // Reset Error Flags
		FLASH->CR |= FLASH_PG ;	                              // Programming Enabled

  while (sz) {
    
		if(sz>=8){
      M32(adr) = *((u32 *)(buf+0 ));                   // Program the first word of the Double Word
			M32(adr+4) = *((u32 *)(buf+4 )); 
			adr += 8;                                           // Go to next DoubleWord
    buf += 8;
    sz  -= 8;
      while (FLASH->SR & FLASH_BSY);
			 while (FLASH->SR & FLASH_BSY2);
	 }
else{
	     for(i=0;i<sz;i++)
        {
          tab[i]= *((unsigned char *)buf);
          buf=buf+1;
        }
        
        for(i=0;i<8-sz;i++)
        {
          tab[i+sz]=0xFF;
        }
 
     
       
       /*wait until the operation ends*/
       while (FLASH->SR & FLASH_BSY){};
         while (FLASH->SR & FLASH_BSY2){};
                  
          M32(adr) = *((u32 *)(&tab+0 ));   
		    M32(adr+4) = *((u32 *)(&tab+4 )); 
         sz =0;
	
}
    if ((FLASH->SR & FLASH_EOP)) {                        // Check for Error
      FLASH->SR  = FLASH_PGERR;                           // Reset Error Flags
      return (1);                                         // Failed
    }
		
 
  }
		FLASH->CR &= (~FLASH_PG) ;	  
	 if ((* (unsigned long *)0x08000000) != 0xFFFFFFFF)
  {
    FLASH->ACR &= 0xFFFEFFFF;
  }

	
	
  return (0);                                           // Done

}

