;/*****************************************************************************/
;/* STM32G0x0_OPT.s: STM32G0x0 Flash Option Bytes                             */
;/*****************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                          */
;/*****************************************************************************/
;/*  This file is part of the uVision/ARM development tools                   */
;/*  Copyright (c) 2015 - 2018 Keil - An ARM Company.                         */
;/*  All rights reserved.                                                     */
;/*****************************************************************************/

;// <e> Flash Option Bytes
FLASH_OPT       EQU     1

;// <h> Flash Read Protection
;//     <i> Read protection is used to protect the software code stored in Flash memory
;//   <o0.0..7> Read Protection Level
;//     <i> Level 0: No Protection
;//     <i> Level 1: Read Protection of Memories (debug features limited)
;//     <i> Level 2: Chip Protection (debug and boot in RAM features disabled)
;//          <0xAA=> Level 0 (No Protection)
;//          <0x00=> Level 1 (Read Protection of Memories)
;//          <0xCC=> Level 2 (Chip Protection)
;// </h>

;// <h> User Configuration
;//   <o0.26> nBOOT0
;//     <i> nBOOT0 option bit
;//   <o0.25> nBOOT1
;//     <i> Boot configuration
;//     <i> Together with the BOOT0 pin or option bit nBOOT0
;//     <i> (depending on nBOOT_SEL option bit configuration), 
;//     <i> this bit selects boot mode from the Main Flash memory, SRAM or the System memory.
;//   <o0.24> nBOOT_SEL
;//     <i> Boot selection
;//          <0=> BOOT0 signal is defined by BOOT0 pin value (legacy mode)
;//          <1=> BOOT0 signal is defined by nBOOT0 option bit
;//   <o0.22> RAM_PARITY_CHECK
;//     <i> SRAM parity check 
;//          <0=> SRAM parity check enabled
;//          <1=> SRAM parity check disabled
;//   <o0.19> WWDG_SW
;//     <i> Window watchdog selection
;//          <0=> Hardware window watchdog
;//          <1=> Software window watchdog
;//   <o0.18> IWDG_STDBY
;//     <i> Independent watchdog counter freeze in Standby mode
;//          <0=> Freeze IWDG counter in Standby mode
;//          <1=> IWDG counter active in Standby mode
;//   <o0.17> IWDG_STOP
;//     <i> Independent watchdog counter freeze in Stop mode
;//          <0=> Freeze IWDG counter in STOP mode
;//          <1=> IWDG counter active in STOP mode
;//   <o0.16> IWDG_SW
;//     <i> Independent watchdog selection
;//          <0=> Hardware independant watchdog
;//          <1=> Software independant watchdog
;//   <o0.14> nRST_STDBY
;//     <i> Generate Reset when entering Standby Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.13> nRST_STOP
;//     <i> Generate Reset when entering STOP Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;// </h>
FLASH_OPTR     EQU     0x08EF10AA       ; reset value b11011xxx1x11xxxx1xx00001xxxxxxxx

;// <h> WRP Configuration
;//   <o0.0..5> WRP1A_STRT   <0x0-0x3F>
;//     <i> WRP A area: Base address + (WRP_A_STRT x 0x800) (included) to Base address + ((WRP_A_END+1) x 0x800) (excluded)
;//   <o0.16..21> WRP1A_END  <0x0-0x3F>
;//     <i> WRP A area: Base address + (WRP_A_STRT x 0x800) (included) to Base address + ((WRP_A_END+1) x 0x800) (excluded)
;//   <o1.0..5> WRP1B_STRT   <0x0-0x3F>
;//     <i> WRP B area: Base address + (WRP_B_STRT x 0x800) (included) to Base address + ((WRP_B_END+1) x 0x800) (excluded)
;//   <o1.16..21> WRP1B_END  <0x0-0x3F>
;//     <i> WRP B area: Base address + (WRP_B_STRT x 0x800) (included) to Base address + ((WRP_B_END+1) x 0x800) (excluded)
;// </h>
FLASH_WRP1AR   EQU     0xFF00FF00       ; reset value 0xFFXXFFXX
FLASH_WRP1BR   EQU     0xFF00FF00       ; reset value 0xFFXXFFXX

;// </e>


                IF      FLASH_OPT <> 0
                AREA    |.ARM.__AT_0x1FFF7800|, CODE, READONLY
                DCD     FLASH_OPTR
                DCD     FLASH_WRP1AR
                DCD     FLASH_WRP1BR
                ENDIF

                END
