;/*****************************************************************************/
;/* STM32G0x1_OPT.s: STM32G0x1 Flash Option Bytes                             */
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
;//   <o0.29> IRHEN:
;//     <i> Internal reset holder enable bit
;//          <0=> Internal resets are propagated as simple pulse on NRST pin
;//          <1=> Internal resets drives NRST pin low until it is seen as low level
;//   <o0.27..28> NRST_MODE
;//     <i> Reset mode.
;//     <i> Reset Input only: (a low level on the NRST pin generates system reset, internal RESET not propagated to the NSRT pin
;//     <i> GPIO: standard GPIO pad functionality, only internal RESET possible
;//     <i> Bidirectional reset: NRST pin configured in reset input/output mode (legacy mode)
;//          <1=> Reset Input only
;//          <2=> GPIO
;//          <3=> Bidirectional reset
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
;//   <o0.15> nRST_SHDW
;//     <i> Generate Reset when entering Shutdown Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.14> nRST_STDBY
;//     <i> Generate Reset when entering Standby Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.13> nRST_STOP
;//     <i> Generate Reset when entering STOP Mode
;//          <0=> Reset generated
;//          <1=> Reset not generated
;//   <o0.11..12> BORR_LEV
;//     <i> These bits contain the VDD supply level threshold that releases the reset.
;//          <0=> BOR rising level 1 with threshold around 2.1 V
;//          <1=> BOR rising level 2 with threshold around 2.3 V
;//          <2=> BOR rising level 3 with threshold around 2.6 V
;//          <3=> BOR rising level 4 with threshold around 2.9 V
;//   <o0.9..10> BORF_LEV
;//     <i> These bits contain the VDD supply level threshold that activates the reset.
;//          <0=> BOR falling level 1 with threshold around 2.0 V
;//          <1=> BOR falling level 2 with threshold around 2.2 V
;//          <2=> BOR falling level 3 with threshold around 2.5 V
;//          <3=> BOR falling level 4 with threshold around 2.8 V
;//   <o0.8> BOR_EN
;//     <i> Brown out reset enable
;//          <0=> Configurable brown out reset disabled
;//          <1=> Configurable brown out reset enabled
;// </h>
FLASH_OPTR     EQU     0xF8FFFEAA       ; reset value 0xFFFFFEAA

;// <h> PCROP / WRP Configuration
;//   <o0.0..7> PCROP1A_STRT  <0x0-0xFF>
;//     <i> Contains the offset of the first subpage of the PCROP1A area.
;//   <o1.0..7> PCROP1A_END  <0x0-0xFF>
;//     <i> Contains the offset of the last subpage of the PCROP1A area.
;//   <o1.31> PCROP_RDP
;//     <i> PCROP zone erase upon RDP level regression
;//   <o2.0..7> PCROP1B_STRT  <0x0-0xFF>
;//     <i> Contains the offset of the first subpage of the PCROP1B area.
;//   <o3.0..7> PCROP1B_END  <0x0-0xFF>
;//     <i> Contains the offset of the last subpage of the PCROP1B area.
;//   <o4.0..5> WRP1A_STRT  <0x0-0x3F>
;//     <i> WRP A area: Base address + (WRP_A_STRT x 0x800) (included) to Base address + ((WRP_A_END+1) x 0x800) (excluded)
;//   <o4.16..21> WRP1A_END  <0x0-0x3F>
;//     <i> WRP A area: Base address + (WRP_A_STRT x 0x800) (included) to Base address + ((WRP_A_END+1) x 0x800) (excluded)
;//   <o5.0..5> WRP1B_STRT  <0x0-0x3F>
;//     <i> WRP B area: Base address + (WRP_B_STRT x 0x800) (included) to Base address + ((WRP_B_END+1) x 0x800) (excluded)
;//   <o5.16..21> WRP1B_END  <0x0-0x3F>
;//     <i> WRP B area: Base address + (WRP_B_STRT x 0x800) (included) to Base address + ((WRP_B_END+1) x 0x800) (excluded)
;// </h>
FLASH_PCROP1ASR EQU     0x0000FFFF       ; reset value 0xFFFFFFFF
FLASH_PCROP1AER EQU     0x80000000       ; reset value 0x00000000
FLASH_PCROP1BSR EQU     0x0000FFFF       ; reset value 0xFFFFFFFF
FLASH_PCROP1BER EQU     0x00000000       ; reset value 0x00000000
FLASH_WRP1AR    EQU     0x000000FF       ; reset value 0x000000FF
FLASH_WRP1BR    EQU     0x000000FF       ; reset value 0x000000FF

;// <h> Flash Security
;//   <o0.16> BOOT_LOCK
;//     <i> used to force boot from user area
;//          <0=> Boot based on the pad/option bit configuration
;//          <1=> Boot forced from Main Flash memory
;//   <o0.0..6> SEC_SIZE  <0x0-0x7F>
;//     <i> Securable memory area size. Contains the number of securable Flash memory pages
;// </h>
FLASH_SECR      EQU     0x00000000       ; reset value 0x00000000

;// </e>


                IF      FLASH_OPT <> 0
                AREA    |.ARM.__AT_0x1FFF7800|, CODE, READONLY
                DCD     FLASH_OPTR
                DCD     FLASH_WRP1AR
                DCD     FLASH_WRP1BR
                DCD     FLASH_PCROP1ASR
                DCD     FLASH_PCROP1AER
                DCD     FLASH_PCROP1BSR
                DCD     FLASH_PCROP1BER
                DCD     FLASH_SECR
                ENDIF

                END
