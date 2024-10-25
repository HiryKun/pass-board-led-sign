[#ftl]
[#list configs as dt]
[#assign data = dt]
[#assign peripheralParams = dt.peripheralParams]
[#assign peripheralGPIOParams = dt.peripheralGPIOParams]
[#assign peripheralDMAParams = dt.peripheralDMAParams]
[#assign peripheralNVICParams = dt.peripheralNVICParams]
[#assign usedIPs = dt.usedIPs]
[#assign adc_hal=false adc_ll=false comp_hal=false comp_ll=false can=false dac_hal=false dac_ll=false]
[#assign dma_hal=false dma_ll=false gpio_hal=false gpio_ll=false i2c_hal=false i2c_ll=false ]
[#assign lptim_hal=false lptim_ll=false lpuart_ll=false opamp_hal=false opamp_ll=false]
[#assign sai=false sdmmc=false spi_hal=false spi_ll=false tim_hal=false tim_ll=false] 
[#assign usart_ll=false uart_hal=false usart_hal=false irda_hal=false smartcard_hal=false]
[#assign pcd=false hcd=false]
[#assign nor=false nand=false pccard=false sram=false sdram=false]
[#assign timebase_tim = false]
[#if timeBaseSource?? && timeBaseSource!="SysTick"]
[#assign timebase_tim = true]
[/#if]
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!-- ******************************************************************************
 * File Name   : FrameworkCubeMX.gpdsc
 * Date        : ${date}
 * Description : Generator PDSC File generated by STM32CubeMX (DO NOT EDIT!)
 ****************************************************************************** -->

<package xmlns:xs="http://www.w3.org/2001/XMLSchema-instance" schemaVersion="1.0" xs:noNamespaceSchemaLocation="PACK.xsd">
  <vendor>Keil</vendor>
  <name>FrameworkCubeMX</name>
  <description>STM32CubeMX generated pack description</description>
  <url>project-path</url>
  <releases>
    <release version="1.0.0">
     - Generated: ${date}
    </release>
  </releases>
  <generators>
    <generator id="STM32CubeMX" Gvendor="STMicroelectronics" Gtool="STM32CubeMX" Gversion="4.10.0">
      <description>STM32CubeMX Environment</description>
      <select Dname="${(dt.device)}" Dvendor="STMicroelectronics:13"/>
      <command>$SMDK\CubeMX\STM32CubeMXLauncher</command>
      <workingDir>$PRTE\Device\${(dt.device)}</workingDir>
      <project_files>
        <file category="include" name="STCubeGenerated/Inc/"/>
        <file category="source" name="STCubeGenerated/Src/main.c" />
        <file category="header" name="STCubeGenerated/Inc/stm32g0xx_it.h"/>
        <file category="source" name="STCubeGenerated/Src/stm32g0xx_it.c"/>
      </project_files>
    </generator>
  </generators>
  <taxonomy>
    <description Cclass="Device" Cgroup="STM32Cube Framework" generator="STM32CubeMX">STM32Cube Framework</description>
  </taxonomy>
  <conditions>
    <condition id="STCubeMX">
      <description>Condition to include CMSIS core and Device Startup components</description>
      <require Dvendor="STMicroelectronics:13" Dname="STM32G0*"/>
      <require Cclass="CMSIS"  Cgroup="CORE"          Csub=""/>
      <require Cclass="Device" Cgroup="Startup"/>
      [#if peripheralParams.get("RCC")??]
      [#if peripheralParams.get("RCC").get("driver") == ("HAL")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="Common"/>
      [#else]
      <require Cclass="Device" Cgroup="STM32Cube LL" Csub="Common"/>
      [/#if]
      [/#if]
[#list usedIPs as ip]
[#if peripheralDMAParams.get(ip)?? && peripheralDMAParams.get(ip).entrySet()?size > 0]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !dma_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="DMA"/>
      [#assign dma_hal=true]
      [/#if]        
    [#elseif !dma_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="DMA"/>
      [#assign dma_ll=true]
    [/#if]
[/#if]
[#if peripheralGPIOParams.get(ip)?? && peripheralGPIOParams.get(ip).entrySet()?size > 0]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !gpio_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="GPIO"/>
      [#assign gpio_hal=true]
      [/#if]        
    [#elseif !gpio_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="GPIO"/>
      [#assign gpio_ll=true]
    [/#if]
[/#if]
[#if peripheralParams.get(ip)??]
[#if ip?starts_with("DMA2D")]
  [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="DMA2D"/>
  [#else]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="DMA2D"/>
  [/#if]
[#elseif ip?starts_with("DMA")]
  [#if peripheralParams.get(ip).get("driver") == ("HAL")]
    [#if !dma_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="DMA"/>
      [#assign dma_hal=true]
    [/#if]        
  [#elseif !dma_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="DMA"/>
      [#assign dma_ll=true]
  [/#if]
[#elseif ip?starts_with("GPIO")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !gpio_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="GPIO"/>
      [#assign gpio_hal=true]
      [/#if]        
    [#elseif !gpio_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="GPIO"/>
      [#assign gpio_ll=true]
    [/#if]
[#elseif ip?starts_with("ADC")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !adc_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="ADC"/>
      [#assign adc_hal=true]
      [/#if]
    [#elseif !adc_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="ADC"/>
      [#assign adc_ll=true]
    [/#if]
[#elseif ip?starts_with("AES")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="CRYP"/>
[#elseif ip?starts_with("CAN")]
  [#if !can]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="CAN"/>
      [#assign can=true]
  [/#if]
[#elseif ip?starts_with("COMP")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !comp_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="COMP"/>
      [#assign comp_hal=true]
      [/#if]      
    [#elseif !comp_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="COMP"/>
      [#assign comp_ll=true]        
    [/#if]      
[#elseif ip?starts_with("DAC")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !dac_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="DAC"/>
      [#assign dac_hal=true]
      [/#if]        
    [#elseif !dac_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="DAC"/>
      [#assign dac_ll=true]        
    [/#if]
[#elseif ip?starts_with("DFSDM")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="DFSDM"/>
[#elseif ip?starts_with("FMC")]
  [#list peripheralParams.get(ip).entrySet() as paramEntry]
  [#if paramEntry.key?starts_with("MemoryType")]
    [#if paramEntry.value?contains("NOR")]
      [#if !nor]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="NOR"/>
      [#assign nor=true]
      [/#if]
    [#elseif paramEntry.value?contains("SRAM")]
      [#if !sram]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SRAM"/>
      [#assign sram=true]
      [/#if]
    [/#if]
  [#elseif paramEntry.key == "NAND_Instance"]
    [#if !nand]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="NAND"/>
      [#assign nand=true]
    [/#if]
  [#elseif paramEntry.key == "PCCARD_Instance"]
    [#if !pccard]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="PC Card"/>
      [#assign pccard=true]
    [/#if]
  [#elseif paramEntry.key == "SDRAM_Instance"]
    [#if !sdram]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SDRAM"/>
      [#assign sdram=true]
    [/#if]
  [/#if]
  [/#list]
[#elseif ip?starts_with("I2C")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !i2c_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="I2C"/>
      [#assign i2c_hal=true]
      [/#if]        
    [#elseif !i2c_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="I2C"/>
      [#assign i2c_ll=true]        
    [/#if]
[#elseif ip?starts_with("LPTIM")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !lptim_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="LPTIM"/>
      [#assign lptim_hal=true]
      [/#if]        
    [#elseif !lptim_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="LPTIM"/>
      [#assign lptim_ll=true]        
    [/#if]
[#elseif ip?starts_with("LPUART")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !uart_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="UART"/>
      [#assign uart_hal=true]
      [/#if]        
    [#elseif !lpuart_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="LPUART"/>
      [#assign lpuart_ll=true]        
    [/#if]
[#elseif ip?starts_with("OPAMP")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !opamp_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="OPAMP"/>
      [#assign opamp_hal=true]
      [/#if]        
    [#elseif !opamp_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="OPAMP"/>
      [#assign opamp_ll=true]        
    [/#if]
[#elseif ip == "NVIC"]
[#elseif ip == "QUADSPI"]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="QSPI"/>
[#elseif ip?starts_with("SAI")]
  [#if !sai]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SAI"/>
      [#assign sai=true]
  [/#if]
[#elseif ip?starts_with("SDMMC")]
  [#if !sdmmc]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SD"/>
      [#assign sdmmc=true]
  [/#if]
[#elseif ip?starts_with("SDIO")]
  [#if !sdio]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SD"/>
      [#assign sdio=true]
  [/#if]
[#elseif ip?starts_with("SPI")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !spi_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SPI"/>
      [#assign spi_hal=true]
      [/#if]        
    [#elseif !spi_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="SPI"/>
      [#assign spi_ll=true]        
    [/#if]
[#elseif ip?starts_with("SWPMI")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="SWPMI"/>
    [#else]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="SWPMI"/>
    [/#if]
[#elseif ip?starts_with("SYS")]
[#elseif ip?starts_with("TIM")]
    [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      [#if !tim_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="TIM"/>
      [#assign tim_hal=true]
      [/#if]        
    [#elseif !tim_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="TIM"/>
      [#assign tim_ll=true]        
    [/#if]
[#elseif ip?starts_with("UART")]
  [#if peripheralParams.get(ip).get("driver") == ("HAL")]
    [#list peripheralParams.get(ip).entrySet() as paramEntry]
    [#if paramEntry.key?starts_with("Mode")]  
      [#if paramEntry.value?starts_with("IRDA")]
      [#if !irda_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="IRDA"/>
      [#assign irda_hal=true]
      [/#if]
      [#else]
      [#if !uart_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="UART"/>
      [#assign uart_hal=true]
      [/#if]
      [/#if]
    [/#if]
    [/#list]
  [#else]
      [#if !usart_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL" Csub="USART"/>
      [#assign usart_ll=true]
      [/#if]
  [/#if]  
[#elseif ip?starts_with("USART")]
  [#if peripheralParams.get(ip).get("driver") == ("HAL")]
    [#list peripheralParams.get(ip).entrySet() as paramEntry]  
    [#if paramEntry.key?starts_with("VirtualMode")]
      [#if paramEntry.value == "VM_IRDA"]
      [#if !irda_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="IRDA"/>
      [#assign irda_hal=true]
      [/#if]
      [#elseif paramEntry.value == "VM_SMARTCARD"]
      [#if !smartcard_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="Smartcard"/>
      [#assign smartcard_hal=true]
      [/#if]
      [#elseif paramEntry.value == "VM_SYNC"]
      [#if !usart_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="USART"/>
      [#assign usart_hal=true]
      [/#if]
      [#else]
      [#if !uart_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="UART"/>
      [#assign uart_hal=true]
      [/#if]
      [/#if]
    [/#if]
    [/#list]
  [#else]
      [#if !usart_ll]
      <require Cclass="Device" Cgroup="STM32Cube LL" Csub="USART"/>
      [#assign usart_ll=true]
      [/#if]
  [/#if]  
[#elseif ip?starts_with("USB")]
  [#list peripheralParams.get(ip).entrySet() as paramEntry]
  [#if paramEntry.key?starts_with("VirtualMode")]
    [#if paramEntry.value?starts_with("Device")]
      [#if !pcd]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="PCD"/>
      [#assign pcd=true]
      [/#if]
    [#elseif paramEntry.value?starts_with("Host")]
      [#if !hcd]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="HCD"/>
      [#assign hcd=true]
      [/#if]
    [#else]
      [#if !pcd]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="PCD"/>
      [#assign pcd=true]
      [/#if]
      [#if !hcd]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="HCD"/>
      [#assign hcd=true]
      [/#if]
    [/#if]
  [/#if]
  [/#list]

[#else]
      [#if peripheralParams.get(ip).get("driver") == ("HAL")]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="${(ip)}"/>
      [#else]
      <require Cclass="Device" Cgroup="STM32Cube LL"  Csub="${(ip)}"/>
      [/#if]
[/#if]
[/#if]
[/#list]
[#if timebase_tim == true]
  [#if !tim_hal]
      <require Cclass="Device" Cgroup="STM32Cube HAL" Csub="TIM"/>
      [#assign tim_hal=true]
  [/#if]
[/#if]
    </condition>
  </conditions>
  <components>
    <component generator="STM32CubeMX" Cvendor="Keil" Cclass="Device" Cgroup="STM32Cube Framework" Csub="STM32CubeMX" Cversion="1.1.0" condition="STCubeMX">
      <description>Configuration via STM32CubeMX</description>
      <RTE_Components_h>
        #define RTE_DEVICE_FRAMEWORK_CUBE_MX
      </RTE_Components_h>
      <files>
        <file category="header" name="MX_Device.h"/>
        <file category="header" name="STCubeGenerated/Inc/stm32g0xx_hal_conf.h"/>
[#if peripheralParams.get("RCC")??]
[#if peripheralParams.get("RCC").get("driver") == ("HAL")]
        <file category="source" name="STCubeGenerated/Src/stm32g0xx_hal_msp.c"/>
[/#if]
[/#if]
[#if timebase_tim == true]
        <file category="source" name="STCubeGenerated/Src/stm32g0xx_hal_timebase_tim.c"/>
[/#if]
      </files>
    </component>
  </components>
</package>
[/#list]
