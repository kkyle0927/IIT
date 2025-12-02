/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2021 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x2000;      /* required amount of heap  */
_Min_Stack_Size = 0x2000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08040400, LENGTH = 1791K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);
  
  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH
 

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1
  
  .AXIBUS_DATA (NOLOAD) :
  {
  	/* WalkONSuit5 Address */
	  /*. = ABSOLUTE(0x24069ae0);  */
  	/* MiniCM Address */
  	. = ABSOLUTE(0x24060000); /*MPU_REGION_NUMBER2 - MPU_InitStruct.BaseAddress 수정 */
  	
  	. = ALIGN(4);
    *(.FATFS_RAMD1_data)
    . = ALIGN(4);
    *(.USBHOST_data)
    . = ALIGN(4);
  } >RAM_D1
  

  /* CM DMA variables -> attribute RAM D2 domain 0x30000000 ~ */
  /* ! This is only for CM */
  /********************************************************************/
  .CM_DMA (NOLOAD) :
  {
  	. = ABSOLUTE(0x30000000);
    /* adc1(CM Board Total Current Sensor, Hip Left Depth ADC, Hip Right Length ADC) & adc2 (Hip Right Depth ADC) & adc3 (Hip Left Length ADC)  */
    *(.adc1DmaBuff)
    *(.adc2DmaBuff)
    *(.adc3DmaBuff)

    /* Battery Status LED & 6Axis IMU & 3Axis IMU */
    *(.i2c1RxBuff)
    *(.i2c2RxBuff)
    *(.i2c3RxBuff)

	/* SPI1 (LED Driver) */
    *(.spi1TxBuff)
    *(.spi1RxBuff)
    
    /* SPI2 (RTC Driver) */
    *(.spi2TxBuff)
    *(.spi2RxBuff)
    
    /* SPI3 (AM Communication) */
    *(.spi3TxBuff)
    *(.spi3TxTestBuff)
    *(.spi3RxBuff)
    *(.esp32spiRxRingBuff)

    /* SPI4 (HIP Left Absolute Encoder) */
    *(.spi4RxBuff)
    
	/* SPI5 (HIP Right Absolute Encoder) */
    *(.spi5RxBuff)

    /* TIM3 NZR LED CH1, 2 */
    *(.nzrLedTxBuff1)
    *(.nzrLedTxBuff2)
    
    /* Uart */
	*(.uart1dmaRxBuff)
	*(.uart1dmaTxBuff)
    
  	/* SAI Buffer */
  	*(.Sai1_RxBuff)
  	
  	/* Ethernet Buffer */
  	*(.RxDecripSection)
  	*(.TxDecripSection)
  	*(.RxArraySection)
  	
  } >RAM_D2

  .CM_GAIT_ANALYSIS_RAMD2 (NOLOAD) :
  {
    . = ABSOLUTE(0x3000A000);
    *(.segmentBuffer)
    *(.velX4AnalysisFiltered)
    *(.ThetaRH4AnalysisFiltered)
    *(.ThetaLH4AnalysisFiltered)
    *(.TorqueRH4AnalysisFiltered)
    *(.TorqueLH4AnalysisFiltered)

    *(.IndexLeftHeelStrike)
    *(.IndexRightHeelStrike)
    
    *(.IndexLeftHeelStrikeCalibrated)
    *(.IndexLeftToeOffCalibrated)
    *(.IndexRightHeelStrikeCalibrated)
    *(.IndexRightToeOffCalibrated)

    *(.ResampledLeftHipAngles)
    *(.ResampledRightHipAngles)
  } >RAM_D2 

  /* CM BDMA variables -> attribute RAM D3 domain 0x38000000 ~ */
  .CM_BDMA (NOLOAD) :
  {
  	. = ABSOLUTE(0x38000000);
    /* i2c4(LTC2944)  */
    *(.i2c4RxBuff)
    
    /* Uart */
	/**(.uart7dmaRxBuff)*/
    
  } >RAM_D3
  
  .CM_GAIT_ANALYSIS_RAMD3 (NOLOAD) :
  {
    . = ABSOLUTE(0x38000020);
    *(.postProcessingCntFiltered)
    *(.ResampledLeftHipTorques)
    *(.ResampledRightHipTorques)

    *(.meanResampledLeftHipAngles)
    *(.stdResampledLeftHipAngles)

    *(.meanResampledRightHipAngles)
    *(.stdResampledRightHipAngles)

    *(.meanResampledLeftHipTorques)
    *(.stdResampledLeftHipTorques)
    
    *(.meanResampledRightHipTorques)
    *(.stdResampledRightHipTorques)
  } >RAM_D3

  .itcm_data (NOLOAD): 
  { 
    *(.ZPF_TempArray)
  } >ITCMRAM

  .dtcm_data (NOLOAD) : 
  { 
    *(.dtcm_data)
  } >DTCMRAM
    
  /********************************************************************/

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


