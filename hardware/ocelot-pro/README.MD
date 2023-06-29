# Ocelot Pro Hardware
This directory contains information related to Ocelot Pro hardare. Here, you'll find information about the hardware itself.
## Header Pin Description
### **H1: CAN**
This header contains the three CAN interfaces on the Ocelot Pro, and is organized like so:


| **CAN1** | **CAN2** | **CAN3** |
|----------|----------|----------|
| CAN1L    | CAN2L    | CAN3L    |
| CAN1H    | CAN2H    | CAN3H    |

### **H2: SWD**
This header contains the pins to connect a SWD Link cable. You can use this along with GDB to debug the running application live.

### **H3: POWER**
This header is connected to all power rails in the Ocelot Pro. Each pin is duplicated in case you want to connect multiple peripherals to a rail.

| **PIN** | **Description**              |
|---------|------------------------------|
| IGN_OUT | 12V 3A, switched by pin PA1  |
| 12V     | 12V input VIN                |
| 5V      | 5V 60mA                      |
| 3.3V    | 3.3V 60mA                    |
| GND     | Ground                       |

### **H4 and H5: GPIO**
All GPIO are 3.3V and connected directly to the STM32F413. Use a level shifter if using 5V logic. Best practice would be to limit the current source/sink to 25mA on a GPIO. More info on this in the datasheet.
Each GPIO can be used as digital I/O or alternate functions depending on the peripheral and the pin. See below:

#### **H4**
| **PIN**   | **Alt Functions**    |
|-----------|----------------|
| **PB10:** | TIM2_CH3, I2C2_SCL, SPI2_SCK/I2S2_CK, I2S3_MCK, USART3_TX, I2CFMP1_SCL, DFSDM2_CKOUT, SDIO_D7, EVENTOUT|
| **PB1:**  | TIM1_CH3N, TIM3_CH4, TIM8_CH3N, SPI5_NSS/I2S5_WS, DFSDM1_DATIN0, QUADSPI_CLK, EVENTOUT |
| **PC5:**  | DFSDM2_DATIN2, I2CFMP1_SMBA, USART3_RX, QUADSPI_BK2_IO3, FSMC_NOE, EVENTOUT |
| **PA7:**  | TIM1_CH1N, TIM3_CH2, TIM8_CH1N, SPI1_MOSI/I2S1_SD, DFSDM2_DATIN1, TIM14_CH1, QUADSPI_BK2_IO1, EVENTOUT |
| **PA5:**  | TIM2_CH1/TIM2_ETR, TIM8_CH1N, SPI1_SCK/I2S1_CK, DFSDM1_CKIN1, FSMC_D7/FSMC_DA7, EVENTOUT |
| **PA3:**  | TIM2_CH4, TIM5_CH4, TIM9_CH2, I2S2_MCK, USART2_RX, SAI1_SD_B, FSMC_D5/FSMC_DA5, EVENTOUT |
| **PC3:**  | LPTIM1_ETR, DFSDM2_CKIN7, SPI2_MOSI/I2S2_SD, SAI1_FS_B, FSMC_A0, EVENTOUT |
| **PC0:**  | LPTIM1_IN1, DFSDM2_CKIN4, SAI1_MCLK_B, EVENTOUT |
| **PC14:** | EVENTOUT |
| **3V3:**  | 3.3V rail |
| **PB2:**  | LPTIM1_OUT, DFSDM1_CKIN0, QUADSPI_CLK, EVENTOUT |
| **PB0:**  | TIM1_CH2N, TIM3_CH3, TIM8_CH2N, SPI5_SCK/I2S5_CK, EVENTOUT |
| **PC4:**  | DFSDM2_CKIN2, I2S1_MCK, QUADSPI_BK2_IO2, FSMC_NE4, EVENTOUT |
| **PA6:**  | TIM1_BKIN, TIM3_CH1, TIM8_BKIN, SPI1_MISO, I2S2_MCK, DFSDM2_CKIN1, TIM13_CH1, QUADSPI_BK2_IO0, SDIO_CMD, EVENTOUT | 
| **PA4:**  | SPI1_NSS/I2S1_WS, SPI3_NSS/I2S3_WS, USART2_CK, DFSDM1_DATIN1, FSMC_D6/FSMC_DA6, EVENTOUT |
| **PA2:**  | TIM2_CH3, TIM5_CH3, TIM9_CH1, I2S2_CKIN, USART2_TX, FSMC_D4/FSMC_DA4, EVENTOUT |
| **PC2:**  | LPTIM1_IN2, DFSDM2_DATIN7, SPI2_MISO, I2S2ext_SD, SAI1_SCK_B, DFSDM1_CKOUT, FSMC_NWE, EVENTOUT |
| **PC15:** | EVENTOUT |
| **NC:**   | N/C |
| **GND:**  | Ground |

#### **H5**    
| **PIN**   | **Alt Functions**    |
|-----------|----------------|
| **PB12:** | TIM1_BKIN, I2C2_SMBA, SPI2_NSS/I2S2_WS, SPI4_NSS/I2S4_WS, SPI3_SCK/I2S3_CK, USART3_CK, CAN2_RX, DFSDM1_DATIN1, UART5_RX, FSMC_D13/FSMC_DA13, EVENTOUT | 
| **PB14:** | TIM1_CH2N, TIM8_CH2N, I2CFMP1_SDA, SPI2_MISO, I2S2ext_SD, USART3_RTS, DFSDM1_DATIN2, TIM12_CH1, FSMC_D0/FSMC_DA0, SDIO_D6, EVENTOUT | 
| **PC6:**  | TIM3_CH1, TIM8_CH1, I2CFMP1_SCL, I2S2_MCK, DFSDM1_CKIN3, DFSDM2_DATIN6, USART6_TX, FSMC_D1/FSMC_DA1, SDIO_D6, EVENTOUT |
| **PC8:**  | TIM3_CH3, TIM8_CH3, DFSDM2_CKIN3, USART6_CK, QUADSPI_BK1_IO2, SDIO_D0, EVENTOUT |
| **PA9:**  | TIM1_CH2, DFSDM2_CKIN3, I2C3_SMBA, SPI2_SCK/I2S2_CK, USART1_TX, USB_FS_VBUS, SDIO_D2, EVENTOUT |
| **PC10:** | DFSDM2_CKIN5, SPI3_SCK/I2S3_CK, USART3_TX, QUADSPI_BK1_IO1, SDIO_D2, EVENTOUT |
| **PC12:** | SPI3_MOSI/I2S3_SD, USART3_CK, UART5_TX, FSMC_D3/FSMC_DA3, SDIO_CK, EVENTOUT |
| **PB3:**  | TDO-SWO, TIM2_CH2, I2CFMP1_SDA, SPI1_SCK/I2S1_CK, SPI3_SCK/I2S3_CK, USART1_RX, UART7_RX, I2C2_SDA, SAI1_SD_A, CAN3_RX, EVENTOUT |
| **PB7:**  | LPTIM1_IN2, TIM4_CH2, I2C1_SDA, DFSDM2_DATIN7, USART1_RX, FSMC_NL, EVENTOUT |
| **3V3**   | 3.3V rail |
|**PB13:**  | TIM1_CH1N, I2CFMP1_SMBA, SPI2_SCK/I2S2_CK, SPI4_SCK/I2S4_CK, USART3_CTS, CAN2_TX, DFSDM1_CKIN1, UART5_TX, EVENTOUT |
|**PB15:**  | RTC_REFIN, TIM1_CH3N, TIM8_CH3N, I2CFMP1_SCL, SPI2_MOSI/I2S2_SD, DFSDM1_CKIN2, TIM12_CH2, SDIO_CK, EVENTOUT |
|**PC7:**   | TIM3_CH2, TIM8_CH2, I2CFMP1_SDA, SPI2_SCK/I2S2_CK, I2S3_MCK, DFSDM2_CKIN6, USART6_RX, DFSDM1_DATIN3, SDIO_D7, EVENTOUT |
|**PC9:**   | MCO_2, TIM3_CH4, TIM8_CH4, I2C3_SDA, I2S2_CKIN, DFSDM2_DATIN3, QUADSPI_BK1_IO0, SDIO_D1, EVENTOUT |
|**PA10:**  | TIM1_CH3, DFSDM2_DATIN3, SPI2_MOSI/I2S2_SD, SPI5_MOSI/I2S5_SD, USART1_RX, USB_FS_ID, EVENTOUT |
|**PC11:**  | DFSDM2_DATIN5, I2S3ext_SD, SPI3_MISO, USART3_RX, UART4_RX, QUADSPI_BK2_NCS, FSMC_D2/FSMC_DA2, SDIO_D3, EVENTOUT |
|**PD2:**   | TIM3_ETR, DFSDM2_CKOUT, UART5_RX, FSMC_NWE, SDIO_CMD, EVENTOUT |
|**PB4:**   | JTRST, TIM3_CH1, SPI1_MISO, SPI3_MISO, I2S3ext_SD, UART7_TX, I2C3_SDA, SAI1_SCK_A, CAN3_TX, SDIO_D0, EVENTOUT |
|**N/C**    | N/C |
|**GND**    | Ground |

More info on GPIOs can be found in the [datasheet](https://www.st.com/resource/en/datasheet/stm32f423ch.pdf)

## Buttons, Connectors, and Jumpers
Ocelot Pro contains the following buttons:
BOOT (SW1)
RESET (SW2)

The RESET button resets the chip, and if the BOOT button is held while resetting, the device goes into DFU mode.

The USB-C connector is specifically used for CAN traffic analysis / serial debugging / firmware updates. Do not connect a Comma OBD-C cable into this port or you will risk burning up everything on the 5V rail.

## Board Files and Schematic
This directory contains the schematics and Gerber files for Ocelot Pro. Use these to customize the board or build one. Pre-made boards can be purchased at [RetroPilot Shop](https://shop.retropilot.org/product/ocelot-pro-devkit/)