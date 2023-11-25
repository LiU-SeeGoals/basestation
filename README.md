# Basestation

## Sheets
[NUCLEO-H563ZI](https://www.st.com/resource/en/user_manual/um3115-stm32h5-nucleo144-board-mb1404-stmicroelectronics.pdf)

## Pins
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       |
|---------------|---------|----------------|---------|-------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     |
| 4             | PB9     | GPIO_EXTI9     | IRQ     | NRF_IRQ     |
| 6             | VDD     | VDD            | VDD     | -           |
| 8             | GND     | GND            | GND     | -           |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     |
| 12            | PA6     | SPI1_MISO      | M1      | NRF_MISO    |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER    |
| -             | PD8     | USART3_RX      | -       | USART3_RX   |
| -             | PD9     | USART3_TX      | -       | USART3_TX   |
| -             | PB0     | GPIO_Output    | -       | LED_GREEN   |
| -             | PF4     | GPIO_Output    | -       | LED_YELLOW  |
| -             | PG4     | GPIO_Output    | -       | LED_RED     |
| -             | PC1     | ETH_MDC        | -       | ETH_MDC     |
| -             | PA1     | ETH_REF_CLK    | -       | ETH_REF_CLK |
| -             | PA2     | ETH_MDIO       | -       | ETH_MDIO    |
| -             | PA7     | ETH_CRS_DV     | -       | ETH_CRS_DV  |
| -             | PC4     | ETH_RXD0       | -       | ETH_RXD0    |
| -             | PC5     | ETH_RXD1       | -       | ETH_RXD1    |
| -             | PB15    | ETH_TXD1       | -       | ETH_TXD1    |
| -             | PG11    | ETH_TX_EN      | -       | ETH_TX_EN   |
| -             | PG13    | ETH_TXD0       | -       | ETH_TXD0    |

## MX configuration

### General stuff
Make sure global interrupts are enabled beneath NVIC (preemption priority set to 7).  
Set Timebase Source in SYS to TIM6.  

### Connectivity

#### ETH
Mode: RMII
NVIC Settings:
~~~
Ethernet Global interrupt: YES
~~~

#### USART3
Mode: Asynchronous

#### SPI1
Mode: Full-Duplex Master
Parameter Settings:
~~~
Data Size: 8 Bits
Prescaler: 32
~~~

### Middleware
Ethernet is used through Azure RTOS middleware, specifically THREADX and NETXDUO.

#### THREADX
Should only have to set mode to core.

#### NETXDUO
Set NETXDUO to NX Core mode, and set the network interface to "LAN8742 Phy Interface".  
NetXDuo:
~~~
NX_ENABLE_INTERFACE_CAPABILITY: YES
NetXDuo Generate Init Code: true
NetXDuo IP Instance Thread Size: 2 * 1024
NetXDuo Application Thread Stack Size: 2* 1024
NetXDuo Protocols: Set all to YES
NetXDuo memory pool size: 30 * 1024
~~~
