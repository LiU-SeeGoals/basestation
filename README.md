# Basestation
This is the firmware for the basestation (NUCLEO-H563ZI) which parses messages from the controller and ssl-vision and then sends it to the robots.

## Contributing
Make sure to follow the [firmware standard](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#seegoal---firmware-standard) and the [feature branch](https://github.com/LiU-SeeGoals/wiki/wiki/1.-Processes-&-Standards#feature-branch-integration) concept.

## Git Submodules 
This project uses several Git Submodules. To get all Submodules at the correct version, use git update --init --recursive.
This has to be done the first time you clone, and everytime you change branch to a branch with different submodule versions.

## Building and flashing
This project has to be compiled and flashed through the STM32CubeIDE.

There are two compiling options used, `Debug` is used to compile the project, `Compiledb` uses [compiledb](https://github.com/nickdiego/compiledb) to produce a `compile_commands.json` file which can be used with your LSP powered IDE of choice.

By running the project through STM32CubeIDE and having the NUCLEO card connected through USB (st-link marked port) the binary is flashed to the MCU.

The project can also be compiled using cmake. 
mkdir build
cmake -B build . -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE
cd build && make

Flashing can be done outside of STM32CubeIDE using STM32_Programmer_CLI. It is bundled with STM32CubeIDE on windows, and is available at [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)
STM32_Programmer_CLI -c port=SWD sn=004C00283232511639353236 ap=1 -w build\basestation.bin 0x08000000 -rst


# Documentation

## Sheets
[NUCLEO-H563ZI](https://www.st.com/resource/en/user_manual/um3115-stm32h5-nucleo144-board-mb1404-stmicroelectronics.pdf)

## Pins
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       | Cable colour |
|---------------|---------|----------------|---------|-------------|--------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      | Yellow       |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     | Orange       |
| 6             | VDD     | VDD            | VDD     | -           | Red          |
| 8             | GND     | GND            | GND     | -           | Black        |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     | Green        |
| 12            | PG9     | SPI1_MISO      | M1      | NRF_MISO    | Purple       |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    | Blue         |
| 20            | PF3     | GPIO_EXTI3     | IRQ     | NRF_IRQ     | Gray         |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER    | -            |
| -             | PD8     | USART3_RX      | -       | USART3_RX   | -            |
| -             | PD9     | USART3_TX      | -       | USART3_TX   | -            |
| -             | PB0     | GPIO_Output    | -       | LED_GREEN   | -            |
| -             | PF4     | GPIO_Output    | -       | LED_YELLOW  | -            |
| -             | PG4     | GPIO_Output    | -       | LED_RED     | -            |
| -             | PC1     | ETH_MDC        | -       | ETH_MDC     | -            |
| -             | PA1     | ETH_REF_CLK    | -       | ETH_REF_CLK | -            |
| -             | PA2     | ETH_MDIO       | -       | ETH_MDIO    | -            |
| -             | PA7     | ETH_CRS_DV     | -       | ETH_CRS_DV  | -            |
| -             | PC4     | ETH_RXD0       | -       | ETH_RXD0    | -            |
| -             | PC5     | ETH_RXD1       | -       | ETH_RXD1    | -            |
| -             | PB15    | ETH_TXD1       | -       | ETH_TXD1    | -            |
| -             | PG11    | ETH_TX_EN      | -       | ETH_TX_EN   | -            |
| -             | PG13    | ETH_TXD0       | -       | ETH_TXD0    | -            |

## MX configuration
If for some reason a new `basestation.ioc` has to be created from scratch, these are the changes needed in of STM32CubeMX.

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
