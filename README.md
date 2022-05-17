# Ear EEG Nordic Dev Directory

This git repo has all the programming and config files neccessary for altering, building, programming firmware for the Ear EEG V2 system developed by UC Berkeley's Muller Group. There are two main firmwares in this repository:
* peripheral
* base station

## Current System Status
* Peripheral firmware can run at full speed on dev kit's onboard NRF52840 (with BLE & SPI)
* Base station firmware can run on a devkit or on an nrf52 dongle

### Implemented & Tested functionality
* Bluetooth (with data throuput of up to 1.2 Mbps)
** BLE optimizations include packing 3 SPI packets into a single BLE packet
** Stack optimizations that queue a number of SPI packets
** Double buffer scheme so that BLE and SPI can operate independantly and not block eachother
* SPI transactions with easydma (NRFX_SPIM)
* Timers to set interval between spi transactions (aka rate of spi transactions)
* Incoming Instruction decoding

### Supported Instructions
| Instruction       | Info                                                                      |
|-------------------|---------------------------------------------------------------------------|
| led #             | toggle LED# (1-4)                                                         |
| read #addr        | Read from IC #addr(decimal)                                               |
| write #addr #data | Write #data (hex) to #addr(decimal)                                       |
| start             | Start streaming data                                                      |
| stop              | Stop streaming data                                                       |
| update            | Put BLE into full speed (2M PHY)                                          |
| startup           | Begin startup routine that writes instructions stored in memory to IC     |
| reset             | reset the bluetooth link (in case pyserial has crashed and locked us out) |

## Peripheral Firmware

## Base Station Firmware

