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

## System Summary and Breakdown
The firmwares in this repository are meant to support the Ear EEG V2 Wireless Readout. Data recorded with the Ear EEG IC is queried by an onboard NRF52840 (peripheral Nordic) via SPI. The peripheral Nordic then sends a bluetooth notification with one or more SPI packets to the NRF52 Dongle (base station). The base station then places the received Ear EEG packets onto the USB bus such that the host computer can plot and store the data on disk.

![system](./images/system_diagram.png?raw=true "system block diagram")

The wireless link provides channel hopping, error correcting codes, can achieve a throughput of 1.2 Mbps which is sufficient for Ear EEG V2.

Pertinent BLE parameters (set by the peripheral device) are:
* 
*
* 

Data throughput information is below:
* 1 SPI packet (520 bits):
** 8 channels  (64 bit outputs for each)
** 8 bit packet ID

* Read out with > 520 KHz clock to achieve system sample rate of 1ksps. 


### Peripheral Firmware

The onboard NRF52840 connected to the Ear EEG IC is referred to as the peripheral. This terminology is taken from the bluetooth protocol where multiple peripherals can connect to a single central base station. This peripheral device has to query the Ear EEG IC (via SPI) for new packets. Every packet is placed into a buffer on the NRF52840's Cortex M4 microcontroller. Once 3 SPI packets have been collected, they're transferred to a 'send buffer' and put on a notification queue to be sent to the base station. This buffering scheme is referred to as double buffering. There are a number of 'clock domain crossings' between the Ear EEG IC and the base station and they've been mitigated with the following techniques:
* The SPI clock is >>>2x faster than the IC output rate
* There is a double buffer for data recieved over SPI MISO
* A large FIFO between SPI and BLE

To retain as stable and robust link, timing in the firmware is crucial. As a result most operations are governed by hardware timers and event interrupts. A hardwire timer triggers an DMA SPI transaction at a given rate. When the SPI transaction is complete, an interrupt is called and places the SPI packet into the send buffer. As the buffer keeps filling, the SPI complete interrupt will try to push the send buffer onto the BLE tx queue. The BLE block will empty the queue as fast as possible. This order of operations ensures minimal interactions with the data/readout chain.

The peripheral firmware handles some simple command processing (supported instructions listed above). 

### Base Station Firmware

The base station is made up of a single NRF52 dongle. It is

