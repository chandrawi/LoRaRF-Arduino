<!-- PROJECT SHIELDS -->
[![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/chandrawi/LoRaRF-Arduino)](https://github.com/chandrawi/LoRaRF-Arduino/releases)
[![GitHub license](https://img.shields.io/github/license/chandrawi/LoRaRF-Arduino)](https://github.com/chandrawi/LoRaRF-Arduino/blob/main/LICENSE)

# LoRa-RF Arduino Library

Arduino LoRa-RF library used for transmitting and receiving data using LoRa module with Semtech SX126x series, SX127x series, or LLCC68. The library works by interfacing SPI port and some I/O pins. Support for configuring frequency, modulation parameter, transmit power, receive gain and other RF parameters on both LoRa and FSK modulation also support for handling transmit and receive using interrupt signal.

This readme is written for quick start guide. Visit this [link](https://github.com/chandrawi/LoRaRF-Arduino/wiki) for complete documentation.

## Hardware Compatibility

Theoritically all LoRa modules using SX126x series (SX1261, SX1262, SX1268), SX127x series (SX1272, SX1276, SX1277, SX1278, SX1279), or LLCC68 will compatible using this library. Arduino LoRa development board which interfaced using SPI to a LoRa module also theoritically compatible.

Some LoRa module which already tested and confirmed compatible are:
* Ebyte: E19-433M20SC, E19-915M20S, E19-433M30S, E19-915M30S, E22-400M22S, E22-900M22S, E22-400M30S, E22-900M30S
* HopeRF: RFM95W, RFM96W, RFM98W

Please note that LoRa modules use 3.3V for power supply and I/O ports. Care must me taken for connecting 5V arduino to LoRa module.

## Installation

### Using the Arduino IDE Library Manager

1. Choose `Sketch` -> `Include Library` -> `Manage Libraries...`
2. Type `LoRaRF` into the search box.
3. Search a row with title `LoRaRF` by Chandra Wijaya.
4. Click the `Install` button to install the library.

### Using Git

Open terminal, Git bash, or command prompt in Arduino library folder then run following command. Default library folder for windows is `C:\Users\{username}\Documents\Arduino` and for linux is `~/Documents/Arduino/libraries/`. 
```sh
git clone https://github.com/chandrawi/LoRaRF-Arduino.git
```

## Initialization

To work with the library, first you must include `SX126x` header or `SX127x` header depending LoRa module you use. For LLCC68 include SX126x header. Then initialize class in the header by creating an object.

```c++
// for SX126x series or LLCC68
#include <SX126x.h>
SX126x LoRa;

// for SX127x series
#include <SX127x.h>
SX127x LoRa;
```

Before calling any configuration methods, doing transmit or receive operation you must call `begin()` method. You can call `begin()` method inside Arduino `setup()` function.

```c++
void setup() {
  LoRa.begin();
  // configuration code goes here
}

void loop() {
  // transmit and receive operation goes here
}
```

## Hardware Configuration

### Wiring Connections

Power pins, SPI pins, and `RESET` pin must be connected between arduino and LoRa module. For SX126x series and LLCC68, a `BUSY` pin also must be connected. If you want to use interrupt operation, you can connect `DIO0` for SX127x series and one of `DIO1`, `DIO2`, or `DIO3` pin for SX126x series. You also should connect `TXEN` and `RXEN` pins if your LoRa module have those pins.

The default Arduino pins used for connecting to SX126x and SX127x are as follows.

| Semtech SX126x | Semtech SX127x | Arduino |
| :------------: | :-------------:| :------:|
| VCC | VCC | 3.3V |
| GND | GND | GND |
| SCK | SCK | SCK |
| MISO | MISO | MISO |
| MOSI | MOSI | MOSI |
| NSS | NSS | 10 |
| RESET | RESET | 9 |
| BUSY | | 4|
| DIO1 | DIO0 | -1 (unused) |
| TXEN | TXEN | -1 (unused) |
| RXEN | RXEN | -1 (unused) |

### SPI Port Configuration

To change Arduino default SPI port or SPI frequency call `setSPI()` method before `begin()` method.
```c++
LoRa.setSPI(SPI2, 16000000);
LoRa.begin();
```

### I/O Pins Configuration

To configure I/O pins (NSS, RESET, BUSY, IRQ, TXEN, RXEN pin) call `setPins()` before `begin()` method.
```c++
// set NSS->10, RESET->9, BUSY->4, DIO1->2, TXEN->8, RXEN->7 for SX126x series
LoRa.setPins(10, 9, 2, 4, 8, 7);
// set NSS->10, RESET->9, DIO0->2, TXEN->8, RXEN->7 for SX127x series
LoRa.setPins(10, 9, 2, 8, 7);
LoRa.begin();
```

## Modem Configuration

Before transmit or receive operation you can configure transmit power and receive gain or matching frequency, modulation parameter, packet parameter, and synchronize word with other LoRa device you want communicate.

### Transmit Power

```c++
// set transmit power to +22 dBm for SX1262
LoRa.setTxPower(22, SX126X_TX_POWER_SX1262);
// set transmit power to +20 dBm for SX127x series using boost pin
LoRa.setTxPower(20, SX127X_TX_POWER_PA_BOOST);
```

### Receive Gain

```c++
// set receive gain to power saving
LoRa.setRxGain(LORA_RX_GAIN_POWER_SAVING);
// set receive gain to boosted and AGC on for SX127x series
LoRa.setRxGain(LORA_RX_GAIN_BOOSTED, true);
```

### Frequency

```c++
// Set frequency to 915 Mhz
LoRa.setFrequency(915000000);
```

### Modulation Parameter

```c++
// set spreading factor 8, bandwidth 125 kHz, coding rate 4/5, and low data rate optimization off
LoRa.setLoRaModulation(8, 125000, 5, false);
```

### Packet Parameter

```c++
// set explicit header mode, preamble length 12, payload length 15, CRC on and no invert IQ operation
LoRa.setLoRaPacket(LORA_HEADER_EXPLICIT, 12, 15, true, false);
```

### Synchronize Word

```c++
// Set syncronize word for public network (0x3444)
LoRa.setSyncWord(0x3444);
```

## Transmit Operation

Transmit operation begin with calling `beginPacket()` method following by `write()` method to write package to be tansmitted and ended with calling `endPacket()` method. For example, to transmit "HeLoRa World!" message and an increment counter you can use following code.

```c++
// message and counter to transmit
char message[] = "HeLoRa World!";
uint8_t counter = 0;

LoRa.beginPacket();
LoRa.write(message, sizeof(message)); // write multiple bytes
LoRa.write(counter++);                // write single byte
LoRa.endPacket();
LoRa.wait();
```

For more detail about transmit operation, please visit this [link](https://github.com/chandrawi/LoRaRF-Arduino/wiki/Transmit-Operation).

## Receive Operation

Receive operation begin with calling `request()` method following by `read()` method to read received package. `available()` method can be used to get length of remaining package. For example, to receive message and a counter in last byte you can use following code.

```c++
LoRa.request();
LoRa.wait();

// get message and counter in last byte
const uint8_t length = LoRa.available() - 1;
char message[length];
uint8_t i=0;
while (LoRa.available() > 1){
  message[i++] = LoRa.read();         // read multiple bytes
}
uint8_t counter = LoRa.read();        // read single byte
```

For more detail about receive operation, please visit this [link](https://github.com/chandrawi/LoRaRF-Arduino/wiki/Receive-Operation).

## Examples

See examples for [SX126x](https://github.com/chandrawi/LoRaRF-Arduino/tree/main/examples/SX126x), [SX127x](https://github.com/chandrawi/LoRaRF-Arduino/tree/main/examples/SX127x), and [simple network implementation](https://github.com/chandrawi/LoRaRF-Arduino/tree/main/examples/Network).

## Contributor

[Chandra Wijaya Sentosa](https://github.com/chandrawi) <<chandra.w.sentosa@gmail.com>>
