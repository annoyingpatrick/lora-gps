LoRaGPS App
===========

This app for the [Heltec WiFi LoRa
V3](https://heltec.org/project/wifi-lora-32-v3/) board collects GPS data and
transmits it over LoRaWAN.

Getting Started
---------------

### Software

1. Install VSCode.
2. Install Platform.io for VSCode.
3. Open this folder in VSCode. It will detect the project as the correct
   platform.io project.

### Hardware

Connect the GPS module to the Heltec V3 with the following pin mappings:

| Heltec V3 | GPS |
|-----------|-----|
| 3V3       | VCC |
| GND       | GND |
| 45        | TXD |

### LoRa / The Things Network

1. Create an application in TTN.
2. Flash the `lora-deveui-display` app to the board. This will show the device's
   EUI.
3. Register the device and its EUI with your application. You will need the
   `JoinEUI` and `AppKey`.


Configuring LoRaWAN
-------------------

Create a file called `lora_parameters.cpp` in the `src/` directory. Copy the following into the file, filling in the parameters for your LoRa application:

```c
#include "stdint.h"

///////////////////////////////////////////////////////////////////////////////
// LoRaWAN Network Settings
///////////////////////////////////////////////////////////////////////////////

// JoinEUI specifies the application used on The Things Network.
// You should set this to whatever your TTN generates.
uint8_t joinEui[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// AppKey is the encryption key used for communication. This must be
// shared with TTN.
uint8_t appKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
```


Build the App and Flash the Board
---------------------------------

Load the LoRaGPS app to the Heltec V3 by pressing the right arrow button on the
bottom blue bar in VSCode.

