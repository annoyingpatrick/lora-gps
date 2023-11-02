#ifndef LoRaWan_APP_H
#define LoRaWan_APP_H

#include "Arduino.h"
#include "driver/board.h"
#include "driver/debug.h"
#include "ESP32_LoRaWan_102.h"
#include "ESP32_Mcu.h"
#include "HardwareSerial.h"
#include "loramac/LoRaMac.h"
#include "loramac/utilities.h"
#include <stdio.h>

enum eDeviceState_LoraWan {
  DEVICE_STATE_OFF,
  DEVICE_STATE_INIT_DONE,
  DEVICE_STATE_JOINING,
  DEVICE_STATE_JOINED,
  DEVICE_STATE_SENDING,
};

// OTAA
extern uint8_t devEui[];
extern uint8_t joinEui[];
extern uint8_t appKey[];
// ABP
extern uint8_t nwkSKey[];
extern uint8_t appSKey[];
extern uint32_t devAddr;
// Frequency settings
extern uint16_t userChannelsMask[6];
// TX data
extern uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];
extern uint8_t appDataSize;

// Helper class for implementing a LoRaWAN device.
class LoRaWanClass {
public:
  void init(DeviceClass_t, LoRaMacRegion_t, bool, void (*JoinedDone)(void), void (*SentDone)(uint8_t, bool),
            void (*SendAcked)(void),
            void (*ReceivedData)(McpsIndication_t*));
  void join(bool, bool);
  void send(bool, uint8_t, uint8_t);
  void cycle(uint32_t timeout_ms);
  void sleep();
  void setDataRateForNoADR(int8_t dataRate);
  void generateDeveuiByChipID();

private:
  DeviceClass_t lorawan_class;
  LoRaMacRegion_t lorawan_region;
};

extern enum eDeviceState_LoraWan deviceState;

extern "C" bool SendFrame( void );
extern "C" void turnOnRGB(uint32_t color, uint32_t time);
extern "C" void turnOffRGB(void);
extern "C" bool checkUserAt(char * cmd, char * content);
extern "C" void downLinkAckHandle();
extern "C" void downLinkDataHandle(McpsIndication_t *mcpsIndication);
extern "C" void lwan_dev_params_update( void );
extern "C" void dev_time_updated( void );


extern LoRaWanClass LoRaWAN;

#endif
