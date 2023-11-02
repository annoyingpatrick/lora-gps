#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "TinyGPS++.h"
#include "Wire.h"
#include <Arduino.h>

// Parser for GPS strings.
TinyGPSPlus gps;
// UART connection to the GPS module.
HardwareSerial gpsSerial(1);

// Pin for receiving data from GPS.
const int GPS_TX_PIN = 45;

// How long to sleep after a packet has been transmitted before
// sending the next packet.
const int SLEEP_TIME_BETWEEN_EVENTS_MS = 10000;

// Use Over the Air Activation for joining the LoRaWAN network.
const bool LORA_OTAA = true;

// For OTAA.
uint8_t devEui[8] = {0};

// These are only used for ABP.
uint8_t nwkSKey[16] = {0};
uint8_t appSKey[16] = {0};
uint32_t devAddr = 0;

// LoRaWAN channel mask
uint16_t userChannelsMask[6] = {0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

// ADR enable
bool loraWanAdr = true;

// Application port
uint8_t appPort = 1;

/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 8;

// Keep a running counter of how many packets have been sent since reset.
RTC_DATA_ATTR uint16_t count = 0;
// Keep a count of how many packets have been acknowledged since reset.
RTC_DATA_ATTR uint16_t acked_count = 0;

// Whether the GPS has a lock or not.
RTC_DATA_ATTR bool gps_locked = false;

// Number of unacked packets in a row.
RTC_DATA_ATTR uint16_t failed_tx_consecutive = 0;

// Driver for the OLED display.
//
// THIS MUST BE DEFINED LAST. (I don't know why.)
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

///////////////////////////////////////////////////////////////////////////////
// Display Functions
///////////////////////////////////////////////////////////////////////////////

// Display on the OLED the current lat/lon coordinates.
void display_gps(long lat, long lng)
{
  char lat_str[25];
  char lng_str[25];
  char cnt_str[25];
  char ack_str[25];

  char lat_dir = 'N';
  char lng_dir = 'E';
  if (lat < 0)
  {
    lat_dir = 'S';
    lat *= -1;
  }
  if (lng < 0)
  {
    lng_dir = 'W';
    lng *= -1;
  }

  long lat_deg = lat / 1000000;
  long lat_deg_rem = lat - (lat_deg * 1000000);
  long lat_min = (lat_deg_rem * 60) / 1000000;
  long lat_min_rem = (lat_deg_rem * 60) - (lat_min * 1000000);
  snprintf(lat_str, 25, "%i°%i.%i'%c", lat_deg, lat_min, lat_min_rem, lat_dir);

  long lng_deg = lng / 1000000;
  long lng_deg_rem = lng - (lng_deg * 1000000);
  long lng_min = (lng_deg_rem * 60) / 1000000;
  long lng_min_rem = (lng_deg_rem * 60) - (lng_min * 1000000);
  snprintf(lng_str, 25, "%i°%i.%i'%c", lng_deg, lng_min, lng_min_rem, lng_dir);

  snprintf(cnt_str, 25, "Count: %i", count);
  snprintf(ack_str, 25, "Acked: %i", acked_count);

  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(4, 5, "GPS Location:");
  display.setFont(ArialMT_Plain_10);
  display.drawString(4, 20, lat_str);
  display.drawString(4, 30, lng_str);
  display.drawString(4, 40, cnt_str);
  display.drawString(4, 50, ack_str);
  display.display();
  delay(2000);
}

// Display on the OLED that a packet was sent and if it was acked.
void display_tx_done(uint8_t tries, bool acked)
{
  char cnt_str[25];
  char ack_str[25];

  snprintf(cnt_str, 25, "TX Count: %i", tries);
  snprintf(ack_str, 25, "Ack: %s", acked ? "Yes" : "No");

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(4, 5, "Packet Sent");
  display.setFont(ArialMT_Plain_10);
  display.drawString(4, 20, cnt_str);
  display.drawString(4, 30, ack_str);
  display.display();
  delay(4000);
}

// Display LoRaWAN Join and GPS status.
void display_status(bool lora_joined, bool gps_found)
{
  char msg[32];
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(4, 10, "GPSLoRa Status");
  display.setFont(ArialMT_Plain_10);
  snprintf(msg, 32, "LoRa: %s", (lora_joined ? "Joined" : "Joining..."));
  display.drawString(4, 25, msg);
  snprintf(msg, 32, "GPS: %s", (gps_found ? "Found" : "Searching..."));
  display.drawString(4, 35, msg);
  snprintf(msg, 32, "Failed Cnt: %i", failed_tx_consecutive);
  display.drawString(4, 45, msg);
  display.display();
  delay(2000);
}

///////////////////////////////////////////////////////////////////////////////
// TX Packet and GPS Functions
///////////////////////////////////////////////////////////////////////////////

// Read the GPS and create an outgoing packet structure.
void prepareTxFrame(void)
{
  // We are going to send 20 bytes.
  appDataSize = 20;

  uint32_t age = 9999999;
  int gps_count = 0;

  // Wait until we have fresh data.
  while (age > 1000)
  {
    // Read in from the GPS and use the library to parse the message.
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }

    age = gps.location.age();
    if (age > 1000)
    {
      gps_count += 1;
      if (gps_count > 3)
      {
        gps_locked = false;
      }
      display_status(true, gps_locked);
      continue;
    }

    gps_locked = true;

    // Retrieve the GPS location in billionths of degrees.
    int16_t lat_whle, lng_whle;
    uint32_t lat_frac, lng_frac;
    lat_whle = (gps.location.rawLat().negative ? -1 : 1) * ((int16_t)gps.location.rawLat().deg);
    lat_frac = gps.location.rawLat().billionths;
    lng_whle = (gps.location.rawLng().negative ? -1 : 1) * ((int16_t)gps.location.rawLng().deg);
    lng_frac = gps.location.rawLng().billionths;

    // Retrieve properties of the GPS reading.
    int16_t altitude, hdop;
    uint8_t satellites;
    altitude = (int16_t)gps.altitude.value();
    hdop = (int16_t)gps.hdop.value();
    satellites = (uint8_t)gps.satellites.value();

    printf("Lat: %i+%i/1000000000, Lng: %i+%i/1000000000, Age: %i\n", lat_whle, lat_frac, lng_whle, lng_frac, age);
    printf("Altitude (cm): %i, HDOP: %i, #satellites: %i\n", altitude, hdop, satellites);

    // Create our packet structure.
    appData[0] = (1 << 7); // our GPS
    // Latitude
    appData[1] = (lat_whle >> 0) & 0xFF;
    appData[2] = (lat_whle >> 8) & 0xFF;
    appData[3] = (lat_frac >> 0) & 0xFF;
    appData[4] = (lat_frac >> 8) & 0xFF;
    appData[5] = (lat_frac >> 16) & 0xFF;
    appData[6] = (lat_frac >> 24) & 0xFF;
    // Longitude
    appData[7] = (lng_whle >> 0) & 0xFF;
    appData[8] = (lng_whle >> 8) & 0xFF;
    appData[9] = (lng_frac >> 0) & 0xFF;
    appData[10] = (lng_frac >> 8) & 0xFF;
    appData[11] = (lng_frac >> 16) & 0xFF;
    appData[12] = (lng_frac >> 24) & 0xFF;
    // Altitude
    appData[13] = (altitude >> 0) & 0xFF;
    appData[14] = (altitude >> 8) & 0xFF;
    // Horizontal Dim. of Precision (HDOP)
    appData[15] = (hdop >> 0) & 0xFF;
    appData[16] = (hdop >> 8) & 0xFF;
    // Satellites
    appData[17] = satellites;
    // Count
    appData[18] = (count >> 0) & 0xFF;
    appData[19] = (count >> 8) & 0xFF;

    for (int i = 0; i < appDataSize; i++)
    {
      Serial.print(appData[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");

    // Display the GPS data to the OLED.
    int32_t lat, lng;
    lat = (((int32_t)lat_whle) * 1000000) + ((lat_whle ? -1 : 1) * ((int32_t)(lat_frac / 1000)));
    lng = (((int32_t)lng_whle) * 1000000) + ((lng_whle ? -1 : 1) * ((int32_t)(lng_frac / 1000)));
    display_gps(lat, lng);
  }
}

static void send_packet(void)
{
  send_packet();

  // Since we are sending a packet, increment our counter.
  count += 1;

  LoRaWAN.send(true, confirmedNbTrials, appPort);
}

///////////////////////////////////////////////////////////////////////////////
// Callback Functions
///////////////////////////////////////////////////////////////////////////////

// Called when we have joined a LoRaWAN network.
static void joined(void)
{
  display_status(true, gps_locked);

  send_packet();
}

// Called after a packet has been sent.
static void sent(uint8_t tries, bool acked)
{
  printf("Packet Sent. TX count: %i, Acked: %i\r\n", tries, acked);

  // Show status on OLED.
  display_tx_done(tries, acked);

  if (!acked)
  {
    // Increment our failed counter.
    failed_tx_consecutive += 1;

    // If we have relatively few failures, just try to send again.
    if (failed_tx_consecutive < 5)
    {
      send_packet();
    }
    else
    {
      // If we have many failed packets, try to re-join the network.
      failed_tx_consecutive = 0;
      LoRaWAN.join(LORA_OTAA, true);
    }
  }
  else
  {
    // Reset failed counter if needed.
    failed_tx_consecutive = 0;

    // Increment number of packets acked.
    acked_count += 1;

    // We want to wait for a period of time and then prepare and send another
    // packet. To do so in a low power mode, we use the LoRaWAN `cycle()` function
    // which will reboot the ESP32 chip after the desired amount of time.
    //
    // Note, all state is NOT lost in this operation. State marked `RTC_DATA_ATTR`
    // will be preserved, and the LoRaWAN stack uses this extensively. So, when
    // the chip restarts we will still be joined to the LoRaWAN network.
    LoRaWAN.cycle(SLEEP_TIME_BETWEEN_EVENTS_MS);

    // send_packet();
  }
}

static void acked(void)
{
}

void received(McpsIndication_t *mcpsIndication)
{
  printf("+REV DATA:%s", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1");
  printf(", RXSIZE %d", mcpsIndication->BufferSize);
  printf(", PORT %d\r\n", mcpsIndication->Port);
  //   printf("+REV DATA:");
  //   for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
  //     printf("%02X", mcpsIndication->Buffer[i]);
  //   }
  //   printf("\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Configure our normal `printf()` UART.
  Serial.begin(115200);

  // Configure the serial port for the GPS.
  gpsSerial.begin(9600, SERIAL_8N1, GPS_TX_PIN, -1);
  delay(10); // give the serial port time to initialize

  // Turn off LED.
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);

  // Initialize MCU.
  Mcu.begin();
  display.init();

  // Configure the DEVEUI to be what is hardcoded in this chip.
  LoRaWAN.generateDeveuiByChipID();

  // Show a title screen.
  if (count == 0)
  {
    char eui_str[100];
    int eui_str_len = snprintf(eui_str, 100, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                               devEui[0], devEui[1], devEui[2], devEui[3], devEui[4], devEui[5], devEui[6], devEui[7]);
    eui_str[eui_str_len] = '\0';

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 5, "hahahahaha");
    display.drawString(64, 40, "Device EUI:");
    display.drawString(64, 50, eui_str);
    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 16, "GPSLoRa");
    display.display();
    delay(2000);
  }

  // Initiate the LoRa process. This will check if the connection
  // is already established, or if we are connecting for the first
  // time (after a full power cycle for example).
  LoRaWAN.init(CLASS_A, LORAWAN_ACTIVE_REGION, loraWanAdr, joined, sent, acked, received);
  LoRaWAN.join(LORA_OTAA, false);
}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////

void loop()
{
  // All operations are event based, so all we do here is
  // let the LoRaWAN stack handle events or go to sleep.
  LoRaWAN.sleep();
}
