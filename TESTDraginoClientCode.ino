/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

static const u1_t PROGMEM APPEUI[8]={ 0xD8, 0x78, 0x94, 0x4E, 0xF1, 0xA8, 0xE3, 0xAE };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x20, 0x85, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x4C, 0x19, 0x4E, 0x20, 0xD3, 0x96, 0xB5, 0xF7, 0xD3, 0xE1, 0x55, 0x1E, 0x4C, 0xD3, 0x20, 0xDD };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

char myLat[10];
char myLon[10];
char myString[25];

SoftwareSerial mySerial1(8, 7);

Adafruit_GPS GPS(&mySerial1);

#define GPSECHO  false

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, myString, sizeof(myString)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);

    while (!Serial);
  
    Serial.println("LoRa Sender");

    if (!LoRa.begin(915E6)) {
      Serial.println("Starting LoRa failed!");
      while (1);
    }
    LoRa.setSyncWord(0x34);

    GPS.begin(9600);
    delay(1000);

    mySerial1.println(PMTK_Q_RELEASE);

    mySerial1.listen();
/*
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
*/
}

uint32_t timer = millis();
int counter = 0;
void loop() {
  
  os_runloop_once();

  char c = GPS.read();
   if(GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) {
        Serial.println("Warning: failed to parse GPS NMEA message");
        return;
      }
    }  

  if (millis() - timer > 500) {
    timer = millis();
  
    if(GPS.fix) {
      Serial.println(GPS.latitude, 4);
      Serial.println(GPS.longitude, 4);
  
      dtostrf(GPS.latitude, 8, 3, myLat);
      dtostrf(GPS.longitude, 8, 3, myLon);
        //Place device ID here
      strcat(myString, "D01,");
      strcat(myString, myLat);
      strcat(myString, ",");
      strcat(myString, myLon);    
  
      Serial.print(myString);
      // send packet
      LoRa.beginPacket();
      LoRa.print(myString);
      LoRa.endPacket();
      Serial.println();
      counter++;
      }
  }
  if ((counter >= 40)&&!LoRa.parsePacket()) {
    LoRa.end();
    os_init();
    LMIC_reset();
    do_send(&sendjob);
    counter = 0;
  }
  memset(myString, 0, strlen(myString));
}
