#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

char myLat[10];
char myLon[10];
char myString[50];

SoftwareSerial mySerial1(5, 4);

Adafruit_GPS GPS(&mySerial1);

#define GPSECHO  false

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
 
}

uint32_t timer = millis();
void loop() {

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
      Serial.println();
      // send packet
      LoRa.beginPacket();
      LoRa.print(myString);
      LoRa.endPacket();
      }
  }
  memset(myString, 0, strlen(myString));
}
