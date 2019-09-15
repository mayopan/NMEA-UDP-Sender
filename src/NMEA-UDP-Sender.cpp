/*
  Reading position data from Ublox GPS module via UBX binary commands using UART @115200 baud
  ...based on Sparkfun example (Thank you very much!!)
  Reading Bearing direction from HMC5883L (it may be on the same GPS module for drone like this https://ja.aliexpress.com/item/32860597703.html)
 And send them after nmea0183 coding (GNRMC, GNGGA, HCHDG) via Wifi through udp protocol.

  Date: September 12, 2019
  Copyright mayopan
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  I want to broadcast NMEA data on Wifi for OpenCPN on other device like android tablet.
  But ESP32 UDP broadcast is NOT stable, so unicast UDP is applied in this program.

  It needs some library below.
  #SparkFun_Ublox_Arduino_Library
  #HMC5883L https://github.com/jarzebski/Arduino-HMC5883L

  It will run on arduino IDE by some mod.
    * Comment out #include <Arduino.h>
    * Chenge file name(extension) to .ino from .cpp
    * 
  Hardware Connections:
  Connect the U-Blox serial TX pin to ESP32 devkitC pin 16
  Connect the U-Blox serial RX pin to ESP32 devkitC pin 17
  Open the serial monitor at 115200 baud to see the output
*/

#include <Arduino.h>

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

/*-----Wifi section----------------------------*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

// WiFi network name and password:
const char *ssid = "esp32";
const char *password = "esp32esp32";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress1 = "192.168.20.2";
//const char *udpAddress2 = "192.168.20.3";
const int udpPort = 2947;
const IPAddress ip(192, 168, 20, 1);
const IPAddress subnet(255, 255, 255, 0);

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

/*-------end Wifi section-------------*/

//Send NMEA data with checksum on UDP

#include "UbloxGpsDataParser.h"

UbloxGPS gpsData;

void sendData(char *buff, bool SerialOut, bool UdpOut, bool OLEDOut);

//Compass section
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
HMC5883Compass compassData;

//End Compass section

//OLED section
#include <U8x8lib.h>
U8X8_SH1107_PIMORONI_128X128_HW_I2C u8x8(/* reset=*/8);

unsigned long lastTime = 0;                 //Simple local timer.
const unsigned long Polling_Interval = 100; // ms = 10Hz It's not sampling interval. It should be faster than GPS sampling.

void setup()
{
  u8x8.begin();
  u8x8.setFont(u8x8_font_8x13B_1x2_f);
  u8x8.clear();
  u8x8.inverse();
  u8x8.print("NMEA-UDP-Sender");
  u8x8.setCursor(0, 2);
  u8x8.noInverse();
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  u8x8.print("Serial connected.");
  u8x8.setCursor(0, 4);
  Serial.println("NMEA-UDP-Sender start.");
  //Connect to the WiFi network
  u8x8.print("Config Wifi AP");
  u8x8.setCursor(0, 6);
  Serial.println("Configuring access point...");
  //Create Access Point
  WiFi.softAP(ssid, password);
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet);
  IPAddress myIP = WiFi.softAPIP();
  u8x8.print("AP IP address: ");
  u8x8.setCursor(0, 8);
  u8x8.print(myIP);
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  /*Initialize the mag sensor */
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 115200 baud.
  //Loop until we're in sync and then ensure it's at 115200 baud.
  do
  {
    Serial.println("GPS: trying 115200 baud");
    Serial2.begin(115200);
    delay(100);
    if (myGPS.begin(Serial2) == true)
      break;
    Serial.println("GPS: trying 9600 baud");
    delay(100);
    Serial2.begin(9600);
    if (myGPS.begin(Serial2) == true)
    {
      Serial.println("GPS: connected at 9600 baud, switching to 115200");
      myGPS.setSerialRate(115200);
      delay(400);
    }
    else
    {

      myGPS.hardReset();
      Serial.println("GPS: hard reset!");
      delay(1000); //Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  udp.beginPacket(udpAddress1, udpPort);
  Serial.println("GPS serial connected @baud 115200");
  udp.print("GPS serial connected @baud 115200");
  udp.endPacket();

  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(5);
  myGPS.setAutoPVT(true, true);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  delay(1000);
  u8x8.clear();
}

void loop()
{
  // Position solution updates every 200ms. So polling every 100ms -> 100ms latency in worst case.
  if (millis() - lastTime > Polling_Interval)
  {
    lastTime = millis(); //Update the timer

    // When GPS has new solution, start encoding position data to NMEA sentences
    if (myGPS.getPVT())
    {
      compassData.parse(compass);
      gpsData.parse(myGPS);
      sendData(gpsData.nmeaRMC, true, true, true);
      sendData(gpsData.nmeaGGA, true, true, false);
      sendData(compassData.nmeaHDG, true, true, false);
    }
  }
}

void sendData(char *buff, bool SerialOut, bool UdpOut, bool OLEDOut)
{
  if (SerialOut == true)
  {
    Serial.println(buff);
  }
  if (UdpOut == true)
  {
    udp.beginPacket(udpAddress1, udpPort);
    udp.print(buff);
    udp.endPacket();
  }
  if (OLEDOut == true)
  {
    u8x8.setCursor(0, 0);
    u8x8.print("Lat:");
    u8x8.setCursor(0, 2);
    u8x8.printf("%3d%c%02.5f'%c", gpsData.latdeg, '\xB0', gpsData.latmin, gpsData.ns);
    u8x8.setCursor(0, 4);
    u8x8.print("Lon:");
    u8x8.setCursor(0, 6);
    u8x8.printf("%3ld%c%02.5f'%c", gpsData.londeg, '\xB0', gpsData.lonmin, gpsData.ew);
    u8x8.setCursor(0, 8);
    u8x8.print("fix SIV pDOP Alt");
    u8x8.setCursor(0, 10);
    float dummyAlt = gpsData.altitude;
    if (dummyAlt > 999)
      dummyAlt = 999;
    u8x8.printf(" %d  %2u  %4lu %3.0f", gpsData.fixtype, gpsData.SIV, gpsData.pDOP, dummyAlt);
    u8x8.setCursor(0, 12);
    u8x8.print("BRG");
    u8x8.setCursor(0, 14);
    u8x8.printf(" %03.0f%c", compassData.headingDegrees, '\xB0');
  }
  return;
}
