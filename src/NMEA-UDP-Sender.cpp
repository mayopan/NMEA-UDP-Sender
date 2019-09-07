/*
  Reading lat and long via UBX binary commands using UART @115200 baud - free from I2C
  ...based on Sparkfun example (Thank you very much!!)
 And send it via Wifi through udp protocol.

  Date: September 7, 2019
  by: mayopan
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  I want to broadcast NMEA data on Wifi for OpenCPN on other device like android tablet.
  But ESP32 UDP broadcast is NOT stable, so unicast UDP is applied in this program.

  It needs some library below.
  #SparkFun_Ublox_Arduino_Library

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

//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11); // RX, TX. Pin 10 on Uno goes to TX pin on GPS module.

long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.

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
const char *udpAddress2 = "192.168.20.3";
const int udpPort = 2947;
const IPAddress ip(192, 168, 20, 1);
const IPAddress subnet(255, 255, 255, 0);

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

//Send NMEA data with checksum on UDP
void udpsend(const char *ip, int port, char *buff, size_t buffMax, bool serial);

/*-------end Wifi section-------------*/

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal

  Serial.println("SparkFun Ublox Example");
  //Connect to the WiFi network
  Serial.println("Configuring access point...");
  //Create Access Point
  WiFi.softAP(ssid, password);
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
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
      delay(1000);
    }
    else
    {
      myGPS.hardReset();
      delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  udp.beginPacket(udpAddress1, udpPort);
  Serial.println("GPS serial connected @baud 115200");
  udp.print("GPS serial connected @baud 115200");
  udp.endPacket();

  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(10);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 200)
  {
    lastTime = millis(); //Update the timer
    char rmcBuff[100];
    char GLLBuff[100];
    uint8_t hour = myGPS.getHour();
    uint8_t minute = myGPS.getMinute();
    uint8_t second = myGPS.getSecond();
    uint16_t msecond = myGPS.getMillisecond();
    uint8_t dd = myGPS.getDay();
    uint8_t mm = myGPS.getMonth();
    uint16_t yy = myGPS.getYear() % 100;
    long latitude = myGPS.getLatitude();   // degrees * 10^7
    long longitude = myGPS.getLongitude(); // degrees * 10^7
    int latdeg = (int)(latitude / 10000000);
    float latmin = (float)(latitude % 10000000) / 10000000 * 60;
    int londeg = (int)(longitude / 10000000);
    float lonmin = (float)(longitude % 10000000) / 10000000 * 60;
//  long spd = myGPS.getGroundSpeed(); // mm/s
    float spdkt = (float)myGPS.getGroundSpeed() * 3600 / 1000000 / 1.6;
//  long hdg = myGPS.getHeading();
    float hdgdeg = (float)myGPS.getHeading() / 100000;
//  long altitude = myGPS.getAltitude();
//  byte SIV = myGPS.getSIV();
    char fix = 'V';
    if (myGPS.getFixType() > 0)
    {
      fix = 'A';
    } //0=no, 3=3D, 4=GNSS+Deadreckoning)"));
    char ns = 'N';
    if(latitude>=0){
      ns = 'N';
    }
    else{
      ns = 'S';
    }
    char ew = 'E';
    if (longitude>=0){
      ew = 'E';
    }else{
      ew = 'W';
    }

    sprintf(rmcBuff, "$GNRMC,%02u%02u%02u.%03u,A,%d%2.5f,%c,%d%2.5f,%c,%1.2f,%03.2f,%02u%02u%02u,,,%c", hour, minute, second, msecond, latdeg, latmin, ns, londeg, lonmin, ew, spdkt, hdgdeg, dd, mm, yy, fix);
    sprintf(GLLBuff, "$GNGLL,%d%2.5f,%c,%d%2.5f,%c,%02u%02u%02u.%02u,%c", latdeg, latmin, ns, londeg, lonmin, ew, hour, minute, second, msecond / 10, fix);
/*
Time (UTC)
Status A - Data Valid, V - Data Invalid, FAA mode indicator (NMEA 2.3 and later)
*/

    //Send a packet
    udpsend(udpAddress1, udpPort, rmcBuff, sizeof(rmcBuff), true);
    udpsend(udpAddress2, udpPort, rmcBuff, sizeof(rmcBuff), false);
    udpsend(udpAddress1, udpPort, GLLBuff, sizeof(GLLBuff), true);
    udpsend(udpAddress2, udpPort, GLLBuff, sizeof(GLLBuff), false);
  }
}

void udpsend(const char *ip, int port, char *buff, size_t buffMax, bool serial)
{
  // calculate checksum
  char cs = 0;
  for (int i = 1; i < strnlen(buff, buffMax); i++)
  {
    cs ^= buff[i];
  }
  udp.beginPacket(ip, port);
  udp.printf("%s*%02X", buff, cs);
  udp.endPacket();
  if (serial)
  {
    Serial.printf("%s*%02X\n", buff, cs);
  }
  return;
}
