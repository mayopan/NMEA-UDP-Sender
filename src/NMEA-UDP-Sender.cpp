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

/*-------end Wifi section-------------*/

//Send NMEA data with checksum on UDP
void udpsend(const char *ip, int port, char *buff, size_t buffMax, bool serial);

//Compass section
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

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

  /* Initialise the mag sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

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

    //GPS data polling and make NMEA0183 sentence
    char RMCBuff[100];
    char GGABuff[100];
    char HDGBuff[50];
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
    float altitude = (float)myGPS.getAltitudeMSL() / 1000; // mm->m
    unsigned long horacc = myGPS.getHorizontalAccuracy();
    long geoidsep = myGPS.getGeoidSeparation();
    byte SIV = myGPS.getSIV();
    char fix = 'V';
    int fixGGA = 0;
    switch (myGPS.getFixType()) //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
    {
    case 1:
      break;
    case 2:
      fix = 'A';
      fixGGA = 1;
      break;
    case 3:
      fix = 'A';
      fixGGA = 2;
      break;
    case 4:
      fix = 'A';
      fixGGA = 2;
      break;
    case 5:
      break;
    }
    char ns = 'N';
    if (latitude >= 0)
    {
      ns = 'N';
    }
    else
    {
      ns = 'S';
    }
    char ew = 'E';
    if (longitude >= 0)
    {
      ew = 'E';
    }
    else
    {
      ew = 'W';
    }

    sprintf(RMCBuff, "$GNRMC,%02u%02u%02u.%03u,A,%d%2.5f,%c,%d%2.5f,%c,%1.2f,%03.2f,%02u%02u%02u,,,%c", hour, minute, second, msecond, latdeg, latmin, ns, londeg, lonmin, ew, spdkt, hdgdeg, dd, mm, yy, fix);
    sprintf(GGABuff, "$GNGGA,%02u%02u%02u.%1u,%d%2.5f,%c,%d%2.5f,%c,%d,%u,%lu,%.1f,M,%ld,M,,,", hour, minute, second, msecond / 100, latdeg, latmin, ns, londeg, lonmin, ew, fixGGA, SIV, horacc, altitude, geoidsep);
    /*
Time (UTC)
Status A - Data Valid, V - Data Invalid, FAA mode indicator (NMEA 2.3 and later)
*/
    //End of GPS

    //Compass data polling
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.x, event.magnetic.y);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.13;
    heading += declinationAngle;
    char ew_magdec = 'E';
    if (declinationAngle >= 0)
    {
      ew_magdec = 'W';
    }
    else
    {
      ew_magdec = 'E';
    }
    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / PI;

    Serial.print("Heading (degrees): ");
    Serial.println(headingDegrees);

    sprintf(HDGBuff, "$HCHDG,%03.1f,,,%.1f,%c", headingDegrees, declinationAngle * 180 / PI, ew_magdec);

    //Send a packet
    udpsend(udpAddress1, udpPort, RMCBuff, sizeof(RMCBuff), true);
//    udpsend(udpAddress2, udpPort, RMCBuff, sizeof(RMCBuff), false);
    udpsend(udpAddress1, udpPort, GGABuff, sizeof(GGABuff), true);
//    udpsend(udpAddress2, udpPort, GGABuff, sizeof(GGABuff), false);
    udpsend(udpAddress1, udpPort, HDGBuff, sizeof(HDGBuff), true);
//    udpsend(udpAddress2, udpPort, HDGBuff, sizeof(HDGBuff), false);
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
