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

unsigned long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.

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
void udpsend(char *buff, bool serial);
char *nmeaEncodeGPS(int id, char *nmeacode, size_t length);
char *nmeaEncodeCompass(char *nmeaCode, size_t length);

const int RMC = 0;
const int GGA = 1;
const int VTG = 2;

//Compass section
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;

//End Compass section

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal

  Serial.println("NMEA-UDP-Sender start.");
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
  myGPS.setNavigationFrequency(5);
  myGPS.setAutoPVT(true, true);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
// Position solution updates every 200ms. So polling every 100ms -> 100ms latancy in worst case.
  if (millis() - lastTime > 100)
  {
    lastTime = millis(); //Update the timer

// When GPS has new solution, start encoding position data to NMEA sentences
    if (myGPS.getPVT())
    {
      char nmeabuff[100];

      udpsend(nmeaEncodeGPS(RMC, nmeabuff, sizeof(nmeabuff)), true);
      udpsend(nmeaEncodeGPS(GGA, nmeabuff, sizeof(nmeabuff)), true);
      udpsend(nmeaEncodeCompass(nmeabuff, sizeof(nmeabuff)), true);

    }
  }
}

char *nmeaEncodeGPS(int id, char *nmeaCode, size_t length)
{
  char buff[length];
  uint8_t hour = myGPS.getHour();
  uint8_t minute = myGPS.getMinute();
  uint8_t second = myGPS.getSecond();
  uint16_t msecond = myGPS.getMillisecond();
  long latitude = myGPS.getLatitude();   // degrees * 10^7
  long longitude = myGPS.getLongitude(); // degrees * 10^7
  uint8_t fixtype = myGPS.getFixType();  //0: no fix 1: dead reckoning only 2: 2D-fix 3: 3D-fix 4: GNSS + dead reckoning combined 5: time only fix
  byte SIV = myGPS.getSIV();
  unsigned long pDOP = myGPS.getPDOP();
  float altitude = (float)myGPS.getAltitudeMSL() / 1000; // mm->m
  float spdkt = (float)myGPS.getGroundSpeed() * 3600 / 1000000 / 1.6;
  float cog = (float)myGPS.getHeading() / 100000;
  uint8_t dd = myGPS.getDay();
  uint8_t mm = myGPS.getMonth();
  uint16_t yy = myGPS.getYear() % 100;

  int latdeg = (int)(latitude / 10000000);
  float latmin = (float)(latitude % 10000000) / 10000000 * 60;
  int londeg = (int)(longitude / 10000000);
  float lonmin = (float)(longitude % 10000000) / 10000000 * 60;

  //long geoidsep = myGPS.getGeoidSeparation(); too long to wait for polling (1sec) so ignored

  char fixRMC = 'V'; //V =No fix/ user limit over, A = 2D/3D/DGPS/RTK/Dead reckoning fix
  int fixGGA = 0;    //0 = No fix, 1 = Autonomous GNSS fix, 2 = Differential GNSS fix, 4 = RTK fixed, 5 = RTK float, 6 = Estimated/Dead reckoning fix
  switch (fixtype)   //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
  {
  case 1:
    fixRMC = 'A';
    break;
  case 2:
    fixRMC = 'A';
    fixGGA = 1;
    break;
  case 3:
    fixRMC = 'A';
    fixGGA = 1;
    break;
  case 4:
    fixRMC = 'A';
    fixGGA = 1;
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
  if (id == RMC)
    snprintf(buff, sizeof(buff), "$GNRMC,%02u%02u%02u.%03u,A,%d%2.5f,%c,%d%2.5f,%c,%1.2f,%03.2f,%02u%02u%02u,,,%c", hour, minute, second, msecond, latdeg, latmin, ns, londeg, lonmin, ew, spdkt, cog, dd, mm, yy, fixRMC);
  if (id == GGA)
    snprintf(buff, sizeof(buff), "$GNGGA,%02u%02u%02u.%1u,%d%2.5f,%c,%d%2.5f,%c,%d,%u,%lu,%.1f,M,,M,,,", hour, minute, second, msecond / 100, latdeg, latmin, ns, londeg, lonmin, ew, fixGGA, SIV, pDOP, altitude);

  char cs = 0;
  for (int i = 1; i < strnlen(buff, sizeof(buff)); i++)
  {
    cs ^= buff[i];
  }
  sprintf(nmeaCode, "%s*%02X", buff, cs);

  return nmeaCode;
}

char *nmeaEncodeCompass(char *nmeaCode, size_t length)
{
  char buff[length];
  char cs = 0;
  //Compass data polling
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.XAxis, norm.YAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = 0.13;
  char ew_magdec = 'E';
  if (declinationAngle >= 0)
  {
    ew_magdec = 'W';
  }
  else
  {
    ew_magdec = 'E';
  }
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180 / PI;
  sprintf(buff, "$HCHDG,%03.1f,,,%.1f,%c", headingDegrees, declinationAngle * 180 / PI, ew_magdec);
  for (int i = 1; i < strnlen(buff, sizeof(buff)); i++)
  {
    cs ^= buff[i];
  }
  sprintf(nmeaCode, "%s*%02X", buff, cs);
  return nmeaCode;
}

void udpsend(char *buff, bool serial)
{
  udp.beginPacket(udpAddress1, udpPort);
  udp.print(buff);
  udp.endPacket();
  if (serial)
    Serial.println(buff);
  return;
}