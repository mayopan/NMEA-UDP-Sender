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

//Are we currently wifiConnected?
boolean wifiConnected = false;

//The udp library class
WiFiUDP udp;

/*-------end Wifi section-------------*/

//Send NMEA data with checksum on UDP

#include "UbloxGpsDataParser.h"

UbloxGPS gpsData;

void sendData(char *buff, bool SerialOut, bool UdpOut, bool OLEDOut);

//Compass section
//#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
HMC5883Compass compassData;

boolean compassConnected = true;

//End Compass section

//OLED section
#include <U8g2lib.h>
U8G2_SH1107_PIMORONI_128X128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/8);
#define U8LOG_WIDTH 30
#define U8LOG_HEIGHT 10
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8G2LOG u8g2log;

#include "satteliteicon.xbm"

unsigned long lastTime = 0;                 //Simple local timer.
const unsigned long Polling_Interval = 100; // ms = 10Hz It's not sampling interval. It should be faster than GPS sampling.

void setup()
{
  uint8_t breakCounter = 0;
  u8g2.begin();
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2.setFontMode(1);
  u8g2.setFont(u8g2_font_7x14_tr);
  u8g2log.println("Booting.."); // output title

  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  u8g2log.println("Serial connected.");
  Serial.println("NMEA-UDP-Sender start.");

  //Create Access Point
  u8g2log.println("Config Wifi AP");
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet);
  IPAddress myIP = WiFi.softAPIP();
  u8g2log.println("AP IP address: ");
  u8g2log.println(myIP);
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  IPAddress broadcastIP = WiFi.softAPBroadcastIP();
  u8g2log.println("Broadcast address: ");
  u8g2log.println(broadcastIP);

  /*Initialize the mag sensor */
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
    breakCounter++;
    if (breakCounter > 4)
    {
      Serial.println("Compass function is disabled.");
      u8g2log.println("Compass is NA.");
      compassConnected = false;
      break;
    }
  }
  if (compassConnected == true)
  {
    u8g2log.println("Compass Connected");
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
    u8g2log.println("Compass config set.");
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
      Serial.println("GPS: Connected at 9600 baud, switching to 115200");
      u8g2log.println("GPS: Connected @9600");
      u8g2log.println("Changing baud to 115200");
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
  Serial.println("GPS serial Connected @baud 115200");
  u8g2log.println("GPS Connected.");
  udp.beginPacket(udpAddress1, udpPort);
  udp.print("GPS serial Connected @baud 115200");
  udp.endPacket();
  Serial.print(F("Version: "));
  u8g2log.print("Protcol Ver: ");
  byte versionHigh = myGPS.getProtocolVersionHigh();
  delay(1000);
  Serial.print(versionHigh);
  u8g2log.print(versionHigh);
  Serial.print(".");
  u8g2log.print(".");
  byte versionLow = myGPS.getProtocolVersionLow();
  delay(1000);
  Serial.println(versionLow);
  u8g2log.println(versionLow);
  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(5);
  myGPS.setAutoPVT(true, true);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  u8g2log.println("GPS config done.");
  delay(5000);
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

      gpsData.parse(myGPS);
      sendData(gpsData.nmeaRMC, true, true, true);
      sendData(gpsData.nmeaGGA, true, true, false);
      if (compassConnected == true)
      {
        compassData.parse(compass);
        sendData(compassData.nmeaHDG, true, true, false);
      }
    }
  }
}

void sendData(char *buff, bool SerialOut, bool UdpOut, bool OLEDOut)
{
  Vector2D arrow_l[3] = {{0, 5}, {-6, 12}, {0, -15}};

  Vector2D arrow_r[3] = {{0, 5}, {6, 12}, {0, -15}};
  Vector2D arrow_now_l[3], arrow_now_r[3], tick[2] = {{0, -12}, {0, -15}};
  char buff_u8g2[16];
  u8g2_uint_t length;
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
    u8g2.firstPage();
    do
    {
      for (int i = 1; i < 128 / 16; i++)
      {
        u8g2.setDrawColor(1);
        if (i == 5 || i == 6)
        {
          u8g2.drawHLine(16 * 3, i * 16, 127 - 16 * 3);
        }
        else
        {
          if (i == 2 || i == 4 || i == 7)
          {
            u8g2.drawHLine(0, i * 16, 127);
          }
        }
      }
      for (int i = 0; i < 3; i++)
      {
        arrow_now_l[i] = arrow_l[i].rotateDeg(-compassData.headingDegrees) + center;
        arrow_now_r[i] = arrow_r[i].rotateDeg(-compassData.headingDegrees) + center;
      }

      u8g2.drawTriangle(arrow_now_l[0].x, arrow_now_l[0].y, arrow_now_l[1].x, arrow_now_l[1].y, arrow_now_l[2].x, arrow_now_l[2].y);
      u8g2.drawTriangle(arrow_now_r[0].x, arrow_now_r[0].y, arrow_now_r[1].x, arrow_now_r[1].y, arrow_now_r[2].x, arrow_now_r[2].y);
      u8g2.drawCircle(center.x, center.y, 16);
      u8g2.setFont(u8g2_font_crox1hb_tr);
      u8g2.drawGlyph(center.x - 4, center.y - 15, 'N');
      u8g2.drawGlyph(center.x - 4, center.y + 16 + 8, 'S');
      u8g2.drawGlyph(center.x + 16, center.y + 4, 'E');
      u8g2.drawGlyph(center.x - 16 - 8, center.y + 4, 'W');
      for (int i = 0; i < 16; i++)
      {
        Vector2D startPt, endPt;
        startPt = tick[0].rotateDeg(i * 360.0 / 12) + center;
        endPt = tick[1].rotateDeg(i * 360.0 / 12) + center;
        u8g2.drawLine(startPt.x, startPt.y, endPt.x, endPt.y);
      }

      u8g2.setCursor(0, 15);
      u8g2.setFont(u8g2_font_crox2hb_tr);
      u8g2.print("DATE");
      u8g2.setCursor(0, 31);
      u8g2.print("TIME");
      u8g2.setCursor(48, 79);
      u8g2.printf("BRG");
      u8g2.setCursor(48, 95);
      u8g2.printf("COG");
      u8g2.setCursor(48, 111);
      u8g2.printf("SOG");

      u8g2.setFont(u8g2_font_crox3hb_tf);
      snprintf(buff_u8g2, 16, "20%02u.%2u.%2u", gpsData.yy, gpsData.mm, gpsData.dd);
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 15);
      u8g2.print(buff_u8g2);
      snprintf(buff_u8g2, 16, "%02u:%02u:%02u", gpsData.hour, gpsData.minute, gpsData.second);
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 31);
      u8g2.print(buff_u8g2);
      u8g2.setCursor(0, 55);
      u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
      u8g2.printf("%c", '\x47'); //gps
      u8g2.setFont(u8g2_font_crox3hb_tf);
      snprintf(buff_u8g2, 16, "%3d%c%02.4f'%c", gpsData.latdeg, '\xB0', gpsData.latmin, gpsData.ns);
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 47);
      u8g2.printf(buff_u8g2);
      snprintf(buff_u8g2, 16, "%3ld%c%02.4f'%c", gpsData.londeg, '\xB0', gpsData.lonmin, gpsData.ew);
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 63);
      u8g2.printf(buff_u8g2);

      snprintf(buff_u8g2, 16, "%03ld%c", long(compassData.headingDegrees), '\xb0');
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 79);
      u8g2.printf(buff_u8g2);

      snprintf(buff_u8g2, 16, "%03ld%c", long(gpsData.cog), '\xb0');
      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 95);
      u8g2.printf(buff_u8g2);

      snprintf(buff_u8g2, 16, "%2.1fkt", gpsData.spdkt);

      length = u8g2.getStrWidth(buff_u8g2);
      u8g2.setCursor(128 - length, 111);
      u8g2.printf(buff_u8g2);
      //      u8g2.setCursor(0, 128);
      //     u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
      //      u8g2.drawGlyph(96, 128, 0x42);//out
      u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
      u8g2.drawGlyph(0, 128, 0x46);                                                          //position fix (arrow icon)
                                                                                             /*      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.setCursor(16, 128);
      u8g2.printf("hac%lum,vac%lum",gpsData.hAcc/1000,gpsData.vAcc/1000);
      */
      u8g2.drawGlyph(78, 128, 0x51);                                                         //Wifi
      u8g2.drawXBMP(102, 113, u8g2_satellite_width, u8g2_satellite_height, u8g2_satellite_bits); //sattelite icon
                                                                                             //      u8g2.drawGlyph(102, 128, 0x47); //sattelite in view (gps icon)
      u8g2.setFont(u8g2_font_7x14_tr);
      u8g2.setCursor(16, 128);
      uint8_t fix = gpsData.fixtype;
      if (fix == 2) //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
      {
        snprintf(buff_u8g2, 16, "2D");
      }
      else if (fix == 3)
      {
        //        snprintf(buff_u8g2, 16, "3D-%u", gpsData.solution);
        if (gpsData.solution == 2)
        {
          snprintf(buff_u8g2, 16, "Dgps");
        }
        else
        {
          snprintf(buff_u8g2, 16, "3D");
        }
      }
      else
      {
        snprintf(buff_u8g2, 16, "x");
      }

      u8g2.printf("%s", buff_u8g2);

      u8g2.setFont(u8g2_font_open_iconic_app_2x_t);
      u8g2.drawGlyph(40, 128, 0x46); //compass
      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.setCursor(58, 128);
      if (compassConnected)
      {
        snprintf(buff_u8g2, 16, "%ld%c", gpsData.magDec / 100, '\xb0');
        u8g2.print(buff_u8g2);
      }
      else
      {
        u8g2.print("x");
      }

      u8g2.setCursor(96, 128);
      u8g2.print(WiFi.softAPgetStationNum());

      u8g2.setCursor(118, 128);
      u8g2.printf("%u", gpsData.SIV);
    } while (u8g2.nextPage());
  }
  return;
}
