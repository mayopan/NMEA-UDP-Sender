# NMEA-UDP-Sender

I made this as "Add on wireless GPS module" for tablet device such as Amazon fire.
Especially for plotting app OpenCPN.

* It polls positioning data from Ublox GPS module with UBX protocol via UART
* Positioning data is converted to NMEA0183 format char buffer including checksum
* It sets ESP32 as an Wifi access point, and unicast the NMEA sentence to client.
 
# DEMO
 
Under construction
 
# Features
 
Using UBX protocol has advantages below.

* Fast transmission in light data stream
* You control polling timing as you like

It polls position data 10 times per second and casts to air.
 
# Requirement
 
Hardware: 
* ESP32
* Ublox GPS module: M8N, M8Q, ...
 
Library:
* Sparkfun Ublox Library for arduino
 
# Installation
 
Under construction
 
```bash

```
 
# Usage
 
Under construction
 
```bash

```
 
# Note
 
Under construction
 
# Author
 
* mayopan
 
# License
ライセンスを明示する
 
"NMEA-UDP-Sender" is under [MIT license](https://en.wikipedia.org/wiki/MIT_License).