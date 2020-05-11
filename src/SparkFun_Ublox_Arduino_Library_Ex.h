#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#include "SparkFun_Ublox_Arduino_Library.h"
#endif

// Extend the class for getModuleInfo
class SFE_UBLOX_GPS_ADD : public SFE_UBLOX_GPS
{
public:
    boolean getModuleInfo(uint16_t maxWait = 1100); //Queries module, texts

    struct minfoStructure // Structure to hold the module info (uses 340 bytes of RAM)
    {
        char swVersion[30];
        char hwVersion[10];
        uint8_t extensionNo = 0;
        char extension[10][30];
    } minfo;
};