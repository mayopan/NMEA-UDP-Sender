#ifndef UBLOX_GPS_DATA_PARSER_H
#define UBLOX_GPS_DATA_PARSER_H

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#include <SparkFun_Ublox_Arduino_Library.h>
#endif

#ifndef HMC5883L
#include <HMC5883L.h>
#endif




class UbloxGPS
{
private:
    long latitude, longitude; // degrees * 10^7

public:
    uint8_t hour, minute, second, fixtype, dd, mm;
    uint16_t msecond, yy;
    byte SIV;
    unsigned long pDOP;
    float altitude, spdkt, cog, latmin, lonmin;
    int latdeg;
    long londeg;
    char ns, ew, nmeaRMC[90], nmeaGGA[90];
    char cs[4];
    //long geoidsep = myGPS.getGeoidSeparation(); too long to wait for polling (1sec) so ignored

    char fixRMC = 'V'; //V =No fix/ user limit over, A = 2D/3D/DGPS/RTK/Dead reckoning fix
    int fixGGA = 0;
    char *csCalc(char *buffin, size_t length);
    void parse(SFE_UBLOX_GPS myGPS);
};

class HMC5883Compass
{
private:
    Vector norm;

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);


public:
    char nmeaHDG[90];
    float heading,headingDegrees;
    float declinationAngle = 0.13;
    char ew_magdec = 'E';
    char cs[4];
    char *csCalc(char *buffin, size_t length);
    void parse(HMC5883L _compass);
};
#endif //UBLOX_GPS_DATA_PARSER_H