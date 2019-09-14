#ifndef UBLOX_GPS_DATA_PARSER_H
#define UBLOX_GPS_DATA_PARSER_H

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#include <SparkFun_Ublox_Arduino_Library.h>
#endif

class UbloxGPS
{
private:
    long latitude, longitude; // degrees * 10^7
    char cs[4];

    char *csCalc(char *buffin, size_t length);

public:
    uint8_t hour, minute, second, fixtype, dd, mm;
    uint16_t msecond, yy;
    byte SIV;
    unsigned long pDOP;
    float altitude, spdkt, cog, latmin, lonmin;
    int latdeg;
    long londeg;
    char ns, ew, nmeaRMC[90], nmeaGGA[90];

    //long geoidsep = myGPS.getGeoidSeparation(); too long to wait for polling (1sec) so ignored

    char fixRMC = 'V'; //V =No fix/ user limit over, A = 2D/3D/DGPS/RTK/Dead reckoning fix
    int fixGGA = 0;

    void parse(SFE_UBLOX_GPS myGPS);
};
#endif //UBLOX_GPS_DATA_PARSER_H