
#ifndef Arduino_h
#include <Arduino.h>
#endif

#include "UbloxGpsDataParser.h"

char *UbloxGPS::csCalc(char *buffin, size_t length)
{

    char csDigit = 0;
    for (int i = 1; i < strnlen(buffin, length); i++)
    {
        csDigit ^= buffin[i];
    }
    sprintf(cs, "*%02X", csDigit);
    return cs;
};

void UbloxGPS::parse(SFE_UBLOX_GPS myGPS)
{
    hour = myGPS.getHour();
    minute = myGPS.getMinute();
    second = myGPS.getSecond();
    msecond = myGPS.getMillisecond();
    latitude = myGPS.getLatitude();   // degrees * 10^7
    longitude = myGPS.getLongitude(); // degrees * 10^7
    fixtype = myGPS.getFixType();     //0: no fix 1: dead reckoning only 2: 2D-fix 3: 3D-fix 4: GNSS + dead reckoning combined 5: time only fix
    SIV = myGPS.getSIV();
    pDOP = myGPS.getPDOP();
    altitude = (float)myGPS.getAltitudeMSL() / 1000; // mm->m
    spdkt = (float)myGPS.getGroundSpeed() * 3600 / 1000000 / 1.6;
    cog = (float)myGPS.getHeading() / 100000;
    dd = myGPS.getDay();
    mm = myGPS.getMonth();
    yy = myGPS.getYear() % 100;

    latdeg = (int)(latitude / 10000000);
    latmin = (float)(latitude % 10000000) / 10000000 * 60;
    ns = 'N';
    if (latitude < 0)
    {
        latdeg *= (-1);
        latmin *= (-1);
        ns = 'S';
    }
    londeg = longitude / 10000000;
    lonmin = (float)(longitude % 10000000) / 10000000 * 60;
    ew = 'E';
    if (longitude < 0)
    {
        londeg *= (-1);
        lonmin *= (-1);
        ew = 'W';
    }
    switch (fixtype) //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
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
    snprintf(nmeaRMC, sizeof(nmeaRMC), "$GNRMC,%02u%02u%02u.%03u,A,%d%2.5f,%c,%ld%2.5f,%c,%1.2f,%03.2f,%02u%02u%02u,,,%c",
             hour, minute, second, msecond, latdeg, latmin, ns, londeg, lonmin, ew, spdkt, cog, dd, mm, yy, fixRMC);
    strcat(nmeaRMC, csCalc(nmeaRMC, sizeof(nmeaRMC)));

    snprintf(nmeaGGA, sizeof(nmeaGGA), "$GNGGA,%02u%02u%02u.%1u,%d%2.5f,%c,%ld%2.5f,%c,%d,%u,%lu,%.1f,M,,M,,,",
             hour, minute, second, msecond / 100, latdeg, latmin, ns, londeg, lonmin, ew, fixGGA, SIV, pDOP, altitude);
    strcat(nmeaGGA, csCalc(nmeaGGA, sizeof(nmeaGGA)));
};
