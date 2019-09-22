
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

void UbloxGPS::parse(SFE_UBLOX_GPS _gps)
{
    hour = _gps.getHour();
    minute = _gps.getMinute();
    second = _gps.getSecond();
    msecond = _gps.getMillisecond();
    latitude = _gps.getLatitude();   // degrees * 10^7
    longitude = _gps.getLongitude(); // degrees * 10^7
    fixtype = _gps.getFixType();     //0: no fix 1: dead reckoning only 2: 2D-fix 3: 3D-fix 4: GNSS + dead reckoning combined 5: time only fix
    SIV = _gps.getSIV();
    pDOP = _gps.getPDOP();
    altitude = (float)_gps.getAltitudeMSL() / 1000; // mm->m
    spdkt = (float)_gps.getGroundSpeed() * 3600 / 1000000 / 1.6;
    cog = (float)_gps.getHeading() / 100000;
    dd = _gps.getDay();
    mm = _gps.getMonth();
    yy = _gps.getYear() % 100;

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

char *HMC5883Compass::csCalc(char *buffin, size_t length)
{

    char csDigit = 0;
    for (int i = 1; i < strnlen(buffin, length); i++)
    {
        csDigit ^= buffin[i];
    }
    sprintf(cs, "*%02X", csDigit);
    return cs;
};

void HMC5883Compass::parse(HMC5883L _compass)
{
    //Compass data polling
    Vector norm = _compass.readNormalize();

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
    headingDegrees = heading * 180 / PI;
    sprintf(nmeaHDG, "$HCHDG,%03.1f,,,%.1f,%c", headingDegrees, declinationAngle * 180 / PI, ew_magdec);
    strcat(nmeaHDG, csCalc(nmeaHDG, sizeof(nmeaHDG)));
};

Vector2D Vector2D::rotateDeg(double deg)
{
    Vector2D new_vec;
    new_vec.x = x * cos(radians(deg)) + y * sin(radians(deg));
    new_vec.y = -x * sin(radians(deg)) + y * cos(radians(deg));
    //x = other.x;
    //y = other.y;
    return new_vec;
};

Vector2D operator+(Vector2D pt, Vector2D mv)
{
    Vector2D new_pt;
    new_pt.x = pt.x + mv.x;
    new_pt.y = pt.y + mv.y;
    return new_pt;
}