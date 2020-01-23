
#ifndef Arduino_h
#include <Arduino.h>
#endif

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#include <SparkFun_Ublox_Arduino_Library.h>
#endif

#include "UbloxGpsDataParser.h"

const int localtimeDiff = 9;

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
    solution = _gps.getCarrierSolutionType();

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
    snprintf(nmeaRMC, sizeof(nmeaRMC), "$GNRMC,%02u%02u%02u.%01u,A,%02d%02.5f,%c,%03ld%02.5f,%c,%1.2f,%03.2f,%02u%02u%02u,,,%c",
             hour, minute, second, msecond, latdeg, latmin, ns, londeg, lonmin, ew, spdkt, cog, dd, mm, yy, fixRMC);
    strcat(nmeaRMC, csCalc(nmeaRMC, sizeof(nmeaRMC)));

    snprintf(nmeaGGA, sizeof(nmeaGGA), "$GNGGA,%02u%02u%02u.%01u,%02d%02.5f,%c,%03ld%02.5f,%c,%d,%u,%lu,%.1f,M,,M,,,",
             hour, minute, second, msecond / 100, latdeg, latmin, ns, londeg, lonmin, ew, fixGGA, SIV, pDOP, altitude);
    strcat(nmeaGGA, csCalc(nmeaGGA, sizeof(nmeaGGA)));
    
    hour += localtimeDiff;
    if (hour >= 24)
    {
        hour -= 24;
        dd++;
        if (dd > 31 && (mm == 1 || mm == 3 || mm == 5 || mm == 7 || mm == 8 || mm == 10 || mm == 12))
        {
            dd -= 31;
            mm++;
            if (mm > 12)
            {
                mm -= 12;
                yy++;
                if (yy >= 100)
                {
                    yy -= 100;
                }
            }
        }
        else if (dd > 30 && (mm == 2 || mm == 4 || mm == 6 || mm == 9 || mm == 11))
        {
            dd -= 30;
            mm++;
        }
    }
};

char *Compass::csCalc(char *buffin, size_t length)
{

    char csDigit = 0;
    for (int i = 1; i < strnlen(buffin, length); i++)
    {
        csDigit ^= buffin[i];
    }
    sprintf(cs, "*%02X", csDigit);
    return cs;
};

void Compass::parse(HMC5883L _compass, QMC5883L _qmc_compass, int compass_selector)
{
    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
//    float declinationAngle = 0.13;
    Vector2D mag;
    char ew_magdec = 'E';
    if (declinationAngle >= 0)
    {
        ew_magdec = 'W';
    }
    else
    {
        ew_magdec = 'E';
    }

    if (compass_selector == 1)
    {
        //Compass data polling
        Vector norm = _compass.readNormalize();

        // Calculate heading
        float heading = (float)atan2(norm.XAxis, norm.YAxis);
        heading += declinationAngle;
        headingDegrees = heading * 180 / PI;
        if(headingDegrees < 0)
        {
            headingDegrees += 360;
        }

        if (headingDegrees > 360)
        {
            headingDegrees -= 360;
        }
    }
    else if (compass_selector == 2)
    {
          int16_t z, t;
        if (_qmc_compass.readRaw(&mag.x, &mag.y, &z, &t) == 0)
        {
            headingDegrees = 0;
        }
        else
        {
            headingDegrees = 180 * (atan2(-mag.x, -mag.y) +declinationAngle)/ PI;
            if(headingDegrees<0)
                headingDegrees+=360;
        }

        //headingDegrees = _qmc_compass.readHeading(); //+declinationAngle*180/PI;
    }
    else
    {
        headingDegrees = 0;
    }

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