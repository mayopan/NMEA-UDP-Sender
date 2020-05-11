#ifndef Arduino_h
#include <Arduino.h>
#endif

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#include <SparkFun_Ublox_Arduino_Library.h>
#endif

#include "SparkFun_Ublox_Arduino_Library_Ex.h"
SFE_UBLOX_GPS_ADD myGPS;

boolean SFE_UBLOX_GPS_ADD::getModuleInfo(uint16_t maxWait)
{
    myGPS.minfo.hwVersion[0] = NULL;
    myGPS.minfo.swVersion[0] = NULL;
    for (int i = 0; i < 10; i++)
        myGPS.minfo.extension[i][0] = NULL;

    // Let's create our custom packet
    uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes

    // The next line creates and initialises the packet information which wraps around the payload
    ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

    // The structure of ubxPacket is:
    // uint8_t cls           : The message Class
    // uint8_t id            : The message ID
    // uint16_t len          : Length of the payload. Does not include cls, id, or checksum bytes
    // uint16_t counter      : Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
    // uint16_t startingSpot : The counter value needed to go past before we begin recording into payload array
    // uint8_t *payload      : The payload
    // uint8_t checksumA     : Given to us by the module. Checked against the rolling calculated A/B checksums.
    // uint8_t checksumB
    // sfe_ublox_packet_validity_e valid            : Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
    // sfe_ublox_packet_validity_e classAndIDmatch  : Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

    // sendCommand will return:
    // SFE_UBLOX_STATUS_DATA_RECEIVED if the data we requested was read / polled successfully
    // SFE_UBLOX_STATUS_DATA_SENT     if the data we sent was writted successfully (ACK'd)
    // Other values indicate errors. Please see the sfe_ublox_status_e enum for further details.

    // Referring to the u-blox M8 Receiver Description and Protocol Specification we see that
    // the navigation rate is configured using the UBX-CFG-RATE message. So let's load our
    // custom packet with the correct information so we can read (poll / get) the current settings.

    customCfg.cls = UBX_CLASS_MON; // This is the message Class
    customCfg.id = UBX_MON_VER;    // This is the message ID
    customCfg.len = 0;             // Setting the len (length) to zero let's us poll the current settings
    customCfg.startingSpot = 0;    // Always set the startingSpot to zero (unless you really know what you are doing)

    // We also need to tell sendCommand how long it should wait for a reply
    //  uint16_t maxWait = 250; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)

    if (sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
        return (false); //If command send fails then bail

    uint16_t position = 0;
    for (int i = 0; i < 30; i++)
    {
        minfo.swVersion[i] = customPayload[position];
        position++;
    }
    for (int i = 0; i < 10; i++)
    {
        minfo.hwVersion[i] = customPayload[position];
        position++;
    }

    while (customCfg.len >= position + 30)
    {
        for (int i = 0; i < 30; i++)
        {
            minfo.extension[minfo.extensionNo][i] = customPayload[position];
            position++;
        }
        minfo.extensionNo++;
        if (minfo.extensionNo > 9)
            break;
    }

    return (true); //Success!
}