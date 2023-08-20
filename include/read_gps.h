#ifndef READ_GPS_H
#define READ_GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <NMEAGPS.h>

class MyGPS {
private:
    HardwareSerial* gpsSerial;
    NMEAGPS gps;
    gps_fix currentFix;

public:
    MyGPS(uint8_t rxPin, uint8_t txPin) {
        gpsSerial = new HardwareSerial(1);
        gpsSerial->begin(9600, SERIAL_8N1, rxPin, txPin);
    }

    void setup();

    gps_fix readFix();

    float getLatitude() {
        return currentFix.latitude();
    }

    float getLongitude() {
        return currentFix.longitude();
    }
    
    float getAltitude() {
        return currentFix.altitude();
    }

};

#endif // READ_GPS_H
