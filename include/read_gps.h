#ifndef READ_GPS_H
#define READ_GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <NMEAGPS.h>

class MyGPS {
private:
    HardwareSerial* gpsSerial = nullptr;
    NMEAGPS gps;
    gps_fix currentFix;

public:
    // Default constructor
    MyGPS() { }

    // Parameterized constructor
    MyGPS(uint8_t rxPin, uint8_t txPin) {
        init(rxPin, txPin);
    }

    // Destructor to free dynamically allocated memory
    ~MyGPS() {
        if(gpsSerial) {
            delete gpsSerial;
        }
    }

    // Method to initialize pins (can be used after default construction)
    void init(uint8_t rxPin, uint8_t txPin) {
        if(!gpsSerial) { // Ensure we don't leak memory by reassigning
            gpsSerial = new HardwareSerial(1);
            gpsSerial->begin(9600, SERIAL_8N1, rxPin, txPin);
        }
    }

    bool setup();

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
