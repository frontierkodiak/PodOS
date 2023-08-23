#ifndef READ_GPS_H
#define READ_GPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <NMEAGPS.h>

class MyGPS {
private:
    HardwareSerial gpsSerial;  // Direct initialization, not a pointer
    NMEAGPS gps;
    gps_fix currentFix;

public:
    // Default constructor
    MyGPS() : gpsSerial(1) { }  // Use initializer list to construct the HardwareSerial

    // Parameterized constructor
    MyGPS(uint8_t rxPin, uint8_t txPin) : gpsSerial(1) {
        init(rxPin, txPin);
    }

    // No need for destructor, so remove it.

    // Method to initialize pins (can be used after default construction)
    void init(uint8_t rxPin, uint8_t txPin) {
        gpsSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
    }

    bool setup();

    gps_fix readFix(); // NOIE: Blocking call, so it must be in Task 2 on second core. Handled correctly in main.cpp but be careful!

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
