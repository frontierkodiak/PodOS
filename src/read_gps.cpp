#include "read_gps.h"

bool MyGPS::setup() {
    Serial.println("Initializing GPS...");
    // TODO: Add logic here to check if the GPS initialization succeeded
    // Not much checking to do, just make sure communication is working, we've selected right baud rate / pins
    return true; 
}


gps_fix MyGPS::readFix() {
    while (gps.available(*gpsSerial)) {
        currentFix = gps.read();
    }
    return currentFix;
}
