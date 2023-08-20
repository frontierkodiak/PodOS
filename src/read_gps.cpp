#include "read_gps.h"

void MyGPS::setup() {
    Serial.println("Initializing GPS...");
}

gps_fix MyGPS::readFix() {
    while (gps.available(*gpsSerial)) {
        currentFix = gps.read();
    }
    return currentFix;
}
