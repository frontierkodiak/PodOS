#ifndef READ_SENSORS_H
#define READ_SENSORS_H

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

// If I2CBME is a two-wire instance, declare it here, otherwise remove or replace with the correct instance.
TwoWire I2CBME = TwoWire(0); 

class MyBME280 {
  private:
    Adafruit_BME280 bme;
    int sdaPin, sclPin;

  public:
    // Default constructor
    MyBME280() {
        // Optionally set default values or leave them uninitialized
        sdaPin = -1;
        sclPin = -1;
    }

    // Parameterized constructor
    MyBME280(int sda, int scl) : sdaPin(sda), sclPin(scl) {}

    // Optionally add a method to initialize pins later if using the default constructor
    void setPins(int sda, int scl) {
        sdaPin = sda;
        sclPin = scl;
    }

    bool setup() {
      if(sdaPin == -1 || sclPin == -1) {
          Serial.println("BME280 pins not set!");
          return false;  // Indicate failure
      }

      Serial.println("Attempting BME280 setup...");
      I2CBME.begin(sdaPin, sclPin, 100000);

      if (!bme.begin(0x76, &I2CBME)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return false;  // Indicate failure
      } else {
        Serial.println("BME280 sensor found!");
        return true;  // Indicate success
      }
    }

    float readTemperature() {
        return 1.8 * bme.readTemperature() + 32;
    }

    float readHumidity() {
        return bme.readHumidity();
    }

    float getTemperature() {
        return 1.8 * bme.readTemperature() + 32;
    }

    float getHumidity() {
        return bme.readHumidity();
    }
    
    float getPressure() {
        return bme.readPressure() / 100.0F;
    }
};

int read_battery_voltage(int vOutPin);



#endif // READ_SENSORS_H
