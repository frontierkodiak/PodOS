#ifndef READ_SENSORS_H
#define READ_SENSORS_H

#include <Arduino.h> // Include Arduino.h to use data types and Serial
#include <Adafruit_BME280.h>
#include <Wire.h>

// Workaround for using BME280 with ESP32-CAM (no SDA/SCL pins available, but second I2C bus available)
#define I2C_SDA 15
#define I2C_SCL 14
TwoWire I2CBME = TwoWire(0); // initialize Two Wire instance in setup()

class MyBME280 {
private:
    Adafruit_BME280 bme; // I2C
    //Adafruit_BME280 bme(BME_CS); // hardware SPI
    //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

public:
    MyBME280() {}
    void setup() {
        Serial.println("Attempting BME280 setup...");
        I2CBME.begin(I2C_SDA, I2C_SCL, 100000); // pass SDA, SCL, and frequency

        if (!bme.begin(0x76, &I2CBME)) // pass address and I2C instance
        {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            while (1);
        }
        else
        {
            Serial.println("BME280 sensor found!");
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
