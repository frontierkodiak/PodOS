#include <read_sensors.h>

int read_battery_voltage(int vOutPin) {
    int rawValue = analogRead(vOutPin); // Read the analog voltage value
    float in_voltage = map(rawValue, 0, 4095, 0, 3.3); // Map the analog value to the expected value range

    // Assuming R1 (voltage divider) = 47k ohms, input range = (1.93v, 3v)
    // Adjust the minVoltage and maxVoltage values according to your specific voltage divider circuit
    float minVoltage = 1.93;
    float maxVoltage = 3.0;

    float battery_level = map(in_voltage, minVoltage, maxVoltage, 0, 100); // Map the voltage to battery level (0-100)
    
    Serial.print("Battery level: ");
    Serial.println(battery_level);

    return static_cast<int>(battery_level);
}
