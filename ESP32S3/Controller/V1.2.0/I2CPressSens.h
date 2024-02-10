#ifndef I2CPressSens_h
#define I2CPressSens_h

#include "Arduino.h"
#include <Wire.h>

class I2CPressSens {
public:
    I2CPressSens();
    void begin();
    float readPressureHigh();
    float readPressureLow();
    float calculateMass(float temperatureCelsius, float volume);
    float estimateFlowRate(float deltaPressure, float temperatureCelsius, float Cv, float SG = 1.0);
    float readTemperatureHigh();
    float readTemperatureLow();

private:
    static TwoWire _wireHigh;
    static TwoWire _wireLow;
    static const uint8_t _deviceAddress = 0x6D; // Fest definierte Sensoradresse
    static const uint8_t _sdaHigh = 4; // SDA-Pin für Hochdruck-I2C-Bus
    static const uint8_t _sclHigh = 5; // SCL-Pin für Hochdruck-I2C-Bus
    static const uint8_t _sdaLow = 6;  // SDA-Pin für Niederdruck-I2C-Bus
    static const uint8_t _sclLow = 7;  // SCL-Pin für Niederdruck-I2C-Bus
    float readPressure(TwoWire &wire);
    const float R = 8.314; // J/(mol·K)
    const float M = 0.04401; // kg/mol
    float readTemperature(TwoWire &wire);
};

#endif
