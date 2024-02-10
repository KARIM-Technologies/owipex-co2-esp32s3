#include "I2CPressSens.h"

TwoWire I2CPressSens::_wireHigh = TwoWire(0);
TwoWire I2CPressSens::_wireLow = TwoWire(1);

const float PRESSURE_RANGE = 500.0; // Beispielwert, anpassen an Ihren Sensor


I2CPressSens::I2CPressSens() {}


void I2CPressSens::begin() {
    _wireHigh.begin(_sdaHigh, _sclHigh); // Initialisiert den Hochdruck-I2C-Bus
    _wireLow.begin(_sdaLow, _sclLow);    // Initialisiert den Niederdruck-I2C-Bus
}

float I2CPressSens::readPressureHigh() {
    return readPressure(_wireHigh);
}

float I2CPressSens::readPressureLow() {
    return readPressure(_wireLow);
}

float I2CPressSens::calculateMass(float temperatureCelsius, float volume) {
    float temperatureKelvin = temperatureCelsius + 273.15; // Umrechnung von °C in Kelvin
    float pressure = readPressureHigh();
    float pressurePa = pressure * 1000.0;
    float volumeM3 = volume / 1000.0;
    return (pressurePa * volumeM3 * M) / (R * temperatureKelvin);
}

float I2CPressSens::estimateFlowRate(float deltaPressure, float temperatureCelsius, float Cv, float SG) {
    float temperatureKelvin = temperatureCelsius + 273.15;
    return Cv * sqrt((SG * deltaPressure) / temperatureKelvin);
}


float I2CPressSens::readPressure(TwoWire &wire) {
    wire.beginTransmission(_deviceAddress);
    wire.write(0x06); // Registeradresse für Druckmessungen, anpassen an Ihren Sensor
    wire.endTransmission(false);
    wire.requestFrom(_deviceAddress, 3);

    if (wire.available() == 3) {
        long dat = wire.read() << 16 | wire.read() << 8 | wire.read();
        if (dat & 0x800000) {
            dat -= 16777216; // Anpassen des 24-bit Zweierkomplement-Werts
        }
        float fadc = dat;
        // Die Berechnung von adc basiert auf einer Referenzspannung von 3.3V; überprüfen Sie dies für Ihren Sensor
        float adc = 3.3 * fadc / 8388608.0;
        // Stellen Sie sicher, dass diese Umrechnung für Ihren Sensor korrekt ist
        float pressure = PRESSURE_RANGE * (adc - 0.5) / 2.0;
        return pressure;
    }
    return NAN; // Falls keine Daten verfügbar
}

float I2CPressSens::readTemperature(TwoWire &wire) {
    wire.beginTransmission(_deviceAddress);
    wire.write(0x09); // Registeradresse für Temperaturmessungen
    wire.endTransmission(false);
    wire.requestFrom(_deviceAddress, 3); // Annahme: Temperaturwert ist 3 Bytes groß

    if (wire.available() == 3) {
        long dat = wire.read() << 16 | wire.read() << 8 | wire.read();
        if (dat & 0x800000) {
        dat = dat - 16777216;
        }
        float fadc = dat;
        float temperature = 25.0 + fadc / 65536.0;
        return temperature;
    }
    return NAN; // Gibt NAN zurück, wenn keine Daten gelesen werden können
}

float I2CPressSens::readTemperatureHigh() {
    // Angenommen, 0x07 ist das Register für die Hochdruck-Temperatur
    return readTemperature(_wireHigh);
}

float I2CPressSens::readTemperatureLow() {
    // Angenommen, 0x07 ist ebenfalls das Register für die Niederdruck-Temperatur
    return readTemperature(_wireLow);
}

