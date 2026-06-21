#include "tof.h"

TOFSensor::TOFSensor(int sda_pin, int scl_pin)
    : _sda_pin(sda_pin), _scl_pin(scl_pin), _initialised(false) {}

bool TOFSensor::begin() {
    if (_sda_pin != -1 && _scl_pin != -1) {
        Wire.begin(_sda_pin, _scl_pin);
    } else {
        Wire.begin();
    }

    Wire.beginTransmission(VL53L0X_I2C_ADDR);
    if (Wire.endTransmission() != 0) return false;

    if (!_sensor.init()) return false;

    _sensor.setTimeout(500);
    // for high accuracy
    _sensor.setMeasurementTimingBudget(200000);

    _initialised = true;
    return true;
}

uint16_t TOFSensor::readDistance() {
    if (!_initialised) return 0xFFFF;

    uint16_t dist = _sensor.readRangeSingleMillimeters();
    if (_sensor.timeoutOccurred()) return 0xFFFF;
    return dist;
}