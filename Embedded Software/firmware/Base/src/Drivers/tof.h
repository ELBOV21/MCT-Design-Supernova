#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <VL53L0X.h>
#define VL53L0X_I2C_ADDR 0x29
class TOFSensor {
public:
    // Constructor: optionally set custom I2C pins (default = -1 means use Wire default)
    TOFSensor(int sda_pin = -1, int scl_pin = -1);

    // Initialize sensor. Returns true if successful.
    bool begin();

    // Read distance in mm. Returns 0xFFFF on error/timeout.
    uint16_t readDistance();

private:
    VL53L0X _sensor;
    int _sda_pin;
    int _scl_pin;
    bool _initialised;
};

#endif