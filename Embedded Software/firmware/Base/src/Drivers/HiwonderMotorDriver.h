#pragma once

#include <Arduino.h>
#include <Wire.h>

class HiwonderMotorDriver {
public:
    // Motor types based on the datasheet
    enum MotorType {
        NO_ENCODER = 0,
        TT_ENCODER = 1,
        N20_ENCODER = 2,
        JGB37_520 = 3
    };

    // Constructor accepts a TwoWire instance and the I2C address
    HiwonderMotorDriver(TwoWire& i2cBus = Wire, uint8_t address = 0x34);

    // Initialization
    bool begin(MotorType type = JGB37_520, uint8_t polarity = 0);

    // Hardware reads
    uint16_t getBatteryVoltage();
    bool readEncoders(int32_t& enc1, int32_t& enc2, int32_t& enc3, int32_t& enc4);
    
    // Motor Control
    // open-loop PWM control, range: -100 to 100
    void setPWM(int8_t m1, int8_t m2, int8_t m3, int8_t m4); 
    // closed-loop speed control
    void setSpeed(int8_t speeds[4]);
    //void setSpeed(int8_t m1, int8_t m2, int8_t m3, int8_t m4);
    void stopAll();
    void resetEncoders();

private:
    TwoWire& _i2c;
    uint8_t _address;

    // Registers
    static const uint8_t REG_ADC_BAT = 0x00;
    static const uint8_t REG_MOTOR_TYPE = 0x14;
    static const uint8_t REG_ENCODER_POLARITY = 0x15;
    static const uint8_t REG_FIXED_PWM = 0x1F;
    static const uint8_t REG_FIXED_SPEED = 0x33;
    static const uint8_t REG_ENCODER_TOTAL = 0x3C;

    // Low-level I2C helpers
    bool writeBytes(uint8_t reg, uint8_t* data, size_t len);
    bool readBytes(uint8_t reg, uint8_t* data, size_t len);
};