#include "HiwonderMotorDriver.h"
#define SPEED_CONVERSION 4
HiwonderMotorDriver::HiwonderMotorDriver(TwoWire& i2cBus, uint8_t address) 
    : _i2c(i2cBus), _address(address) {}

bool HiwonderMotorDriver::begin(MotorType type, uint8_t polarity) {
    uint8_t t = static_cast<uint8_t>(type);
    
    
    if (!writeBytes(REG_MOTOR_TYPE, &t, 1)) return false;
    delay(10);
    
    // Set encoder polarity. Make sure to set to 0, otherwise motor may keep rotating 
    return writeBytes(REG_ENCODER_POLARITY, &polarity, 1);
}

uint16_t HiwonderMotorDriver::getBatteryVoltage() {
    uint8_t data[2] = {0, 0};
    if (readBytes(REG_ADC_BAT, data, 2)) {
        return data[0] | (data[1] << 8); // Combine LSB and MSB 
    }
    return 0;
}

void HiwonderMotorDriver::setPWM(int8_t m1, int8_t m2, int8_t m3, int8_t m4) {
    int8_t speeds[4] = {m1, m2, m3, m4};
    writeBytes(REG_FIXED_PWM, (uint8_t*)speeds, 4); 
}

void HiwonderMotorDriver::setSpeed(int8_t speeds[4]) {
   for(int i=0;i<4;i++){
    speeds[i] /= SPEED_CONVERSION;
   }
    writeBytes(REG_FIXED_SPEED, (uint8_t*)speeds, 4); // Closed-loop control 
}

void HiwonderMotorDriver::stopAll() {
    int8_t zeroSpeeds[4] = {0, 0, 0, 0};
    setSpeed(zeroSpeeds);
}

bool HiwonderMotorDriver::readEncoders(int32_t& enc1, int32_t& enc2, int32_t& enc3, int32_t& enc4) {
    int32_t data[4] = {0, 0, 0, 0};
    
    
    if (readBytes(REG_ENCODER_TOTAL, (uint8_t*)data, 16)) {
        enc1 = data[0];
        enc2 = data[1];
        enc3 = data[2];
        enc4 = data[3];
        return true;
    }
    return false;
}

void HiwonderMotorDriver::resetEncoders() {
    uint8_t resetData[16] = {0};
    
    writeBytes(REG_ENCODER_TOTAL, resetData, 16); 
}

// --- Private I2C Wrappers ---

bool HiwonderMotorDriver::writeBytes(uint8_t reg, uint8_t* data, size_t len) {
    _i2c.beginTransmission(_address);
    _i2c.write(reg);
    for (size_t i = 0; i < len; i++) {
        _i2c.write(data[i]);
    }
    return (_i2c.endTransmission() == 0);
}

bool HiwonderMotorDriver::readBytes(uint8_t reg, uint8_t* data, size_t len) {
    _i2c.beginTransmission(_address);
    _i2c.write(reg);
    if (_i2c.endTransmission() != 0) return false;

    _i2c.requestFrom(_address, (uint8_t)len);
    size_t i = 0;
    while (_i2c.available() && i < len) {
        data[i++] = _i2c.read();
    }
    return (i == len);
}