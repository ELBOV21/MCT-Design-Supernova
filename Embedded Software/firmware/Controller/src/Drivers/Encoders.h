#pragma once

#include <Arduino.h>   
#include <RotaryEncoder.h>

class Encoders
{
public:
    Encoders(uint8_t pinA, uint8_t pinB);
    void begin();
    int32_t getPosition();
    void reset();
private:
    RotaryEncoder encoder;
};