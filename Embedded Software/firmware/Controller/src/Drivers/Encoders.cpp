#include "Encoders.h"

/**
 * Constructor
 * Uses an initializer list to pass the pins to the RotaryEncoder object.
 */
Encoders::Encoders(uint8_t pinA, uint8_t pinB) 
    : encoder(pinA, pinB, RotaryEncoder::LatchMode::FOUR3) 
{
    // Note: LatchMode::FOUR3 is common for most HW-040 encoders, 
    // but you can adjust based on your specific hardware.
}

/**
 * Initialize pins
 */
void Encoders::begin() {
    // Most encoder libraries don't require internal pullups if 
    // the breakout board has them, but it's safe to define pins here if needed.
}

/**
 * Updates the encoder state and returns the current position.
 */
int32_t Encoders::getPosition() {
    encoder.tick(); // Checks the pins for state changes
    return (int32_t)encoder.getPosition();
}

/**
 * Resets the internal counter to zero.
 */
void Encoders::reset() {
    encoder.setPosition(0);
}