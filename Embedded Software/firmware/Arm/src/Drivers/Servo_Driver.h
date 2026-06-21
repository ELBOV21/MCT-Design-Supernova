#pragma once
#include <Arduino.h>
#include "Config/Config.h"

/**
 * @brief Initialise the PCA9685 PWM servo driver.
 *
 * Must be called once in setup() before any Servo_SetAngle() calls.
 */
void Servo_Init();

/**
 * @brief Command a servo to a target angle.
 *
 * Converts the angle to the corresponding PWM pulse width using the
 * configured min/max pulse-width constants and sends it to the PCA9685.
 *
 * @param servoNum  Channel index on the PCA9685 (0-based)
 * @param angle     Target angle in degrees [0.0 – 180.0]
 */
void Servo_SetAngle(uint8_t servoNum, float angle);
