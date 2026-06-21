#include "Servo_Driver.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Single global instance of the Adafruit PCA9685 driver
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void Servo_Init()
{
    pwm.begin();
    pwm.setOscillatorFrequency(CFG_PWM_OSCILLATOR_HZ);
    pwm.setPWMFreq(CFG_PWM_FREQ_HZ);
}


void Servo_SetAngle(uint8_t servoNum, float angle)
{
    // Map angle [0°, 180°] → pulse width [CFG_SERVO_PULSE_MIN_US, CFG_SERVO_PULSE_MAX_US]
    uint16_t pulseWidth = static_cast<uint16_t>(
        map(static_cast<long>(angle), 0, 180,
            CFG_SERVO_PULSE_MIN_US, CFG_SERVO_PULSE_MAX_US));

    pwm.writeMicroseconds(servoNum, pulseWidth);
}
