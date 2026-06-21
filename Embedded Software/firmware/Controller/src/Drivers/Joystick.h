#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

class Joystick {
private:
    int _pinX;
    int _pinY;
    int _pinBtn;
    bool _hasButton;

    // State variables for non-blocking debounce
    bool _btnState;
    bool _lastBtnState;
    unsigned long _lastDebounceTime;
    const unsigned long _debounceDelay = 50; // 50ms delay

public:
    // Constructor. pinBtn is optional; defaults to -1 if not connected.
    Joystick(int pinX, int pinY, int pinBtn = -1) {
        _pinX = pinX;
        _pinY = pinY;
        _pinBtn = pinBtn;
        _hasButton = (pinBtn != -1);
        
        _btnState = HIGH;
        _lastBtnState = HIGH;
        _lastDebounceTime = 0;
    }

    // Initialize the pins
    void begin() {
        pinMode(_pinX, INPUT);
        pinMode(_pinY, INPUT);
        if (_hasButton) {
            pinMode(_pinBtn, INPUT_PULLUP);
        }
    }

    // Get raw analog reading for X axis (0 - 4095 for ESP32)
    int getRawX() {
        return analogRead(_pinX);
    }

    // Get raw analog reading for Y axis (0 - 4095 for ESP32)
    int getRawY() {
        return analogRead(_pinY);
    }

    // Returns true ONLY once per press (falling edge detected)
    bool wasPressed() {
        if (!_hasButton) return false;

        bool pressed = false;
        int reading = digitalRead(_pinBtn);

        // Reset debounce timer if the switch changed due to noise or pressing
        if (reading != _lastBtnState) {
            _lastDebounceTime = millis();
        }

        // Whatever the reading is at, it's been there for longer than the debounce delay
        if ((millis() - _lastDebounceTime) > _debounceDelay) {
            // If the button state has changed
            if (reading != _btnState) {
                _btnState = reading;

                // Only trigger on the falling edge (HIGH to LOW)
                if (_btnState == LOW) {
                    pressed = true;
                }
            }
        }
        
        _lastBtnState = reading;
        return pressed;
    }
};

#endif