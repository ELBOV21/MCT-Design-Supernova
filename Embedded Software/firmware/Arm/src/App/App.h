#pragma once

// =============================================================================
// App.h  —  Top-level application interface
//
// The application layer owns all firmware logic: hardware init, ESP-NOW
// communication, autonomous sequences, and manual control dispatch.
// =============================================================================

/**
 * @brief Initialise all hardware and communication subsystems.
 *
 * Must be called once from Arduino setup().
 * Initialises Serial, WiFi, ESP-NOW peers + callback, servos, and joint state.
 * Halts with a serial error message if ESP-NOW fails to start.
 */
void App_Init();

/**
 * @brief Execute one application cycle.
 *
 * Must be called repeatedly from Arduino loop().
 * Handles:
 *   1. Autonomous pick-and-place commands from the base car
 *   2. Manual preset position commands from the OLED controller menu
 *   3. Gripper open/close
 *   4. Continuous joint control (rate-limited)
 */
void App_Run();
