#include "Navigator.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "GUI.h" 

// Externs to hook into the global telemetry and synchronization systems
extern SemaphoreHandle_t i2c_mutex;
extern GUI_par LiveValues;

Navigator::Navigator(HiwonderMotorDriver *drv, Kinematics *kin, MPU9250 *imu_ptr) {
    driver = drv;
    kinematics = kin;
    mpu = imu_ptr;
    for (int i = 0; i < 4; i++) prev_enc[i] = 0;
    current_yaw_rads = 0.0;
    last_imu_time = 0;
    gyro_z_bias = 0.0;
    global_x = 0.0; 
    global_y = 0.0;
    move_local_x = 0.0; 
    move_local_y = 0.0;
}
void Navigator::resetGlobalOdometry() {
    global_x = 0.0;
    global_y = 0.0;
}
void Navigator::updateOdometry() {
    int32_t current_enc[4];
    
    // 1. Safely read encoders
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        driver->readEncoders(current_enc[0], current_enc[1], current_enc[2], current_enc[3]);
        LiveValues.encoder1 = current_enc[0];
        LiveValues.encoder2 = current_enc[1];
        LiveValues.encoder3 = current_enc[2];
        LiveValues.encoder4 = current_enc[3];
        xSemaphoreGive(i2c_mutex);
    }
// 2. Calculate local deltas
    float delta_rad[4];
    for (int i = 0; i < 4; i++) {
        int32_t delta_ticks = -(current_enc[i] - prev_enc[i]);
        delta_rad[i] = delta_ticks * RAD_PER_TICK;
        prev_enc[i] = current_enc[i];
    }

    float r = kinematics->getWheelRadius();
    float delta_x_local = (r / 4.0) * (delta_rad[1] + delta_rad[0] + delta_rad[3] + delta_rad[2]);
    float delta_y_local = (r / 4.0) * (-delta_rad[1] + delta_rad[0] + delta_rad[3] - delta_rad[2]);

    // 3. Accumulate local trackers for autonomous move_distance PID
    move_local_x += delta_x_local;
    move_local_y += delta_y_local;

    // 4. Rotate to Global Map Frame using current yaw
    float yaw = current_yaw_rads; 
    float delta_x_global = (delta_x_local * cos(yaw)) - (delta_y_local * sin(yaw));
    float delta_y_global = (delta_x_local * sin(yaw)) + (delta_y_local * cos(yaw));

    // 5. Update Global Position
    global_x += delta_x_global;
    global_y += delta_y_global;

    // 6. Push to GUI
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        LiveValues.x = global_x;
        LiveValues.y = global_y;
        xSemaphoreGive(i2c_mutex);
    }
}
// ====================================================================
// GYRO CALIBRATION (Run once at startup while robot is completely still)
// ====================================================================
void Navigator::calibrateGyro() {
    Serial.println("--- Calibrating Gyroscope: DO NOT MOVE THE ROBOT ---");
    float total_bias = 0.0;
    const int sample_count = 500; // Take 500 readings

    for (int i = 0; i < sample_count; i++) {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            mpu->update();
            total_bias += mpu->getGyroZ();
            xSemaphoreGive(i2c_mutex);
        }
        delay(2); 
    }

    // Calculate the average resting offset
    gyro_z_bias = total_bias / (float)sample_count;
    Serial.printf("Gyro Calibration Complete! Bias: %.4f dps\n", gyro_z_bias);
}
// ====================================================================
// SENSOR: GYROSCOPE ONLY (With I2C Mutex Protection)
// ====================================================================
float Navigator::updateIMUYaw() {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        mpu->update(); 
        LiveValues.yaw = mpu->getYaw();
        LiveValues.pitch = mpu->getPitch();
        LiveValues.roll = mpu->getRoll();
        xSemaphoreGive(i2c_mutex);
    }
    
    unsigned long current_time = micros();
    float dt = (current_time - last_imu_time) / 1000000.0;
    
    if (dt <= 0.0 || dt > 0.5) dt = 0.02; 
    last_imu_time = current_time;
    
    float raw_gyro_z = mpu->getGyroZ();
    float clean_gyro_z_dps = raw_gyro_z - gyro_z_bias; 
    
    if (abs(clean_gyro_z_dps) < 0.15) { 
        clean_gyro_z_dps = 0.0; 
    }
    
    float gyro_z_rads = clean_gyro_z_dps * (PI / 180.0);
    current_yaw_rads += gyro_z_rads * dt; 
    
    LiveValues.yaw = current_yaw_rads * (180.0 / PI); 
    
    return current_yaw_rads;
}

// ====================================================================
// FORWARD/STRAFE (With Live GUI Updates & Mutex)
// ====================================================================
void Navigator::move_distance(float target_x, float target_y) {
    target_y *= 1.17; 

    // 🌟 REMOVED driver->resetEncoders(). We just reset the local software trackers.
    move_local_x = 0.0;
    move_local_y = 0.0;

    last_imu_time = micros(); 
    float target_yaw = updateIMUYaw(); 

    double current_cmd_vx = 0.0;
    double current_cmd_vy = 0.0;
    float prev_vel_x = 0.0;
    unsigned long prev_time = micros(); 
    
    float integral_x = 0.0;
    float integral_y = 0.0;

    const double MAX_ACCEL_STEP = 0.015; 
    const float SLIP_ACCEL_THRESHOLD = 1.5; 
    
    Serial.println("--- Starting Gyro-Locked Straight Move with Traction Control ---");

    while (true) {
        
        // 🌟 ALL ENCODER READING AND MATH IS NOW HANDLED BY THIS SINGLE CALL
        updateOdometry();
        float current_yaw = updateIMUYaw();
        
        unsigned long current_time = micros();
        float dt = (current_time - prev_time) / 1000000.0;
        if (dt <= 0.001) dt = 0.02; 
        prev_time = current_time;

        // Calculate Errors using our local accumulators
        float error_x = target_x - move_local_x;
        float error_y = target_y - move_local_y;
        float error_yaw = target_yaw - current_yaw;

        while (error_yaw > PI)  error_yaw -= 2.0 * PI;
        while (error_yaw < -PI) error_yaw += 2.0 * PI;

        if (abs(error_x) < TOLERANCE_XY && abs(error_y) < TOLERANCE_XY && abs(error_yaw) < TOLERANCE_YAW) {
            int8_t car_stop[4] = {0, 0, 0, 0};
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                driver->setSpeed(car_stop);
                xSemaphoreGive(i2c_mutex);
            }
            Serial.printf("Target Reached! Global X: %.3f, Global Y: %.3f\n", global_x, global_y);
            break;
        }

        if (abs(error_x) < 0.2) integral_x += error_x * dt;
        else integral_x = 0.0;
        
        if (abs(error_y) < 0.2) integral_y += error_y * dt;
        else integral_y = 0.0;

        double raw_cmd_vx = (error_x * KP_X) + (integral_x * KI_X);
        double raw_cmd_vy = (error_y * KP_Y) + (integral_y * KI_Y);
        double cmd_w  = error_yaw * KP_YAW;

        raw_cmd_vx += (raw_cmd_vy * Y_TO_X_DRIFT_COMPENSATION);

        // ... SLEW RATE LIMITING ...
        if (raw_cmd_vx > current_cmd_vx + MAX_ACCEL_STEP) current_cmd_vx += MAX_ACCEL_STEP;
        else if (raw_cmd_vx < current_cmd_vx - MAX_ACCEL_STEP) current_cmd_vx -= MAX_ACCEL_STEP;
        else current_cmd_vx = raw_cmd_vx;

        if (raw_cmd_vy > current_cmd_vy + MAX_ACCEL_STEP) current_cmd_vy += MAX_ACCEL_STEP;
        else if (raw_cmd_vy < current_cmd_vy - MAX_ACCEL_STEP) current_cmd_vy -= MAX_ACCEL_STEP;
        else current_cmd_vy = raw_cmd_vy;

        current_cmd_vx = constrain(current_cmd_vx, -0.4, 0.4);
        current_cmd_vy = constrain(current_cmd_vy, -0.4, 0.4);
        cmd_w  = constrain(cmd_w, -0.5, 0.5);

        double motor_speeds_rads[4];
        kinematics->calculateMotorSpeeds(current_cmd_vx, current_cmd_vy, cmd_w, motor_speeds_rads);
        
        int8_t driver_speeds[4];
        for (int i = 0; i < 4; i++) {
            int scaled_speed = (int)(motor_speeds_rads[i] * -40.0);
            if (abs(scaled_speed) < 2) scaled_speed = 0;
            scaled_speed = constrain(scaled_speed, -100, 100);
            driver_speeds[i] = (int8_t)scaled_speed;
        }

        // Safely dispatch motor commands AND update GUI velocities
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            LiveValues.vx = current_cmd_vx;
            LiveValues.vy = current_cmd_vy;
            LiveValues.omega = cmd_w;
            driver->setSpeed(driver_speeds);
            xSemaphoreGive(i2c_mutex);
        }
        
        delay(10);
    }
}

// ====================================================================
// ROTATION
// ====================================================================
void Navigator::rotate_angle(float target_yaw_relative) {
    last_imu_time = micros(); 
    float start_yaw = updateIMUYaw(); 
    float final_target_yaw = start_yaw + target_yaw_relative;

    float prev_error_yaw = target_yaw_relative; 
    unsigned long prev_time = micros(); 
    float integral_yaw = 0.0; 

    Serial.printf("--- Starting Gyro-Only Rotation: Target %.3f rad ---\n", final_target_yaw);

    while (true) {
        float current_yaw = updateIMUYaw();
        float error_yaw = final_target_yaw - current_yaw; 

        while (error_yaw > PI)  error_yaw -= 2.0 * PI;
        while (error_yaw < -PI) error_yaw += 2.0 * PI;

        if (abs(error_yaw) < TOLERANCE_YAW) {
            int8_t car_stop[4] = {0, 0, 0, 0};
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                driver->setSpeed(car_stop); 
                xSemaphoreGive(i2c_mutex);
            }
            Serial.printf("Rotation Complete! Final GyroYaw: %.3f\n", current_yaw);
            break;
        }

        unsigned long current_time = micros(); 
        float dt = (current_time - prev_time) / 1000000.0; 
        if (dt <= 0.0) dt = 0.001; 
        prev_time = current_time;

        float derivative_yaw = (error_yaw - prev_error_yaw) / dt;
        prev_error_yaw = error_yaw;

        if (abs(error_yaw) < 0.3) {
            integral_yaw += error_yaw * dt;
        } else {
            integral_yaw = 0.0; 
        }

        double cmd_w  = (error_yaw * KP_YAW) + (integral_yaw * KI_YAW) + (derivative_yaw * KD_YAW);
        cmd_w = constrain(cmd_w, -0.5, 0.5);  
        
        double motor_speeds_rads[4];
        kinematics->calculateMotorSpeeds(0.0, 0.0, cmd_w, motor_speeds_rads);

        int8_t driver_speeds[4];
        for(int i=0; i<4; i++){
             int scaled_speed = (int)(motor_speeds_rads[i] * -40.0); 
             if (abs(scaled_speed) < 2) scaled_speed = 0; 
             scaled_speed = constrain(scaled_speed, -100, 100);
             driver_speeds[i] = (int8_t)scaled_speed; 
        }

        // Safely dispatch motor commands
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            driver->setSpeed(driver_speeds);
            xSemaphoreGive(i2c_mutex);
        }
        
        delay(10); 
    }
}