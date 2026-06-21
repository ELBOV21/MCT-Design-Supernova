#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "MPU9250.h" 
#include "HiwonderMotorDriver.h"
#include "Kinematics.h"

#define TICKS_PER_REV (11.0 * 4.0 * 34.0) 
#define RAD_PER_TICK ((2.0 * PI) / TICKS_PER_REV)

// Tuning Parameters 
#define KP_X 1.5             
#define KI_X 0.5   // ADDED: Integral term for X to prevent stopping short
#define KP_Y 1.5          
#define KI_Y 0.5   // ADDED: Integral term for Y to prevent stopping short
#define KP_YAW 0.5        
#define KD_YAW 0.5        
#define KI_YAW 1.2        

#define TOLERANCE_XY 0.02 
#define TOLERANCE_YAW 0.05 

// UPDATED: Changed from 0.1 to -0.1 (tune this based on your physical tests)
#define Y_TO_X_DRIFT_COMPENSATION 0.0

class Navigator {
private:
    HiwonderMotorDriver* driver;
    Kinematics* kinematics;
    MPU9250* mpu;
    
    int32_t prev_enc[4];
    float current_yaw_rads;
    unsigned long last_imu_time;
    float gyro_z_bias;
   float global_x;
    float global_y;
    float move_local_x; 
    float move_local_y;

public:
    float updateIMUYaw();
    void calibrateGyro();
    void updateOdometry(); 
    void resetGlobalOdometry();
    Navigator(HiwonderMotorDriver* drv, Kinematics* kin, MPU9250* imu_ptr);
    void move_distance(float target_x, float target_y);
    void rotate_angle(float target_yaw_relative);
};

#endif