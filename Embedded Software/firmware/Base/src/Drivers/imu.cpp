// #include <Wire.h>
// #include "imu.h"
// #include "MPU9250.h"
// #include <HardwareSerial.h>
// #include <Preferences.h>
// #define MPU_ADDR 0x68
// Preferences prefs;
// MPU9250 mpu;
// float static yaw_offset = 0.0; // To store the initial yaw reference


// void mpuSetup() {
//     if (!mpu.setup(MPU_ADDR)) {
//         Serial.println("MPU9250 connection failed");
//         while (1) {}
//     }
//     mpu.setMagneticDeclination(4.46);    
// }    
// void mpuCalibration() {
//     Serial.println("Accel Gyro calibration will start in 5sec.");
//     Serial.println("Please leave the device still on the flat plane.");
//     mpu.verbose(true);
//     delay(5000);
//     mpu.calibrateAccelGyro();

//     Serial.println("Mag calibration will start in 5sec.");
//     Serial.println("Please Wave device in a figure eight until done.");
//     delay(5000);
//     mpu.calibrateMag();

//     print_calibration();
//     mpu.verbose(false);
// }    

// void mpuMadgwickFilter()
// {
//     mpu.selectFilter(QuatFilterSel::MADGWICK);
//     mpu.setFilterIterations(15);
// }    

// void mpuMahonyFilter()
// {
//     mpu.selectFilter(QuatFilterSel::MAHONY);
// }    

// void mpuSaveCalibrationData(){
//     mpuCalibration();
//     prefs.begin("mpu", false);
//     prefs.putFloat("acc_bias_x", mpu.getAccBiasX());
//     prefs.putFloat("acc_bias_y", mpu.getAccBiasY());   
//     prefs.putFloat("acc_bias_z", mpu.getAccBiasZ());
//     prefs.putFloat("gyro_bias_x", mpu.getGyroBiasX());
//     prefs.putFloat("gyro_bias_y", mpu.getGyroBiasY());
//     prefs.putFloat("gyro_bias_z", mpu.getGyroBiasZ());
//     prefs.putFloat("mag_bias_x", mpu.getMagBiasX());
//     prefs.putFloat("mag_bias_y", mpu.getMagBiasY());
//     prefs.putFloat("mag_bias_z", mpu.getMagBiasZ());
//     prefs.putFloat("mag_scale_x", mpu.getMagScaleX());
//     prefs.putFloat("mag_scale_y", mpu.getMagScaleY());
//     prefs.putFloat("mag_scale_z", mpu.getMagScaleZ());
//     prefs.end();
//     Serial.println("Calibration saved!");
// }
// void mpuLoadCalibrationData(){
//     prefs.begin("mpu", true);
//     mpu.setAccBias(prefs.getFloat("acc_bias_x", 0.0f),
//                    prefs.getFloat("acc_bias_y", 0.0f),   
//                    prefs.getFloat("acc_bias_z", 0.0f));
//     mpu.setGyroBias(prefs.getFloat("gyro_bias_x", 0.0f),
//                     prefs.getFloat("gyro_bias_y", 0.0f),
//                     prefs.getFloat("gyro_bias_z", 0.0f));
//     mpu.setMagBias(prefs.getFloat("mag_bias_x", 0.0f),
//                    prefs.getFloat("mag_bias_y", 0.0f),
//                    prefs.getFloat("mag_bias_z", 0.0f));
//     mpu.setMagScale(prefs.getFloat("mag_scale_x", 1.0f),
//                     prefs.getFloat("mag_scale_y", 1.0f),
//                     prefs.getFloat("mag_scale_z", 1.0f));
//     prefs.end();
//     Serial.println("Calibration loaded!");
// }
// float mpuGetYaw(){
//     static uint32_t prev_ms = millis(); 
//      if (mpu.update()) {
//         if (millis() > prev_ms + 25) {
//             prev_ms = millis();
//         }   
//     } 
//     float yaw = mpu.getYaw()-yaw_offset; // Get yaw relative to the initial reference
//     if(yaw <0) yaw += 360.0; // Wrap yaw to [0, 360)
//     Serial.print("Yaw: ");
//     Serial.println(yaw);

//     return yaw; // Return yaw relative to the initial reference
// }    


// float angleToTarget(float targetYaw)
// {
//     float currentYaw = mpuGetYaw();    
//     float error = targetYaw - currentYaw;
//     return error;
// }
// void resetYawReference() {
//     yaw_offset = mpu.getYaw();    
// } 


// void print_calibration() {
//     Serial.println("< calibration parameters >");
//     Serial.println("accel bias [g]: ");
//     Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//     Serial.print(", ");
//     Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//     Serial.print(", ");
//     Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//     Serial.println();
//     Serial.println("gyro bias [deg/s]: ");
//     Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//     Serial.println();
//     Serial.println("mag bias [mG]: ");
//     Serial.print(mpu.getMagBiasX());
//     Serial.print(", ");
//     Serial.print(mpu.getMagBiasY());
//     Serial.print(", ");
//     Serial.print(mpu.getMagBiasZ());
//     Serial.println();
//     Serial.println("mag scale []: ");
//     Serial.print(mpu.getMagScaleX());
//     Serial.print(", ");
//     Serial.print(mpu.getMagScaleY());
//     Serial.print(", ");
//     Serial.print(mpu.getMagScaleZ());
//     Serial.println();
// }
