#ifndef IMU_H
#define IMU_H

void mpuSetup();
void mpuCalibration();
void mpuMadgwickFilter();
void mpuMahonyFilter();
float mpuGetYaw();
void print_calibration();
void mpuSaveCalibrationData();
void mpuLoadCalibrationData();

float angleToTarget(float targetYaw);
void resetYawReference();


float getfilteredYaw();

#endif // IMU_H 