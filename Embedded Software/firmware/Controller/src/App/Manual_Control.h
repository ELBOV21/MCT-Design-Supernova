#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <Arduino.h>


void Car_Control_Init();


void vCarControlTask(void *pvParameters);

#endif // CAR_CONTROL_H