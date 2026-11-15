// Copyright 2026 Naman Harshwal
// Licensed under the Apache License, Version 2.0
// C/C++ Linux User-Space Driver for ST VL53L0X Time-of-Flight Sensor
#ifndef VL53L0X_SENSOR_CPP__TOF_H_
#define VL53L0X_SENSOR_CPP__TOF_H_

#ifdef __cplusplus
extern "C" {
#endif

int tofInit(int iChan, int iAddr, int bLongRange);
int tofGetModel(int *model, int *revision);
int tofReadDistance(void);

#ifdef __cplusplus
}
#endif

#endif  // VL53L0X_SENSOR_CPP__TOF_H_
