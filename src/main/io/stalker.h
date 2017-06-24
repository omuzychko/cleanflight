#pragma once

#include "common/time.h"

typedef struct stalkerSource3D_s{
    // sensor output angles
    float a; // angle returned by X sensor
    float b; // angle returned by Y sensor
    float c; // angle returned by Z sensor
    
    // cartesian coordinates
    float x;
    float y;
    float z;
} stalkerSource3D_t;


typedef struct stalkerTargetRAW_s {
    stalkerSource3D_t src0;
    stalkerSource3D_t src1;
    stalkerSource3D_t src2;
} stalkerTargetRAW_t;

typedef struct stalkerTargetUAV_s {
    // Averaged value of "45 deg - atan(X/Y)" for all given sources
    // thousands of a radian (~0.0573 degree)
    int16_t azimuth;
  
    // Averged value of the Z-angle sensor for all given sources
    // thousands of a radian (~0.0573 degree)
    int16_t elevation;
  
    // Averaged value of "45 deg - atan(X/Y)" for all given sources
    // thousands of a radian (~0.0573 degree)
    int16_t headingAzimuth;
    
    // Averged value of the Z-angle sensor for all given sources
    // thousands of a radian (~0.0573 degree)
    int16_t headingElevation;
  
    // Averaged cartesian Z value for all given sources
    // in millimeters
    int16_t altitude;
  
    // Averaged sqrt(X^2 + Y^2) for all given soruces
    // in millimeters
    int16_t distance;
} stalkerTargetUAV_t;

typedef struct stalkerConfig_s {
    uint16_t target_distance;
    uint8_t target_deadband;
    uint8_t throttle_filter;
    uint8_t distance_rate;
    uint8_t heading_rate;
    uint8_t rollyaw_ratio;
} stalkerConfig_t;

extern stalkerTargetRAW_t  STALKER_TARGET_RAW;
extern stalkerTargetUAV_t  STALKER_TARGET_UAV;

PG_DECLARE(stalkerConfig_t, stalkerConfig);

void stalkerInit(void);
void stalkerUpdate(timeUs_t currentTimeUs);