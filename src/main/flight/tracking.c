/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/tracking.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/stalker.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#define TRACKING_SETPOINT_LIMIT     (1440.0f)
#define TRACKING_SENSOR_PITCH_SIN   (0.577350269f)
#define TRACKING_SENSOR_PITCH_COS   (0.816496581f)
#define TRACKING_SENSOR_PITCH_TAN   (0.707106781f)
#define TRACKING_SENSOR_PITCH_RAD   (0.615479708f)
#define DEG                         (180.0f/M_PIf)
#define SIN_MAX_PITCH_ANGLE         (0.8660f) // the SIN of 60 DEG - maximum Pitch angle "by math model" for distance control

float TRACKING_setpoint[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0.0f, 0.0f, 0.0f }; 

typedef struct {
    float integrator;     // integrator value
    float derivative;     // last derivative calculation
    float error;          // last input for derivative
    float trim;
} PID;

typedef struct {
    float kP;
    float kI;
    float kD;
    float fD;
    float iMin;
    float iMax;
    float initialTrim;  // this is used to attenuate P and I terms while inital lock on target
} PID_PARAM;

static PID_PARAM pidParamRoll;
static PID_PARAM pidParamPitch;
static PID_PARAM pidParamYaw;
static PID_PARAM pidParamThrottle;

static PID pidRoll      = {0,0,0,0};
static PID pidPitch     = {0,0,0,0};
static PID pidYaw       = {0,0,0,0};
static PID pidThrottle  = {0,0,0,0};

static float targetDistance;
static float targetAltitude;
static float deadbandDistance;
static float deadbandAltitude;
static float ratePitch;
static float rateRoll;
static float ratioRollToYaw;

static float setpointRoll;
static float setpointPitch;
static float setpointYaw;
static float setpointThrottle;
static int16_t originalThrottleRcData;

static bool targetLocked;
static bool pitchLocked;
static bool rollLocked;



static int32_t get_P(float error, PID *pid, PID_PARAM *pid_param)
{
    return pid_param->kP * pid->trim * error;
}

static int32_t get_I(float error, float dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += pid->trim * error * dt;
    pid->integrator = constrainf(pid->integrator, pid_param -> iMin, pid_param -> iMax);
    return pid_param->kI * pid->integrator;
}

static int32_t get_D(float error, float dt, PID *pid, PID_PARAM *pid_param)
{
    float d = (error - pid->error) / dt;

    // low pass filter for 10 samples (250 ms)
    pid->derivative = pid->derivative + (d - pid->derivative)/pid_param->fD;
    // remember last error
    pid->error = error;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid,  PID_PARAM * pidParam)
{
    pid->integrator = 0;
    pid->error = 0;
    pid->derivative = 0;
    pid->trim = pidParam -> initialTrim;
}

static void LoadPidParam(pid8_t pidProfile, PID_PARAM * pidParam) {
    pidParam->kP = pidProfile.P;
    pidParam->kI = pidProfile.I;
    pidParam->kD = pidProfile.D;
}

static void trackingCleanup(void) {
    uint32_t i;
    for (i = 0; i < FLIGHT_DYNAMICS_INDEX_COUNT; i++) {
        TRACKING_setpoint[i] = 0;
    }

    setpointRoll = 0;
    setpointPitch = 0;
    setpointYaw = 0;
    setpointThrottle = 0;
    targetLocked = false;
    pitchLocked = false;
    rollLocked = false;

    reset_PID(&pidRoll, &pidParamRoll);
    reset_PID(&pidPitch, &pidParamPitch);
    reset_PID(&pidYaw, &pidParamYaw);
    reset_PID(&pidThrottle, &pidParamThrottle);
}

void trackingInit(const pidProfile_t *pidProfile) {
    targetDistance =    stalkerConfig() -> target_distance;
    targetAltitude =    TRACKING_SENSOR_PITCH_TAN * targetDistance;

    LoadPidParam(pidProfile->pid[PID_ST_ELV], &pidParamThrottle); 
    LoadPidParam(pidProfile->pid[PID_ST_AZM], &pidParamYaw);
    LoadPidParam(pidProfile->pid[PID_ST_DST], &pidParamPitch);
    LoadPidParam(pidProfile->pid[PID_ST_HDN], &pidParamRoll);

    // Pitch-specific trim coefficients
    pidParamPitch.initialTrim = 0.25;
    pidParamPitch.fD =  4.0f;
    pidParamPitch.kP *= 0.002000f;
    pidParamPitch.kI *= 0.000002f;
    pidParamPitch.kD *= 0.080000f;
    pidParamPitch.iMin = - 0.25f * targetDistance/pidParamPitch.kI; // approx -15 deg
    pidParamPitch.iMax =   0.25f * targetDistance/pidParamPitch.kI; // approx +15 deg

    // Roll-specific trim coefficients
    pidParamRoll.initialTrim = 0.25;
    pidParamRoll.fD = 4.0f;
    pidParamRoll.kP *= 0.0040000f;
    pidParamRoll.kI *= 0.0000005f;
    pidParamRoll.kD *= 0.0016000f;
    pidParamRoll.iMin = -TRACKING_SETPOINT_LIMIT/pidParamRoll.kI;
    pidParamRoll.iMax =  TRACKING_SETPOINT_LIMIT/pidParamRoll.kI;

    // Yaw-specific trim coefficients
    pidParamYaw.initialTrim = 0.5;
    pidParamYaw.fD = 4.0f;
    pidParamYaw.kP *= 0.0100f;
    pidParamYaw.kI *= 0.0001f;
    pidParamYaw.kD *= 0.0005f;
    pidParamYaw.iMin = -TRACKING_SETPOINT_LIMIT/pidParamYaw.kI;
    pidParamYaw.iMax =  TRACKING_SETPOINT_LIMIT/pidParamYaw.kI;
    
    // Throttle-specific trim coefficients
    pidParamThrottle.initialTrim = 0.5;
    pidParamThrottle.fD = stalkerConfig() -> throttle_filter;
    pidParamThrottle.kP *= 0.000800f;
    pidParamThrottle.kI *= 0.000005f;
    pidParamThrottle.kD *= 0.080000f;
    pidParamThrottle.iMin = 100.0f/pidParamThrottle.kI;
    pidParamThrottle.iMax = 1000.0f/pidParamThrottle.kI;

    deadbandDistance =  stalkerConfig() -> target_deadband ;
    deadbandAltitude =  TRACKING_SENSOR_PITCH_TAN * stalkerConfig() -> target_deadband;

    ratePitch =  0.1f * stalkerConfig() -> distance_rate;
    rateRoll  =  0.1f * stalkerConfig() -> heading_rate;

    // what fraction of Azimuth error value will be fed into Roll control instead of Yaw, 
    // remaining percentage will go to Yaw PID.
    ratioRollToYaw = 0.01f*stalkerConfig() -> rollyaw_ratio;

    setpointRoll = 0;
    setpointPitch = 0;
    setpointRoll = 0;
    setpointThrottle = 0;
    targetLocked = false;
}

static float GetNextPID(float error, float dt, PID *pid, PID_PARAM *pid_param) {
    // stop triming after inital lock-on
    if (pid->error * error < 0)
        pid -> trim = 1.0f;

    return get_P(error, pid, pid_param) + get_I(error, dt, pid, pid_param) + get_D(error, dt, pid, pid_param);
}

static float ClipDeadband(float error, float deadband){
    float errorAbs = ABS(error);

    if (errorAbs > deadband) {
        errorAbs = errorAbs - deadband;
        return (error > 0) ? errorAbs : -errorAbs;
    } else {
        return 0.0f;
    }
}

static void CalculateThrottle(float dt, float targetAngle){
    UNUSED(targetAngle);
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use altitude displacement in [mm]
    // otherwise - we assume distance is correct (equal to target one), so we only adjusting the altitude displacement by its angle
    float errorAlt = //(targetAngle < 140.0f && STALKER_TARGET_UAV.distance > (0.5f*targetDistance) && STALKER_TARGET_UAV.distance < (2.0f*targetDistance))
                     //   ? (targetAltitude - STALKER_TARGET_UAV.altitude) 
                     //   : 
                    (targetAltitude - targetDistance * tanf(TRACKING_SENSOR_PITCH_RAD - 0.001f * STALKER_TARGET_UAV.elevation));

    float errorThrottle = ClipDeadband(errorAlt, deadbandAltitude);

    setpointThrottle = GetNextPID(errorThrottle, dt, &pidThrottle, &pidParamThrottle);
    setpointThrottle = constrainf(setpointThrottle, 0.0f, PWM_RANGE_MAX - PWM_RANGE_MIN);
}

static void CalculatePitch(float dt, float targetAngle, const rollAndPitchTrims_t * attitudeAngleTrim){
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use distance displacement in [mm]//
    // otherwise - we set Distance error to 0, letting Yaw and Throttle to correct first.
    float errorDistance = 0.0f;
    
    if (targetAngle < 140.0f 
        && STALKER_TARGET_UAV.distance > (0.5f*targetDistance) 
        && STALKER_TARGET_UAV.distance < (4.0f*targetDistance)){

        errorDistance = ClipDeadband(STALKER_TARGET_UAV.distance - targetDistance, deadbandDistance);

        if (!pitchLocked) {
            pidPitch.error = errorDistance;
            pidPitch.trim = pidParamPitch.initialTrim;
            pitchLocked = true;
        }
    }
    else {
        // we lost the target (or it was lost)
        // reset previous error to 0 so we will not get a D-term spike
        pidPitch.error = 0.0f;
        pidPitch.trim = pidParamPitch.initialTrim;
        pitchLocked = false;
    }

    // forward acceleration/speed is a function of thottle mutiplied by SIN of Pitch Angle
    // so we normalize by target distance, and constrain it -60..+60 DEG

    float pidDistance = GetNextPID(errorDistance, dt, &pidPitch, &pidParamPitch);
    float pidDistanceNormalized = constrainf(pidDistance/targetDistance, -SIN_MAX_PITCH_ANGLE,  SIN_MAX_PITCH_ANGLE);
    
    float errorPitch = asinf(pidDistanceNormalized)*DEG - ((attitude.raw[FD_PITCH] - attitudeAngleTrim->raw[FD_PITCH]) / 10.0f);
    setpointPitch = constrainf(ratePitch*errorPitch, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

static void CalculateRoll(float dt, bool isLocked, const rollAndPitchTrims_t *attitudeAngleTrim){
    // target heading is very inaccurate
    // PID should be tunned very sluggish
    // and with high deadband 140 mRAD = +/- 8 degrees
    float errorHeading = (isLocked) 
                        ? ClipDeadband(-STALKER_TARGET_UAV.headingAzimuth/1000.0f, asinf(deadbandDistance/targetDistance))
                        : 0.0f; 

    float rollRatio = constrainf(ratioRollToYaw, 0.0f, 1.0f);
    float errorAzimuth = rollRatio * STALKER_TARGET_UAV.azimuth/1000.0f; 

    // Azimuth and Heading compaund signal to Roll input
    float error = errorAzimuth + 0.1f*errorHeading;

    if (isLocked){
        if (!rollLocked) {
            pidRoll.error = error;
            pidRoll.trim = pidParamRoll.initialTrim;
            rollLocked = true;
        }
    } else {
        // we lost the target (or it was lost)
        // reset previous error to 0 so we will not get a D-term spike
        pidRoll.error = 0.0f;
        pidRoll.trim = pidParamRoll.initialTrim;
        rollLocked = false;
    }

    float pidHeading = GetNextPID(error, dt, &pidRoll, &pidParamRoll);
    float errorRoll = constrainf(pidHeading*DEG, -30.0f, 30.0f) - ((attitude.raw[FD_ROLL] - attitudeAngleTrim->raw[FD_ROLL]) / 10.0f);
    setpointRoll = constrainf(rateRoll*errorRoll, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

static void CalculateYaw(float dt){
    // this is like +/- 0.6 degree
    float yawRatio = constrainf(1.0f - ratioRollToYaw, 0.0f, 1.0f);
    float errorYaw = -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed) * yawRatio * STALKER_TARGET_UAV.azimuth; 
    setpointYaw = GetNextPID(errorYaw, dt, &pidYaw, &pidParamYaw);
    setpointYaw = constrainf(setpointYaw, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

void onStalkerNewData(void)
{
    const float deltaTime = 1.0f; // 42 Hz is a latest Stalker version framerate
    if (FLIGHT_MODE(STALKER_MODE)){
         const rollAndPitchTrims_t * accTrims = &accelerometerConfig()->accelerometerTrims;

        // this value is used to figure out do we have true measurement of distance to target and heading value.
        float targetAngle = sqrtf(STALKER_TARGET_UAV.azimuth*STALKER_TARGET_UAV.azimuth + STALKER_TARGET_UAV.elevation*STALKER_TARGET_UAV.elevation);
        
        // FOV limit on correct Distance and Altitude calculations
        // distance lock when in reasonable range
        // 0.20 and 0.30 are derived from 140 mRAD Elevation decrease or increase on FIXED Altitude but varying Distance
        bool isLocked = (targetAngle < 100.0f && STALKER_TARGET_UAV.distance > (0.8f*targetDistance) && STALKER_TARGET_UAV.distance <  (1.30f*targetDistance));

        if (isLocked) {
            if (!targetLocked) {
                beeper(BEEPER_ARMING_GPS_FIX);
                targetLocked = true;
            }
        } else {
            if (targetLocked) {
                beeper(BEEPER_DISARMING);
                targetLocked = false;
            }
        }
      
        CalculateYaw(deltaTime);
        CalculateThrottle(deltaTime, targetAngle);  
        CalculatePitch(deltaTime, targetAngle, accTrims);
        CalculateRoll(deltaTime, isLocked, accTrims);
    }
}

void updateTrackingSetpoints(void) {
    TRACKING_setpoint[ROLL]  = setpointRoll;
    TRACKING_setpoint[PITCH] = setpointPitch;
    TRACKING_setpoint[YAW]   = setpointYaw;

    // Throttle controlled via overwriting RC data
    if (ABS(rcData[THROTTLE] - originalThrottleRcData) < 10){
        rcCommand[THROTTLE] = constrainf(PWM_RANGE_MIN + setpointThrottle, PWM_RANGE_MIN + 50, PWM_RANGE_MAX - 50);
    }
}

void updateTrackingMode(void) {
    if (IS_RC_MODE_ACTIVE(BOXSTALKER)) {
        if (!FLIGHT_MODE(STALKER_MODE)) {
            ENABLE_FLIGHT_MODE(STALKER_MODE);
            trackingCleanup();

            // pre-set PID to avoid crazy shit
            // TODO: add some conditions chack as we have with tragteAngle in main code
            originalThrottleRcData = rcData[THROTTLE];
            setpointThrottle =  rcCommand[THROTTLE] - PWM_RANGE_MIN;
            
            // pre-init basic error to avoid D-spikes and craft jerks when starting tracking.
            pidThrottle.integrator = setpointThrottle / pidParamThrottle.kI;
            pidThrottle.error =  ClipDeadband(targetAltitude - targetDistance * tanf(TRACKING_SENSOR_PITCH_RAD - 0.001f * STALKER_TARGET_UAV.elevation), deadbandAltitude);
            pidYaw.error = -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed) *  constrainf(1.0f - ratioRollToYaw, 0.0f, 1.0f) * STALKER_TARGET_UAV.azimuth; 

            beeper(BEEPER_ARMED);
        }
    } else {
        if (FLIGHT_MODE(STALKER_MODE)) {
            DISABLE_FLIGHT_MODE(STALKER_MODE);
            trackingCleanup();
        }
    }   
}
