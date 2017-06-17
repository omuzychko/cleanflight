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

#define TRACKING_SETPOINT_LIMIT     (400.0f)
#define TRACKING_SENSOR_PITCH_SIN   (0.577350269f)
#define TRACKING_SENSOR_PITCH_COS   (0.816496581f)
#define TRACKING_SENSOR_PITCH_TAN   (0.707106781f)
#define TRACKING_SENSOR_PITCH_RAD   (0.615479708f)

float TRACKING_setpoint[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0.0f, 0.0f, 0.0f }; 

typedef struct {
    float output;
    float derivative;
    float integrator;          // integrator value
    float last_derivative;     // last derivative for low-pass filter
    float last_error;          // last input for derivative
} PID;

typedef struct {
    float kP;
    float kI;
    float kD;
    float maxI;
} PID_PARAM;

static PID_PARAM pidParamRoll;
static PID_PARAM pidParamPitch;
static PID_PARAM pidParamYaw;
static PID_PARAM pidParamThrottle;

static PID pidRoll      = {0,0,0,0,0};
static PID pidPitch     = {0,0,0,0,0};
static PID pidYaw       = {0,0,0,0,0};
static PID pidThrottle  = {0,0,0,0,0};


static float targetDistance;
static float targetAltitude;
static float deadbandDistance;
static float deadbandAltitude;

static float setpointRoll;
static float setpointPitch;
static float setpointYaw;
static float setpointThrottle;
static bool targetLocked;

static int32_t get_P(float error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(float error, float dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += pid_param->kI * error * dt;
    pid->integrator = constrain(pid->integrator, -pid_param->maxI, pid_param->maxI);
    return pid->integrator;
}

static int32_t get_D(float error, float dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (error - pid->last_error) / dt;

    // low pass filter for 10 samples (250 ms)
    pid->derivative = pid->last_derivative + (pid->derivative - pid->last_derivative)/10.0f;
    // update state
    pid->last_error = error;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->output = 0;
    pid->integrator = 0;
    pid->last_error = 0;
    pid->last_derivative = 0;
}

static void LoadPidParam(pid8_t pidProfile, PID_PARAM * pidParam, float trimCoefficient, float maxI) {
    pidParam->kP =        trimCoefficient * pidProfile.P;
    pidParam->kI = 0.1f * trimCoefficient * pidProfile.I;
    pidParam->kD = 0.1f * trimCoefficient * pidProfile.D;
    pidParam->maxI = maxI;
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

    reset_PID(&pidRoll);
    reset_PID(&pidPitch);
    reset_PID(&pidYaw);
    reset_PID(&pidThrottle);
}

void trackingInit(const pidProfile_t *pidProfile) {
    LoadPidParam(pidProfile->pid[PID_ST_ELV], &pidParamThrottle, 0.010f, 300.0f); 
    LoadPidParam(pidProfile->pid[PID_ST_AZM], &pidParamYaw,      0.010f, 500.0f); // with Yaw we deal with mRAD (thausand fraction of radian) 
    LoadPidParam(pidProfile->pid[PID_ST_DST], &pidParamPitch,    0.100f, 300.0f); // with Pitch we deal with RAD (not mRAD) and 10 times less aggressive than Yaw
    LoadPidParam(pidProfile->pid[PID_ST_HDN], &pidParamRoll,     0.001f, 200.0f); // with Roll, whic is targetHeading control we should be very sluggish. Its very noisy

    pidParamThrottle.kI*=10.0f;
    pidParamThrottle.kD*=10.0f;

    targetDistance =    stalkerConfig() -> target_distance;
    deadbandDistance =  stalkerConfig() -> target_deadband ;
    targetAltitude =    TRACKING_SENSOR_PITCH_TAN * stalkerConfig() -> target_distance;
    deadbandAltitude =  TRACKING_SENSOR_PITCH_TAN * stalkerConfig() -> target_deadband;
    setpointRoll = 0;
    setpointPitch = 0;
    setpointRoll = 0;
    setpointThrottle = 0;
}

void updateTrackingSetpoints(void) {
    const int STICK_DEADBAND = 40;

    TRACKING_setpoint[ROLL]  =  (ABS(rcData[ROLL]  - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointRoll  : 0;
    TRACKING_setpoint[PITCH] =  (ABS(rcData[PITCH] - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointPitch : 0;
    TRACKING_setpoint[YAW]   =  (ABS(rcData[YAW]   - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointYaw   : 0;

    // throttle stick should be down
    // otherwise - user tries to override Stalker from TX
    if (rcData[THROTTLE] < PWM_RANGE_MIN + STICK_DEADBAND){
        rcCommand[THROTTLE] = constrain(PWM_RANGE_MIDDLE + setpointThrottle, PWM_RANGE_MIN + 100, PWM_RANGE_MAX - 100);
    }
}

void updateTrackingMode(void) {
    if (IS_RC_MODE_ACTIVE(BOXSTALKER)) {
        if (!FLIGHT_MODE(STALKER_MODE)) {
            ENABLE_FLIGHT_MODE(STALKER_MODE);
            setpointThrottle =  rcCommand[THROTTLE] - PWM_RANGE_MIDDLE;
            pidThrottle.integrator = setpointThrottle;
            beeper(BEEPER_ARMED);
        }
    } else {
        if (FLIGHT_MODE(STALKER_MODE)) {
            DISABLE_FLIGHT_MODE(STALKER_MODE);
            trackingCleanup();
        }
    }   
}
static float GetNextSetpoint(float error, float dt, PID *pid, PID_PARAM *pid_param) {
    return get_P(error, pid_param) + get_I(error, dt, pid, pid_param) + get_D(error, dt, pid, pid_param);
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
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use altitude displacement in [mm]
    // otherwise - we assume distance is correct (equal to target one), so we only adjusting the altitude displacement by its angle
    float errorAlt = //targetAngle < 140.0f ? (targetAltitude - STALKER_TARGET_UAV.altitude) :
                    (targetAltitude - targetDistance * tanf(TRACKING_SENSOR_PITCH_RAD - 0.001f * STALKER_TARGET_UAV.elevation));

    float errorThrottle = ClipDeadband(errorAlt, deadbandAltitude);    

    setpointThrottle = GetNextSetpoint(errorThrottle, dt, &pidThrottle, &pidParamThrottle);
    setpointThrottle = constrain(setpointThrottle, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

static void CalculatePitch(float dt, float targetAngle){
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use distance displacement in [mm]//
    // otherwise - we set Distance error to 0, letting Yaw and Throttle to correct first.
    float errorDistance = targetAngle < 140.0f 
                                ? ClipDeadband(STALKER_TARGET_UAV.distance - targetDistance, deadbandDistance)
                                : 0.0f;
    // forward acceleration/speed is a function of thottle mutiplied by SIN of Pitch Angle
    // so we normalize by target distance, and constrain it -60..+60 DEG
    float errorDistanceNormalized = constrain(errorDistance/targetDistance, -0.8660f,  0.8660f) ;
    float errorPitch = asinf(errorDistanceNormalized);
    
    setpointPitch = GetNextSetpoint(errorPitch, dt, &pidPitch, &pidParamPitch);
    setpointPitch = constrain(setpointPitch, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}


static void CalculateRoll(float dt){
    // target heading is very inaccurate
    // PID should be tunned very sluggish
    // and with high deadband 140 mRAD = +/- 8 degrees
    float errorRoll = ClipDeadband(STALKER_TARGET_UAV.headingAzimuth, 140.0f); 
    setpointRoll = GetNextSetpoint(errorRoll, dt, &pidRoll, &pidParamRoll);
    setpointRoll = constrain(setpointRoll, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

static void CalculateYaw(float dt){
    // this is like +/- 0.6 degree
    float errorYaw = -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed) * ClipDeadband(STALKER_TARGET_UAV.azimuth, 10.0f); 
    setpointYaw = GetNextSetpoint(errorYaw, dt, &pidYaw, &pidParamYaw);
    setpointYaw = constrain(setpointYaw, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

void onStalkerNewData(void)
{
    const float deltaTime = 1.0f; // 42 Hz is a latest Stalker version framerate
    if (FLIGHT_MODE(STALKER_MODE)){

        // this value is used to figure out do we have true measurement of distance to target and heading value.
        float targetAngle = sqrtf(STALKER_TARGET_UAV.azimuth*STALKER_TARGET_UAV.azimuth + STALKER_TARGET_UAV.elevation*STALKER_TARGET_UAV.elevation);
        float reach = sqrtf(STALKER_TARGET_UAV.distance*STALKER_TARGET_UAV.distance + STALKER_TARGET_UAV.altitude*STALKER_TARGET_UAV.altitude);
        float reachDiff = reach - targetDistance/TRACKING_SENSOR_PITCH_COS; 
        
        bool isLocked = 
                // FOV limit on correct Distance and Altitude calculations
                (targetAngle < 140.0f) 
                // distance lock when in reasonable range
                // 0.20 and 0.30 are derived from 140 mRAD Elevation decrease or increase on FIXED Altitude but varying Distance
                && reachDiff > -0.20*targetDistance 
                && reachDiff <  0.30*targetDistance;

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
      
        CalculateThrottle(deltaTime, targetAngle);
        CalculatePitch(deltaTime, targetAngle);
        CalculateRoll(deltaTime);
        CalculateYaw(deltaTime);
     
    }
}
