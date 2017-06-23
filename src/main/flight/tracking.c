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
#define DEG                         (180.0f/M_PIf)

float TRACKING_setpoint[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0.0f, 0.0f, 0.0f }; 

typedef struct {
    float integrator;     // integrator value
    float derivative;     // last derivative calculation
    float error;          // last input for derivative
} PID;

typedef struct {
    float kP;
    float kI;
    float kD;
    float fD;
    float iMin;
    float iMax;
} PID_PARAM;

static PID_PARAM pidParamRoll;
static PID_PARAM pidParamPitch;
static PID_PARAM pidParamYaw;
static PID_PARAM pidParamThrottle;

static PID pidRoll      = {0,0,0};
static PID pidPitch     = {0,0,0};
static PID pidYaw       = {0,0,0};
static PID pidThrottle  = {0,0,0};


static float targetDistance;
static float targetAltitude;
static float deadbandDistance;
static float deadbandAltitude;
static float defaultThrottleIntegral;

static float setpointRoll;
static float setpointPitch;
static float setpointYaw;
static float setpointThrottle;
static bool targetLocked;

static int32_t get_P(float error, PID_PARAM *pid)
{
    return pid->kP * error;
}

static int32_t get_I(float error, float dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += error * dt;
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

static void reset_PID(PID *pid)
{
    pid->integrator = 0;
    pid->error = 0;
    pid->derivative = 0;
}

static void LoadPidParam(pid8_t pidProfile, PID_PARAM * pidParam, float trimCoefficient) {
    pidParam->kP = trimCoefficient * pidProfile.P;
    pidParam->kI = trimCoefficient * pidProfile.I;
    pidParam->kD = trimCoefficient * pidProfile.D;
    pidParam->fD = 10.0f;
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
    LoadPidParam(pidProfile->pid[PID_ST_ELV], &pidParamThrottle, 1.000f); 
    LoadPidParam(pidProfile->pid[PID_ST_AZM], &pidParamYaw,      0.010f); // with Yaw we deal with mRAD (thausand fraction of radian) 
    LoadPidParam(pidProfile->pid[PID_ST_DST], &pidParamPitch,    5.730f); // with Pitch we deal with RAD (not mRAD)
    LoadPidParam(pidProfile->pid[PID_ST_HDN], &pidParamRoll,     0.010f); // with Roll, whic is targetHeading control we should be very sluggish. Its very noisy

    pidParamPitch.kI *= 0.1f;
    pidParamPitch.kD *= 0.1f;
    pidParamPitch.iMin = -TRACKING_SETPOINT_LIMIT/pidParamPitch.kI;
    pidParamPitch.iMax =  TRACKING_SETPOINT_LIMIT/pidParamPitch.kI;

    pidParamRoll.kI *= 0.1f;
    pidParamRoll.kD *= 0.1f;
    pidParamRoll.iMin = -TRACKING_SETPOINT_LIMIT/pidParamRoll.kI;
    pidParamRoll.iMax =  TRACKING_SETPOINT_LIMIT/pidParamRoll.kI;

    pidParamYaw.kI *= 0.1f;
    pidParamYaw.kD *= 0.1f;
    pidParamYaw.iMin = -TRACKING_SETPOINT_LIMIT/pidParamYaw.kI;
    pidParamYaw.iMax =  TRACKING_SETPOINT_LIMIT/pidParamYaw.kI;

    
    // Throttle-specific coefficients
    
    pidParamThrottle.fD = stalkerConfig() -> throttle_filter;
    pidParamThrottle.kP *= 0.00200f;
    pidParamThrottle.kI *= 0.00001f;
    pidParamThrottle.kD *= 0.12000f;
    pidParamThrottle.iMin = 100.0f/pidParamThrottle.kI;
    pidParamThrottle.iMax = 1000.0f/pidParamThrottle.kI;
    defaultThrottleIntegral = pidParamThrottle.kI;

    targetDistance =    stalkerConfig() -> target_distance;
    deadbandDistance =  stalkerConfig() -> target_deadband ;
    targetAltitude =    TRACKING_SENSOR_PITCH_TAN * stalkerConfig() -> target_distance;
    deadbandAltitude =  TRACKING_SENSOR_PITCH_TAN * stalkerConfig() -> target_deadband;
    setpointRoll = 0;
    setpointPitch = 0;
    setpointRoll = 0;
    setpointThrottle = 0;
    targetLocked = false;
}

void updateTrackingSetpoints(void) {
    const int STICK_DEADBAND = 40;

    TRACKING_setpoint[ROLL]  =  (ABS(rcData[ROLL]  - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointRoll  : 0;
    TRACKING_setpoint[PITCH] =  (ABS(rcData[PITCH] - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointPitch : 0;
    TRACKING_setpoint[YAW]   =  (ABS(rcData[YAW]   - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointYaw   : 0;

    // throttle stick should be down
    // otherwise - user tries to override Stalker from TX
    if (rcData[THROTTLE] < PWM_RANGE_MIN + STICK_DEADBAND){
        rcCommand[THROTTLE] = constrainf(PWM_RANGE_MIN + setpointThrottle, PWM_RANGE_MIN + 50, PWM_RANGE_MAX - 50);
    }
}

void updateTrackingMode(void) {
    if (IS_RC_MODE_ACTIVE(BOXSTALKER)) {
        if (!FLIGHT_MODE(STALKER_MODE)) {
            ENABLE_FLIGHT_MODE(STALKER_MODE);
            setpointThrottle =  rcCommand[THROTTLE] - PWM_RANGE_MIN;
            pidThrottle.integrator = setpointThrottle / pidParamThrottle.kI;
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
    UNUSED(targetAngle);
    static float errorTrackingFilter = 0;
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use altitude displacement in [mm]
    // otherwise - we assume distance is correct (equal to target one), so we only adjusting the altitude displacement by its angle
    float errorAlt = //targetAngle < 140.0f ? (targetAltitude - STALKER_TARGET_UAV.altitude) :
                    (targetAltitude - targetDistance * tanf(TRACKING_SENSOR_PITCH_RAD - 0.001f * STALKER_TARGET_UAV.elevation));

    float errorThrottle = ClipDeadband(errorAlt, deadbandAltitude);  

    // alghorithm for adaptive I-term grows
    // if big error persists for a long time and doesn't changes much (within 10%) - increase kI 100 times so we get there faster
    errorTrackingFilter = errorTrackingFilter + (errorThrottle - errorTrackingFilter)/(10.0f*stalkerConfig() -> throttle_filter);
    if (ABS(errorThrottle) > (targetDistance/3.0f) && ABS((errorTrackingFilter-errorThrottle)/errorThrottle) < 0.10f) {
        pidThrottle.integrator += (50.0f*errorThrottle); 
    } else {
        // reset the filter if target is locked
        errorTrackingFilter = 0.0f;
    }

    setpointThrottle = GetNextSetpoint(errorThrottle, dt, &pidThrottle, &pidParamThrottle);
    setpointThrottle = constrainf(setpointThrottle, 0.0f, PWM_RANGE_MAX - PWM_RANGE_MIN);
}

static void CalculatePitch(float dt, float targetAngle, const rollAndPitchTrims_t * attitudeAngleTrim){
    // 140 mRAD = 8 DEG. x2 = 16 DEG FOV where the distance and altitude is calculated more or less correctly
    // so we use distance displacement in [mm]//
    // otherwise - we set Distance error to 0, letting Yaw and Throttle to correct first.
    float errorDistance = (targetAngle < 140.0f && STALKER_TARGET_UAV.distance > (0.5f*targetDistance) && STALKER_TARGET_UAV.distance < (3.0f*targetDistance))
                                ? ClipDeadband(STALKER_TARGET_UAV.distance - targetDistance, deadbandDistance)
                                : 0.0f;
    // forward acceleration/speed is a function of thottle mutiplied by SIN of Pitch Angle
    // so we normalize by target distance, and constrain it -60..+60 DEG
    float errorDistanceNormalized = constrainf(errorDistance/targetDistance, -0.8660f,  0.8660f) ;
    float errorAngle = asinf(errorDistanceNormalized)*DEG - ((attitude.raw[FD_PITCH] - attitudeAngleTrim->raw[FD_PITCH]) / 10.0f);

    setpointPitch = GetNextSetpoint(errorAngle, dt, &pidPitch, &pidParamPitch);
    setpointPitch = constrainf(setpointPitch, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}


static void CalculateRoll(float dt, bool isLocked, const rollAndPitchTrims_t *attitudeAngleTrim){
    // target heading is very inaccurate
    // PID should be tunned very sluggish
    // and with high deadband 140 mRAD = +/- 8 degrees
    float errorHeading = (isLocked) 
                        ? ClipDeadband(-STALKER_TARGET_UAV.headingAzimuth/1000.0f, asinf(deadbandDistance/targetDistance))*DEG
                        : 0.0f; 

    // TODO: 
    // mix-in Yaw as complimentary input to help manuvering on target sharp turns
    // mix level should depend on current ongoing speed (e.g. movinf average pitch angle)
    float errorAngle = errorHeading/10.0f - ((attitude.raw[FD_ROLL] - attitudeAngleTrim->raw[FD_ROLL]) / 10.0f);

    setpointRoll = GetNextSetpoint(errorAngle, dt, &pidRoll, &pidParamRoll);
    setpointRoll = constrainf(setpointRoll, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

static void CalculateYaw(float dt){
    // this is like +/- 0.6 degree
    float errorYaw = -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed) * STALKER_TARGET_UAV.azimuth; 
    setpointYaw = GetNextSetpoint(errorYaw, dt, &pidYaw, &pidParamYaw);
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
