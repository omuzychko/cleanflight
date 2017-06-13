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

#define TRACKING_SETPOINT_LIMIT 500.0f


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

static float setpointRoll;
static float setpointPitch;
static float setpointYaw;
static float setpointThrottle;
static float rcCommandThrottleAdjustment;

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

    // low pass filter for 10 samples
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
    pidParam->kP = pidProfile.P * trimCoefficient;
    pidParam->kI = pidProfile.I * trimCoefficient / 10.0f;
    pidParam->kD = pidProfile.D * trimCoefficient / 100.0f;
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
    rcCommandThrottleAdjustment = 0;

    reset_PID(&pidRoll);
    reset_PID(&pidPitch);
    reset_PID(&pidYaw);
    reset_PID(&pidThrottle);
}

void trackingInit(const pidProfile_t *pidProfile) {
    LoadPidParam(pidProfile->pid[PID_ST_AZM], &pidParamYaw, 0.05f, 500.0f); 
    LoadPidParam(pidProfile->pid[PID_ST_ELV], &pidParamThrottle, 0.05f, 400.0f); 
    LoadPidParam(pidProfile->pid[PID_ST_DST], &pidParamPitch, 0.01f, 250.0f); 
    LoadPidParam(pidProfile->pid[PID_ST_HDN], &pidParamRoll, 0.005f, 250.0f); 

    setpointRoll = 0;
    setpointPitch = 0;
    setpointRoll = 0;
    setpointThrottle = 0;
    rcCommandThrottleAdjustment = 0;
}

void updateTrackingSetpoints(void) {
    const int STICK_DEADBAND = 20;

    TRACKING_setpoint[ROLL]  =  (ABS(rcData[ROLL]  - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointRoll  : 0;
    TRACKING_setpoint[PITCH] =  (ABS(rcData[PITCH] - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointPitch : 0;
    TRACKING_setpoint[YAW]   =  (ABS(rcData[YAW]   - PWM_RANGE_MIDDLE) < STICK_DEADBAND) ? setpointYaw   : 0;

    // throttle stick should be down
    // otherwise - user tries to override Stalker from TX
    if (rcData[THROTTLE] < 2*STICK_DEADBAND){
        rcCommand[THROTTLE] = constrainf(PWM_RANGE_MIDDLE + rcCommandThrottleAdjustment, PWM_RANGE_MIN, PWM_RANGE_MAX);
    }
}

void updateTrackingMode(void) {
    if (IS_RC_MODE_ACTIVE(BOXSTALKER)) {
        if (!FLIGHT_MODE(STALKER_MODE)) {
            ENABLE_FLIGHT_MODE(STALKER_MODE);
            rcCommandThrottleAdjustment =  rcCommand[THROTTLE] - PWM_RANGE_MIDDLE;
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

static void CalculateThrottle(float dt){
    // TODO: Adjust by distance  when possible
    float errorThrottle = STALKER_TARGET_UAV.elevation;
    float errorThrottleAbs = ABS(errorThrottle);
    //beeper(BEEPER_ACC_CALIBRATION);
    if (errorThrottleAbs > 2.0f) {
        errorThrottleAbs = errorThrottleAbs - 2.0f;
        errorThrottle = (errorThrottle > 0) ? errorThrottleAbs : -errorThrottleAbs;
    } else {
        errorThrottle = 0;
    }

    setpointThrottle = GetNextSetpoint(errorThrottle, dt, &pidThrottle, &pidParamThrottle);
    setpointThrottle = constrain(setpointThrottle, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
    rcCommandThrottleAdjustment  += setpointThrottle;
}

static void CalculateYaw(float dt){
        float errorYaw = STALKER_TARGET_UAV.azimuth;
        float errorYawAbs = ABS(errorYaw);

        if (errorYawAbs > 2.0f) {
            errorYawAbs = errorYawAbs - 2.0f;
            errorYaw = (errorYaw > 0) ? errorYawAbs : -errorYawAbs;
        } else {
            errorYaw = 0;
        }

        setpointYaw = GetNextSetpoint(errorYaw, dt, &pidYaw, &pidParamYaw);
        setpointYaw = constrain(setpointYaw, -TRACKING_SETPOINT_LIMIT, TRACKING_SETPOINT_LIMIT);
}

void onStalkerNewData(void)
{
    const float deltaTime = 1.0f; // 42 Hz is a latest Stalker version framerate
    if (FLIGHT_MODE(STALKER_MODE)){
        CalculateYaw(deltaTime);
        CalculateThrottle(deltaTime);
    }
}
