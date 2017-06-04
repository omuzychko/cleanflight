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



int16_t TRACKING_angle[ANGLE_INDEX_COUNT] = { 0, 0 }; 

typedef struct {
    float output;
    float derivative;
    float integrator;          // integrator value
    float last_derivative;     // last derivative for low-pass filter
    float last_input;          // last input for derivative
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



static int32_t get_P(int32_t error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(int32_t error, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->maxI, pid_param->maxI);
    return pid->integrator;
}

static int32_t get_D(int32_t input, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (input - pid->last_input) / *dt;

    // Low pass filter cut frequency for derivative calculation
    // Set to  "1 / ( 2 * PI * gps_lpf )
    float pidFilter = (1.0f / (2.0f * M_PIf * (float)navigationConfig()->gps_lpf));
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->last_derivative + (*dt / (pidFilter + *dt)) * (pid->derivative - pid->last_derivative);
    // update state
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->output = 0;
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}


static void LoadPidParam(pid8_t * pidProfile, PID_PARAM * pidParam, float trimCoefficient, float maxI) {
    pidParam->kP = pidProfile -> P * trimCoefficient;
    pidParam->kI = pidProfile -> I * trimCoefficient;
    pidParam->kD = pidProfile -> D * trimCoefficient;
    pidParam->maxI = maxI;
}

static void trackingCleanup(void) {
    uint32_t i;
    for (i = 0; i < ANGLE_INDEX_COUNT; i++) {
        TRACKING_angle[i] = 0;
    }

    reset_PID(&pidRoll);
    reset_PID(&pidPitch);
    reset_PID(&pidYaw);
    reset_PID(&pidThrottle);
}

void trackingInit(void) {
    LoadPidParam(&(currentPidProfile->pid[PID_ROLL]),&pidParamRoll, 1.0f, 30.0f); // 30 deg roll
    LoadPidParam(&(currentPidProfile->pid[PID_PITCH]),&pidParamPitch, 1.0f, 30.0f); // 30 deg pitch
    LoadPidParam(&(currentPidProfile->pid[PID_YAW]) ,&pidParamYaw, 1.0f, 0.0f); // 30 deg pitch
    LoadPidParam(&(currentPidProfile->pid[PID_ALT]) ,&pidParamThrottle, 1.0f, 0.0f); // 2000 - max Throttle value
}

void updateTrackingControls(void) {
// TODO: assign rcCommands and TRACKING_angle from PID controller

    rcCommand[YAW] += constrain(pidYaw.output, PWM_RANGE_MIN, PWM_RANGE_MAX);
    rcCommand[THROTTLE] = constrain(pidThrottle.output, PWM_RANGE_MIN, PWM_RANGE_MAX);
}

void updateTrackingMode(void) {
    if (IS_RC_MODE_ACTIVE(BOXSTALKER)) {
        if (!FLIGHT_MODE(STALKER_MODE)) {
            pidThrottle.output = rcCommand[THROTTLE];
            ENABLE_FLIGHT_MODE(STALKER_MODE);
        }
    } else {
        if (FLIGHT_MODE(STALKER_MODE)) {
            DISABLE_FLIGHT_MODE(STALKER_MODE);
            trackingCleanup();
        }
    }   
}

void onStalkerNewData(void)
{
    if (FLIGHT_MODE(STALKER_MODE)){
            pidThrottle.output = constrain(pidThrottle.output + STALKER_TARGET_UAV.elevation, PWM_RANGE_MIN, PWM_RANGE_MAX);
            pidThrottle.output = constrain(pidThrottle.output + STALKER_TARGET_UAV.elevation, PWM_RANGE_MIN, PWM_RANGE_MAX);
    }
}
