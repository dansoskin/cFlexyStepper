#include "cFlexyStepper.h"

//-------------------
static FlexyStepper_log_fn_t log_fn = NULL;

void FlexyStepper_attach_logger(FlexyStepper_log_fn_t fn) {
    log_fn = fn;
}

void FlexyStepper_logf(FlexyStepper* stepper, const char *format, ...) {
    if (!stepper->log_enabled || log_fn == NULL) {
        return;
    }

    char buffer[96];

    int n = snprintf(buffer, sizeof(buffer), "[%s] ", stepper->motorName);
    if (n < 0 || (size_t)n >= sizeof(buffer)) {
        n = 0;
    }

    va_list args;
    va_start(args, format);
    vsnprintf(buffer + n, sizeof(buffer) - (size_t)n, format, args);
    va_end(args);

    log_fn(buffer);
}

void FlexyStepper_enable_logging(FlexyStepper* stepper, bool enable) {
    if (stepper->log_enabled == enable)
        return;

    stepper->log_enabled = true;
    FlexyStepper_logf(stepper, "Logging %s\r\n", enable ? "on" : "off");
    stepper->log_enabled = enable;
}

bool FlexyStepper_logging_enabled(FlexyStepper* stepper) {
    return stepper->log_enabled;
}

//-------------------
void FlexyStepper_setConversion(FlexyStepper* stepper, float conversion) {
    stepper->conversion = conversion;
}

void FlexyStepper_setDefaults(FlexyStepper* stepper, float speed, float acceleration, float deceleration) {
    stepper->default_speed = speed;
    stepper->default_acceleration = acceleration;
    stepper->default_deceleration = deceleration;

    FlexyStepper_logf(stepper, "Defaults: %.2f / %.2f / %.2f\r\n", speed, acceleration, deceleration);
    FlexyStepper_restoreDefaults(stepper);
}

void FlexyStepper_restoreDefaults(FlexyStepper* stepper) {
    FlexyStepper_setSpeed(stepper, stepper->default_speed);
    FlexyStepper_setAcceleration(stepper, stepper->default_acceleration);
    FlexyStepper_setDeceleration(stepper, stepper->default_deceleration);
}

float FlexyStepper_getDefaultSpeed(FlexyStepper* stepper) {
    return stepper->default_speed;
}

float FlexyStepper_getDefaultAcceleration(FlexyStepper* stepper) {
    return stepper->default_acceleration;
}

float FlexyStepper_getDefaultDeceleration(FlexyStepper* stepper) {
    return stepper->default_deceleration;
}

//-------------------
float FlexyStepper_getCurrentPosition(FlexyStepper* stepper) {
    return ((float)FlexyStepper_getCurrentPositionInSteps(stepper) / stepper->conversion);
}

void FlexyStepper_setCurrentPosition(FlexyStepper* stepper, float position) {
    FlexyStepper_logf(stepper, "Current Position: %.4f\r\n", position);
    FlexyStepper_setCurrentPositionInSteps(stepper, (int32_t)round(position * stepper->conversion));
}

//-------------------
float FlexyStepper_getCurrentVelocity(FlexyStepper* stepper) {
    return FlexyStepper_getCurrentVelocityInStepsPerSecond(stepper) / fabsf(stepper->conversion);
}

void FlexyStepper_setSpeed(FlexyStepper* stepper, float speed) {
    FlexyStepper_logf(stepper, "Speed: %.4f\r\n", speed);
    FlexyStepper_setSpeedInStepsPerSecond(stepper, speed * fabsf(stepper->conversion));
}

void FlexyStepper_setAcceleration(FlexyStepper* stepper, float acceleration) {
    FlexyStepper_logf(stepper, "Acceleration: %.4f\r\n", acceleration);
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, acceleration * fabsf(stepper->conversion));
}

void FlexyStepper_setDeceleration(FlexyStepper* stepper, float deceleration) {
    FlexyStepper_logf(stepper, "Deceleration: %.4f\r\n", deceleration);
    FlexyStepper_setDecelerationInStepsPerSecondPerSecond(stepper, deceleration * fabsf(stepper->conversion));
}

float FlexyStepper_getTargetSpeed(FlexyStepper* stepper)
{
	return stepper->desiredSpeed_InStepsPerSecond / fabsf(stepper->conversion);
}

float FlexyStepper_getAcceleration(FlexyStepper* stepper)
{
	return stepper->acceleration_InStepsPerSecondPerSecond / fabsf(stepper->conversion);
}

float FlexyStepper_getDeceleration(FlexyStepper* stepper)
{
	return stepper->deceleration_InStepsPerSecondPerSecond / fabsf(stepper->conversion);
}

//-------------------
void FlexyStepper_jog(FlexyStepper * stepper, float speed)
{
	FlexyStepper_logf(stepper, "Jog: %.4f\r\n", speed);

    if(stepper->status == FLEXY_STATUS_FAULT)
    {
        FlexyStepper_logf(stepper, "in fault, jog refused\r\n");
        return;
    }

    if(speed == 0)
    {
        FlexyStepper_Estop(stepper, true);
        return;
    }
    FlexyStepper_setCurrentPositionInSteps(stepper, 0);
    // stepper->currentStepPeriod_InUS = 0.0;
    // stepper->nextStepPeriod_InUS = 0.0;
    // stepper->directionOfMotion = 0;

    FlexyStepper_setStatus(stepper, FLEXY_STATUS_MOVING);
    stepper->should_release = 0;
    FlexyStepper_setSpeedInStepsPerSecond(stepper, fabsf(speed * stepper->conversion));


    FlexyStepper_en_motor(stepper, 1);
	if(speed > 0)
        FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(9999999.0 * stepper->conversion));
    else
        FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(-9999999.0 * stepper->conversion));
}

//-------------------
void FlexyStepper_setTargetPositionRelative(FlexyStepper* stepper, float distanceToMove, bool should_release) {
    FlexyStepper_logf(stepper, "Target Position Relative: %.4f\r\n", distanceToMove);
    if(stepper->status == FLEXY_STATUS_FAULT)
    {
        FlexyStepper_logf(stepper, "in fault, move refused\r\n");
        return;
    }
    /* MOVING before en_motor: en_motor only tracks the IDLE<->ENABLED edge, so
     * claiming the state first keeps a move from announcing "enabled" on its
     * way to "moving". */
    FlexyStepper_setStatus(stepper, FLEXY_STATUS_MOVING);
    FlexyStepper_en_motor(stepper, 1);
    FlexyStepper_setTargetPositionRelativeInSteps(stepper, (int32_t)round(distanceToMove * stepper->conversion));
    stepper->should_release = should_release;
}

void FlexyStepper_setTargetPosition(FlexyStepper* stepper, float absolutePositionToMoveTo, bool should_release) {
    FlexyStepper_logf(stepper, "Target Position: %.4f\r\n", absolutePositionToMoveTo);
    if(stepper->status == FLEXY_STATUS_FAULT)
    {
        FlexyStepper_logf(stepper, "in fault, move refused\r\n");
        return;
    }
    FlexyStepper_setStatus(stepper, FLEXY_STATUS_MOVING);
    FlexyStepper_en_motor(stepper, 1);
    FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(absolutePositionToMoveTo * stepper->conversion));
    stepper->should_release = should_release;
}



//-------------------------------------------------------------------------------
// Timed moves
//
// Under a trapezoidal profile a move of distance d at cruise speed v takes
//
//      T(v) = d/v + v/2 * (1/accel + 1/decel)
//
// so with k = (1/accel + 1/decel)/2 the cruise speed for a wanted T is a root of
//
//      k*v^2 - T*v + d = 0
//
// Only the smaller root is a real trapezoid. T(v) bottoms out at v = sqrt(d/k),
// which is exactly where the cruise phase vanishes and the profile turns
// triangular; the larger root sits beyond that, where the formula no longer
// describes the motion. That minimum, T = 2*sqrt(k*d), is also the fastest the
// move can possibly be done, so any shorter time is rejected rather than
// silently approximated.

const char* FlexyStepper_move_status_str(FlexyStepper_move_status status) {
    switch (status) {
        case FLEXY_MOVE_OK:            return "ok";
        case FLEXY_MOVE_ALREADY_THERE: return "already at target";
        case FLEXY_MOVE_BAD_ARG:       return "bad distance or time";
        case FLEXY_MOVE_BAD_SETUP:     return "conversion/accel/decel not set";
        case FLEXY_MOVE_TOO_FAST:      return "time too short for the ramps";
        case FLEXY_MOVE_FAULTED:       return "axis in fault";
    }
    return "unknown";
}

FlexyStepper_move_status FlexyStepper_solveTimedMove(FlexyStepper* stepper,
                                                    float distance,
                                                    float seconds,
                                                    float* speed_out,
                                                    float* min_seconds_out) {
    if (!isfinite(distance) || !isfinite(seconds) || seconds <= 0.0f)
        return FLEXY_MOVE_BAD_ARG;

    const float accel = stepper->acceleration_InStepsPerSecondPerSecond;
    const float decel = stepper->deceleration_InStepsPerSecondPerSecond;
    const float scale = fabsf(stepper->conversion);

    if (!(accel > 0.0f) || !(decel > 0.0f) || !(scale > 0.0f))
        return FLEXY_MOVE_BAD_SETUP;

    /* Solved in steps so the stored ramps are used as-is, with no round trip
     * through user units. Distance is a magnitude here -- direction is the
     * target's business, not the speed's. */
    const float d = fabsf(distance) * scale;

    if (roundf(d) < 1.0f)   /* under half a step: the move would not turn the motor */
    {
        if (min_seconds_out)
            *min_seconds_out = 0.0f;
        return FLEXY_MOVE_ALREADY_THERE;
    }

    const float k = 0.5f * (1.0f / accel + 1.0f / decel);
    const float min_seconds = 2.0f * sqrtf(k * d);

    if (min_seconds_out)
        *min_seconds_out = min_seconds;

    if (seconds < min_seconds)
        return FLEXY_MOVE_TOO_FAST;

    /* The min_seconds test above is the single authority on feasibility, so
     * clamp instead of letting float noise at seconds == min_seconds hand a
     * negative value to sqrtf. At disc == 0 this yields the triangular peak
     * speed sqrt(d/k), which is the correct answer for that case. */
    float disc = seconds * seconds - 4.0f * k * d;
    if (disc < 0.0f)
        disc = 0.0f;

    const float speedInStepsPerSecond = (seconds - sqrtf(disc)) / (2.0f * k);

    if (!(speedInStepsPerSecond > 0.0f))
        return FLEXY_MOVE_TOO_FAST;

    if (speed_out)
        *speed_out = speedInStepsPerSecond / scale;

    return FLEXY_MOVE_OK;
}

/* Shared tail: solve, report, and only then command. Nothing is applied unless
 * the whole solve succeeds, so a rejected move cannot leave the motor holding a
 * half-configured speed. */
static FlexyStepper_move_status FlexyStepper_startTimedMove(FlexyStepper* stepper,
                                                           float distance,
                                                           float target,
                                                           float seconds,
                                                           bool relative,
                                                           bool should_release) {
    float speed = 0.0f;
    float min_seconds = 0.0f;

    if (stepper->status == FLEXY_STATUS_FAULT) {
        FlexyStepper_logf(stepper, "Timed move: %s\r\n",
                         FlexyStepper_move_status_str(FLEXY_MOVE_FAULTED));
        return FLEXY_MOVE_FAULTED;
    }

    FlexyStepper_move_status status =
        FlexyStepper_solveTimedMove(stepper, distance, seconds, &speed, &min_seconds);

    if (status == FLEXY_MOVE_TOO_FAST) {
        FlexyStepper_logf(stepper, "Timed move: %.3fs needs >= %.3fs\r\n",
                         seconds, min_seconds);
        return status;
    }

    if (status != FLEXY_MOVE_OK) {
        FlexyStepper_logf(stepper, "Timed move: %s\r\n",
                         FlexyStepper_move_status_str(status));
        return status;
    }

    FlexyStepper_logf(stepper, "Timed move: %.4f in %.3fs\r\n", distance, seconds);
    FlexyStepper_setSpeed(stepper, speed);

    if (relative)
        FlexyStepper_setTargetPositionRelative(stepper, distance, should_release);
    else
        FlexyStepper_setTargetPosition(stepper, target, should_release);

    return FLEXY_MOVE_OK;
}

FlexyStepper_move_status FlexyStepper_setTimedTargetPositionRelative(FlexyStepper* stepper,
                                                                    float distanceToMove,
                                                                    float seconds,
                                                                    bool should_release) {
    return FlexyStepper_startTimedMove(stepper, distanceToMove, 0.0f, seconds,
                                       /*relative=*/true, should_release);
}

FlexyStepper_move_status FlexyStepper_setTimedTargetPosition(FlexyStepper* stepper,
                                                            float absolutePositionToMoveTo,
                                                            float seconds,
                                                            bool should_release) {
    /* Distance comes off the live position, so the solve accounts for where the
     * motor actually is rather than where the last command aimed it. */
    const float distance = absolutePositionToMoveTo - FlexyStepper_getCurrentPosition(stepper);

    return FlexyStepper_startTimedMove(stepper, distance, absolutePositionToMoveTo, seconds,
                                       /*relative=*/false, should_release);
}

//-------------------------------------------------------------------------------

void FlexyStepper_en_motor(FlexyStepper* stepper, uint8_t state) {
    //XXX
    // stepper->inverse_enablePin ?
    // HAL_GPIO_WritePin(stepper->enablePort, stepper->enablePin, !state) :
    // HAL_GPIO_WritePin(stepper->enablePort, stepper->enablePin, state);

    stepper->inverse_enablePin ?
        WRITE_PIN(stepper->enablePort, stepper->enablePin, !state) :
        WRITE_PIN(stepper->enablePort, stepper->enablePin, state);

    /* Only the IDLE<->ENABLED edge is tracked here: a move in progress owns
     * MOVING, and only clearFault() may lift FAULT, so toggling enable must
     * disturb neither. */
    if (state && stepper->status == FLEXY_STATUS_IDLE)
        FlexyStepper_setStatus(stepper, FLEXY_STATUS_ENABLED);
    else if (!state && stepper->status == FLEXY_STATUS_ENABLED)
        FlexyStepper_setStatus(stepper, FLEXY_STATUS_IDLE);
}
//-------------------------------------------------------------------------------
void FlexyStepper_Estop(FlexyStepper* stepper, bool should_release) {
    stepper->directionOfMotion = 0;
    stepper->targetPosition_InSteps = stepper->currentPosition_InSteps;
    
    // Reset current motion state but preserve desiredSpeed for future movements
    stepper->currentStepPeriod_InUS = 0.0;
    stepper->nextStepPeriod_InUS = 0.0;

    if(should_release) {
        FlexyStepper_en_motor(stepper, 0);
    }

    FlexyStepper_loop(stepper);
    FlexyStepper_logf(stepper, "Estop\r\n");

}


//-------------------------------------------------------------------------------
// Fault handling

void FlexyStepper_setStatus(FlexyStepper* stepper, FlexyStepper_status status) {
    if (stepper->status == status)
        return;

    stepper->status = status;
    stepper->status_timer = GET_MICROS;
    FlexyStepper_logf(stepper, "status: %s\r\n", FlexyStepper_status_str(status));
}

FlexyStepper_status FlexyStepper_getStatus(FlexyStepper* stepper) {
    return stepper->status;
}

bool FlexyStepper_isMoving(FlexyStepper* stepper) {
    return stepper->status == FLEXY_STATUS_MOVING;
}

const char* FlexyStepper_status_str(FlexyStepper_status status) {
    switch (status) {
        case FLEXY_STATUS_IDLE:    return "idle";
        case FLEXY_STATUS_ENABLED: return "enabled";
        case FLEXY_STATUS_MOVING:  return "moving";
        case FLEXY_STATUS_FAULT:   return "fault";
    }
    return "unknown";
}

static bool FlexyStepper_faultPinActive(FlexyStepper* stepper) {
    bool level = READ_PIN(stepper->faultPort, stepper->faultPin) != 0;
    return stepper->inverse_faultPin ? !level : level;
}

static void FlexyStepper_writeFaultClearPin(FlexyStepper* stepper, bool asserted) {
    bool level = stepper->inverse_faultClearPin ? !asserted : asserted;
    WRITE_PIN(stepper->faultClearPort, stepper->faultClearPin, level);
}

void FlexyStepper_clearFault(FlexyStepper* stepper) {
    if (!stepper->fault_clear_pin_connected) {
        FlexyStepper_logf(stepper, "clear fault: no clear pin connected\r\n");
        return;
    }

    FlexyStepper_logf(stepper, "clearing fault\r\n");
    FlexyStepper_writeFaultClearPin(stepper, true);
    DELAY_MICROS(FLEXY_FAULT_CLEAR_PULSE_US);
    FlexyStepper_writeFaultClearPin(stepper, false);

    /* Drop the latch on faith: if the driver is still in alarm, the next loop
     * pass just latches FAULT again through the ordinary debounce. */
    stepper->fault_pin_was_active = false;
    if (stepper->status == FLEXY_STATUS_FAULT)
        FlexyStepper_setStatus(stepper, FLEXY_STATUS_IDLE);
}

static void FlexyStepper_serviceFault(FlexyStepper* stepper) {
    if (!stepper->fault_pin_connected)
        return;

    if (!FlexyStepper_faultPinActive(stepper)) {
        stepper->fault_pin_was_active = false;
        return;
    }

    if (!stepper->fault_pin_was_active) {
        stepper->fault_pin_was_active = true;
        stepper->fault_debounce_start_us = GET_MICROS;
        return;
    }

    if (GET_MICROS - stepper->fault_debounce_start_us < FLEXY_FAULT_DEBOUNCE_US)
        return;

    if (stepper->status == FLEXY_STATUS_FAULT)
        return;

    stepper->fault_count++;
    stepper->last_fault_us = GET_MICROS;

    /* FAULT before Estop: Estop() releases the motor and runs the loop once,
     * and neither the en_motor(0) inside it nor the recursive loop pass may
     * lift or miss the latch. */
    FlexyStepper_setStatus(stepper, FLEXY_STATUS_FAULT);
    FlexyStepper_Estop(stepper, true);
}

//-------------------------------------------------------------------------------

void FlexyStepper_loop(FlexyStepper* stepper) {
    FlexyStepper_serviceFault(stepper);

    //auto disable
    if(stepper->status == FLEXY_STATUS_MOVING)
    {
        if(!FlexyStepper_motionComplete(stepper)) {
            FlexyStepper_processMovement(stepper);
        }
        else
        {
            if(stepper->should_release)
            {
                FlexyStepper_setStatus(stepper, FLEXY_STATUS_IDLE);
                FlexyStepper_en_motor(stepper, 0);
                stepper->should_release = false;
            }
            else
            {
                FlexyStepper_setStatus(stepper, FLEXY_STATUS_ENABLED);
            }
        }
    }
}
