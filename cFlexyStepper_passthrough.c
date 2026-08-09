#include "cFlexyStepper.h"

//
// Passthrough streaming -- see the header for the calling pattern.
//
// The mode exists because a streamed axis is being told where it should be
// *right now*, not where to end up. A trapezoid planned toward each setpoint
// would spend its whole life braking toward a point a few steps ahead, so the
// ramps have to get out of the way. The interesting part is that simply making
// them enormous does not work, and the reason is worth writing down.
//
// Two quantities in DeterminePeriodOfNextStep depend on the ramps:
//
//   decelerationDistance   = 5E11 / (decel * period^2)
//   decelPeriodOfSlowestStep = 1e6 / sqrt(2 * decel)      (and the same for accel,
//                                                          as periodOfSlowestStep)
//
// The first must stay below the distance covered in one tick, or the slow-down
// branch fires every tick and the axis crawls. That pushes decel up. But the
// second is a *speed floor*: the first step from rest, and the clamp the
// slow-down ramp cannot go below. Push the ramps up far enough and that floor
// rises above the rate actually being asked for -- so every tick the axis dashes
// at the floor, arrives early, stops and waits, which is a square wave on the
// velocity and exactly what streaming was meant to avoid.
//
// Setting acceleration = deceleration = v^2 / 2 resolves both at once:
//
//   decelPeriodOfSlowestStep = 1e6 / sqrt(2 * v^2/2) = 1e6/v   -- the floor IS the
//                                                                commanded period
//   periodOfSlowestStep      = the same                        -- the first step
//                                                                from rest is
//                                                                already at rate
//   decelerationDistance     = 5e11 / ((v^2/2)(1e6/v)^2) = 1   -- brakes only
//                                                                within one step
//
// Both ramp branches then land back on the commanded period: the speed-up path
// computes period/2 and is clamped up by desiredPeriod, the slow-down path
// computes 1.5*period and is clamped down by decelPeriodOfSlowestStep. The axis
// runs at the rate it was given, and when it overshoots the setpoint the
// reverse-direction branch fires immediately instead of dithering -- which is
// the position correction doing its job.
//
// Because the ramps follow v, this holds at every rate in a sweep, which no
// fixed pair of values can do.
//

/* Small enough to be indistinguishable from stopped, large enough to keep
 * 1e6/v and v^2/2 finite when a stream commands a rate of exactly zero. */
#define PASSTHROUGH_MIN_STEPS_PER_SECOND    (1.0f)

void FlexyStepper_beginPassthrough(FlexyStepper* stepper)
{
    if (stepper->passthrough)
        return;

    stepper->passthrough_saved_speed        = FlexyStepper_getTargetSpeed(stepper);
    stepper->passthrough_saved_acceleration = FlexyStepper_getAcceleration(stepper);
    stepper->passthrough_saved_deceleration = FlexyStepper_getDeceleration(stepper);
    stepper->passthrough_saved_logging      = stepper->log_enabled;

    FlexyStepper_logf(stepper, "Passthrough on\r\n");

    /* Muted for the duration. Every setter on the streaming path logs, and a
     * single line blocks the caller for milliseconds -- at streaming rates that
     * is a visible gap in the pulse train, so this is a correctness
     * requirement rather than a matter of noise. */
    stepper->log_enabled = false;

    /* The axis has to hold whatever it is carrying between setpoints, so it
     * stays energised and must not release when a setpoint happens to be
     * reached exactly. */
    FlexyStepper_en_motor(stepper, 1);
    stepper->should_release = false;

    stepper->passthrough = true;
}

void FlexyStepper_endPassthrough(FlexyStepper* stepper)
{
    if (!stepper->passthrough)
        return;

    stepper->passthrough = false;

    /* Stop where the axis actually is. Leaving the last streamed setpoint in
     * place would let it run on toward a position the sequence no longer
     * believes in. The motor stays energised: releasing it here would drop
     * whatever it is holding. */
    stepper->targetPosition_InSteps = stepper->currentPosition_InSteps;
    stepper->directionOfMotion      = 0;
    stepper->currentStepPeriod_InUS = 0.0f;
    stepper->nextStepPeriod_InUS    = 0.0f;
    stepper->is_moving              = false;

    stepper->log_enabled = stepper->passthrough_saved_logging;

    /* Acceleration before deceleration: setAcceleration resets the decel ramp
     * to match itself, so the other order would silently discard the saved
     * deceleration. */
    FlexyStepper_setSpeed(stepper, stepper->passthrough_saved_speed);
    FlexyStepper_setAcceleration(stepper, stepper->passthrough_saved_acceleration);
    FlexyStepper_setDeceleration(stepper, stepper->passthrough_saved_deceleration);

    FlexyStepper_logf(stepper, "Passthrough off\r\n");
}

bool FlexyStepper_inPassthrough(FlexyStepper* stepper)
{
    return stepper->passthrough;
}

void FlexyStepper_streamSetpoint(FlexyStepper* stepper, float position, float rate)
{
    if (!stepper->passthrough)
        return;

    const float scale = fabsf(stepper->conversion);

    if (!(scale > 0.0f) || !isfinite(position) || !isfinite(rate))
        return;

    /* A magnitude: which way to turn is settled by the target relative to the
     * current position, so a signed rate would only fight it. */
    float v = fabsf(rate) * scale;

    if (!(v > PASSTHROUGH_MIN_STEPS_PER_SECOND))
        v = PASSTHROUGH_MIN_STEPS_PER_SECOND;

    /* Written straight into the struct rather than through setSpeed(): the
     * public setters log, and setTargetPosition() also re-energises the motor
     * and rewrites should_release. All of that is right once per command and
     * wasteful hundreds of times a second. */
    stepper->desiredSpeed_InStepsPerSecond = v;
    stepper->desiredPeriod_InUSPerStep     = 1000000.0f / v;

    /* The whole of passthrough, derived at the top of this file. */
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, 0.5f * v * v);

    stepper->targetPosition_InSteps = (int32_t)lroundf(position * stepper->conversion);
    stepper->is_moving = true;
}
