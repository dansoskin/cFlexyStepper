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
// One caveat, and it bites: v here must be the larger of the rate just
// commanded and the rate the axis is actually travelling at. Deriving the ramps
// from the command alone leaves the axis unable to stop at the moment the
// command drops away underneath it. See streamSetpoint.
//

/* Small enough to be indistinguishable from stopped, large enough to keep
 * 1e6/v and v^2/2 finite when a stream commands a rate of exactly zero. */
#define PASSTHROUGH_MIN_STEPS_PER_SECOND    (1.0f)

/* Seconds over which a standing position error is closed. The feedforward says
 * how fast the setpoint is moving; it says nothing about being in the wrong
 * place to begin with, so an axis fed a pure feedforward has no reason to ever
 * catch up. Adding error/TAU to the commanded speed gives it one.
 *
 * Short enough to close a gap promptly, long enough that the extra speed is
 * small next to the feedforward once tracking. The result is capped -- see
 * streamSetpoint -- so a large error slews at the axis's own speed limit rather
 * than at whatever error/TAU happens to work out to. */
#define PASSTHROUGH_CATCHUP_SECONDS         (0.05f)

void FlexyStepper_beginPassthrough(FlexyStepper* stepper)
{
    if (stepper->passthrough)
        return;

    stepper->passthrough_saved_logging = stepper->log_enabled;

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
    /* Energised and holding -- unless a fault latched mid-stream, which must
     * survive leaving the mode. */
    if (stepper->status != FLEXY_STATUS_FAULT)
        FlexyStepper_setStatus(stepper, FLEXY_STATUS_ENABLED);

    stepper->log_enabled = stepper->passthrough_saved_logging;

    /* The stream drove speed and both ramps far away from anything the axis
     * would use on its own (see the derivation at the top of this file), so
     * they have to be put back -- the axis defaults are that baseline. */
    FlexyStepper_restoreDefaults(stepper);

    FlexyStepper_logf(stepper, "Passthrough off\r\n");
}

bool FlexyStepper_inPassthrough(FlexyStepper* stepper)
{
    return stepper->passthrough;
}

void FlexyStepper_streamSetpoint(FlexyStepper* stepper, float position, float rate)
{
    if (!stepper->passthrough || stepper->status == FLEXY_STATUS_FAULT)
        return;

    const float scale = fabsf(stepper->conversion);

    if (!(scale > 0.0f) || !isfinite(position) || !isfinite(rate))
        return;

    const int32_t target = (int32_t)lroundf(position * stepper->conversion);

    /* A magnitude: which way to turn is settled by the target relative to the
     * current position, so a signed rate would only fight it. */
    const float feedforward = fabsf(rate) * scale;

    /* Feedforward alone is only ever enough to keep pace with a setpoint the
     * axis is already sitting on. Start a sequence with the mechanism somewhere
     * the masters do not believe it to be -- an aborted run, a hand jog, a fresh
     * datum -- and the error stands there forever: the rate is right, the place
     * is wrong, and nothing in the command says so.
     *
     * Worse, at the start of a move the feedforward is near zero, so the whole
     * commanded speed collapses to the floor, and with it the acceleration and
     * therefore periodOfSlowestStep -- which is the delay before the first step
     * from rest. Measured: a full second of dead time before the axis so much as
     * twitched, while the master ran away from it. */
    const int32_t error_steps = target - stepper->currentPosition_InSteps;
    const float error = fabsf((float)error_steps);

    float v = feedforward + error / PASSTHROUGH_CATCHUP_SECONDS;

    /* Capped at this axis's default speed, so closing a large gap is a
     * deliberate slew at a known rate rather than a lunge. Never below the
     * feedforward though: during a fast sweep the feedforward legitimately
     * exceeds the axis's ordinary working speed, and clamping to that would
     * throttle the very motion being tracked. */
    const float limit = fmaxf(feedforward, FlexyStepper_getDefaultSpeed(stepper) * scale);
    if (v > limit)
        v = limit;

    if (!(v > PASSTHROUGH_MIN_STEPS_PER_SECOND))
        v = PASSTHROUGH_MIN_STEPS_PER_SECOND;

    /* Written straight into the struct rather than through setSpeed(): the
     * public setters log, and setTargetPosition() also re-energises the motor
     * and rewrites should_release. All of that is right once per command and
     * wasteful hundreds of times a second. */
    stepper->desiredSpeed_InStepsPerSecond = v;
    stepper->desiredPeriod_InUSPerStep     = 1000000.0f / v;

    /* The ramps follow v^2/2 as derived at the top of this file -- but sized
     * against whichever is larger, the speed just commanded or the speed the
     * axis is actually travelling at.
     *
     * The two agree for as long as the stream is following something that
     * moves. They come apart the instant the thing being followed stops: the
     * feedforward drops to zero while the motor is still running at full rate,
     * and a deceleration derived from that new near-zero command is far too
     * weak to stop it. DeterminePeriodOfNextStep's "moving away from the
     * target" branch then takes the decelerate path instead of reversing,
     *
     *     if (currentStepPeriod_InUS < decelPeriodOfSlowestStep_InUS)
     *         slowDownFlag = true;
     *     else
     *         directionOfMotion = -1;
     *
     * because the collapsed ramp puts decelPeriodOfSlowestStep enormously above
     * the period the axis is actually stepping at. The axis coasts straight past
     * the setpoint, shedding speed so slowly it would take minutes to stop.
     * Measured in simulation: 1.51 degrees of overshoot and still climbing at
     * the end of a tilt move, against 0.05 with this line as it stands. */
    float v_ramp = v;
    if (stepper->currentStepPeriod_InUS > 0.0f)
    {
        const float actual = 1000000.0f / stepper->currentStepPeriod_InUS;
        if (actual > v_ramp)
            v_ramp = actual;
    }

    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, 0.5f * v_ramp * v_ramp);

    stepper->targetPosition_InSteps = target;
    FlexyStepper_setStatus(stepper, FLEXY_STATUS_MOVING);
}
