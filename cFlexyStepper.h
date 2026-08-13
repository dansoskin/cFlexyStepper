#ifndef FLEXY_STEPPER_H
#define FLEXY_STEPPER_H

//#define MCU_ARDUINO
 #define MCU_STM32

#ifdef MCU_ARDUINO
    #include <Arduino.h>
#else
    #include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


// Direction signal level for "step and direction"
#define POSITIVE_DIRECTION 0
#define NEGATIVE_DIRECTION 1

/* Result of a timed move. FLEXY_MOVE_OK is the only one where the motor was
 * actually commanded. */
typedef enum FlexyStepper_move_status
{
	FLEXY_MOVE_OK = 0,
	FLEXY_MOVE_ALREADY_THERE,	/* distance is under one step, nothing to do */
	FLEXY_MOVE_BAD_ARG,			/* distance/time not finite, or time <= 0 */
	FLEXY_MOVE_BAD_SETUP,		/* conversion, accel or decel not positive */
	FLEXY_MOVE_TOO_FAST,		/* time is shorter than the ramps can achieve */
	FLEXY_MOVE_FAULTED			/* axis is latched in fault, command refused */
} FlexyStepper_move_status;

/* Axis state. FAULT latches when the fault pin stays active for the debounce
 * window and only leaves through a successful FlexyStepper_clearFault() pulse;
 * every motion command is refused while it holds. */
typedef enum FlexyStepper_status
{
	FLEXY_STATUS_IDLE = 0,		/* driver disabled, no motion */
	FLEXY_STATUS_ENABLED,		/* enabled / holding torque, no motion */
	FLEXY_STATUS_MOVING,		/* profile in progress */
	FLEXY_STATUS_FAULT			/* fault latched, motion commands rejected */
} FlexyStepper_status;

/* The fault pin must read active this long, continuously, before FAULT latches,
 * so a single glitch on the alarm wire does not estop the axis. */
#ifndef FLEXY_FAULT_DEBOUNCE_US
	#define FLEXY_FAULT_DEBOUNCE_US		2000
#endif

/* Width of the reset pulse FlexyStepper_clearFault() puts on the clear pin. */
#ifndef FLEXY_FAULT_CLEAR_PULSE_US
	#define FLEXY_FAULT_CLEAR_PULSE_US	10000
#endif

typedef enum cFlexyStepper_homing_sm_states
{
	HOMING_IDLE, HOMING_MOVE_TOWARDS_LIMIT, HOMING_DELAY1,
	HOMING_MOVE_AWAY_FROM_LIMIT, HOMING_DELAY2,
	HOMING_ADJUST_POSITION, HOMING_DELAY3, HOMING_ERROR
} cFlexyStepper_homing_sm_states;


// FlexyStepper structure
typedef struct {

    #ifdef MCU_ARDUINO
        uint8_t stepPin;
        uint8_t directionPin;
        uint8_t enablePin;
        uint8_t faultPin;
        uint8_t faultClearPin;
    #else
        // Pin configuration
        GPIO_TypeDef* stepPort;
        uint16_t stepPin;
        GPIO_TypeDef* directionPort;
        uint16_t directionPin;
        GPIO_TypeDef* enablePort;
        uint16_t enablePin;
        GPIO_TypeDef* faultPort;
        uint16_t faultPin;
        GPIO_TypeDef* faultClearPort;
        uint16_t faultClearPin;
    #endif

    bool log_enabled;
    
    // Motion parameters
    float stepsPerMillimeter;
    float stepsPerRevolution;
    float conversion;
    int directionOfMotion;
    int32_t currentPosition_InSteps;
    int32_t targetPosition_InSteps;
    float desiredSpeed_InStepsPerSecond;
    float desiredPeriod_InUSPerStep;
    float acceleration_InStepsPerSecondPerSecond;
    float acceleration_InStepsPerUSPerUS;
    float deceleration_InStepsPerSecondPerSecond;
    float deceleration_InStepsPerUSPerUS;
    /* periodOfSlowestStep_InUS is the period of the first step taken from rest,
     * so it follows acceleration. decelPeriodOfSlowestStep_InUS is the floor the
     * slow-down ramp clamps to, so it follows deceleration.
     * minimumPeriodForAStoppedMotion decides when a move counts as finished and
     * MUST derive from the same ramp as the clamp -- if the ramp cannot reach
     * the threshold the move never completes and the motor overshoots, reverses
     * and oscillates around the target. */
    float periodOfSlowestStep_InUS;
    float decelPeriodOfSlowestStep_InUS;
    float minimumPeriodForAStoppedMotion;
    float nextStepPeriod_InUS;
    uint32_t lastStepTime_InUS;
    float currentStepPeriod_InUS;

    bool inverse_enablePin;
    FlexyStepper_status status;
    bool should_release;
    char motorName[20];

    /* Fault handling. The *_connected flags exist because a port/pin pair of
     * zero is indistinguishable from "never configured". */
    bool fault_pin_connected;
    bool inverse_faultPin;
    bool fault_clear_pin_connected;
    bool inverse_faultClearPin;
    /* Debounce: when the pin first read active, and whether it read active on
     * the previous sample at all. */
    bool fault_pin_was_active;
    uint32_t fault_debounce_start_us;
    /* Diagnostics, so an intermittent alarm is visible from the console. */
    uint32_t fault_count;
    uint32_t last_fault_us;

    float default_speed;
    float default_acceleration;
    float default_deceleration;

    uint32_t homing_sm_timer;
    cFlexyStepper_homing_sm_states homing_sm_state;
    int homing_direction;
    float homing_speed;
    float homing_adjust_position;
    uint8_t* homing_limit_switch_ptr;
    float zero_pos;
    float speed_after_homing;

    /* Passthrough streaming. The saved_* fields hold what the axis was
     * configured with before the stream took over, so leaving the mode restores
     * it exactly. */
    bool  passthrough;
    bool  passthrough_saved_logging;
    float passthrough_saved_speed;
    float passthrough_saved_acceleration;
    float passthrough_saved_deceleration;
} FlexyStepper;

// Logging sink: the application supplies a callback that transports a
// fully-formatted line (UART, Serial, whatever); the driver never touches
// hardware directly.
typedef void (*FlexyStepper_log_fn_t)(const char *message);

void FlexyStepper_attach_logger(FlexyStepper_log_fn_t log_fn);

// Per-motor trace: adds the "[name] " prefix and honours that motor's
// log_enabled flag, so muting one motor mutes all of its output.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
void FlexyStepper_logf(FlexyStepper* stepper, const char *format, ...);

#ifdef MCU_ARDUINO
	#define WRITE_PIN(port, pin, value) digitalWrite(pin, value)
	#define READ_PIN(port, pin) digitalRead(pin)
    #define GET_MICROS micros()
    #define DELAY_MICROS(micros) delayMicroseconds(micros)
#else
    uint32_t HAL_GetMicros(void);
    void HAL_DelayMicros(uint32_t micros);

    void FlexyStepper_attach_timer_for_micros(TIM_HandleTypeDef* htim);

	#define WRITE_PIN(port, pin, value) HAL_GPIO_WritePin(port, pin, value)
	#define READ_PIN(port, pin) HAL_GPIO_ReadPin(port, pin)
	#define GET_MICROS HAL_GetMicros()
	#define DELAY_MICROS(micros) HAL_DelayMicros(micros)
#endif


//----------------------------------------------------------------
// Setup functions
void FlexyStepper_Init(FlexyStepper* stepper, char* name);
void FlexyStepper_en_motor(FlexyStepper* stepper, uint8_t state);
void FlexyStepper_set_homing(FlexyStepper* stepper,
							int homing_direction,
							float  homing_speed,
							float  homing_adjust_position,
							uint8_t* homing_limit_switch_ptr,
							float new_zero_position);
void stop_cFlexyStepper_homing_sm(FlexyStepper* stepper);

void FlexyStepper_enable_logging(FlexyStepper* stepper, bool enable);
bool FlexyStepper_logging_enabled(FlexyStepper* stepper);

/* Fault pins follow the enable pin's polarity convention: inverse = false means
 * active high. The fault pin is the driver's alarm output; the clear pin is a
 * dedicated reset output pulsed by FlexyStepper_clearFault(). */
#ifdef MCU_ARDUINO
    void FlexyStepper_connectToPins(FlexyStepper* stepper, uint8_t stepPin, uint8_t directionPin);
    void FlexyStepper_connectEnablePin(FlexyStepper* stepper, uint8_t pin, bool inverse);
    void FlexyStepper_connectFaultPin(FlexyStepper* stepper, uint8_t pin, bool inverse);
    void FlexyStepper_connectFaultClearPin(FlexyStepper* stepper, uint8_t pin, bool inverse);
#else
    void FlexyStepper_connectToPins(FlexyStepper* stepper, GPIO_TypeDef* stepPort, uint16_t stepPin,
                                GPIO_TypeDef* directionPort, uint16_t directionPin);
    void FlexyStepper_connectEnablePin(FlexyStepper* stepper, GPIO_TypeDef* port, uint16_t pin, bool inverse);
    void FlexyStepper_connectFaultPin(FlexyStepper* stepper, GPIO_TypeDef* port, uint16_t pin, bool inverse);
    void FlexyStepper_connectFaultClearPin(FlexyStepper* stepper, GPIO_TypeDef* port, uint16_t pin, bool inverse);
#endif


/*
// Functions with units in millimeters
void FlexyStepper_setStepsPerMillimeter(FlexyStepper* stepper, float motorStepPerMillimeter);
float FlexyStepper_getCurrentPositionInMillimeters(FlexyStepper* stepper);
void FlexyStepper_setCurrentPositionInMillimeters(FlexyStepper* stepper, float currentPositionInMillimeters);
void FlexyStepper_setSpeedInMillimetersPerSecond(FlexyStepper* stepper, float speedInMillimetersPerSecond);
void FlexyStepper_setAccelerationInMillimetersPerSecondPerSecond(FlexyStepper* stepper, float accelerationInMillimetersPerSecondPerSecond);
void FlexyStepper_moveRelativeInMillimeters(FlexyStepper* stepper, float distanceToMoveInMillimeters);
void FlexyStepper_setTargetPositionRelativeInMillimeters(FlexyStepper* stepper, float distanceToMoveInMillimeters);
void FlexyStepper_moveToPositionInMillimeters(FlexyStepper* stepper, float absolutePositionToMoveToInMillimeters);
void FlexyStepper_setTargetPositionInMillimeters(FlexyStepper* stepper, float absolutePositionToMoveToInMillimeters);
float FlexyStepper_getCurrentVelocityInMillimetersPerSecond(FlexyStepper* stepper);
*/
/*
// Functions with units in revolutions
void FlexyStepper_setStepsPerRevolution(FlexyStepper* stepper, float motorStepPerRevolution);
float FlexyStepper_getCurrentPositionInRevolutions(FlexyStepper* stepper);
void FlexyStepper_setCurrentPositionInRevolutions(FlexyStepper* stepper, float currentPositionInRevolutions);
void FlexyStepper_setSpeedInRevolutionsPerSecond(FlexyStepper* stepper, float speedInRevolutionsPerSecond);
void FlexyStepper_setAccelerationInRevolutionsPerSecondPerSecond(FlexyStepper* stepper, float accelerationInRevolutionsPerSecondPerSecond);
void FlexyStepper_moveRelativeInRevolutions(FlexyStepper* stepper, float distanceToMoveInRevolutions);
void FlexyStepper_setTargetPositionRelativeInRevolutions(FlexyStepper* stepper, float distanceToMoveInRevolutions);
void FlexyStepper_moveToPositionInRevolutions(FlexyStepper* stepper, float absolutePositionToMoveToInRevolutions);
void FlexyStepper_setTargetPositionInRevolutions(FlexyStepper* stepper, float absolutePositionToMoveToInRevolutions);
float FlexyStepper_getCurrentVelocityInRevolutionsPerSecond(FlexyStepper* stepper);
*/

// Functions with units in steps
void FlexyStepper_setCurrentPositionInSteps(FlexyStepper* stepper, int32_t currentPositionInSteps);
int32_t FlexyStepper_getCurrentPositionInSteps(FlexyStepper* stepper);
void FlexyStepper_setSpeedInStepsPerSecond(FlexyStepper* stepper, float speedInStepsPerSecond);
/* Sets acceleration AND deceleration to the same value, so existing callers are
 * unaffected. To use different ramps, call this first and then
 * FlexyStepper_setDecelerationInStepsPerSecondPerSecond(). */
void FlexyStepper_setAccelerationInStepsPerSecondPerSecond(FlexyStepper* stepper, float accelerationInStepsPerSecondPerSecond);
/* Deceleration only. Must be called AFTER setAcceleration*, which resets it. */
void FlexyStepper_setDecelerationInStepsPerSecondPerSecond(FlexyStepper* stepper, float decelerationInStepsPerSecondPerSecond);
void FlexyStepper_moveRelativeInSteps(FlexyStepper* stepper, int32_t distanceToMoveInSteps);
void FlexyStepper_setTargetPositionRelativeInSteps(FlexyStepper* stepper, int32_t distanceToMoveInSteps);
void FlexyStepper_moveToPositionInSteps(FlexyStepper* stepper, int32_t absolutePositionToMoveToInSteps);
void FlexyStepper_setTargetPositionInSteps(FlexyStepper* stepper, int32_t absolutePositionToMoveToInSteps);
void FlexyStepper_setTargetPositionToStop(FlexyStepper* stepper);
bool FlexyStepper_motionComplete(FlexyStepper* stepper);
float FlexyStepper_getCurrentVelocityInStepsPerSecond(FlexyStepper* stepper);

// Helper functions
bool FlexyStepper_processMovement(FlexyStepper* stepper);
void FlexyStepper_DeterminePeriodOfNextStep(FlexyStepper* stepper);

// Basic
void FlexyStepper_setConversion(FlexyStepper* stepper, float conversion);

float FlexyStepper_getCurrentPosition(FlexyStepper* stepper);
void FlexyStepper_setCurrentPosition(FlexyStepper* stepper, float position);

float FlexyStepper_getCurrentVelocity(FlexyStepper* stepper);
void FlexyStepper_setSpeed(FlexyStepper* stepper, float speed);

/* Sets both ramps. Call before FlexyStepper_setDeceleration(), not after. */
void FlexyStepper_setAcceleration(FlexyStepper* stepper, float acceleration);
float FlexyStepper_getAcceleration(FlexyStepper* stepper);

void FlexyStepper_setDeceleration(FlexyStepper* stepper, float deceleration);
float FlexyStepper_getDeceleration(FlexyStepper* stepper);

float FlexyStepper_getTargetSpeed(FlexyStepper* stepper);
void FlexyStepper_jog(FlexyStepper * stepper, float speed);

void FlexyStepper_setDefaults(FlexyStepper* stepper, float speed, float acceleration, float deceleration);
void FlexyStepper_restoreDefaults(FlexyStepper* stepper);
float FlexyStepper_getDefaultSpeed(FlexyStepper* stepper);
float FlexyStepper_getDefaultAcceleration(FlexyStepper* stepper);
float FlexyStepper_getDefaultDeceleration(FlexyStepper* stepper);

void FlexyStepper_setTargetPositionRelative(FlexyStepper* stepper, float distanceToMove, bool should_release);
void FlexyStepper_setTargetPosition(FlexyStepper* stepper, float absolutePositionToMoveTo, bool should_release);

/* Timed moves: give a target and how long the move should take, and the cruise
 * speed is solved from the ramps currently set and applied before starting.
 * Returns FLEXY_MOVE_OK only if the move was actually commanded; every other
 * status leaves the motor untouched. The solve assumes the motor starts from
 * rest -- retargeting mid-move still works, but the time will not be honoured.
 * Speed is left at the solved value afterwards, so follow a timed move with
 * FlexyStepper_restoreDefaults() when the baseline should come back. */
FlexyStepper_move_status FlexyStepper_setTimedTargetPositionRelative(FlexyStepper* stepper, float distanceToMove, float seconds, bool should_release);
FlexyStepper_move_status FlexyStepper_setTimedTargetPosition(FlexyStepper* stepper, float absolutePositionToMoveTo, float seconds, bool should_release);

/* Solve only -- nothing is commanded. Either output may be NULL. speed_out is
 * the cruise speed in user units; min_seconds_out is the fastest this distance
 * can be covered under the current ramps, and is written even when the answer
 * is FLEXY_MOVE_TOO_FAST so a caller can report what it should have asked for. */
FlexyStepper_move_status FlexyStepper_solveTimedMove(FlexyStepper* stepper, float distance, float seconds, float* speed_out, float* min_seconds_out);
const char* FlexyStepper_move_status_str(FlexyStepper_move_status status);

void FlexyStepper_Estop(FlexyStepper* stepper, bool should_release);
void FlexyStepper_loop(FlexyStepper* stepper);

//----------------------------------------------------------------
// Fault handling
//
// FlexyStepper_loop() samples the fault pin (once connected) and, after the
// debounce window, estops the axis, releases the motor and latches
// FLEXY_STATUS_FAULT. While latched, every motion command is refused.
// FlexyStepper_clearFault() pulses the clear pin and drops the latch back to
// IDLE -- a driver still in alarm simply re-latches on the next loop pass.
// Clearing a fault does NOT re-home: position is not to be trusted after a
// driver alarm, and what to do about that is the application's call.
//----------------------------------------------------------------

FlexyStepper_status FlexyStepper_getStatus(FlexyStepper* stepper);
bool FlexyStepper_isMoving(FlexyStepper* stepper);
const char* FlexyStepper_status_str(FlexyStepper_status status);

/* Blocking: holds the clear pin active for FLEXY_FAULT_CLEAR_PULSE_US (10 ms)
 * before returning. Ignored when no clear pin is connected. */
void FlexyStepper_clearFault(FlexyStepper* stepper);

//----------------------------------------------------------------
// Passthrough streaming
//
// For an axis whose position is dictated by an outside calculation rather than
// by a destination of its own -- a coordinated motion recomputed every few
// milliseconds. The trapezoid is bypassed: each setpoint is a position to be at
// *now* and the rate to get there at, and the axis simply runs at that rate.
//
//   FlexyStepper_beginPassthrough(&ax);
//   /* every tick, e.g. 200 Hz */
//   FlexyStepper_streamSetpoint(&ax, position, rate);
//   /* every pass of the main loop, as usual */
//   FlexyStepper_loop(&ax);
//   /* when the sequence ends */
//   FlexyStepper_endPassthrough(&ax);
//
// begin mutes this axis's logging and end restores it: one log line blocks for
// milliseconds, which at streaming rates is dozens of missed steps.
//----------------------------------------------------------------

void FlexyStepper_beginPassthrough(FlexyStepper* stepper);
void FlexyStepper_endPassthrough(FlexyStepper* stepper);
bool FlexyStepper_inPassthrough(FlexyStepper* stepper);

/* One streamed setpoint. `position` is absolute, in user units. `rate` is the
 * speed to travel at, signed or not -- direction comes from the target, so only
 * the magnitude is used. Ignored unless the axis is in passthrough. */
void FlexyStepper_streamSetpoint(FlexyStepper* stepper, float position, float rate);

//----------------------------------------------------------------


void set_cFlexyStepper_homing_sm_state(FlexyStepper* stepper, cFlexyStepper_homing_sm_states st);
cFlexyStepper_homing_sm_states get_cFlexyStepper_homing_sm_state(FlexyStepper* stepper);
void start_cFlexyStepper_homing_sm(FlexyStepper* stepper);
void cFlexyStepper_homing_sm_loop(FlexyStepper* stepper);

//----------------------------------------------------------------
void FlexyStepper_decode_menu(char * buffer, size_t buffer_size);
void FlexyStepper_decode(FlexyStepper * stpr, char cmd, char* arg);


#ifdef __cplusplus
}
#endif

#endif // FLEXY_STEPPER_H

