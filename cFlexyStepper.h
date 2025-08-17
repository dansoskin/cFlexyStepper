#ifndef FLEXY_STEPPER_H
#define FLEXY_STEPPER_H

#define MCU_ARDUINO
// #define MCU_STM32

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
#include <stdint.h>
#include <math.h>


// Direction signal level for "step and direction"
#define POSITIVE_DIRECTION 0
#define NEGATIVE_DIRECTION 1

// FlexyStepper structure
typedef struct {

    #ifdef MCU_ARDUINO
        uint8_t stepPin;
        uint8_t directionPin;
        uint8_t enablePin;
    #else
        // Pin configuration
        GPIO_TypeDef* stepPort;
        uint16_t stepPin;
        GPIO_TypeDef* directionPort;
        uint16_t directionPin;
        GPIO_TypeDef* enablePort;
        uint16_t enablePin;
    #endif
    
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
    float periodOfSlowestStep_InUS;
    float minimumPeriodForAStoppedMotion;
    float nextStepPeriod_InUS;
    uint32_t lastStepTime_InUS;
    float currentStepPeriod_InUS;

    bool inverse_enablePin;
    bool is_moving;
    bool should_release;
    char motorName[20];
} FlexyStepper;

#ifdef MCU_ARDUINO
    void FlexyStepper_log(const char *format, ...);
#else
    void FlexyStepper_attach_timer_for_micros(TIM_HandleTypeDef* htim);
    void FlexyStepper_attach_logger(UART_HandleTypeDef * uart);
    void FlexyStepper_log(const char *format, ...);
#endif



//----------------------------------------------------------------
// Setup functions
void FlexyStepper_Init(FlexyStepper* stepper, char* name);
void FlexyStepper_en_motor(FlexyStepper* stepper, uint8_t state);
#ifdef MCU_ARDUINO
    void FlexyStepper_connectToPins(FlexyStepper* stepper, uint8_t stepPin, uint8_t directionPin);
    void FlexyStepper_connectEnablePin(FlexyStepper* stepper, uint8_t pin, bool inverse);
#else
    void FlexyStepper_connectToPins(FlexyStepper* stepper, GPIO_TypeDef* stepPort, uint16_t stepPin, 
                                GPIO_TypeDef* directionPort, uint16_t directionPin);
    void FlexyStepper_connectEnablePin(FlexyStepper* stepper, GPIO_TypeDef* port, uint16_t pin, bool inverse);
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
void FlexyStepper_setAccelerationInStepsPerSecondPerSecond(FlexyStepper* stepper, float accelerationInStepsPerSecondPerSecond);
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

void FlexyStepper_setSpeed(FlexyStepper* stepper, float speed);
void FlexyStepper_setAcceleration(FlexyStepper* stepper, float acceleration);

void FlexyStepper_setTargetPositionRelative(FlexyStepper* stepper, float distanceToMove, bool should_release);
void FlexyStepper_setTargetPosition(FlexyStepper* stepper, float absolutePositionToMoveTo, bool should_release);
float FlexyStepper_getCurrentVelocity(FlexyStepper* stepper);

void FlexyStepper_Estop(FlexyStepper* stepper);
void FlexyStepper_loop(FlexyStepper* stepper);


// MACROS

#ifdef MCU_ARDUINO
    #define WRITE_PIN(port, pin, value) digitalWrite(pin, value)
    #define GET_MICROS micros()
    #define DELAY_MICROS(micros) delayMicroseconds(micros)
#else
    #define WRITE_PIN(port, pin, value) HAL_GPIO_WritePin(port, pin, value)
    #defube GET_MICROS HAL_GetMicros()
    #define DELAY_MICROS(micros) HAL_DelayMicros(micros)
#endif

#ifdef __cplusplus
}
#endif

#endif // FLEXY_STEPPER_H

