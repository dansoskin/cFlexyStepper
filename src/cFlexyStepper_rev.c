#include "cFlexyStepper.h"


// ---------------------------------------------------------------------------------
//                     Functions with units in revolutions 
// ---------------------------------------------------------------------------------

//
// Set the number of steps the motor has per revolution
//
void FlexyStepper_setStepsPerRevolution(FlexyStepper* stepper, float motorStepPerRevolution) {
    stepper->stepsPerRevolution = motorStepPerRevolution;
}

//
// Get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float FlexyStepper_getCurrentPositionInRevolutions(FlexyStepper* stepper) {
    return ((float)FlexyStepper_getCurrentPositionInSteps(stepper) / stepper->stepsPerRevolution);
}

//
// Set the current position of the motor in revolutions, this does not move the motor
//
void FlexyStepper_setCurrentPositionInRevolutions(
                     FlexyStepper* stepper, float currentPositionInRevolutions) {
    FlexyStepper_setCurrentPositionInSteps(stepper, (int32_t)round(currentPositionInRevolutions * 
                                          stepper->stepsPerRevolution));
}

//
// Set the maximum speed, units in revolutions/second, this is the maximum speed  
// reached while accelerating
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in 
//            revolutions/second
//
void FlexyStepper_setSpeedInRevolutionsPerSecond(FlexyStepper* stepper, float speedInRevolutionsPerSecond) {
    FlexyStepper_setSpeedInStepsPerSecond(stepper, speedInRevolutionsPerSecond * stepper->stepsPerRevolution);
}

//
// Set the rate of acceleration, units in revolutions/second/second
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration,  
//          units in revolutions/second/second
//
void FlexyStepper_setAccelerationInRevolutionsPerSecondPerSecond(
       FlexyStepper* stepper, float accelerationInRevolutionsPerSecondPerSecond) {
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper,
      accelerationInRevolutionsPerSecondPerSecond * stepper->stepsPerRevolution);
}


//
// Move relative to the current position, units are in revolutions, this function  
// does not return until the move is complete
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the  
//          current position in revolutions
//
void FlexyStepper_moveRelativeInRevolutions(FlexyStepper* stepper, float distanceToMoveInRevolutions) {
    FlexyStepper_setTargetPositionRelativeInRevolutions(stepper, distanceToMoveInRevolutions);
    
    while(!FlexyStepper_processMovement(stepper)) {
        // Keep checking and updating position until move is complete
    }
}

//
// Setup a move relative to the current position, units are in revolutions, no   
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the  
//            currentposition in revolutions
//
void FlexyStepper_setTargetPositionRelativeInRevolutions(
                     FlexyStepper* stepper, float distanceToMoveInRevolutions) {
    FlexyStepper_setTargetPositionRelativeInSteps(stepper, (int32_t)round(distanceToMoveInRevolutions * 
                                                stepper->stepsPerRevolution));
}

//
// Move to the given absolute position, units are in revolutions, this function 
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to 
//            move to in units of revolutions
//
void FlexyStepper_moveToPositionInRevolutions(
                    FlexyStepper* stepper, float absolutePositionToMoveToInRevolutions) {
    FlexyStepper_setTargetPositionInRevolutions(stepper, absolutePositionToMoveToInRevolutions);
    
    while(!FlexyStepper_processMovement(stepper)) {
        // Keep checking and updating position until move is complete
    }
}

//
// Setup a move, units are in revolutions, no motion occurs until processMove() 
// is called
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to  
//          move to in units of revolutions
//
void FlexyStepper_setTargetPositionInRevolutions(
       FlexyStepper* stepper, float absolutePositionToMoveToInRevolutions) {
    FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(absolutePositionToMoveToInRevolutions * 
                                        stepper->stepsPerRevolution));
}

//
// Get the current velocity of the motor in revolutions/second
//
float FlexyStepper_getCurrentVelocityInRevolutionsPerSecond(FlexyStepper* stepper) {
    return FlexyStepper_getCurrentVelocityInStepsPerSecond(stepper) / stepper->stepsPerRevolution;
}