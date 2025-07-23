//      ******************************************************************
//      *                                                                *
//      *                    FlexyStepper Motor Driver                   *
//      *                                                                *
//      *            Stan Reifel                     12/8/2014           *
//      *               Copyright (c) S. Reifel & Co, 2014               *
//      *                                                                *
//      ******************************************************************


// MIT License
// 
// Copyright (c) 2014 Stanley Reifel & Co.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


//
// This library is used to control one or more stepper motors.  It requires a 
// stepper driver board that has a Step and Direction interface.  The motors are 
// accelerated and decelerated as they travel to the final position.  This driver
// supports changing the target position, speed or rate of acceleration while a
// motion is in progress.
//
// Because the library allows making changes while moving, it can not generate
// as fast of a step rate as a driver that requires each motion to complete.
//

#include "cFlexyStepper.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>



//-----------------------------------------------------------------

//
// Initialize the stepper structure
//
void FlexyStepper_Init(FlexyStepper* stepper, char* name) {
    // Initialize constants
    stepper->stepsPerRevolution = 200;
    stepper->stepsPerMillimeter = 25.0;
    stepper->conversion = 1.0;
    
    stepper->directionOfMotion = 0;
    stepper->currentPosition_InSteps = 0;
    stepper->targetPosition_InSteps = 0;
    
    // Set default speed
    FlexyStepper_setSpeedInStepsPerSecond(stepper, 200);
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, 200.0);
    
    stepper->currentStepPeriod_InUS = 0.0;
    stepper->nextStepPeriod_InUS = 0.0;

    // Save the name of the motor
    strncpy(stepper->motorName, name, sizeof(stepper->motorName) - 1);
    stepper->motorName[sizeof(stepper->motorName) - 1] = '\0'; // Ensure null termination
}




// ---------------------------------------------------------------------------------
//                        Functions with units in steps 
// ---------------------------------------------------------------------------------

//
// Set the current position of the motor in steps, this does not move the motor
// Note: This function should only be called when the motor is stopped
//    Enter:  currentPositionInSteps = the new position of the motor in steps
//
void FlexyStepper_setCurrentPositionInSteps(FlexyStepper* stepper, int32_t currentPositionInSteps) {
    stepper->currentPosition_InSteps = currentPositionInSteps;
}

//
// Get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
int32_t FlexyStepper_getCurrentPositionInSteps(FlexyStepper* stepper) {
    return stepper->currentPosition_InSteps;
}

//
// Set the maximum speed, units in steps/second, this is the maximum speed reached  
// while accelerating
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
//TODO see what happens if speed is set to 0
void FlexyStepper_setSpeedInStepsPerSecond(FlexyStepper* stepper, float speedInStepsPerSecond) {
    stepper->desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
    stepper->desiredPeriod_InUSPerStep = 1000000.0 / speedInStepsPerSecond;
}

//
// Set the rate of acceleration, units in steps/second/second
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in 
//          steps/second/second
//
void FlexyStepper_setAccelerationInStepsPerSecondPerSecond(
                     FlexyStepper* stepper, float accelerationInStepsPerSecondPerSecond) {
    stepper->acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
    stepper->acceleration_InStepsPerUSPerUS = accelerationInStepsPerSecondPerSecond / 1E12;

    stepper->periodOfSlowestStep_InUS = 
        1000000.0 / sqrt(2.0 * accelerationInStepsPerSecondPerSecond);
    stepper->minimumPeriodForAStoppedMotion = stepper->periodOfSlowestStep_InUS / 2.8;
}

//
// Move relative to the current position, units are in steps, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//          position in steps
//
void FlexyStepper_moveRelativeInSteps(FlexyStepper* stepper, int32_t distanceToMoveInSteps) {
    FlexyStepper_setTargetPositionRelativeInSteps(stepper, distanceToMoveInSteps);
    
    while(!FlexyStepper_processMovement(stepper)) {
        // Keep checking and updating position until move is complete
    }
}

//
// Setup a move relative to the current position, units are in steps, no motion  
// occurs until processMove() is called
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//            positionin steps
//
void FlexyStepper_setTargetPositionRelativeInSteps(FlexyStepper* stepper, int32_t distanceToMoveInSteps) {
    FlexyStepper_setTargetPositionInSteps(stepper, stepper->currentPosition_InSteps + distanceToMoveInSteps);
}

//
// Move to the given absolute position, units are in steps, this function does not 
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in unitsof steps
//
void FlexyStepper_moveToPositionInSteps(FlexyStepper* stepper, int32_t absolutePositionToMoveToInSteps) {
    FlexyStepper_setTargetPositionInSteps(stepper, absolutePositionToMoveToInSteps);
    
    while(!FlexyStepper_processMovement(stepper)) {
        // Keep checking and updating position until move is complete
    }
}

//
// Setup a move, units are in steps, no motion occurs until processMove() is called
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in units of steps
//
void FlexyStepper_setTargetPositionInSteps(FlexyStepper* stepper, int32_t absolutePositionToMoveToInSteps) {
    stepper->targetPosition_InSteps = absolutePositionToMoveToInSteps;
}

//
// Setup a "Stop" to begin the process of decelerating from the current velocity  
// to zero, decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps 
// or revolutions
//
void FlexyStepper_setTargetPositionToStop(FlexyStepper* stepper) {
    int32_t decelerationDistance_InSteps;
    
    // Move the target position so that the motor will begin deceleration now
    decelerationDistance_InSteps = (int32_t)round(
        5E11 / (stepper->acceleration_InStepsPerSecondPerSecond * stepper->currentStepPeriod_InUS * 
        stepper->currentStepPeriod_InUS));

    if (stepper->directionOfMotion > 0)
        FlexyStepper_setTargetPositionInSteps(stepper, stepper->currentPosition_InSteps + decelerationDistance_InSteps);
    else
        FlexyStepper_setTargetPositionInSteps(stepper, stepper->currentPosition_InSteps - decelerationDistance_InSteps);
}

//
// Check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool FlexyStepper_motionComplete(FlexyStepper* stepper) {
    if ((stepper->directionOfMotion == 0) && 
        (stepper->currentPosition_InSteps == stepper->targetPosition_InSteps))
        return true;
    else
        return false;
}

//
// Get the current velocity of the motor in steps/second
//
float FlexyStepper_getCurrentVelocityInStepsPerSecond(FlexyStepper* stepper) {
    if (stepper->currentStepPeriod_InUS == 0.0)
        return 0;
    else {
        if (stepper->directionOfMotion > 0)
            return 1000000.0 / stepper->currentStepPeriod_InUS;
        else
            return -1000000.0 / stepper->currentStepPeriod_InUS;
    }
}

//
// Helper function for computing the period of the next step, 
// either speed up a little, slow down a little or go the same speed
//
void FlexyStepper_DeterminePeriodOfNextStep(FlexyStepper* stepper) {
    int32_t distanceToTarget_Signed;
    uint32_t distanceToTarget_Unsigned;
    // float currentSpeed_InStepsPerSecond;
    int32_t decelerationDistance_InSteps;
    float currentStepPeriodSquared;
    bool speedUpFlag = false;
    bool slowDownFlag = false;
    bool targetInPositiveDirectionFlag = false;
    bool targetInNegativeDirectionFlag = false;

    // Determine the distance to the target position
    distanceToTarget_Signed = stepper->targetPosition_InSteps - stepper->currentPosition_InSteps;
    if (distanceToTarget_Signed >= 0) {
        distanceToTarget_Unsigned = distanceToTarget_Signed;
        targetInPositiveDirectionFlag = true;
    }
    else {
        distanceToTarget_Unsigned = -distanceToTarget_Signed;
        targetInNegativeDirectionFlag = true;
    }

    // Determine the number of steps needed to go from the current speed down to a 
    // velocity of 0, Steps = Velocity^2 / (2 * Acceleration)
    currentStepPeriodSquared = stepper->currentStepPeriod_InUS * stepper->currentStepPeriod_InUS;
    decelerationDistance_InSteps = (int32_t)round(
        5E11 / (stepper->acceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));
    
    // Check if: Moving in a positive direction & Moving toward the target
    if ((stepper->directionOfMotion == 1) && (targetInPositiveDirectionFlag)) {
        // Check if need to start slowing down as we reach the target, or if we 
        // need to slow down because we are going too fast
        if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) || 
            (stepper->nextStepPeriod_InUS < stepper->desiredPeriod_InUSPerStep))
            slowDownFlag = true;
        else 
            speedUpFlag = true;
    }
    // Check if: Moving in a positive direction & Moving away from the target
    else if ((stepper->directionOfMotion == 1) && (targetInNegativeDirectionFlag)) {
        // Need to slow down, then reverse direction
        if (stepper->currentStepPeriod_InUS < stepper->periodOfSlowestStep_InUS) {
            slowDownFlag = true;
        }
        else {
            stepper->directionOfMotion = -1;
            
            //XXX
            WRITE_PIN(stepper->directionPort, stepper->directionPin, NEGATIVE_DIRECTION == 0 ? 0 : 1);

            // HAL_GPIO_WritePin(stepper->directionPort, stepper->directionPin, 
            //                 NEGATIVE_DIRECTION == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
    // Check if: Moving in a negative direction & Moving toward the target
    else if ((stepper->directionOfMotion == -1) && (targetInNegativeDirectionFlag)) {
        // Check if need to start slowing down as we reach the target, or if we 
        // need to slow down because we are going too fast
        if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) || 
            (stepper->nextStepPeriod_InUS < stepper->desiredPeriod_InUSPerStep))
            slowDownFlag = true;
        else 
            speedUpFlag = true;
    }
    // Check if: Moving in a negative direction & Moving away from the target
    else if ((stepper->directionOfMotion == -1) && (targetInPositiveDirectionFlag)) {
        // Need to slow down, then reverse direction
        if (stepper->currentStepPeriod_InUS < stepper->periodOfSlowestStep_InUS) {
            slowDownFlag = true;
        }
        else {
            stepper->directionOfMotion = 1;
            //XXX
            WRITE_PIN(stepper->directionPort, stepper->directionPin, POSITIVE_DIRECTION == 0 ? 0 : 1);
            // HAL_GPIO_WritePin(stepper->directionPort, stepper->directionPin, 
            //                 POSITIVE_DIRECTION == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }                      
    }

    // Check if accelerating
    if (speedUpFlag) {
        // StepPeriod = StepPeriod(1 - a * StepPeriod^2)
        stepper->nextStepPeriod_InUS = stepper->currentStepPeriod_InUS - stepper->acceleration_InStepsPerUSPerUS * 
        currentStepPeriodSquared * stepper->currentStepPeriod_InUS;

        if (stepper->nextStepPeriod_InUS < stepper->desiredPeriod_InUSPerStep)
            stepper->nextStepPeriod_InUS = stepper->desiredPeriod_InUSPerStep;
    }
    
    // Check if decelerating
    if (slowDownFlag) {
        // StepPeriod = StepPeriod(1 + a * StepPeriod^2)
        stepper->nextStepPeriod_InUS = stepper->currentStepPeriod_InUS + stepper->acceleration_InStepsPerUSPerUS * 
        currentStepPeriodSquared * stepper->currentStepPeriod_InUS;

        if (stepper->nextStepPeriod_InUS > stepper->periodOfSlowestStep_InUS)
            stepper->nextStepPeriod_InUS = stepper->periodOfSlowestStep_InUS;
    }
}

//
// If it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target 
//           position yet
//
bool FlexyStepper_processMovement(FlexyStepper* stepper) { 
    uint32_t currentTime_InUS;
    uint32_t periodSinceLastStep_InUS;
    int32_t distanceToTarget_Signed;

    // Check if currently stopped
    if (stepper->directionOfMotion == 0) {
        distanceToTarget_Signed = stepper->targetPosition_InSteps - stepper->currentPosition_InSteps;

        // Check if target position in a positive direction
        if (distanceToTarget_Signed > 0) {
            stepper->directionOfMotion = 1;
            //XXX
            WRITE_PIN(stepper->directionPort, stepper->directionPin, POSITIVE_DIRECTION == 0 ? 0 : 1);
            // HAL_GPIO_WritePin(stepper->directionPort, stepper->directionPin, 
            //                 POSITIVE_DIRECTION == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
            stepper->nextStepPeriod_InUS = stepper->periodOfSlowestStep_InUS;

            //XXX
            stepper->lastStepTime_InUS = GET_MICROS;
            // stepper->lastStepTime_InUS = HAL_GetMicros(); 
            return false;
        }
        
        // Check if target position in a negative direction
        else if (distanceToTarget_Signed < 0) {
            stepper->directionOfMotion = -1;
            //XXX
            WRITE_PIN(stepper->directionPort, stepper->directionPin, NEGATIVE_DIRECTION == 0 ? 0 : 1);
            // HAL_GPIO_WritePin(stepper->directionPort, stepper->directionPin, 
            //                 NEGATIVE_DIRECTION == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
            stepper->nextStepPeriod_InUS = stepper->periodOfSlowestStep_InUS;
            //XXX
            stepper->lastStepTime_InUS = GET_MICROS;
            // stepper->lastStepTime_InUS = HAL_GetMicros(); 
            return false;
        }
        
        else
            return true;
    }
       
    // Determine how much time has elapsed since the last step
    //XXX
    // currentTime_InUS = HAL_GetMicros();
    currentTime_InUS =  GET_MICROS;
    periodSinceLastStep_InUS = currentTime_InUS - stepper->lastStepTime_InUS;

    // If it is not time for the next step, return
    if (periodSinceLastStep_InUS < (uint32_t)stepper->nextStepPeriod_InUS)
        return false;
    
    // Execute the step on the rising edge
    // HAL_GPIO_WritePin(stepper->stepPort, stepper->stepPin, GPIO_PIN_SET);
    WRITE_PIN(stepper->stepPort, stepper->stepPin, 1);
    
    // This delay is almost nothing because there's so much code between rising & falling edges
    //XXX
    // HAL_DelayMicros(2); 
    DELAY_MICROS(2); // Delay for 2 microseconds to allow the stepper driver to register the step      
    
    // Update the current position and speed
    stepper->currentPosition_InSteps += stepper->directionOfMotion;
    stepper->currentStepPeriod_InUS = stepper->nextStepPeriod_InUS;

    // Remember the time that this step occurred
    stepper->lastStepTime_InUS = currentTime_InUS;
 
    // Figure out how long before the next step
    FlexyStepper_DeterminePeriodOfNextStep(stepper);
 
    // Return the step line low
    // HAL_GPIO_WritePin(stepper->stepPort, stepper->stepPin, GPIO_PIN_RESET);
    WRITE_PIN(stepper->stepPort, stepper->stepPin, 0);

    // Check if the move has reached its final target position, return true if all done
    if (stepper->currentPosition_InSteps == stepper->targetPosition_InSteps) {
        // At final position, make sure the motor is not going too fast
        if (stepper->nextStepPeriod_InUS >= stepper->minimumPeriodForAStoppedMotion) {
            stepper->currentStepPeriod_InUS = 0.0;
            stepper->nextStepPeriod_InUS = 0.0;
            stepper->directionOfMotion = 0;
            return true;
        }
    }
        
    return false;
}