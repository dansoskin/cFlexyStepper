#include "cFlexyStepper.h"


void FlexyStepper_setConversion(FlexyStepper* stepper, float conversion) {
    stepper->conversion = conversion;
}

float FlexyStepper_getCurrentPosition(FlexyStepper* stepper) {
    return ((float)FlexyStepper_getCurrentPositionInSteps(stepper) / stepper->conversion);
}

void FlexyStepper_setCurrentPosition(FlexyStepper* stepper, float position) {
    FlexyStepper_log("[%s] Current Position: %.4f\r\n", stepper->motorName, position);
    FlexyStepper_setCurrentPositionInSteps(stepper, (int32_t)round(position * stepper->conversion));
}

void FlexyStepper_setSpeed(FlexyStepper* stepper, float speed) {
    FlexyStepper_log("[%s] Speed: %.4f\r\n", stepper->motorName, speed);
    FlexyStepper_setSpeedInStepsPerSecond(stepper, speed * stepper->conversion);
}

void FlexyStepper_setAcceleration(FlexyStepper* stepper, float acceleration) {
    FlexyStepper_log("[%s] Acceleration: %.4f\r\n", stepper->motorName, acceleration);
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, acceleration * stepper->conversion);
}

/*
void FlexyStepper_moveRelativeInMillimeters(FlexyStepper* stepper, float distanceToMoveInMillimeters) {
    FlexyStepper_setTargetPositionRelativeInMillimeters(stepper, distanceToMoveInMillimeters);
    
    while(!FlexyStepper_processMovement(stepper)) {
        // Keep checking and updating position until move is complete
    }
}
void FlexyStepper_moveToPositionInMillimeters(
    FlexyStepper* stepper, float absolutePositionToMoveToInMillimeters) {
        FlexyStepper_setTargetPositionInMillimeters(stepper, absolutePositionToMoveToInMillimeters);
        
        while(!FlexyStepper_processMovement(stepper)) {
            // Keep checking and updating position until move is complete
        }
    }
*/

void FlexyStepper_setTargetPositionRelative(FlexyStepper* stepper, float distanceToMove, bool should_release) {
    FlexyStepper_log("[%s] Target Position Relative: %.4f\r\n", stepper->motorName, distanceToMove);
    FlexyStepper_en_motor(stepper, 1);
    FlexyStepper_setTargetPositionRelativeInSteps(stepper, (int32_t)round(distanceToMove * stepper->conversion));
    stepper->is_moving = true;
    stepper->should_release = should_release;
}
    
void FlexyStepper_setTargetPosition(FlexyStepper* stepper, float absolutePositionToMoveTo, bool should_release) {
    FlexyStepper_log("[%s] Target Position: %.4f\r\n", stepper->motorName, absolutePositionToMoveTo);
    FlexyStepper_en_motor(stepper, 1);
    FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(absolutePositionToMoveTo * stepper->conversion));
    stepper->is_moving = true;
    stepper->should_release = should_release;
}

float FlexyStepper_getCurrentVelocity(FlexyStepper* stepper) {
    return FlexyStepper_getCurrentVelocityInStepsPerSecond(stepper) / stepper->conversion;
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

    
}
//-------------------------------------------------------------------------------
void FlexyStepper_Estop(FlexyStepper* stepper) {
    stepper->directionOfMotion = 0;
    stepper->targetPosition_InSteps = stepper->currentPosition_InSteps;

    FlexyStepper_loop(stepper);
    FlexyStepper_log("[%s] Estop\r\n", stepper->motorName);
}


void FlexyStepper_loop(FlexyStepper* stepper) {
    //auto disable
    if(stepper->is_moving)
    {
        if(!FlexyStepper_motionComplete(stepper)) {
            FlexyStepper_processMovement(stepper);
        }
        else
        {
            FlexyStepper_log("[%s] movement finished\r\n", stepper->motorName);
            stepper->is_moving = false;

            if(stepper->should_release)
            {
                FlexyStepper_en_motor(stepper, 0);
                stepper->should_release = false;
            }
        }
    }
}