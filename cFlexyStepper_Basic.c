#include "cFlexyStepper.h"


void FlexyStepper_setConversion(FlexyStepper* stepper, float conversion) {
    stepper->conversion = conversion;
}

//-------------------
float FlexyStepper_getCurrentPosition(FlexyStepper* stepper) {
    return ((float)FlexyStepper_getCurrentPositionInSteps(stepper) / stepper->conversion);
}

void FlexyStepper_setCurrentPosition(FlexyStepper* stepper, float position) {
    FlexyStepper_log("[%s] Current Position: %.4f\r\n", stepper->motorName, position);
    FlexyStepper_setCurrentPositionInSteps(stepper, (int32_t)round(position * stepper->conversion));
}

//-------------------
float FlexyStepper_getCurrentVelocity(FlexyStepper* stepper) {
    return FlexyStepper_getCurrentVelocityInStepsPerSecond(stepper) / stepper->conversion;
}

void FlexyStepper_setSpeed(FlexyStepper* stepper, float speed) {
    FlexyStepper_log("[%s] Speed: %.4f\r\n", stepper->motorName, speed);
    FlexyStepper_setSpeedInStepsPerSecond(stepper, speed * stepper->conversion);
}

void FlexyStepper_setAcceleration(FlexyStepper* stepper, float acceleration) {
    FlexyStepper_log("[%s] Acceleration: %.4f\r\n", stepper->motorName, acceleration);
    FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper, acceleration * stepper->conversion);
}

float FlexyStepper_getTargetSpeed(FlexyStepper* stepper)
{
	return stepper->desiredSpeed_InStepsPerSecond / stepper->conversion;
}

void FlexyStepper_jog(FlexyStepper * stepper, float speed)
{
	FlexyStepper_log("[%s] Jog: %.4f\r\n", stepper->motorName, speed);

	if(speed > 0)
	{
		FlexyStepper_setSpeedInStepsPerSecond(stepper, speed * stepper->conversion);

		// Reset current motion state to start acceleration from zero
		stepper->currentStepPeriod_InUS = 0.0;
		stepper->nextStepPeriod_InUS = 0.0;
		stepper->directionOfMotion = 0;

		FlexyStepper_en_motor(stepper, 1);
		FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(9999999 * stepper->conversion));
		stepper->is_moving = true;
		stepper->should_release = 1;
	}
	else if(speed < 0)
	{
		FlexyStepper_setSpeedInStepsPerSecond(stepper, -1*speed * stepper->conversion);

		// Reset current motion state to start acceleration from zero
		stepper->currentStepPeriod_InUS = 0.0;
		stepper->nextStepPeriod_InUS = 0.0;
		stepper->directionOfMotion = 0;

		FlexyStepper_en_motor(stepper, 1);
		FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(-9999999 * stepper->conversion));
		stepper->is_moving = true;
		stepper->should_release = 1;
	}
	else
	{
		FlexyStepper_Estop(stepper);
	}
}

//-------------------
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
    
    // Reset current motion state but preserve desiredSpeed for future movements
    stepper->currentStepPeriod_InUS = 0.0;
    stepper->nextStepPeriod_InUS = 0.0;

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
