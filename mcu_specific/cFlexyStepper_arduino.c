#include "../cFlexyStepper.h"

#ifdef MCU_ARDUINO

void FlexyStepper_log(const char *format, ...)
{

}

void FlexyStepper_connectToPins(FlexyStepper* stepper, uint8_t stepPin, uint8_t directionPin)
{
    // Remember the pin configurations
    stepper->stepPin = stepPin;
    stepper->directionPin = directionPin;

    // Set step pin to output and default LOW
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);

    // Set direction pin to output and default LOW (positive direction)
    pinMode(directionPin, OUTPUT);
    digitalWrite(directionPin, LOW);
}

void FlexyStepper_connectEnablePin(FlexyStepper* stepper, uint8_t pin, bool inverse)
{
    // Remember the pin configuration
    stepper->enablePin = pin;
    stepper->inverse_enablePin = inverse;

    // Set enable pin to output and default HIGH (disabled)
    pinMode(pin, OUTPUT);
    digitalWrite(pin, inverse ? HIGH : LOW);
}



#endif