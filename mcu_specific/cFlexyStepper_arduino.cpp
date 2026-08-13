#include "../cFlexyStepper.h"
#ifdef MCU_ARDUINO

extern "C" void FlexyStepper_connectToPins(FlexyStepper* stepper, uint8_t stepPin, uint8_t directionPin)
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

extern "C" void FlexyStepper_connectEnablePin(FlexyStepper* stepper, uint8_t pin, bool inverse)
{
    // Remember the pin configuration
    stepper->enablePin = pin;
    stepper->inverse_enablePin = inverse;

    pinMode(stepper->enablePin, OUTPUT);
    digitalWrite(stepper->enablePin, stepper->inverse_enablePin ? HIGH : LOW);
}

// Driver alarm input. An active-low alarm is typically an open-collector
// output, so inverse gets the internal pullup; active high is left floating
// for the driver to drive.
extern "C" void FlexyStepper_connectFaultPin(FlexyStepper* stepper, uint8_t pin, bool inverse)
{
    stepper->faultPin = pin;
    stepper->inverse_faultPin = inverse;
    stepper->fault_pin_was_active = false;

    pinMode(pin, inverse ? INPUT_PULLUP : INPUT);
    stepper->fault_pin_connected = true;
}

// Driver reset output, pulsed by FlexyStepper_clearFault(). Parked at its
// inactive level here so connecting it never resets the driver by accident.
extern "C" void FlexyStepper_connectFaultClearPin(FlexyStepper* stepper, uint8_t pin, bool inverse)
{
    stepper->faultClearPin = pin;
    stepper->inverse_faultClearPin = inverse;
    stepper->fault_clear_pulse_active = false;

    pinMode(pin, OUTPUT);
    digitalWrite(pin, inverse ? HIGH : LOW);
    stepper->fault_clear_pin_connected = true;
}
#endif
