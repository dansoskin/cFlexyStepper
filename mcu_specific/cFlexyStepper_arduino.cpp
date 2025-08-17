#include "../cFlexyStepper.h"
#ifdef MCU_ARDUINO


// Logging function for FlexyStepper using Arduino Serial
extern "C" void FlexyStepper_log(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
}

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
}
#endif
