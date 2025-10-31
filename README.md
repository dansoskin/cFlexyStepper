# FlexyStepper Library for STM32

A C port of the amazing FlexyStepper library originally created by Stan Reifel, with some personal adjustments.

source: https://github.com/Stan-Reifel/FlexyStepper

## Overview

This library is designed for STM32 microcontrollers but can be easily adapted to other platforms. It provides smooth stepper motor control with acceleration and deceleration profiles.

## Example Usage

```c
// Initialize the stepper motor
FlexyStepper_attach_timer_for_micros(&htim23);
FlexyStepper_attach_logger(&huart3);

FlexyStepper stepper;
FlexyStepper_Init(&stepper, "Stepper1");

// Connect to hardware pins
FlexyStepper_connectToPins(&stepper, PUL_GPIO_Port, PUL_Pin, DIR_GPIO_Port, DIR_Pin);
FlexyStepper_connectEnablePin(&stepper, EN_GPIO_Port, EN_Pin, true);

// Configure motion parameters
FlexyStepper_setConversion(&stepper, 3200.0);  // Steps per unit (e.g., steps per revolution or mm)
FlexyStepper_setAcceleration(&stepper, 10);    // Units per second²
FlexyStepper_setSpeed(&stepper, 4);            // Units per second

// Move 10 units in the positive direction
FlexyStepper_setTargetPositionRelative(&stepper, 10, true);

// Main loop
while (1) {
    FlexyStepper_loop(&stepper);
}
```

## License

This library is based on FlexyStepper by Stan Reifel, which is available under the MIT License.