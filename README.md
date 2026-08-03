# FlexyStepper Library for STM32

A C port of the amazing FlexyStepper library originally created by Stan Reifel, with some personal adjustments.

source: https://github.com/Stan-Reifel/FlexyStepper

## Overview

This library is designed for STM32 microcontrollers but can be easily adapted to other platforms. It provides smooth stepper motor control with acceleration and deceleration profiles.

## Example Usage

```c
// Initialize the stepper motor
FlexyStepper_attach_timer_for_micros(&htim23);
FlexyStepper_attach_logger(&huart6);

FlexyStepper stepper;
FlexyStepper_Init(&stepper, "Stepper1");

// Connect to hardware pins
FlexyStepper_connectToPins(&stepper, PUL_GPIO_Port, PUL_Pin, DIR_GPIO_Port, DIR_Pin);
FlexyStepper_connectEnablePin(&stepper, EN_GPIO_Port, EN_Pin, true);

// Configure motion parameters
FlexyStepper_setConversion(&stepper, 3200.0);  // Steps per unit (e.g., steps per revolution or mm)

// Applies speed/accel/decel now and records them as this motor's baseline
FlexyStepper_setDefaults(&stepper, /*speed=*/4, /*accel=*/10, /*decel=*/15);

// Move 10 units in the positive direction
FlexyStepper_setTargetPositionRelative(&stepper, 10, true);

// Main loop
while (1) {
    FlexyStepper_loop(&stepper);
}
```

## Defaults

`FlexyStepper_setDefaults()` records a baseline and applies it. Override speed or
either ramp freely for one move, then come back with one call:

```c
FlexyStepper_setSpeed(&stepper, 20);           // fast approach, just this once
FlexyStepper_setDeceleration(&stepper, 200);
FlexyStepper_setTargetPosition(&stepper, 90, false);
// ... move finishes ...
FlexyStepper_restoreDefaults(&stepper);        // speed, accel and decel all back
```

To put only part of the baseline back, read it and apply what you want —
useful when e.g. the operator's chosen speed must survive the move:

```c
FlexyStepper_setAcceleration(&stepper, FlexyStepper_getDefaultAcceleration(&stepper));
FlexyStepper_setDeceleration(&stepper, FlexyStepper_getDefaultDeceleration(&stepper));
```

Note the ordering rule this handles for you: `setAcceleration` resets the decel
ramp to match itself, so deceleration must always be applied *after* it.

## Logging

Logging is on after `FlexyStepper_Init()`. Every trace carries a `[motorName]`
prefix and is gated per motor, so a motor commanded at high rate can be muted
without silencing the others:

```c
FlexyStepper_enable_logging(&stepper, false);
for (;;) {                                  // e.g. a cyclic setpoint stream
    FlexyStepper_setTargetPosition(&stepper, next_setpoint(), false);
    FlexyStepper_loop(&stepper);
    if (done) break;
}
FlexyStepper_enable_logging(&stepper, true);
```

The call is idempotent — re-asserting the current state emits nothing, so it is
safe to call every pass of a loop.

## License

This library is based on FlexyStepper by Stan Reifel, which is available under the MIT License.