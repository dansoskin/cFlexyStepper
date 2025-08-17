
// #include "cFlexyStepper.h"

// // ---------------------------------------------------------------------------------
// //                     Functions with units in millimeters 
// // ---------------------------------------------------------------------------------

// //
// // Set the number of steps the motor has per millimeters
// //
// void FlexyStepper_setStepsPerMillimeter(FlexyStepper* stepper, float motorStepsPerMillimeter) {
//     stepper->stepsPerMillimeter = motorStepsPerMillimeter;
// }

// //
// // Get the current position of the motor in millimeters, this functions is updated
// // while the motor moves
// //  Exit:  a signed motor position in millimeters returned
// //
// float FlexyStepper_getCurrentPositionInMillimeters(FlexyStepper* stepper) {
//     return ((float)FlexyStepper_getCurrentPositionInSteps(stepper) / stepper->stepsPerMillimeter);
// }

// //
// // Set the current position of the motor in millimeters, this does not move the motor
// //
// void FlexyStepper_setCurrentPositionInMillimeters(FlexyStepper* stepper, float currentPositionInMillimeters) {
//     FlexyStepper_setCurrentPositionInSteps(stepper, (int32_t)round(currentPositionInMillimeters * 
//                                           stepper->stepsPerMillimeter));
// }

// //
// // Set the maximum speed, units in millimeters/second, this is the maximum speed  
// // reached while accelerating
// //  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in 
// //            millimeters/second
// //
// void FlexyStepper_setSpeedInMillimetersPerSecond(FlexyStepper* stepper, float speedInMillimetersPerSecond) {
//     FlexyStepper_setSpeedInStepsPerSecond(stepper, speedInMillimetersPerSecond * stepper->stepsPerMillimeter);
// }

// //
// // Set the rate of acceleration, units in millimeters/second/second
// //  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration,  
// //          units in millimeters/second/second
// //
// void FlexyStepper_setAccelerationInMillimetersPerSecondPerSecond(
//                       FlexyStepper* stepper, float accelerationInMillimetersPerSecondPerSecond) {
//     FlexyStepper_setAccelerationInStepsPerSecondPerSecond(stepper,
//       accelerationInMillimetersPerSecondPerSecond * stepper->stepsPerMillimeter);
// }


// //
// // Move relative to the current position, units are in millimeters, this function  
// // does not return until the move is complete
// //  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the  
// //          current position in millimeters
// //
// void FlexyStepper_moveRelativeInMillimeters(FlexyStepper* stepper, float distanceToMoveInMillimeters) {
//     FlexyStepper_setTargetPositionRelativeInMillimeters(stepper, distanceToMoveInMillimeters);
    
//     while(!FlexyStepper_processMovement(stepper)) {
//         // Keep checking and updating position until move is complete
//     }
// }

// //
// // Setup a move relative to the current position, units are in millimeters, no   
// // motion occurs until processMove() is called
// //  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the  
// //          current position in millimeters
// //
// void FlexyStepper_setTargetPositionRelativeInMillimeters(
//                      FlexyStepper* stepper, float distanceToMoveInMillimeters) {
//     FlexyStepper_setTargetPositionRelativeInSteps(stepper, (int32_t)round(distanceToMoveInMillimeters * 
//                                                 stepper->stepsPerMillimeter));
// }

// //
// // Move to the given absolute position, units are in millimeters, this function 
// // does not return until the move is complete
// //  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to  
// //          move to in units of millimeters
// //
// void FlexyStepper_moveToPositionInMillimeters(
//                     FlexyStepper* stepper, float absolutePositionToMoveToInMillimeters) {
//     FlexyStepper_setTargetPositionInMillimeters(stepper, absolutePositionToMoveToInMillimeters);
    
//     while(!FlexyStepper_processMovement(stepper)) {
//         // Keep checking and updating position until move is complete
//     }
// }

// //
// // Setup a move, units are in millimeters, no motion occurs until processMove() 
// // is called
// //  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to  
// //          move to in units of millimeters
// //
// void FlexyStepper_setTargetPositionInMillimeters(
//                     FlexyStepper* stepper, float absolutePositionToMoveToInMillimeters) {
//     FlexyStepper_setTargetPositionInSteps(stepper, (int32_t)round(absolutePositionToMoveToInMillimeters * 
//                                         stepper->stepsPerMillimeter));
// }

// //
// // Get the current velocity of the motor in millimeters/second
// //
// float FlexyStepper_getCurrentVelocityInMillimetersPerSecond(FlexyStepper* stepper) {
//     return FlexyStepper_getCurrentVelocityInStepsPerSecond(stepper) / stepper->stepsPerMillimeter;
// }