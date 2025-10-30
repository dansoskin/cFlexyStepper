#include "../cFlexyStepper.h"

#ifdef MCU_STM32

// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------
TIM_HandleTypeDef * micros_tim = NULL;

//provide a timer with a 1us tick time
void FlexyStepper_attach_timer_for_micros(TIM_HandleTypeDef* htim) {
    micros_tim = htim;
    HAL_TIM_Base_Start(micros_tim);
}

// Get microseconds timing (similar to Arduino's micros())
uint32_t HAL_GetMicros(void) {
    if (micros_tim == NULL) {
        // Fallback to HAL_GetTick if timer not initialized
        return HAL_GetTick() * 1000;
        
    }
    
    // Use the hardware timer for microsecond precision
    return __HAL_TIM_GET_COUNTER(micros_tim);
}

// Microseconds delay (similar to Arduino's delayMicroseconds())
void HAL_DelayMicros(uint32_t micros) {
    uint32_t start = HAL_GetMicros();
    while ((HAL_GetMicros() - start) < micros);
}

//-------------------------------------------------
UART_HandleTypeDef * logger_uart = NULL;

void FlexyStepper_attach_logger(UART_HandleTypeDef * uart) {
    logger_uart = uart;
}

void FlexyStepper_log(const char *format, ...)
{
    if(logger_uart == NULL) {
        return; // No UART handler attached
    }

	char buffer[64]; // Adjust the buffer size as needed
	
    va_list args;     // Declare a variable of type va_list
	va_start(args, format); // Initialize args to store all values after format
	
    // Convert float values in the format string to double
    // This ensures proper handling of floating-point values
    int len = vsnprintf(buffer, sizeof(buffer), format, args); // Format the string with the arguments
	
    va_end(args); // Clean up the va_list variable

    // Check if formatting was successful
    if (len > 0) {
        HAL_UART_Transmit(logger_uart, (uint8_t *) buffer, len, 0xFFFF);
    }
}

//-------------------------------------------------------

//
// Connect the stepper object to the IO pins
//  Enter:  stepper = pointer to the stepper object
//          stepPort = GPIO port for the Step pin
//          stepPin = GPIO pin number for the Step
//          directionPort = GPIO port for the Direction pin
//          directionPin = GPIO pin number for the direction bit
//
void FlexyStepper_connectToPins(FlexyStepper* stepper, GPIO_TypeDef* stepPort, uint16_t stepPin, 
                             GPIO_TypeDef* directionPort, uint16_t directionPin) {
    // Remember the pin configurations
    stepper->stepPort = stepPort;
    stepper->stepPin = stepPin;
    stepper->directionPort = directionPort;
    stepper->directionPin = directionPin;
    
    // Set step pin to output and default LOW
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);
    
    // Set direction pin to output and default LOW (positive direction)
    HAL_GPIO_WritePin(directionPort, directionPin, GPIO_PIN_RESET);
}

void FlexyStepper_connectEnablePin(FlexyStepper* stepper, GPIO_TypeDef* port, uint16_t pin, bool inverse) {
    stepper->inverse_enablePin = inverse;
    stepper->enablePort = port;
    stepper->enablePin = pin;

    FlexyStepper_en_motor(stepper, 0);
}

#endif
