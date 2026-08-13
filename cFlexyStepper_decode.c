#include "cFlexyStepper.h"

/*requires buffer size of at least 526 bytes */
void FlexyStepper_decode_menu(char * buffer, size_t buffer_size)
{
    snprintf(buffer, buffer_size,
        "Stepper commands:\n"
        "E <0|1> - Enable/disable motor\n"
        "P <position> - Set current position\n"
        "p - Get current position\n"
        "V <speed> - Set speed\n"
        "v - Get current velocity\n"
        "A <acceleration> - Set acceleration\n"
        "a - Get current acceleration\n"
        "D <deceleration> - Set deceleration\n"
        "d - Get current deceleration\n"
        "J <speed> - Jog at speed\n"
        "R <distance> - Move relative to current position\n"
        "T <position> - Move to absolute position\n"
        "F - Restore default speed, acceleration, and deceleration\n"
        "k - Stop at target position\n"
        "K - Emergency stop and release motor\n"
        "s - Get status and fault count\n"
        "C - Clear fault (pulse the clear pin)\n");
}

void FlexyStepper_decode(FlexyStepper * stpr, char cmd, char* arg)
{
    switch(cmd)
	{
        case 'E':
            FlexyStepper_en_motor(stpr, atoi(arg));
            break;      

        case 'P':
            FlexyStepper_setCurrentPosition(stpr, atof(arg));
            break;

        case 'p':
            FlexyStepper_logf(stpr, "Current position: %.2f\n", FlexyStepper_getCurrentPosition(stpr));
            break;

        case 'V':
            FlexyStepper_setSpeed(stpr, atof(arg));
            break;

        case 'v':
            FlexyStepper_logf(stpr, "Current velocity: %.2f\n", FlexyStepper_getCurrentVelocity(stpr));
            break;
        
        case 'A':   /* also resets deceleration to match -- send @D after, not before */
            FlexyStepper_setAcceleration(stpr, atof(arg));
            break;

        case 'a':
            FlexyStepper_logf(stpr, "Current acceleration: %.2f\n", FlexyStepper_getAcceleration(stpr));
            break;

        case 'D':
            FlexyStepper_setDeceleration(stpr, atof(arg));
            break;

        case 'd':
            FlexyStepper_logf(stpr, "Current deceleration: %.2f\n", FlexyStepper_getDeceleration(stpr));
            break;

        case 'J':
            FlexyStepper_jog(stpr, atof(arg));
            break;

        case 'R':
            FlexyStepper_setTargetPositionRelative(stpr, atof(arg), false);
            break;

        case 'T':
            FlexyStepper_setTargetPosition(stpr, atof(arg), false);
            break;

        case 'F':   
            FlexyStepper_restoreDefaults(stpr);
            break;

        case 'k':
            FlexyStepper_setTargetPositionToStop(stpr);
            break;

        case 'K':
            FlexyStepper_Estop(stpr, true);
            break;

        case 's':
            FlexyStepper_logf(stpr, "Status: %s, faults: %lu\n",
                              FlexyStepper_status_str(FlexyStepper_getStatus(stpr)),
                              (unsigned long)stpr->fault_count);
            break;

        case 'C':
            FlexyStepper_clearFault(stpr);
            break;


        default:
            FlexyStepper_logf(stpr, "Unknown stepper command: %c\n", cmd);
            break;
    }
}