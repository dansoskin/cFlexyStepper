#include "cFlexyStepper.h"

#define HOMING_DELAY_US 1000000	

const char *cFlexyStepper_homing_sm_states_strings[] =
	{	"HOMING_IDLE", "HOMING_MOVE_TOWARDS_LIMIT", "HOMING_DELAY1",
	"HOMING_MOVE_AWAY_FROM_LIMIT", "HOMING_DELAY2",
	"HOMING_ADJUST_POSITION", "HOMING_DELAY3"};


void set_cFlexyStepper_homing_sm_state(FlexyStepper* stepper, cFlexyStepper_homing_sm_states st)
{
	stepper->homing_sm_state = st;
	FlexyStepper_log("[%s]_homing_sm_state: %s\n", stepper->motorName, cFlexyStepper_homing_sm_states_strings[stepper->homing_sm_state]);
	stepper->homing_sm_timer = GET_MICROS;
}

cFlexyStepper_homing_sm_states get_cFlexyStepper_homing_sm_state(FlexyStepper* stepper)
{
	return stepper->homing_sm_state;
}

/*
	direction = 1 forward, direction = -1 backward, direction = 0 not set
*/
void FlexyStepper_set_homing(FlexyStepper* stepper, int8_t homing_direction, uint8_t homing_speed, float homing_adjust_position)
{
	stepper->homing_direction = homing_direction;
	stepper->homing_speed = homing_speed;
	stepper->homing_adjust_position = homing_adjust_position;
}


void start_cFlexyStepper_homing_sm(FlexyStepper* stepper)
{
	FlexyStepper_setSpeed(stepper, stepper->homing_speed);
}

void cFlexyStepper_homing_sm_loop(FlexyStepper* stepper, uint8_t limit_switch_state)
{
	if(stepper->homing_direction == 0)	//homing not set
		return;

	switch (stepper->homing_sm_state)
	{
		case HOMING_IDLE:
			break;
		case HOMING_MOVE_TOWARDS_LIMIT:
			break;
		case HOMING_DELAY1:
			// if(GET_MICROS - stepper->homing_sm_timer > HOMING_DELAY_US)
			break;
		case HOMING_MOVE_AWAY_FROM_LIMIT:
			break;
		case HOMING_DELAY2:
			break;
		case HOMING_ADJUST_POSITION:
			break;
		case HOMING_DELAY3:
			break;
		default:
			break;

	}
}