#include "cFlexyStepper.h"

#define HOMING_DELAY_US 1000000	
#define MOVING_TOWARDS_LIMIT_TIMEOUT_US 30000000 
#define MOVING_AWAY_FROM_LIMIT_TIMEOUT_US 50000000 
#define HOMING_ERROR_TIMEOUT_US 2 * 60000000

const char *cFlexyStepper_homing_sm_states_strings[] =
	{	"HOMING_IDLE", "HOMING_MOVE_TOWARDS_LIMIT", "HOMING_DELAY1",
	"HOMING_MOVE_AWAY_FROM_LIMIT", "HOMING_DELAY2",
	"HOMING_ADJUST_POSITION", "HOMING_DELAY3", "HOMING_ERROR"};


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
void FlexyStepper_set_homing(FlexyStepper* stepper, int8_t homing_direction, uint8_t homing_speed, float homing_adjust_position, uint8_t* homing_limit_switch_ptr)
{
	stepper->homing_direction = homing_direction;
	stepper->homing_speed = homing_speed;
	stepper->homing_adjust_position = homing_adjust_position;
	stepper->homing_limit_switch_ptr = homing_limit_switch_ptr;
}

void start_cFlexyStepper_homing_sm(FlexyStepper* stepper)
{
	FlexyStepper_setSpeed(stepper, stepper->homing_speed);
	FlexyStepper_setTargetPositionRelative(stepper, stepper->homing_direction * 100, false);
	set_cFlexyStepper_homing_sm_state(stepper, HOMING_MOVE_TOWARDS_LIMIT);
}

void cFlexyStepper_homing_sm_loop(FlexyStepper* stepper)
{
	if(stepper->homing_direction == 0)	//homing not set
		return;

	switch (stepper->homing_sm_state)
	{
		case HOMING_IDLE:
			break;

		case HOMING_MOVE_TOWARDS_LIMIT:
			if (*(stepper->homing_limit_switch_ptr) == 1)
			{
				FlexyStepper_Estop(stepper);
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY1);
			}

			if (GET_MICROS - stepper->homing_sm_timer >= MOVING_TOWARDS_LIMIT_TIMEOUT_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ERROR); 
			}
			break;

		case HOMING_DELAY1:
			if (GET_MICROS - stepper->homing_sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_MOVE_AWAY_FROM_LIMIT);
				FlexyStepper_setTargetPositionRelative(stepper, -1 * stepper->homing_direction * 5, false);
			}
			break;

		case HOMING_MOVE_AWAY_FROM_LIMIT:
			if (*(stepper->homing_limit_switch_ptr) == 0 && !stepper->is_moving)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY2);
			}

			if(GET_MICROS - stepper->homing_sm_timer >= MOVING_AWAY_FROM_LIMIT_TIMEOUT_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ERROR); 
			}
			break;

		case HOMING_DELAY2:
			if (GET_MICROS - stepper->homing_sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ADJUST_POSITION);
				FlexyStepper_setTargetPositionRelative(stepper, stepper->homing_adjust_position, false);
			}
			break;

		case HOMING_ADJUST_POSITION:
			if (!stepper->is_moving)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY3);
			}
			break;

		case HOMING_DELAY3:
			if (GET_MICROS - stepper->homing_sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_IDLE);
			}
			break;

		case HOMING_ERROR:
			if(GET_MICROS - stepper->homing_sm_timer >= HOMING_ERROR_TIMEOUT_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_IDLE);
			}
			break;
	}
}