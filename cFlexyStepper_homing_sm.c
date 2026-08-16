#include "cFlexyStepper.h"
#include <stdbool.h>

#define HOMING_DELAY_US 1000000	
#define MOVING_TOWARDS_LIMIT_TIMEOUT_US 2 * 60000000 
#define MOVING_AWAY_FROM_LIMIT_TIMEOUT_US 2 * 60000000 

const char *cFlexyStepper_homing_sm_states_strings[] =
{	"HOMING_IDLE", "HOMING_MOVE_TOWARDS_LIMIT", "HOMING_DELAY1",
	"HOMING_MOVE_AWAY_FROM_LIMIT", "HOMING_DELAY2",
	"HOMING_ADJUST_POSITION", "HOMING_DELAY3", "HOMING_ERROR"};
	
	
void set_cFlexyStepper_homing_sm_state(FlexyStepper* stepper, cFlexyStepper_homing_sm_states st)
{
	stepper->homing.sm_state = st;
	FlexyStepper_logf(stepper, "homing_sm_state: %s\r\n", cFlexyStepper_homing_sm_states_strings[stepper->homing.sm_state]);
	stepper->homing.sm_timer = GET_MICROS;
}

cFlexyStepper_homing_sm_states get_cFlexyStepper_homing_sm_state(FlexyStepper* stepper)
{
	return stepper->homing.sm_state;
}

/*
	direction = 1 forward, direction = -1 backward, direction = 0 not set
*/
void FlexyStepper_set_homing(FlexyStepper* stepper,
							int homing_direction,
							float homing_speed,
							float homing_adjust_position,
							uint8_t* homing_limit_switch_ptr,
							float new_zero_position)
{
	stepper->homing.direction = homing_direction;
	stepper->homing.speed = homing_speed;
	stepper->homing.adjust_position = homing_adjust_position;
	stepper->homing.limit_switch_ptr = homing_limit_switch_ptr;
	stepper->homing.zero_pos = new_zero_position;
}

void start_cFlexyStepper_homing_sm(FlexyStepper* stepper)
{
	stepper->homing.did_home = false;
	FlexyStepper_jog(stepper, stepper->homing.speed * stepper->homing.direction);
	set_cFlexyStepper_homing_sm_state(stepper, HOMING_MOVE_TOWARDS_LIMIT);
}

void stop_cFlexyStepper_homing_sm(FlexyStepper* stepper)
{
	FlexyStepper_Estop(stepper, false);
	set_cFlexyStepper_homing_sm_state(stepper, HOMING_IDLE);
}

void cFlexyStepper_homing_sm_loop(FlexyStepper* stepper)
{
	if(stepper->homing.direction == 0)	//homing not set
		return;

	/* A fault estops the axis, so any move the sequence is waiting on will
	 * never complete -- bail out instead of waiting forever. */
	if (stepper->status == FLEXY_STATUS_FAULT &&
		stepper->homing.sm_state != HOMING_IDLE &&
		stepper->homing.sm_state != HOMING_ERROR)
	{
		set_cFlexyStepper_homing_sm_state(stepper, HOMING_ERROR);
	}

	switch (stepper->homing.sm_state)
	{
		case HOMING_IDLE:
			break;

		case HOMING_MOVE_TOWARDS_LIMIT:
			if (*(stepper->homing.limit_switch_ptr) == 1)
			{
				FlexyStepper_Estop(stepper, false);
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY1);
			}

			if (GET_MICROS - stepper->homing.sm_timer >= MOVING_TOWARDS_LIMIT_TIMEOUT_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ERROR); 
			}
			break;

		case HOMING_DELAY1:
			if (GET_MICROS - stepper->homing.sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_MOVE_AWAY_FROM_LIMIT);
				FlexyStepper_jog(stepper, (-1 * stepper->homing.direction) * 0.2 * stepper->homing.speed);
			}
			break;

		case HOMING_MOVE_AWAY_FROM_LIMIT:
			if (*(stepper->homing.limit_switch_ptr) == 0)
			{
				FlexyStepper_Estop(stepper, false);
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY2);
			}

			if(GET_MICROS - stepper->homing.sm_timer >= MOVING_AWAY_FROM_LIMIT_TIMEOUT_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ERROR); 
			}
			break;

		case HOMING_DELAY2:
			if (GET_MICROS - stepper->homing.sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_ADJUST_POSITION);
				FlexyStepper_restoreDefaults(stepper);
				FlexyStepper_setTargetPositionRelative(stepper, stepper->homing.adjust_position, false);
			}
			break;

		case HOMING_ADJUST_POSITION:
			if (!FlexyStepper_isMoving(stepper))
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_DELAY3);
			}
			break;

		case HOMING_DELAY3:
			if (GET_MICROS - stepper->homing.sm_timer >= HOMING_DELAY_US)
			{
				set_cFlexyStepper_homing_sm_state(stepper, HOMING_IDLE);
				FlexyStepper_setCurrentPosition(stepper, stepper->homing.zero_pos);

				stepper->homing.did_home = true;
				FlexyStepper_logf(stepper, "homed at %.4f\r\n", stepper->homing.zero_pos);
			}
			break;

		case HOMING_ERROR:
			break;
	}
}