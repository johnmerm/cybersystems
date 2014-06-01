#include "irobotNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

/// Program States
typedef enum{
	INITIAL = 0,						///< Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			///< Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			///< Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		///< Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,MOVE_RIGHT,MOVE_LEFT,								///< Drive straight
	TURN_LEFT,TURN_RIGHT,
	ADJUST_LEFT,ADJUST_RIGHT///< Turn
} robotState_t;

void irobotNavigationStatechart(
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const irobotSensorGroup6_t 	sensors,
	const accelerometer_t 		accel,
	const bool					isSimulator,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed
){
	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************


	//*****************************************************
	// state transition - pause region (highest priority) *
	//*****************************************************
	if(   state == INITIAL
	   || state == PAUSE_WAIT_BUTTON_RELEASE
	   || state == UNPAUSE_WAIT_BUTTON_PRESS
	   || state == UNPAUSE_WAIT_BUTTON_RELEASE
	   || sensors.buttons.play				// pause button
	){
		switch(state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if(isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if(!sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if(!sensors.buttons.play){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if(sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************
	else if( sensors.bumps_wheelDrops.bumpLeft>0){
		state = TURN_RIGHT;
	}else if (sensors.bumps_wheelDrops.bumpRight>0){
		state = TURN_LEFT;
	}else if(state == TURN_LEFT && abs(netAngle - angleAtManeuverStart) >= 90){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = MOVE_LEFT;
	}else if(state == TURN_RIGHT && abs(netAngle - angleAtManeuverStart) >= 90){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = MOVE_RIGHT;
	}
	else if(state == MOVE_LEFT && abs(netDistance - distanceAtManeuverStart) >= 250){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = ADJUST_RIGHT;
	}else if(state == MOVE_RIGHT && abs(netDistance - distanceAtManeuverStart) >= 250){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = ADJUST_LEFT;
	}
	else if((state == ADJUST_LEFT ||state == ADJUST_RIGHT) && abs(netAngle - angleAtManeuverStart) >= 90){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	
	

	if (state == DRIVE || state == MOVE_LEFT || state == MOVE_RIGHT){
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
	}else if (state ==TURN_LEFT || state == ADJUST_LEFT){
		leftWheelSpeed = -100;
		rightWheelSpeed = -leftWheelSpeed;
	}else if (state ==TURN_RIGHT || state == ADJUST_RIGHT){
		leftWheelSpeed = 100;
		rightWheelSpeed = -leftWheelSpeed;
	}else{
		leftWheelSpeed = rightWheelSpeed = 0;
	}


	// write outputs
	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
