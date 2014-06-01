#include "irobotNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

/// Program States
typedef enum{
	INITIAL = 0,						///< Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			///< Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			///< Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		///< Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,MOVE_RIGHT,MOVE_LEFT,MOVE,BACK,								///< Drive straight
	TURN_LEFT,TURN_RIGHT,TURN_AROUND,
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
	static robotState_t 		prev_state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s
	int16_t speed = 200;
	static int16_t				error = 0;
	static int16_t				d_error = 0;
	static double accel_x = 0;
	//update error
	double d_accel_x = accel_x - accel.x;
	bool onramp = (abs(accel.x)>0.1 );
	int16_t maneuv = onramp?50:50;
	
	accel_x = accel.x;
	error += sensors.distance*sin(netAngle*M_PI/180);
	d_error -= error;
	if (state !=prev_state){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
	}
	prev_state = state;
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
	else if (sensors.bumps_wheelDrops.wheeldropLeft || sensors.bumps_wheelDrops.wheeldropRight){
		state = BACK;
	}
	else if( sensors.bumps_wheelDrops.bumpLeft>0 ||
		sensors.cliffFrontLeft || sensors.cliffLeft ){
		state = TURN_RIGHT;
	}else if (sensors.bumps_wheelDrops.bumpRight>0 ||
		sensors.cliffFrontRight ||sensors.cliffRight ||
		sensors.bumps_wheelDrops.wheeldropRight){
		state = TURN_LEFT;
	}else if(state == TURN_LEFT && abs(netAngle - angleAtManeuverStart) >= 90){
		
		state = MOVE_LEFT;
	}else if(state == TURN_RIGHT && abs(netAngle - angleAtManeuverStart) >= 90){
		state = MOVE_RIGHT;
	}
	else if(state == MOVE_LEFT && abs(netDistance - distanceAtManeuverStart) >= maneuv){
		state = ADJUST_RIGHT;
	}else if(state == MOVE_RIGHT && abs(netDistance - distanceAtManeuverStart) >= maneuv){
		state = ADJUST_LEFT;
	}
	else if((state == ADJUST_LEFT ||state == ADJUST_RIGHT) && abs(netAngle - angleAtManeuverStart) >= 90){
		state = DRIVE;
	}else if (state == DRIVE && (d_accel_x<-0.1 || accel_x<-0.1)){
		state = TURN_AROUND;
	}else if(state == TURN_AROUND && abs(netAngle + angleAtManeuverStart)<5){ 	// else, no transitions are taken
		state = MOVE;
	}else if ((state == MOVE ) && abs(netDistance - distanceAtManeuverStart) >= maneuv){
		state = DRIVE;
	}else if ((state == BACK ) && abs(netDistance - distanceAtManeuverStart) >= 3*maneuv){
		state = DRIVE;
	}


	//*****************
	//* state actions *
	//*****************
	
	speed = (onramp?100:200);



	if (state == MOVE_LEFT || state == MOVE_RIGHT || state ==MOVE){
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = speed;
	}else if (state == BACK){
		leftWheelSpeed = rightWheelSpeed = -speed;
	
	}else if (state == DRIVE){
		leftWheelSpeed = 150 + 0.05*error;
		rightWheelSpeed = 150 - 0.05*error;
		
	
	}else if (state ==TURN_LEFT || state == ADJUST_LEFT){
		leftWheelSpeed = -speed;
		rightWheelSpeed = -leftWheelSpeed;
	}else if (state ==TURN_RIGHT || state == ADJUST_RIGHT){
		leftWheelSpeed = speed;
		rightWheelSpeed = -leftWheelSpeed;
	}else if (state == TURN_AROUND){
		leftWheelSpeed = speed;
		rightWheelSpeed = -leftWheelSpeed;
	}else{
		leftWheelSpeed = rightWheelSpeed = 0;
	}


	// write outputs
	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
