// code as of end of week 12 workshop 

// remember to comment out the LED line of code in main.c to ensure LEDs display correctly

/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>


// Program States
typedef enum {
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state

	DRIVE,								// Drive straight on the ground
	TURN,								// Turn on the ground
	REVERSE,							// Reverse on the ground
	DRIVE_A_BIT,						// Drive a bit on the ground
	REORIENT,							// Reorient on the ground

	HILL_CORRECT,						// on a hill, but facing the wrong direction so need to turn to face the correct direction
	HILL_DRIVE_A_BIT,					// Drive forward a bit to ensure the entire robot is on the ramp
	DRIVE_UPHILL,						// Drive up the ramp
	CLIFF_AVOID,						// Any cliff detector set off, reverse a bit and turn until ground orientation reached

	DRIVE_PLATEAU,						// Drive straight on the plateau
	TURN_PLATEAU,						// Turn on the plateau
	REVERSE_PLATEAU,					// Reverse on the plateau
	DRIVE_A_BIT_PLATEAU,				// Drive a bit on the plateau
	REORIENT_PLATEAU,					// Reorient on the plateau

	HILL_CORRECT_DOWN,					// on the downhill, but facing the wrong direction so need to turn to face the correct direction
	HILL_DRIVE_A_BIT_DOWN,				// Drive forward a bit to ensure the entire robot is on the downhill ramp
	DRIVE_DOWNHILL,						// Drive down the ramp
	CLIFF_AVOID_DOWN,					// Any cliff detector set off, reverse a bit and turn until ground orientation reached

	FINISH								// Returned to ground off down-ramp, so stop
} robotState_t;

typedef enum {
	BUMP_LEFT,
	BUMP_CENTRE,
	BUMP_RIGHT
} bump_state_t;

typedef enum {
	CLIFF_LEFT,
	CLIFF_CENTRE,
	CLIFF_RIGHT
} cliff_state_t;


#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree
#define HILL_TURN_THRESHOLD		0.05				// threshold value for accelerometer reading to turn
#define HILL_DETECT_THRESHOLD	0.15				// threshold value for accelerometer reading to detect a ramp
#define K 						200
#define int16_t short
#define int32_t int
#define int64_t long int

extern NiFpga_Session myrio_session;
void KobukiNavigationStatechart(
	const int16_t 				maxWheelSpeed,
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	int16_t* const 			pRightWheelSpeed,
	int16_t* const 			pLeftWheelSpeed,
	const bool					isSimulator
) {

	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static bump_state_t			bump_state;						// stores which sensor was bumped
	static cliff_state_t		cliff_state;					// stores which cliff detector was set off
	// static robotState_t			hill_state;						// stores whether we were going uphill or downhill
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg
	static int32_t				init_accelx;					// accelAxes.x reading at transition into DRIVE_UPHILL

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************



	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		) {
		switch (state) {
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator) {
			}
			else {
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0) {
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0) {
				// state = unpausedState; REMEMBER TO CHANGE THIS BACK FOR THE PROJECT
				state = DRIVE;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0) {
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

	// from previous workshop where robot automatically turns after driving forward a certain distance
	//else if (state == DRIVE && abs(netDistance - distanceAtManeuverStart) >= 1000){
	//	angleAtManeuverStart = netAngle;
	//	distanceAtManeuverStart = netDistance;
	//	state = TURN;
	//}

	else if (state == TURN
			&& abs(netAngle - angleAtManeuverStart) >= 60
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE_A_BIT;
	}
	else if (state == DRIVE_A_BIT
			&& abs(netDistance - distanceAtManeuverStart) >= 400 // shortened from 500 to help robot get through 2 obstacles that are close together
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = REORIENT;
	}
	else if (state == REORIENT
			&& (netAngle) == 0
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}
	else if ((state == DRIVE
				|| state == TURN
				|| state == DRIVE_A_BIT
				|| state == REORIENT)
			&& (sensors.bumps_wheelDrops.bumpCenter
				|| sensors.bumps_wheelDrops.bumpLeft
				|| sensors.bumps_wheelDrops.bumpRight)
			) {
		if (sensors.bumps_wheelDrops.bumpCenter) {
			bump_state = BUMP_CENTRE;
		}
		else if (sensors.bumps_wheelDrops.bumpLeft) {
			bump_state = BUMP_LEFT;
		}
		else {
			bump_state = BUMP_RIGHT;
		}
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = REVERSE;
	}
	else if (state == REVERSE
			&& abs(netDistance - distanceAtManeuverStart) >= 100 // was 250 but shortened to help robot get through 2 obstacles that are close together
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN;
	}
	else if ((state == DRIVE
				|| state == TURN
				|| state == DRIVE_A_BIT
				|| state == REORIENT)
			&& ((fabs(accelAxes.x) >= HILL_DETECT_THRESHOLD)
				|| accelAxes.y > HILL_DETECT_THRESHOLD)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_DRIVE_A_BIT;
	}

	else if (state == HILL_DRIVE_A_BIT
			&& (abs(netDistance - distanceAtManeuverStart) >= 20)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_CORRECT;
	}
	else if (state == HILL_CORRECT
			&& ((accelAxes.x < HILL_TURN_THRESHOLD)
					&& (accelAxes.x > -HILL_TURN_THRESHOLD))
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		init_accelx = accelAxes.x;
		state = DRIVE_UPHILL;
	}
	else if ((state == HILL_CORRECT
				|| state == HILL_DRIVE_A_BIT
				|| state == DRIVE_UPHILL)
			&& (sensors.cliffLeft
				|| sensors.cliffCenter
				|| sensors.cliffRight)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		if (sensors.cliffCenter) {
			cliff_state = CLIFF_CENTRE;
		}
		else if (sensors.cliffLeft) {
			cliff_state = CLIFF_LEFT;
		}
		else if (sensors.cliffRight) {
			cliff_state = CLIFF_RIGHT;
		}
		state = CLIFF_AVOID;
	}
	else if (state == CLIFF_AVOID
			&& (abs(netAngle - angleAtManeuverStart) >= 45)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_DRIVE_A_BIT;
	}
	else if ((state == HILL_CORRECT
				|| state == HILL_DRIVE_A_BIT
				|| state == DRIVE_UPHILL)
			&& (accelAxes.y < HILL_DETECT_THRESHOLD-0.08)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE_PLATEAU;
	}

	else if (state == REVERSE_PLATEAU
			&& abs(netDistance - distanceAtManeuverStart) >= 150
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN_PLATEAU;
	}
	else if (state == TURN_PLATEAU
			&& abs(netAngle - angleAtManeuverStart) >= 45
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE_A_BIT_PLATEAU;
	}
	else if (state == DRIVE_A_BIT_PLATEAU
			&& abs(netDistance - distanceAtManeuverStart) >= 100
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = REORIENT_PLATEAU;
	}
	else if (state == REORIENT_PLATEAU
			&& (netAngle) == 0
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE_PLATEAU;
	}
	else if ((state == DRIVE_PLATEAU
				|| state == TURN_PLATEAU
				|| state == DRIVE_A_BIT_PLATEAU
				|| state == REORIENT_PLATEAU)
			&& (sensors.cliffCenter
				|| sensors.cliffLeft
				|| sensors.cliffRight)
			) {
		if (sensors.cliffCenter) {
			cliff_state = CLIFF_CENTRE;
		}
		else if (sensors.cliffLeft) {
			cliff_state = CLIFF_LEFT;
		}
		else if (sensors.cliffRight) {
			cliff_state = CLIFF_RIGHT;
		}
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = REVERSE_PLATEAU;
	}

	else if ((state == DRIVE_PLATEAU
				|| state == TURN_PLATEAU
				|| state == DRIVE_A_BIT_PLATEAU
				|| state == REORIENT_PLATEAU)
			&& ((fabs(accelAxes.x) >= HILL_DETECT_THRESHOLD)
				|| accelAxes.y < -HILL_DETECT_THRESHOLD)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_DRIVE_A_BIT_DOWN;
	}

	else if (state == HILL_DRIVE_A_BIT_DOWN
			&& (abs(netDistance - distanceAtManeuverStart) >= 50)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_CORRECT_DOWN;
	}
	else if (state == HILL_CORRECT_DOWN
			&& ((accelAxes.x < HILL_TURN_THRESHOLD+0.25)
				&& (accelAxes.x > -HILL_TURN_THRESHOLD))
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		init_accelx = accelAxes.x;
		state = DRIVE_DOWNHILL;
	}
	else if ((state == HILL_CORRECT_DOWN
				|| state == HILL_DRIVE_A_BIT_DOWN
				|| state == DRIVE_DOWNHILL)
			&& (sensors.cliffLeft
				|| sensors.cliffCenter
				|| sensors.cliffRight)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = CLIFF_AVOID_DOWN;
	}
	else if (state == CLIFF_AVOID_DOWN
			&& (abs(netAngle - angleAtManeuverStart) >= 45)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = HILL_CORRECT_DOWN;
	}
	else if (state == DRIVE_DOWNHILL
			&& (accelAxes.y > -0.05)
			) {
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = FINISH;
	}

	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch (state) {
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 600;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x00);
		break;

	case TURN:
		if (bump_state == BUMP_LEFT) {
			leftWheelSpeed = 100;
			rightWheelSpeed = -leftWheelSpeed;
		}
		else {
			rightWheelSpeed = 100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		break;

	case REVERSE:
		leftWheelSpeed = -300;
		rightWheelSpeed = -300;
		break;

	case DRIVE_A_BIT:
		// half speed ahead!
		leftWheelSpeed = rightWheelSpeed = 400;
		break;

	case REORIENT:
		// reorient by turning left
		if (netAngle < 0) {
			rightWheelSpeed = 100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		// reorient by turning right
		else {
			rightWheelSpeed = -100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		break;


	case HILL_DRIVE_A_BIT:
		leftWheelSpeed = rightWheelSpeed = 200;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x01);
		break;

	case HILL_CORRECT:
		// facing right uphill, reorient by turning left
		if (accelAxes.x < -HILL_TURN_THRESHOLD) {
			rightWheelSpeed = 100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		// facing left uphill, reorient by turning right
		else if (accelAxes.x > HILL_TURN_THRESHOLD) {
			rightWheelSpeed = -100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x00);
		break;
	case DRIVE_UPHILL: // include a way to correct orientation while going uphill - P controller
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x04);
		// full speed ahead!
		leftWheelSpeed = 420 + K * (accelAxes.x + init_accelx);
		rightWheelSpeed = 400 - K * (accelAxes.x + init_accelx);
		break;
	case CLIFF_AVOID: // turning and reversing at the same time while on uphill and cliff detected
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x08);
		if (cliff_state == CLIFF_RIGHT) {
			leftWheelSpeed = -100;
			rightWheelSpeed = -50;
		}
		else if (cliff_state == CLIFF_LEFT) {
			leftWheelSpeed = -50;
			rightWheelSpeed = -100;
		}
		else if (cliff_state == CLIFF_CENTRE) {
			leftWheelSpeed = -200;
			rightWheelSpeed = -200;
		}
		break;


	case DRIVE_PLATEAU:
		// half speed ahead!
		leftWheelSpeed = rightWheelSpeed = 400;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x03);
		break;

	case REVERSE_PLATEAU:
		leftWheelSpeed = -200;
		rightWheelSpeed = -200;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x09);
		break;

	case TURN_PLATEAU:
		if (cliff_state == CLIFF_RIGHT) {
			rightWheelSpeed = 200;
			leftWheelSpeed = -rightWheelSpeed;
		}
		else {
			leftWheelSpeed = 200;
			rightWheelSpeed = -leftWheelSpeed;
		}
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x05);
		break;

	case DRIVE_A_BIT_PLATEAU:
		// half speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x06);
		break;

	case REORIENT_PLATEAU:
		// reorient by turning left
		if (netAngle < 0) {
			rightWheelSpeed = 100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		// reorient by turning right
		else {
			rightWheelSpeed = -100;
			leftWheelSpeed = -rightWheelSpeed;
		}
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x0A);
		break;


	case HILL_DRIVE_A_BIT_DOWN:
		leftWheelSpeed = rightWheelSpeed = 100;
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x07);
		break;

	case HILL_CORRECT_DOWN:
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x0B);
		// facing right downhill, reorient by turning left
		if (accelAxes.x > HILL_TURN_THRESHOLD) {
			rightWheelSpeed = 50;
			leftWheelSpeed = -rightWheelSpeed;
		}
		// facing left downhill, reorient by turning right
		else if (accelAxes.x < -HILL_TURN_THRESHOLD) {
			rightWheelSpeed = -50;
			leftWheelSpeed = -rightWheelSpeed;
		}
		break;

	case DRIVE_DOWNHILL: // include a way to correct orientation while going downhill - P controller
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x0D);
		// full speed ahead!
		leftWheelSpeed = 400 - K * (accelAxes.x + init_accelx);
		rightWheelSpeed = 420 + K * (accelAxes.x + init_accelx);
		break;

	case CLIFF_AVOID_DOWN: // turning and reversing at the same time while on hill and cliff detected
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x0E);
		if (cliff_state == CLIFF_RIGHT) {
			leftWheelSpeed = -100;
			rightWheelSpeed = -50;
		}
		else if (cliff_state == CLIFF_LEFT) {
			leftWheelSpeed = -50;
			rightWheelSpeed = -100;
		}
		else if (cliff_state == CLIFF_CENTRE) {
			leftWheelSpeed = -200;
			rightWheelSpeed = -200;
		}
		break;


	case FINISH:
		NiFpga_WriteU8 (myrio_session , DOLED30, 0x0F);
		if (0) {
			leftWheelSpeed = rightWheelSpeed = 200;
		}
		else {
			leftWheelSpeed = rightWheelSpeed = 0;
		}
		break;
	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 200;
		break;
	}

	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
