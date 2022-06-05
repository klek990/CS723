#include <stdio.h>
#include <stdlib.h>
#include "cruiseController.h"

/* Check if acceleerator pedal pressed */
int checkAccelPedal(float accel)
{
	return accel >= 3;
}

/* Check if brake pedal pressed */
int checkBrakePedal(float brake)
{
    return brake >= 3;
}

/* Return true if speed within range, else false */
int checkSpeedInRange(float speed)
{
    return (speed >= 30.0 && speed <= 150.0);
}

/* Check if result of increment */
int checkSpeedIncrement(float cruiseSpeed)
{
	return (cruiseSpeed + 2.5 <= 150.0); 
}

/* Check if result of decrement */
int checkSpeedDecrement(float cruiseSpeed)
{
	return (cruiseSpeed - 2.5 >= 30.0);
}

/*
DESCRIPTION: Saturate the throttle command to limit the acceleration.
PARAMETERS: throttleIn - throttle input
            saturate - true if saturated, false otherwise
RETURNS: throttle output (ThrottleCmd)
*/
float saturateThrottle(float throttleIn, int* saturate)
{
	static const float THROTTLESATMAX = 45.0;
	if (throttleIn > THROTTLESATMAX) {
		*saturate = 1;
		return THROTTLESATMAX;
	}
	else if (throttleIn < 0) {
		*saturate = 1;
		return 0;
	}
	else {
		*saturate = 0;
		return throttleIn;
	}
}

/*
DESCRIPTION: Saturate the throttle command to limit the acceleration.
PARAMETERS: isGoingOn - true if the cruise control has just gone into the ON state 
                        from another state; false otherwise
            saturate - true if saturated, false otherwise
RETURNS: throttle output (ThrottleCmd)
*/
float regulateThrottle(int isGoingOn, float cruiseSpeed, float vehicleSpeed)
{
	static const float KP = 8.113;
	static const float KI = 0.5;
	static int saturate = 1;
	static float iterm = 0;
	
	if (isGoingOn) {
		iterm = 0;	// reset the integral action
		saturate = 1;	
	}
	float error = cruiseSpeed - vehicleSpeed;
	float proportionalAction = error * KP;
	if (saturate)
		error = 0;	// integral action is hold when command is saturated
	iterm = iterm + error;
	float integralAction = KI * iterm;
	return saturateThrottle(proportionalAction + integralAction, &saturate);
}