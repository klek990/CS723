#include <stdio.h>
#include <stdlib.h>
#include "cruiseController.h"

/* Check if acceleerator pedal pressed */
int checkAccelPedal(float accel)
{
	return accel >= 0.03;
}

/* Check if brake pedal pressed */
int checkBrakePedal(float brake)
{
    return brake >= 0.03;
}

/* Return true if speed within range, else false */
int checkSpeedInRange(float speed)
{
    return (speed >= 30.0 && speed <= 150.0);
}

/* Check if result of increment/decrement is within bounds */
int checkSpeedIncrement(float cruiseSpeed, int quickAccel, int quickDecel)
{
    if (quickAccel == 1)
    {
        return (cruiseSpeed + 2.5 <= 150.0); 
    }
    else if (quickDecel == 1)
    {
        return (cruiseSpeed - 2.5 >= 30.0);
    }
}