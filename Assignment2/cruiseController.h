int checkAccelPedal(float accel);
int checkBrakePedal(float brake);
int checkSpeedInRange(float speed);
int checkSpeedIncrement(float cruiseSpeed, int quickAccel, int quickDecel);
float saturateThrottle(float throttleIn, int* saturate);
float regulateThrottle(int isGoingOn, float cruiseSpeed, float vehicleSpeed);