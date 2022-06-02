//The type 'string' is defined as 'char*' in Esterel, hence passing a string by referenc  is 'char**'
//Aviod naming a function 'recv'
void send(int data);
void recv1(char** data);
void adding(unsigned int data);
int remov();
int outCount();
int checkAccelPedal(float accel);
int checkBrakePedal(float brake);
int checkSpeedInRange(float speed);
int checkSpeedIncrement(float cruiseSpeed, int quickAccel, int quickDecel);