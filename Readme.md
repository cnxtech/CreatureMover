#CreatureMover

Arduino class to move a suspended item in a 3x3 grid of positions using 2 stepper motors.

Requires AccelStepper or EightAccelStepper objects, array of positions, maximum speeds, idle distance and acceleration values.

##Example Usage

```C++
int aSteps[3][3] = {
    {-320,   -230,   -95} // LEFT
  , {-70,     0,     106}     // MID
  , {240,     308,     350}    // RIGHT
};

int bSteps[3][3] = {
    {-310,     -380,   -450}  // LEFT
  , {62,     0,      -128}    // MID
  , {267,    128,    12}  // RIGHT
};

int aMaxSpeed = 100;
int bMaxSpeed = 100;
int idleDistance = 0;
int acceleration = 25;

EightAccelStepper stepperA(4, 28, 30, 32, 34);
EightAccelStepper stepperB(4, 23, 25, 27, 29);

CreatureMover creature(stepperA, aMaxSpeed, aSteps,
                       stepperB, bMaxSpeed, bSteps,
                       idleDistance, acceleration);
```