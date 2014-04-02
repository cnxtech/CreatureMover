//
//  CreatureMover.h
//  CreatureMover
//
//  Created by Sam McHardy on 1/10/12.
//  Copyright (c) 2012 Josephmark. All rights reserved.
//

#ifndef CreatureMover_CreatureMover_h
#define CreatureMover_CreatureMover_h

#include "Arduino.h"
#include <EightAccelStepper.h>

class CreatureMover
{
public:
    typedef enum
    {
        XLEFT  = 0,
        XMID   = 1,
        XRIGHT = 2
    } XPosition;
    typedef enum
    {
        YTOP    = 0,
        YMID    = 1,
        YBOTTOM = 2
    } YPosition;
    
    typedef enum
    {
        STOPPED = 0,
        MOVING  = 1,
        IDLE    = 2
    } States;
    
    CreatureMover(EightAccelStepper &stepperA, int aMaxSpeed, int _aSteps[3][3],
                  EightAccelStepper &stepperB, int bMaxSpeed, int _bSteps[3][3],
                  int idleDistance, int accelration);
    
    void moveToPosition(XPosition xPos, YPosition yPos);
    
    void idle();
    
    void run();
    
    int getPositionX();
    
    int getPositionY();
    
    int getMaxSpeedA();
    int getMaxSpeedB();
    
    States getState();
    
    void _loadPositions();  

    int aSteps[3][3];
    
    int bSteps[3][3]; 
    
protected:
    void _setWaitDelay(int delayA, int delayB);
    void _setState(States state);    
    void _savePositions();      
    bool _positionReached();    
    void _setNextIdlePosition();
    bool _isWaitingA();    
    bool _isWaitingB();
    
private:
    int _xPosition;
    int _yPosition;
    int _aIdleRefPosition;
    int _bIdleRefPosition;
    int _creatureState;
    int _aSteps;
    int _bSteps;
    int _waitTillA;
    int _waitTillB;
    unsigned long _lastSaved;
    unsigned long _saveInterval;
    int _maxSpeedA;
    int _maxSpeedB;
    int _acceleration;
    int _idleDistance;
    States _state;
    EightAccelStepper _stepperA;
    EightAccelStepper _stepperB;
};

#endif
