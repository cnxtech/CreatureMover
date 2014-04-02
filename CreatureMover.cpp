//
//  CreatureMover.cpp
//  CreatureMover
//
//  Created by Sam McHardy on 1/10/12.
//  Copyright (c) 2012 Josephmark. All rights reserved.
//

#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <math.h>
#include "CreatureMover.h"
    
CreatureMover::CreatureMover(EightAccelStepper &stepperA, int aMaxSpeed, int _aSteps[3][3],
                             EightAccelStepper &stepperB, int bMaxSpeed, int _bSteps[3][3],
                             int idleDistance, int acceleration) {
    
    _saveInterval = 1000;
    
    // setup motors
    _stepperA = stepperA;
    _stepperB = stepperB;
    
    _idleDistance = idleDistance;
    
    // load the previous position from memory
    _loadPositions();
    
    // set this as the current position    
    //_stepperA.setCurrentPosition(0);
    //_stepperB.setCurrentPosition(0);
        
    _maxSpeedA = aMaxSpeed;
    _maxSpeedB = bMaxSpeed;
    _acceleration = acceleration;
    _stepperA.setMaxSpeed(_maxSpeedA);
    _stepperA.setAcceleration(_acceleration);
    
    _stepperB.setMaxSpeed(_maxSpeedB);
    _stepperB.setAcceleration(_acceleration);
    
    for (int x=0; x<3; x++){
        for (int y=0; y<3; y++){
            aSteps[x][y] = _aSteps[x][y];
            bSteps[x][y] = _bSteps[x][y];
        }
    }
    // move to closest position
    _setState(STOPPED);    
    
    randomSeed(analogRead(0));
    
}
    
void CreatureMover::moveToPosition(XPosition xPos, YPosition yPos) {
    
    if (getState() != MOVING) {
        // start moving to the specified position

        Serial.print("Moving to position(");
        Serial.print(xPos);
        Serial.print(", ");        
        Serial.print(yPos);
        Serial.println(")");
        
        Serial.print("Current position(");
        Serial.print(_stepperA.currentPosition());
        Serial.print(", ");        
        Serial.print(_stepperB.currentPosition());
        Serial.println(")");
        
        Serial.print("New position(");
        Serial.print(aSteps[xPos][yPos]);
        Serial.print(", ");        
        Serial.print(bSteps[xPos][yPos]);
        Serial.println(")"); 
        
        int steps2TakeA = aSteps[xPos][yPos] - _stepperA.currentPosition();
        int steps2TakeB = bSteps[xPos][yPos] - _stepperB.currentPosition();
                
        Serial.print("Difference(");
        Serial.print(steps2TakeA);
        Serial.print(", ");        
        Serial.print(steps2TakeB);
        Serial.println(")");
        
        _stepperA.move(steps2TakeA);
        _stepperB.move(steps2TakeB);
        
        int maxSpeedA = _maxSpeedA;
        int maxSpeedB = _maxSpeedB;
        // if moving from left bottom to a right position
        if (_xPosition == 0 && _yPosition == 2 && xPos == 2) {
            maxSpeedA = round((float)maxSpeedA / 7);    
        // if moving from right bottom to left position
        } else if (_xPosition == 2 && _yPosition == 2 && xPos == 0) {  
            maxSpeedB = round((float)maxSpeedB / 7); 
        // if moving from right middle to middle          
        } else if (_xPosition == 2 && _yPosition == 1 && xPos == 1 && yPos == 1) {  
            maxSpeedB = round((float)maxSpeedB / 5);
        // if moving from left middle to middle
        } else if (_xPosition == 0 && _yPosition == 1 && xPos == 1 && yPos == 1) {
            maxSpeedA = round((float)maxSpeedA / 5); 
        // if moving from right position to another
        } else if (_xPosition == 2 && xPos == 2) {
            maxSpeedA = round((float)maxSpeedA / 3);  
        // if moving from left position to another
        } else if (_xPosition == 0 && xPos == 0) {
            maxSpeedB = round((float)maxSpeedB / 3);          
        } else if (abs(steps2TakeA) > abs(steps2TakeB)) {             
            // Attempt to scale back the speed of the shorter step motor
            maxSpeedB = round(maxSpeedB * abs((float)steps2TakeB / (float)steps2TakeA));
        } else {        
            // Attempt to scale back the speed of the shorter step motor
            maxSpeedA = round(maxSpeedA * abs((float)steps2TakeA / (float)steps2TakeB));
        }
        Serial.print("Speeds(");
        Serial.print(maxSpeedA);
        Serial.print(", ");        
        Serial.print(maxSpeedB);
        Serial.println(")");
   
        _stepperA.setMaxSpeed(maxSpeedA);
        _stepperB.setMaxSpeed(maxSpeedB);
        
        _xPosition = xPos;
        _yPosition = yPos;
        
        _setWaitDelay(0, 0);
        _setState(MOVING);
    } else {
        // ignore call
    }
}
    
void CreatureMover::idle() {    
    _aIdleRefPosition = _stepperA.currentPosition();
    _bIdleRefPosition = _stepperB.currentPosition();
    _setState(IDLE);
    _setNextIdlePosition();
}

void CreatureMover::run() {
    int currState = getState();
    if (_positionReached()) {
        switch(currState) {
            case MOVING:
                _setState(STOPPED);
                break;
            case IDLE:
                _setNextIdlePosition();
                break;
            default:
                break;
        }
    } else {
        if (currState == MOVING || currState == IDLE) {
            //if (!_isWaitingA()) {
            if (_stepperA.distanceToGo() != 0) {
                _stepperA.run();
            }
            //    Serial.println("RunningA");  
            //} else {
            //    Serial.println("WaitingA");                
            //}
            //if (!_isWaitingB()) {
            if (_stepperB.distanceToGo() != 0) {
                _stepperB.run();
            }
            //    Serial.println("RunningB");
            //} else {
            //    Serial.println("WaitingB");
            //}
            // if we haven't saved positions for a while
            if (currState == MOVING) {
                _savePositions();
            }
        } else {
            //Serial.println("State STOPPED");            
        }
    }
}
                         
int CreatureMover::getPositionX() {
    return _xPosition;
}
    
int CreatureMover::getPositionY() {
    return _yPosition;
}
        
CreatureMover::States CreatureMover::getState() {
    return _state;
}

bool CreatureMover::_isWaitingA() {
    if (millis() < _waitTillA) {
        return true;
    }
    return false;
}

bool CreatureMover::_isWaitingB() {
    if (millis() < _waitTillB) {
        return true;
    }
    return false;
}

void CreatureMover::_setWaitDelay(int delayA, int delayB) {
    _waitTillA = millis() + delayA;
    _waitTillB = millis() + delayB;

}

void CreatureMover::_setState(States state) {
    _state = state;
}

void CreatureMover::_savePositions() {
    if (millis() > _lastSaved + _saveInterval) {
        _lastSaved = millis();
        EEPROM_writeAnything(0, (int) _stepperA.currentPosition());
        EEPROM_writeAnything(2, (int) _stepperB.currentPosition());
/*
        Serial.print("Saving positions(");
        Serial.print(_stepperA.currentPosition());
        Serial.print(", ");        
        Serial.print(_stepperB.currentPosition());
        Serial.println(")");
*/
    }
}

void CreatureMover::_loadPositions() {
    // load the positions and update the stepper position
    
    int posA, posB;
    EEPROM_readAnything(0, posA);
    EEPROM_readAnything(2, posB);
    
    _stepperA.setCurrentPosition(posA);
    _stepperB.setCurrentPosition(posB);
/*    
    Serial.print("Read positions(");
    Serial.print(posA);
    Serial.print(", ");
    Serial.print(posB);
    Serial.println();
*/
}

bool CreatureMover::_positionReached() {
    //Serial.print("Distance to go(");
    //Serial.print(_stepperA.distanceToGo());
    //Serial.print(", ");        
    //Serial.print(_stepperB.distanceToGo());
    //Serial.println(")");

    if (_stepperA.distanceToGo() == 0 && _stepperB.distanceToGo() == 0) {
        return true;
    }
    return false;
}

void CreatureMover::_setNextIdlePosition() {
    if (_idleDistance != 0) {
        // use 1 of 5 positions, above, right, below, left, center
        int position = random(5);
        int aOffset = 0;
        int bOffset = 0;    
        switch(position) {
            case 0: // above
                aOffset = -_idleDistance;
                bOffset = _idleDistance;
                break;
            case 1: // right
                aOffset = _idleDistance;
                bOffset = _idleDistance;
                break;
            case 2: // below
                aOffset = _idleDistance;
                bOffset = -_idleDistance;
                break;
            case 3: // left
                aOffset = -_idleDistance;
                bOffset = -_idleDistance;
                break;
            case 4: // center
            default:
                break;
            
        }
        //int stepToA = aSteps[getPositionX()][getPositionY()] + aOffset;
        //int stepToB = bSteps[getPositionX()][getPositionY()] + bOffset;
        int stepToA = _aIdleRefPosition + aOffset;
        int stepToB = _bIdleRefPosition + bOffset;
    
        _stepperA.setMaxSpeed(getMaxSpeedA());    
        _stepperB.setMaxSpeed(getMaxSpeedB());
        _stepperA.moveTo(stepToA);
        _stepperB.moveTo(stepToB);
    }

}

int CreatureMover::getMaxSpeedA() {
    return _maxSpeedA;
}
int CreatureMover::getMaxSpeedB() {
    return _maxSpeedB;
}
