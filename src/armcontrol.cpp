#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"



const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, 17, 135};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 2.75;
    double error = target - encoder.get_value();
    double velocity = kp * error;
    arm.move(velocity);
}

void setArmPosition(int targetDegrees){
    target = targetDegrees;
}


