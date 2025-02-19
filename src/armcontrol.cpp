#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"


extern pros::adi::DigitalOut brownLady;

const int numStates = 3;
//make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {3, 19,150};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
    
  /*  if (currState == 0 || currState == 2){
        brownLady.set_value(false);
    }
    if (currState == 3)*/

  
}

void liftControl() {
    double kp = 1;
    if ( target > 100 || target < 5){
        kp = 1;
    }
    else{
        kp = 1.7;
    }
   
    double error = target - encoder.get_value();
    double velocity = kp * error;
    arm.move(-velocity);

    if (target > 30){
        brownLady.set_value(true);
    }
    else {
        brownLady.set_value(false);
    }



   
}

void setArmPosition(int targetDegrees){
    target = targetDegrees;
}


