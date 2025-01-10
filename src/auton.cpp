#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "armcontrol.hpp"

extern pros::adi::DigitalOut doinker; // Reference to doinker defined in constants.cpp
extern pros::adi::DigitalOut rushMech;

void redRingRush() {

  setArmPosition(70);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);

  //initial rush
  rushMech.set_value(true);
  //chassis.moveToPoint(-17.9, 40,  2700, {.maxSpeed = 80},false);
  chassis.moveToPoint(-8, 40.,  2700, {.maxSpeed = 80},false);
  pros::delay(10);

  //back up after rush into mogo
  chassis.moveToPose(-0.7, 29.8, -75, 1000, {.forwards = false, .maxSpeed = 60}, false);
  intake1.move_velocity(0);
  intake2.move_velocity(0);

  
  //drive backwards while clamping mogo and do other bot functions
  chassis.moveToPoint(12.6, 27.8, 500, {.forwards = false, .maxSpeed = 75}, true);
    while (chassis.isInMotion())
    {
      if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(100);
        chassis.cancelMotion();

       
        break;
      }
      pros::delay(20);
    }

  //get ready to pick up rings
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);

  //mogoClamp.set_value(true);

  //go pick up 2 rings after clamping mogo
  chassis.moveToPoint(2.2, 31.4, 1000, {}, false);
  chassis.moveToPoint(-8.6, 30.4, 1000, {}, false);
 
  //go towards and score wall stake
  chassis.moveToPose(-30.2, 42.0, -50.4, 2500, {.maxSpeed = 50}, false);
  setArmPosition(155);
  pros::delay(200);
  chassis.turnToHeading(-51, 200);
  
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

void match_awp() {
    chassis.setPose(0,0,0);

   // setArmPosition(130);
    pros::delay(500);
   // setArmPosition(0);
    
    chassis.moveToPoint(16.13, -36.7, 2500, {.forwards = false, .maxSpeed = 70}, true );

    while (chassis.isInMotion() && distance.get() > 32) {
      pros::delay(10); // save cpu resources
    }
    // cancel the motion once the robot detects mogo is in the bot and clamped
    // pros::delay(100); // wait for mogo to clamp and settle
    mogoClamp.set_value(true);

    pros::delay(250);
    chassis.cancelMotion();


    mogoClamp.set_value(true);
    pros::delay(250);

    chassis.turnToHeading(98.2, 1000, {.maxSpeed = 100, .minSpeed = 10}, false);
    chassis.moveToPoint(33.5, -34.24, 2000, {.forwards = true, .maxSpeed = 100});

    intake1.move_velocity(600);
    intake2.move_velocity(600);
    // AWP (Alliance Win Point) autonomous routine
    // Add your AWP-specific autonomous code here
}

void skills() {

  chassis.setPose(0,0,0);
  // Set initial arm position
  setArmPosition(130);
  pros::delay(750);
  setArmPosition(0);
  pros::delay(750);

  // Move to goal


  /**/
  chassis.moveToPose(19, -12.7, -90, 3000, {.forwards = false, .maxSpeed = 60},
                     false);

  while (chassis.isInMotion() && distance.get() > 32) {
    pros::delay(10); // save cpu resources
  }

  pros::delay(750);
  // Mogo
  mogoClamp.set_value(true);

  intake1.move_velocity(600);
  intake2.move_velocity(600);

  // chassis.turnToHeading(65, 1000, {.maxSpeed = 80}, false);

  pros::delay(750);

  chassis.moveToPose(55, -11, 90, 3000, {.maxSpeed = 80}, false);

  chassis.moveToPose(60, 2, 219, 3000, {.forwards = false, .maxSpeed = 80},
                     false);

  // After setting goal
  mogoClamp.set_value(false);

  arm.move_absolute(260, 80);

  // chassis.moveToPoint(60, -40, 3000, {.forwards = true, .maxSpeed = 80},
  // false);

  chassis.turnToHeading(220, 2000, {.maxSpeed = 80}, false);

  chassis.moveToPoint(54, -55, 2000, {.forwards = true, .maxSpeed = 80}, false);

  chassis.turnToHeading(90, 2000, {.maxSpeed = 80}, false);

  chassis.moveToPoint(65, -55, 2000, {.forwards = true, .maxSpeed = 80}, false);

  intake1.move_velocity(0);
  intake2.move_velocity(0);

  arm.move_absolute(3000, 80);
}

void (*autonFunctions[])() = {redRingRush, match_awp, skills};

int autonSelect = 1;
std::string autonNames[3] = {"redRingRush", "Match AWP", "Skills"};

void previousAuton() {
  if (autonSelect == 0) {
    autonSelect = sizeof(autonNames) / sizeof(autonNames[0]) - 1;
  } else {
    autonSelect--;
  }
  pros::lcd::set_text(0, "Auton Selected = " + autonNames[autonSelect]);
}

void nextAuton() {
  if (autonSelect == sizeof(autonNames) / sizeof(autonNames[0]) - 1) {
    autonSelect = 0;
  } else {
    autonSelect++;
  }
  pros::lcd::set_text(0, "Auton Selected = " + autonNames[autonSelect]);
}