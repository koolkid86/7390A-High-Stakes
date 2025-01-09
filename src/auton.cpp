#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "armcontrol.hpp"

extern pros::adi::DigitalOut doinker; // Reference to doinker defined in constants.cpp
extern pros::adi::DigitalOut rushMech;

void redRingRush() {
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  rushMech.set_value(true);
  chassis.moveToPoint(-17.9, 40,  2700, {.maxSpeed = 80},false);
  pros::delay(1000);

  chassis.moveToPoint(-11.9 - 7.5, 20, 2000, {.forwards = false}, false);

  chassis.moveToPose(0, 0, 0, 5000);

  
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