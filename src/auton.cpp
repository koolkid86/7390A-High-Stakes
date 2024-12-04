#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "arm_control.hpp"

void match() {
  chassis.moveToPoint(0, -41.5, 2500,
                      {
                          .forwards = false,
                          .maxSpeed = 80,
                      },
                      true);

  while (chassis.isInMotion() && distance.get() > 32) {
    pros::delay(10); // save cpu resources
  }
  // cancel the motion once the robot detects mogo is in the bot and clamped
  // pros::delay(100); // wait for mogo to clamp and settle
  mogoClamp.set_value(true);

  pros::delay(250);
  chassis.cancelMotion();

  //
  // pros::delay(200); // wait for mogo to clamp and settle

  // chassis.moveToPoint(9.4, -32.36,  3000, {.maxSpeed=90}, true);
  intake1.move_velocity(600);
  intake2.move_velocity(600);

  chassis.moveToPoint(18.7, -30.6, 1000, {.maxSpeed = 80}, false);

  chassis.moveToPose(23.75, -50.7, 180, 1500, {.maxSpeed = 100}, false);
  pros::delay(1000);
  chassis.turnToHeading(200, 1000);
  chassis.moveToPoint(26.75, -40, 1500, {.forwards = false}, false);

  // chassis.moveToPoint(0, 0, 4000);
}

void skills() {
  // Set initial arm position
  setArmPosition(130);
  pros::delay(750);
  setArmPosition(0);
  pros::delay(750);

  // Move to goal

  chassis.moveToPose(19, -12.7, -90, 3000, {.forwards = false, .maxSpeed = 60},
                     true);

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

void (*autonFunctions[])() = {match, skills};

int autonSelect = 1;
std::string autonNames[2] = {"Match", "Skills"};

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