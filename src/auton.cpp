#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "armcontrol.hpp"
#include "globals.hpp"

extern pros::adi::DigitalOut doinker; // Reference to doinker defined in constants.cpp
extern pros::adi::DigitalOut rushMech;
extern pros::Optical optical;

#include "colorsort.hpp"

void redRingRush() {
  chassis.setPose(0,0,0);
  chassis.moveToPoint(0,-37.1,4000, {.forwards = false, .maxSpeed = 60}, false);

  mogoClamp.set_value(true);
  pros::delay(750);
  intake1.move_velocity(600);
  chassis.turnToHeading(90,2000);

  chassis.moveToPoint(26, -37.1, 4000, {.maxSpeed = 60});
 /*  mogoClamp.set_value(true); 
 
   // Start the colorsort task to eject red rings
   startColorSortTask(ColorSortMode::RED);
 
   pros::delay(100000);

   stopColorSortTask(); */
  //intake2.move_velocity(-600);
 //chassis.moveToPose(-7.2, 31.05,-77.7, 3000, {.forwards = false, .maxSpeed = 40}, false);
 /* chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  setArmPosition(70);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);

  //initial rush
  rushMech.set_value(true);
  chassis.moveToPoint(-17.9, 40,  2700, {.maxSpeed = 80},false);
  //chassis.moveToPoint(-8, 40.,  2700, {.maxSpeed = 80},false);
  pros::delay(10);
  
  //back up after rush into mogo
  chassis.moveToPose(-5.8, 30,-83.4, 3000, {.forwards = false, .maxSpeed = 40}, false);
  intake1.move_velocity(0);
  intake2.move_velocity(0);

  
  //drive backwards while clamping mogo and do other bot functions
  chassis.moveToPoint(7, 28.3, 500, {.forwards = false, .maxSpeed = 75}, true);
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
  chassis.moveToPoint(-12.8, 23.1, 1000, {.maxSpeed = 60}, false);
  /*
  chassis.moveToPoint(-12, 28.4, 1000, {.maxSpeed = 60}, false);
  pros::delay(100);

  intake2.move_velocity(-600);
  //go towards and score wall stake
  chassis.moveToPose(-26.5, 45.5, -50.4, 2500, {.maxSpeed = 50}, false);
  setArmPosition(155);
  pros::delay(200);

  //back up from wall stake and touch climb ladder structure
  //chassis.moveToPoint(-10.6,18.7,1000);
  //chassis.moveToPoint(7.4,31.2,1000);

  */
  
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

void BGoalRush() {
  color = 0;
  //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0,0,0);

  doinker.set_value(true);
  rushMech.set_value(true);

  chassis.moveToPoint(0,35, 3000,{}, true);
  pros::delay(830);
  rushMech.set_value(false);
  pros::delay(500);

  
  chassis.moveToPoint(-3, 25, 8000,{.forwards = false}, false);

  rushMech.set_value(true);

  chassis.moveToPoint(-3, 17, 7000,{.forwards = false}, false);
  
  doinker.set_value(false);
  chassis.turnToHeading(-106.7, 1500, {.minSpeed = 20}, false);

  chassis.moveToPoint(12.1, 25.2, 2000, {.forwards = false, .maxSpeed = 70}, true);

  while (chassis.isInMotion() && distance.get() > 40) {
    pros::delay(10); // save cpu resources
  }
  chassis.cancelMotion();
  mogoClamp.set_value(true);

  pros::delay(700);
  intake1.move_velocity(500);

 // pros::delay(1000);
  //intake1.move_velocity(0);

  


  chassis.moveToPose(-26.7, 20, -124.9, 2500, {}, false);
  intake1.move_velocity(-600);



  doinker.set_value(true);
  rushMech.set_value(false);

  pros::delay(500);
  chassis.moveToPoint(-32.6, 11.4, 1000,  {.minSpeed = 20}, false);

  chassis.turnToHeading(-208, 500, {.maxSpeed = 100});
   
  chassis.moveToPoint(-27.7, 2.3, 1000, {}, false);
    // AWP (Alliance Win Point) autonomous routine
    // Add your AWP-specific autonomous code here
 
  doinker.set_value(false);

  

 // chassis.moveToPoint(-26.5, -1.1, 1500);

//  chassis.moveToPoint(-23.9, 4.7, 1500);


  chassis.moveToPose(-36.9, 7.7, -280.3, 1000, {.forwards = false}, false);

  mogoClamp.set_value(false);

  pros::delay(500);

  chassis.moveToPoint(-5.1, 8.7, 500, {.maxSpeed = 120});

  chassis.moveToPoint(-11.3, 11.8, 500, {.maxSpeed = 120});

 // chassis.turnToHeading(-189.3, 1500);

  chassis.moveToPoint(8.7, 36.0, 2000, {.forwards = false, .maxSpeed = 80}, true);

  while (chassis.isInMotion() && distance.get() > 40) {
    pros::delay(10); // save cpu resources
  }
  chassis.cancelMotion();
  mogoClamp.set_value(true);
  pros::delay(500);
  //chassis.moveToPoint(8.2, 32.0, 2000, {.forwards = true, .maxSpeed = 80}, true);

  //chassis.turnToHeading(-82.9, 2000, {.maxSpeed = 80} , false);

  //intake1.move_velocity(600);

  //chassis.moveToPoint(0, 39.2, 2000);
}


void RGoalRush(){
  color = 0;
  chassis.setPose(0,0,0); //set origin to 0,0,0

  doinker.set_value(true); //enable piston for doinker
  rushMech.set_value(true); //enable piston for rushMech

  chassis.moveToPoint(0,35, 3000,{}, true); // move to x-0, y-35, with timeout of 3000
  pros::delay(830); //delay to save cpu resources
  rushMech.set_value(false); //disable piston for rushMech
  pros::delay(500); //delay to save cpu resources

  
  chassis.moveToPoint(3, 25, 8000,{.forwards = false}, false); // 

  rushMech.set_value(true);

  chassis.moveToPoint(3, 17, 7000,{.forwards = false}, false);
  
  doinker.set_value(false);
  chassis.turnToHeading(106.7, 1500, {.minSpeed = 20}, false);

  chassis.moveToPoint(-12.1, 25.2, 2000, {.forwards = false, .maxSpeed = 70}, true);

  while (chassis.isInMotion() && distance.get() > 40) {
    pros::delay(10); // save cpu resources
  }
  chassis.cancelMotion();
  mogoClamp.set_value(true);

  pros::delay(700);
  intake1.move_velocity(500);

 // pros::delay(1000);
  //intake1.move_velocity(0);

  


  chassis.moveToPose(26.7, 20, 124.9, 2500, {}, false);
  intake1.move_velocity(-600);



  doinker.set_value(true);
  rushMech.set_value(false);

  pros::delay(500);
  chassis.moveToPoint(32.6, 11.4, 1000,  {.minSpeed = 20}, false);

  chassis.turnToHeading(208, 500, {.maxSpeed = 100});
   
  chassis.moveToPoint(27.7, 2.3, 1000, {}, false);
    // AWP (Alliance Win Point) autonomous routine
    // Add your AWP-specific autonomous code here
 
  doinker.set_value(false);

  

 // chassis.moveToPoint(-26.5, -1.1, 1500);

//  chassis.moveToPoint(-23.9, 4.7, 1500);


  chassis.moveToPose(36.9, 7.7, 280.3, 1000, {.forwards = false}, false);

  mogoClamp.set_value(false);

  pros::delay(500);

  chassis.moveToPoint(5.1, 8.7, 500, {.maxSpeed = 120});

  chassis.moveToPoint(11.3, 11.8, 500, {.maxSpeed = 120});

 // chassis.turnToHeading(-189.3, 1500);

  chassis.moveToPoint(-8.7, 36.0, 2000, {.forwards = false, .maxSpeed = 80}, true);

  while (chassis.isInMotion() && distance.get() > 40) {
    pros::delay(10); // save cpu resources
  }
  chassis.cancelMotion();
  mogoClamp.set_value(true);
  pros::delay(500);
  //chassis.moveToPoint(8.2, 32.0, 2000, {.forwards = true, .maxSpeed = 80}, true);

  //chassis.turnToHeading(-82.9, 2000, {.maxSpeed = 80} , false);

  //intake1.move_velocity(600);

  //chassis.moveToPoint(0, 39.2, 2000);
}

void blueRingRush(){
  // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);  
  chassis.setPose(0,0,0);
  chassis.moveToPoint(0,-3,4000, {.forwards = false, .maxSpeed = 50}, false);

  mogoClamp.set_value(true);
  pros::delay(750);
  intake1.move_velocity(600);
  chassis.turnToHeading(-90,2000);

  chassis.moveToPoint(-10, -35, 4000, {.maxSpeed = 60});
  
}


/*
 void blueGoalRush(){
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0,0,0);
  rushMech.set_value(true);
  doinker.set_value(true);
  chassis.moveToPoint(0, 38.8, 10000, {.forwards = true, .maxSpeed = 50});
  //moves to mid mogo and grabs it w doinker
} 
*/






void skills() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  usePiston = false;


  chassis.setPose(0,0,0);
  // Set initial arm position
  setArmPosition(225);
  pros::delay(1000);
  chassis.moveToPoint(0,-5.7,500, {.forwards = false, .maxSpeed = 20});
  setArmPosition(0);
  chassis.turnToHeading(-90,2000);
  intake1.move_velocity(600);

  chassis.moveToPoint(18.7,-5.7, 3000, { .forwards = false, .maxSpeed = 70}, false);
  mogoClamp.set_value(true);
  pros::delay(750);

  chassis.turnToHeading(-180, 1500);

  chassis.moveToPoint(22.5, -20.9, 2000);

  pros::delay(200);

  chassis.turnToHeading(-270, 750);

  chassis.moveToPoint(42, -23.4, 1500);

  pros::delay(300);

  chassis.turnToHeading(0, 500);

  chassis.moveToPoint(42, 7.8, 2000);



}

void (*autonFunctions[])() = {redRingRush, BGoalRush, RGoalRush, blueRingRush, skills};

int autonSelect = 1;
std::string autonNames[5] = {"redRingRush", "BGoalRush", "RGoalRush", "blueRingRush", "Skills"};

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