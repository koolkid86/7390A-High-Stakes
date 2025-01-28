#include "lemlib/api.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "armcontrol.hpp"

extern pros::adi::DigitalOut doinker; // Reference to doinker defined in constants.cpp
extern pros::adi::DigitalOut rushMech;


void redRingRush() {




  //with comments added :(

  //setArmPosition(60);
   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  //setArmPosition(60);

  rushMech.set_value(true);
//first rush motion thing:
  chassis.moveToPoint(-7.5, 39.0, 1400, {.maxSpeed = 70},false);
  //running all the way to the back
  chassis.turnToHeading(-64.0, 1400, {.maxSpeed = 30});
  //cool turn
  chassis.moveToPoint(-5.0, 28.4, 700, {.forwards = false, .maxSpeed = 40},false);
  //woah where's the mogo

  chassis.moveToPoint(15.2, 13.9, 2500, {.forwards = false, .maxSpeed =40}, true );
  //run towards mogo and clamp
  while (chassis.isInMotion()){

    if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }

  mogoClamp.set_value(true);
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);
  //intake stuff for getting more rings
  chassis.moveToPoint(-10.6, 25.7, 1000, {.forwards = true, .maxSpeed = 60}, false );
 // chassis.moveToPoint(-19.2, 29.5, 1000, {.forwards = true, .maxSpeed = 60}, false );
 //get the last extra ring
  pros::delay(100);


//last ring again oops
  chassis.moveToPoint(-24.7, 15.6, 1000, {.forwards = true, .maxSpeed = 60}, false );

  pros::delay(2000);

  intake2.move_velocity(-600);
//touch the
  chassis.moveToPoint(8.2, 16.2, 1100, {.forwards = false, .maxSpeed = 80}, false);
  setArmPosition(130);











  chassis.moveToPoint(25.0, 46.5, 1100, {.forwards = true, .maxSpeed = 80}, false);/*
 

  /*

  chassis.turnToHeading(-40, 900, {.maxSpeed = 50});
  chassis.moveToPose(-39, 40.1, -45.3, 1800, {.maxSpeed =100}, false);

  pros::delay(100);
  
  setArmPosition(155);
  chassis.turnToHeading(-45.7, 400, {. maxSpeed =60});
  pros::delay(200);1☺
  chassis.turnToHeading(44.9, 400, {. maxSpeed = 60});
  pros::delay(500);
  */
 /**/

  /*

  chassis.turnToHeading(-40, 900, {.maxSpeed = 50});
  chassis.moveToPose(-39, 40.1, -45.3, 1800, {.maxSpeed =100}, false);

  pros::delay(100);
  
  setArmPosition(155);
  chassis.turnToHeading(-45.7, 400, {. maxSpeed =60});
  pros::delay(200);
  chassis.turnToHeading(44.9, 400, {. maxSpeed = 60});
  pros::delay(500);
  */
/*
  chassis.moveToPoint(-8.2, 16.2, 1500, {.forwards = false, .maxSpeed = 80}, false);
  setArmPosition(130);
  
  chassis.moveToPoint(25.0, 46.5, 1500, {.forwards = true, .maxSpeed = 80}, false);
*/
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

void redGoalRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);





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


void blueRingRush(){
    
    

  //with comments added :)

  //setArmPosition(60);
   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  //setArmPosition(60);

  rushMech.set_value(true);
//first rush motion thing:
  chassis.moveToPoint(7.5, 39.0, 1400, {.maxSpeed = 70},false);
  //running all the way to the back
  chassis.turnToHeading(64.0, 1400, {.maxSpeed = 30});
  //cool turn
  chassis.moveToPoint(5.0, 28.4, 700, {.forwards = false, .maxSpeed = 40},false);
  //woah where's the mogo

  chassis.moveToPoint(-15.2, 13.9, 2500, {.forwards = false, .maxSpeed =40}, true );
  //run towards mogo and clamp
  while (chassis.isInMotion()){

    if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }

  mogoClamp.set_value(true);
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);
  //intake stuff for getting more rings
  chassis.moveToPoint(10.6, 25.7, 1000, {.forwards = true, .maxSpeed = 60}, false );
 // chassis.moveToPoint(-19.2, 29.5, 1000, {.forwards = true, .maxSpeed = 60}, false );
 //get the last extra ring
  pros::delay(100);


//last ring again oops
  chassis.moveToPoint(26.7, 15.6, 1000, {.forwards = true, .maxSpeed = 60}, false );

  pros::delay(2000);

  intake2.move_velocity(-600);
//touch the
  chassis.moveToPoint(-8.2, 16.2, 1100, {.forwards = false, .maxSpeed = 80}, false);
  setArmPosition(130);

  chassis.moveToPoint(-25.0, 46.5, 1100, {.forwards = true, .maxSpeed = 80}, false);/*
 
    //thought this worked but turned out bad
    /*
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);


  rushMech.set_value(true);
  chassis.moveToPoint(7.5, 39.0, 1400, {.maxSpeed = 70},false);
  chassis.turnToHeading(64.0, 1400, {.maxSpeed = 30});
  chassis.moveToPoint(5.0, 28.4, 700, {.forwards = false, .maxSpeed = 40},false);

  chassis.moveToPoint(-15.2, 13.9, 2500, {.forwards = false, .maxSpeed =40}, true );
  while (chassis.isInMotion()){

    if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }
  mogoClamp.set_value(true);
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);
  chassis.moveToPoint(10.6, 25.7, 1000, {.forwards = true, .maxSpeed = 60}, false );
 // chassis.moveToPoint(19.2, 29.5, 1000, {.forwards = true, .maxSpeed = 60}, false );
  pros::delay(100);



  chassis.moveToPoint(24.7, 15.6, 1000, {.forwards = true, .maxSpeed = 60}, false );

  pros::delay(2000);

  intake2.move_velocity(-600);

  chassis.moveToPoint(-8.2, 16.2, 1100, {.forwards = false, .maxSpeed = 80}, false);
  setArmPosition(130);
  
  //chassis.moveToPoint(28.0, 46.5, 1100, {.forwards = true, .maxSpeed = 80}, false);

  /*
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  setArmPosition(60);

  rushMech.set_value(true);
  chassis.moveToPoint(7.5, 39.0, 1400, {.maxSpeed = 70},false);
  chassis.turnToHeading(64.0, 1400, {.maxSpeed = 30});
  chassis.moveToPoint(5.0, 28.4, 700, {.forwards = false, .maxSpeed = 40},false);

  chassis.moveToPoint(-15.2, 13.9, 2500, {.forwards = false, .maxSpeed =40}, true );
  while (chassis.isInMotion()){

    if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }
  mogoClamp.set_value(true);
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);
  chassis.moveToPoint(10.6, 27.7, 1400, {.forwards = true, .maxSpeed = 60}, false );
  chassis.moveToPoint(19.2, 29.5, 1400, {.forwards = true, .maxSpeed = 60}, false );
  pros::delay(100);

  intake2.move_velocity(-600);


  chassis.turnToHeading(40, 900, {.maxSpeed = 50});
  chassis.moveToPose(39, 40.1, 45.3, 1800, {.maxSpeed =100}, false);
  pros::delay(100);
  setArmPosition(155);
  chassis.turnToHeading(45.7, 400, {. maxSpeed =60});
  pros::delay(200);
  chassis.turnToHeading(-44.9, 400, {. maxSpeed = 60});
  pros::delay(500);

  chassis.moveToPoint(8.2, 16.2, 1500, {.forwards = false, .maxSpeed = 127}, false);
  setArmPosition(130);
  chassis.moveToPoint(-28.0, 49.5, 1500, {.forwards = true, .maxSpeed = 127}, false);

*/
}



void redRingRushV2(){
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); 

  
  rushMech.set_value(true);
  chassis.moveToPoint(-8.3, 25.8, 700, {.maxSpeed = 50, .minSpeed = 30},false);
  chassis.moveToPose(-17.8, 46.6, -5.0, 1900, {.maxSpeed = 70},false);
  chassis.moveToPoint(-14.8, 34, 700, {.forwards = false, .maxSpeed = 50, .minSpeed = 30},false);

  chassis.turnToHeading(-64.0, 1400, {.maxSpeed = 20});

  chassis.moveToPoint(15.2, 13.9, 2500, {.forwards = false, .maxSpeed =40}, true );
  while (chassis.isInMotion()){

    if (distance.get() < 22){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }
  mogoClamp.set_value(true);
  rushMech.set_value(false);
  intake1.move_velocity(600);
  intake2.move_velocity(600);

  /*
  chassis.moveToPoint(-10.6, 27.7, 1400, {.forwards = true, .maxSpeed = 60}, false );
  chassis.moveToPoint(-19.2, 29.5, 1400, {.forwards = true, .maxSpeed = 60}, false );
  pros::delay(100);

  intake2.move_velocity(-600);
  */

 /* chassis.turnToHeading(-40, 900, {.maxSpeed = 50});
  chassis.moveToPose(-39, 40.1, -45.3, 1800, {.maxSpeed =100}, false);
  pros::delay(100);
  setArmPosition(155);
  chassis.turnToHeading(-45.7, 400, {. maxSpeed =60});
  pros::delay(200);
  chassis.turnToHeading(44.9, 400, {. maxSpeed = 60});
  pros::delay(500);

  chassis.moveToPoint(-8.2, 16.2, 1500, {.forwards = false, .maxSpeed = 127}, false);
  setArmPosition(130);
  chassis.moveToPoint(28.0, 49.5, 1500, {.forwards = true, .maxSpeed = 127}, false); */
}






void skills() {
  
 
  setArmPosition(135);
  pros::delay(1000);
  chassis.moveToPoint(0, 2.5, 500, {.maxSpeed = 50}, false);
  chassis.moveToPoint(0, -11.7, 1500, {.forwards = false, .maxSpeed = 50}, false);
  setArmPosition(1);
  chassis.turnToHeading(-90, 1500, {.maxSpeed = 50}); // turn to face mogo

  chassis.moveToPoint(25, -22.1, 5000, {.forwards = false, .maxSpeed = 50}, true); // go back to clamp mogo
  
   while (chassis.isInMotion()){

    if (distance.get() < 30){
        mogoClamp.set_value(true);
        pros::delay(200);
        chassis.cancelMotion();
        break;
    }

    pros::delay(20);
  }

  mogoClamp.set_value(true);

  chassis.turnToHeading(90, 1500, {.maxSpeed = 50}); // turn to face rings

  intake1.move_velocity(600);
  intake2.move_velocity(600);

  chassis.moveToPoint(35,-22, 1500, {.maxSpeed = 30}, false); // go pick up the 2 rings

  //scuffed antijam code
  pros::delay(1000);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  pros::delay(400);
  intake1.move_velocity(600);
  intake2.move_velocity(600);


  chassis.moveToPoint(50,-22, 2000, {.maxSpeed = 30}, false); // go pick up the 2 rings

   pros::delay(1000);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  pros::delay(400);
  intake1.move_velocity(600);
  intake2.move_velocity(600);
  
  //back up after picking up the rings
  chassis.moveToPoint(22, -22, 1500, {.forwards = false, .maxSpeed = 50}, false); // go pick up the last ring in the cornerish
  chassis.turnToHeading(59.4, 1500, {.maxSpeed = 50},false);
  chassis.moveToPoint(45.9, -7.4, 1500, {.maxSpeed = 50},false);
  //scuffed antijam code
  pros::delay(1000);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  pros::delay(400);
  intake1.move_velocity(600);
  intake2.move_velocity(600);

  //back up and angle to pick up other two rings towards wallstake
  chassis.moveToPoint(27.5, -14.5, 2000, {.forwards = false, .maxSpeed = 50}, false);
  chassis.turnToHeading(144.7, 2000);

  chassis.moveToPoint(37.4, -40.1, 2000, {.forwards = true, .maxSpeed = 50}, false);
  //scuffed antijam code
  pros::delay(1000);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  pros::delay(400);
  intake1.move_velocity(600);
  intake2.move_velocity(600);


chassis.moveToPoint(52.4,-67.8,2000, {.maxSpeed = 50}, false);
  //scuffed antijam code
  pros::delay(1000);
  intake1.move_velocity(-600);
  intake2.move_velocity(-600);
  pros::delay(400);
  intake1.move_velocity(600);
  intake2.move_velocity(600);

 chassis.moveToPoint(56,-8, 4000, {.forwards = false, .maxSpeed = 80},false);
 mogoClamp.set_value(false);
 chassis.moveToPoint(0,-8, 5000, {.maxSpeed = 30});

intake1.move_velocity(0);
intake2.move_velocity(0);

//REMOVE FOR ACTUAL COMP
while (distance.get() > 30){
  pros::delay(25);

}

pros::delay(1000);

chassis.moveToPoint(-27.6, -11.9, 5000, {.maxSpeed = 50});
chassis.moveToPoint(-70.6, 8.2, 5000, {.maxSpeed = 100});

  

  

}

void (*autonFunctions[])() = { redGoalRush, skills, blueRingRush, redRingRushV2, redRingRush};
 
int autonSelect = 1;
std::string autonNames[5] = { "redGoalRush", "skills", "blueRingRush", "RED RINGRUSHV2", "redringRush"};

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