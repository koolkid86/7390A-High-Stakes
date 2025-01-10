#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/adi.hpp"

// MOTORS
pros::Motor arm(14);
pros::Motor intake1(2);
pros::Motor intake2(8);
#define QUAD_TOP_PORT 'C'
#define QUAD_BOTTOM_PORT 'B'

// PNEUMATICS
pros::adi::DigitalOut mogoClamp('A');
pros::adi::DigitalOut doinker('B'); // Doinker pneumatic on port B
pros::adi::DigitalOut rushMech('H');

// SENSORS
pros::Distance distance(19);
pros::Imu imu(11);
pros::adi::Encoder encoder('C', 'D', true);

// DRIVE
pros::MotorGroup left_motors({-9, 20, -10},
                             pros::MotorGearset::blue); // left motors
pros::MotorGroup right_motors({-1, 5, 7},
                              pros::MotorGearset::blue); // right motors

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,             // left motor group
                              &right_motors,            // right motor group
                              12.5,                     // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              300,                      // drivetrain rpm is 300
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// need to add wallstake pid with the vex encoders

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(30, // proportional gain (kP)
                       0,  // integral gain (kI)
                       10, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       60  // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(4,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       24, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       60  // maximum acceleration (slew)
    );

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,   // joystick deadband out of 127
                   10,  // minimum output where drivetrain will move out of 127
                   1.03 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,   // joystick deadband out of 127
                10,  // minimum output where drivetrain will move out of 127
                1.01 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);