#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/adi.hpp"


// MOTORS
pros::Motor arm(14);
pros::Motor intake1(2);
pros::Motor intake2(6);


// PNEUMATICS
pros::adi::DigitalOut mogoClamp('A');
pros::adi::DigitalOut doinker('B'); // Doinker pneumatic on port B
pros::adi::DigitalOut rushMech('H');

// SENSORS
pros::Distance distance(19);
pros::Imu imu(11);
pros::adi::Encoder encoder('C', 'D', true);