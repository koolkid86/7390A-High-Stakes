#include "constants.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"


// MOTORS
pros::Motor arm(19);
pros::Motor intake1(7);

#define QUAD_TOP_PORT 'C'
#define QUAD_BOTTOM_PORT 'B'

// PNEUMATICS
pros::adi::DigitalOut mogoClamp('A');
pros::adi::DigitalOut doinker('E'); // Doinker pneumatic on port B
pros::adi::DigitalOut rushMech('G');
pros::adi::DigitalOut brownLady('F');

// SENSORS
pros::Distance distance(11);
pros::Imu imu(18);
pros::adi::Encoder encoder('C', 'D', true);

pros::Optical optical(12);
