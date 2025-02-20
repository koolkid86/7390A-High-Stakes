#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "pros/adi.hpp"
#include "pros/optical.hpp"



#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/adi.hpp"
// MOTORS
extern pros::Motor arm;
extern pros::Motor intake1;

// PNEUMATICS
extern pros::adi::DigitalOut mogoClamp;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut rushMech;
extern pros::adi::DigitalOut brownLady;

// SENSORS
extern pros::Distance distance;
extern pros::Imu imu;
extern pros::adi::Encoder encoder;
extern pros::Optical optical;

#endif // CONSTANTS_HPP
