#include "main.h"
#include "auton.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <string>
#include "pros/misc.h"
#include <vector>
#include <fstream>
#include "armcontrol.hpp"
#include "pros/rtos.hpp"



#define ts std::to_string

extern pros::adi::DigitalOut doinker; // Reference to doinker defined in constants.cpp
extern pros::adi::DigitalOut rushMech;


pros::Rotation horizontalSensor(17);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalSensor, lemlib::Omniwheel::OLD_275, -3);
//



// DRIVE
pros::MotorGroup left_motors({-9, 20, -10},
                             pros::MotorGearset::blue); // left motors
pros::MotorGroup right_motors({-1, 5, 7},
                              pros::MotorGearset::blue); // right motors

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,             // left motor group
                              &right_motors,            // right motor group
                              13,                     // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              300,                      // drivetrain rpm is 300
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
     &horizontal_tracking_wheel, // horizontal tracking wheel 1
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



void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); 
    chassis.setPose(0, 0, 0);
    encoder.reset();
    pros::delay(5000); //give time for imu to calibrate
    pros::lcd::set_text(0, "Auton Selected = " + autonNames[autonSelect]);

    pros::lcd::register_btn0_cb(previousAuton);
    pros::lcd::register_btn2_cb(nextAuton);

    pros::lcd::print(5, "Initial Encoder Ticks: %d", encoder.get_value());

    pros::Task screen_task([&]() {
        while (true) {
            // Combine position data onto one line
            pros::lcd::print(1, "X:%.1f Y:%.1f T:%.1f", 
                           chassis.getPose().x,
                           chassis.getPose().y,
                           chassis.getPose().theta);
            // Move encoder to line 2
            pros::lcd::print(2, "Enc:%d", encoder.get_value());
            pros::lcd::print(3, "Rotation Sensor: %i", horizontalSensor.get_position());
            pros::delay(20);
        }
    });

     pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() { autonFunctions[autonSelect](); }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // Set the arm motor to hold its position when stopped
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Define target angles for the arm
    const int ARM_ANGLE_ONE = 120; // Replace with actual encoder value for position 1
    const double ARM_ANGLE_TWO = 15.5; // Replace with actual encoder value for position 2

    // Variables for arm control
    static bool armTargetState = false; // Arm toggle state (starts at 0)
    bool armLockedAtZero = true; // Tracks if arm is locked at angle 0
    bool manualArmControl = false;
    const int MANUAL_ARM_SPEED = 100;  // Speed for manual control

    // Variables for mogo clamp control
    bool currentL1State = false;    // Tracks the current state of the L1 button
    bool lastL1State = false;       // Tracks the previous state of the L1 button
    bool manualOverride = false;    // Manual override state for mogo clamp
    bool mogoState = false;         // Current clamp state (open/closed)
    bool autoClamped = false;       // Tracks if the auto-clamp is engaged
    int autoClampLastActivated = 0; // Last activation time of auto-clamp

    // Auto-clamp settings
    const int AUTO_CLAMP_COOLDOWN = 5000; // Cooldown period for auto-clamp (ms)
    const double DISTANCE_THRESHOLD =
        30; // Distance threshold for auto-clamp (mm)

    // Main control loop
    while (true) {
        /////////////////////////// CHASSIS CONTROL //////////////////////////
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        ///////////////// ARM CONTROL /////////////////////////////
         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			nextState();
		}


        //////////////////////// INTAKE CONTROL //////////////////////////////
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake1.move_velocity(600); // Intake forward
            intake2.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            intake1.move_velocity(-600); // Intake reverse
            intake2.move_velocity(-600);
        } else {
            intake1.move_velocity(0); // Stop intake
            intake2.move_velocity(0);
        }

        ///////////////////// MOGO CLAMP CONTROL ////////////////////////////////
        currentL1State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // Manual toggle of mogo clamp
        if (currentL1State && !lastL1State) {
            manualOverride = true;          // Enable manual override
            mogoState = !mogoState;         // Toggle clamp state
            mogoClamp.set_value(mogoState); // Apply the toggle state

            if (!mogoState) {
                autoClamped = false; // Reset auto-clamp flag if manually unclamped
                manualOverride = false;
            }
        }

        // Auto-clamp logic (if manual override is not engaged)
        int currentTime = pros::millis();
        if (!manualOverride &&
            (currentTime - autoClampLastActivated >= AUTO_CLAMP_COOLDOWN)) {
            double distanceValue = distance.get(); // Get distance sensor value

            // Trigger auto-clamping if object is detected within threshold
            if (distanceValue < DISTANCE_THRESHOLD && !autoClamped) {
                mogoState = true;                     // Close clamp
                mogoClamp.set_value(mogoState);       // Activate pneumatic clamp
                autoClamped = true;                   // Mark auto-clamp as active
                controller.rumble("--");              // Haptic feedback
                autoClampLastActivated = currentTime; // Record activation time
            }
        }
        lastL1State = currentL1State; // Update last button state

        // Doinker control (toggle with L2)
        static bool doinkerState = false;
        static bool lastL2State = false;
        bool currentL2State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        
        if (currentL2State && !lastL2State) {  // Button just pressed
            doinkerState = !doinkerState;  // Toggle state
            doinker.set_value(doinkerState);
        }
        lastL2State = currentL2State;

         // rushMech control (toggle with B)
        static bool dState = false;
        static bool lL2State = false;
        bool cL2State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        
        if (cL2State && !lL2State) {  // Button just pressed
            dState = !dState;  // Toggle state
            rushMech.set_value(dState);
        }
        lL2State = cL2State;

        //////////////////////////////// LOOP DELAY ////////////////////////////////
        pros::delay(20); // Delay to prevent overloading cpu resources
    }
}
