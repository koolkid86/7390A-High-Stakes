#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// motorsssssssss

pros::MotorGroup
    left_motors({-9, 20, -10},
                pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup
    right_motors({-1, 5, 7},
                 pros::MotorGearset::blue); // right motors on ports 4, 5, 6

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,             // left motor group
                              &right_motors,            // right motor group
                              12.5,                     // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              300,                      // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// create an imu on port 11
pros::Imu imu(11);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(7,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       30,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller( 2, // proportional gain (kP)
						0, // integral gain (kI)
						10, // derivative gain (kD)
						0, // anti windup
						0, // small error range, in inches
						0, // small error range timeout, in milliseconds
						0, // large error range, in inches
						0, // large error range timeout, in milliseconds
						0 // maximum acceleration (slew)
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

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();

  pros::lcd::register_btn1_cb(on_center_button);

  pros::adi::DigitalOut mogo('A');
  pros::Motor intake(2);

  pros::Task screen_task([&](){
	while(true){
		pros::lcd::print(1, "X: %f", chassis.getPose().x);
		pros::lcd::print(2, "Y: %f", chassis.getPose().y);
		pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);

        


		pros::delay(20);
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
//load "first.txt"
// the file should be in the "static" folder in the project root directory
// this should also be done outside of any functions, otherwise it won't compile
ASSET(first_txt); // we replace "." with "_" to make the asset name valid

// autonomous function in your project. The function that runs during the autonomous period
void autonomous() {
    // Motors and clamp initialization
    pros::Motor intake(2);
    pros::adi::DigitalOut mogoClamp('A'); // Mogo clamp mechanism control

    // Step 1: Move to the mogo and clamp it
    chassis.moveToPoint(0, -23, 4000, {.forwards = true}, true);
    pros::delay(300); // Allow time to settle
    mogoClamp.set_value(true); // Clamp the mogo
    pros::delay(500); // Allow time for clamping

    // Step 2: Move to the next position to potentially score or reposition
    chassis.moveToPoint(-23, -23, 4000, {.forwards = true}, true);

    // Step 3: Navigate towards the scoring area or designated target zone
    chassis.moveToPoint(-23, -46, 4000, {.forwards = true}, true);

    // Step 4: Automated intake for picking up rings/disks
    intake.move_velocity(500); // Activate intake to collect rings/disks
    pros::delay(2000); // Adjust time based on field setup and scoring opportunity
    intake.move_velocity(0); // Stop intake

    // Step 5: Position near a scoring zone and release the mogo (if applicable)
    chassis.moveToPoint(0, -45, 4000, {.forwards = true}, true);
    mogoClamp.set_value(false); // Release the mogo
    pros::delay(500); // Allow time for release

    // Optional additional actions (e.g., realignment, further intake, etc.)
}


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


bool lastL1 = false; // Keep track of the last state of the button



pros::Controller controller(pros::E_CONTROLLER_MASTER); // define controller
pros::Motor intake (2);// intake motor
pros::adi::DigitalOut mogo ('A'); // mogo clamp pneumatics
pros::Distance distance (19); // distance sensor

 // Declare the controller once
//#include "pros/tasks.hpp"

// Variables shared between tasks
bool currentButtonState = false;      // Current state of the L1 button
bool lastButtonState = false;         // Last state of the L1 button
bool manualOverride = false;          // Indicates if manual override is active
bool mogoState = false;               // Pneumatic clamp state
bool autoClamped = false;             // Flag indicating if the clamp was automatically triggered
pros::Mutex mogoMutex;                // Mutex to prevent race conditions

const double DISTANCE_THRESHOLD = 30; // Example threshold in your distance sensor's unit

// Task to handle clamp reset logic
void clampResetTask(void* param) {
    while (true) {
        mogoMutex.take();
        if (!mogoState) {
            autoClamped = false;      // Reset auto-clamp flag
            manualOverride = false;   // Disable manual override
        }
        mogoMutex.give();
        pros::delay(10);              // Short delay to reduce CPU usage
    }
}

// Main operator control function
void opcontrol() {
    // Start the clamp reset task
    pros::Task resetTask(clampResetTask);

    // loop forever
    while (true) {
        // Get left Y and right X positions for arcade drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Move the robot using arcade drive
        chassis.arcade(leftY, rightX);

        // Intake control block (hold)
        if (controller.get_digital(DIGITAL_R1)) {
            intake.move_velocity(600);
        } else if (controller.get_digital(DIGITAL_R2)) {
            intake.move_velocity(-600);
        } else {
            intake.move_velocity(0);
        }

        //////////////////////////////////////////////////////////////////////////
        // Get the current state of the L1 button
        currentButtonState = controller.get_digital(DIGITAL_L1);

        // Check if the L1 button was just pressed (edge detection)
        if (currentButtonState && !lastButtonState) {
            mogoMutex.take();
            manualOverride = !manualOverride;  // Toggle manual override state
            mogoState = !mogoState;            // Toggle the clamp state manually
            mogo.set_value(mogoState);         // Apply the manual state to the clamp
            mogoMutex.give();
        }

        // Auto-clamp logic (only active if manual override is not engaged)
        if (!manualOverride) {
            double distanceValue = distance.get();  // Get the distance from the sensor

            // Trigger auto-clamping if the robot is close enough to an object
            if (distanceValue < DISTANCE_THRESHOLD && !autoClamped) {
                mogoMutex.take();
                mogoState = true;            // Close the clamp automatically
                mogo.set_value(mogoState);   // Activate pneumatic clamp
                autoClamped = true;          // Mark auto-clamp as triggered
                mogoMutex.give();
            }
        }

        // Update the last button state for the next loop iteration
        lastButtonState = currentButtonState;

        // Delay to save resources
        pros::delay(25);  // Delay for stability
    }
}
