#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/llemu.hpp"

pros::adi::DigitalOut mogo('A');
pros::Motor intake(2);
#define QUAD_TOP_PORT 'C'
#define QUAD_BOTTOM_PORT 'B'



pros::adi::Encoder encoder('C', 'D', true); // Replace 'A' and 'B' with the actual ports.


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

//need to add wallstake pid with the vex encoders

lemlib::ControllerSettings lateral_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              70 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              24, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew)
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

  encoder.reset(); 


  pros::lcd::print(5, "Initial Encoder Ticks: %d", encoder.get_value());

  pros::Task screen_task([&]() {
    while (true) {
        pros::lcd::print(1, "X: %f", chassis.getPose().x);
        pros::lcd::print(2, "Y: %f", chassis.getPose().y);
        pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);
        pros::lcd::print(4, "Encoder Ticks: %d", encoder.get_value());
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
// we replace "." with "_" to make the asset name valid
pros::Distance distance (19); 
// autonomous function in your project. The function that runs during the autonomous period
void autonomous() {
    // Motors and clamp initialization
    pros::Motor intake(2);
    pros::adi::DigitalOut mogoClamp('A'); // Mogo clamp mechanism control

    chassis.setPose(0, 0, 0);

   
    chassis.moveToPoint(0, -45, 2500,    {.forwards = false, .maxSpeed=75, }, true);
    while(chassis.isInMotion() && distance.get() > 30){ 
        pros::delay(10); // save cpu resources
    }
    mogoClamp.set_value(true);
     pros::delay(100); // wait for mogo to clamp and settle
    chassis.cancelMotion(); // cancel the motion once the robot detects mogo is in the bot and clamped


    //pros::delay(200); // wait for mogo to clamp and settle

    chassis.moveToPoint(17, -38,  3000, {.maxSpeed=90}, true);
    intake.move_velocity(600);
    chassis.moveToPoint(18, -40, 1000) ;

   
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




pros::Controller controller(pros::E_CONTROLLER_MASTER); // define controller

// distance sensor
const double DISTANCE_THRESHOLD = 30; // Example threshold in your distance sensor's unit
bool currentButtonState = false;      // Current state of the L1 button
bool lastButtonState = false;         // Last state of the L1 button
bool manualOverride = false;          // Indicates if manual override is active
bool mogoState = false;               // Pneumatic clamp state
bool autoClamped = false;             // Flag indicating if the clamp was automatically triggered
int autoClampLastActivated = 0;       // Timestamp of the last auto-clamp action in milliseconds
const int AUTO_CLAMP_COOLDOWN = 4000;  // Cooldown period in milliseconds
pros::Motor arm(8); 



// Define PID constants
const double kP = 0.5;   // Proportional gain
const double kI = 0.001; // Integral gain
const double kD = 0.1;   // Derivative gain

// Define target positions
const int ARM_POSITION_1 = 120; // Replace with your desired encoder value
const int ARM_POSITION_2 = 17; // Replace with your desired encoder value

// PID controller variables
double targetPosition = ARM_POSITION_1; // Initial target position
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;
double output = 0;

// Button toggle logic
bool armButtonState = false;
bool lastArmButtonState = false;
bool xButtonState = false;


void opcontrol() {

    

    pros::Motor arm(8);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // Set the arm motor to hold its position when stopped
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);




    // loop forever
    while (true) {
        // Get left Y and right X positions for arcade drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Move the robot using arcade drive
        chassis.arcade(leftY, rightX);

   


        //////////////////////////////////////////////////////////////////////////////////////////////////////



          // Toggle target position with L1 button
        armButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        if (armButtonState && !lastArmButtonState) {
            // Toggle between positions
            targetPosition = (targetPosition == ARM_POSITION_1) ? ARM_POSITION_2 : ARM_POSITION_1;
        }
        lastArmButtonState = armButtonState;

        // Move to zero position with X button
        xButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (xButtonState) {
            targetPosition = 0; // Override target position to zero
        }

        // PID control for the arm
        int currentPosition = encoder.get_value(); // Read the encoder
        error = targetPosition - currentPosition;     // Calculate error
        integral += error;                            // Update integral
        derivative = error - lastError;               // Calculate derivative
        output = (kP * error) + (kI * integral) + (kD * derivative); // PID formula
        lastError = error;                            // Update last error

        // Limit output to prevent excessive speed
        output = std::clamp(output, -127.0, 127.0);

        // Set motor power
        arm.move_voltage(output * 120); // Convert to millivolts for PROS

        // Optional: Reset integral if the error is negligible to avoid windup
        if (std::abs(error) < 10) {
            integral = 0;
        }


        ///////////////////////////////////////////////////////////////////////////////////////////





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
            manualOverride = true;            // Enable manual override
            mogoState = !mogoState;           // Toggle the clamp state manually
            mogo.set_value(mogoState);        // Apply the manual state to the clamp

            if (!mogoState) {
                autoClamped = false;          // Reset auto-clamp flag if manually unclamped
                manualOverride = false;
            }
        }

        // Auto-clamp logic (only active if manual override is not engaged)
        int currentTime = pros::millis();     // Get the current time in milliseconds
        if (!manualOverride && (currentTime - autoClampLastActivated >= AUTO_CLAMP_COOLDOWN)) {
            double distanceValue = distance.get();  // Get the distance from the sensor

            // Trigger auto-clamping if the robot is close enough to an object
            if (distanceValue < DISTANCE_THRESHOLD && !autoClamped) {
                mogoState = true;             // Close the clamp automatically
                mogo.set_value(mogoState);    // Activate pneumatic clamp
                autoClamped = true;           // Mark auto-clamp as triggered
                controller.rumble("--");       // Provide haptic feedback
                autoClampLastActivated = currentTime; // Record the time of this action
            }
        }

         

        // Update the last button state for the next loop iteration
        lastButtonState = currentButtonState;
/////////////////////////////////////////////////////////////////////////////////////////


    



    pros::delay(25);  // Delay for stability
}
}






