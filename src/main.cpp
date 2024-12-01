#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/llemu.hpp"

pros::adi::DigitalOut mogo('A');
pros::Motor intake(2);
pros::Motor intake1(8);
#define QUAD_TOP_PORT 'C'
#define QUAD_BOTTOM_PORT 'B'



pros::adi::Encoder encoder('C', 'D', true); // Replace 'A' and 'B' with the actual ports.
pros::Motor arm(14); 

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
  chassis.setPose(0, 0, 0);
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


   // Derivative gain

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
    pros::Motor intake1(8);
    pros::adi::DigitalOut mogoClamp('A'); // Mogo clamp mechanism control

  
   
   
    chassis.moveToPoint(0, -41.5,  2500,    {.forwards = false, .maxSpeed=80, }, true);
    while(chassis.isInMotion() && distance.get() > 32){ 
        pros::delay(10); // save cpu resources
    }
     // cancel the motion once the robot detects mogo is in the bot and clamped
     //pros::delay(100); // wait for mogo to clamp and settle
    mogoClamp.set_value(true);

    pros::delay(250);
    chassis.cancelMotion();


    //pros::delay(200); // wait for mogo to clamp and settle

    //chassis.moveToPoint(9.4, -32.36,  3000, {.maxSpeed=90}, true);
    intake.move_velocity(600);
    intake1.move_velocity(600);

    chassis.moveToPoint(18.7, -30.6, 1000, {.maxSpeed = 80}, false);

   
    
    
    chassis.moveToPose(23.75, -50.7, 180, 1500, {.maxSpeed = 100}, false);
    pros::delay(1000);
    chassis.turnToHeading(200, 1000);
    chassis.moveToPoint(26.75, -40, 1500, {.forwards = false}, false);

   
    
    //chassis.moveToPoint(0, 0, 4000); 


   
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



void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // Set the arm motor to hold its position when stopped
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    // Define target angles for the arm
    const int ARM_ANGLE_ONE = 120;  // Replace with actual encoder value for position 1
    const double ARM_ANGLE_TWO = 15.5; // Replace with actual encoder value for position 2

    // PID constants for arm control
    const double ARM_KP = 5;       // Proportional gain
    const double ARM_KI = 0.0;       // Integral gain
    const double ARM_KD = 5;       // Derivative gain

    // Variables for arm control
    int targetAngle = 0;             // Target position for the arm
    int lastError = 0;               // Previous error for PID calculation
    double integral = 0;             // Integral term for PID
    static bool armTargetState = false; // Arm toggle state (starts at 0)

    // Arm lock state
    bool armLockedAtZero = true;     // Tracks if arm is locked at angle 0

    // Variables for mogo clamp control
    bool currentButtonState = false; // Tracks the current state of the L1 button
    bool lastButtonState = false;    // Tracks the previous state of the L1 button
    bool manualOverride = false;     // Manual override state for mogo clamp
    bool mogoState = false;          // Current clamp state (open/closed)
    bool autoClamped = false;        // Tracks if the auto-clamp is engaged
    int autoClampLastActivated = 0;  // Last activation time of auto-clamp

    // Auto-clamp settings
    const int AUTO_CLAMP_COOLDOWN = 5000; // Cooldown period for auto-clamp (ms)
    const double DISTANCE_THRESHOLD = 30; // Distance threshold for auto-clamp (mm)

    // Main control loop
    while (true) {
        //////////////////////////////// CHASSIS CONTROL ////////////////////////////////
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        //////////////////////////////// ARM CONTROL (TOGGLE + PID) ////////////////////////////////
        static bool lastL2State = false; // Tracks the previous state of the L2 button
        bool currentL2State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // Reset arm lock state and toggle target when L2 is pressed
        if (currentL2State && !lastL2State) {
            armTargetState = !armTargetState;  // Toggle arm state
            armLockedAtZero = false;          // Unlock arm from angle 0
        }
        lastL2State = currentL2State;         // Update the last state

        // Set target angle based on toggle state or lock state
        if (armLockedAtZero) {
            targetAngle = 0;                  // Lock arm at angle 0
        } else {
            targetAngle = armTargetState ? ARM_ANGLE_TWO : ARM_ANGLE_ONE;
        }

        // Check if button A is pressed to reset and lock arm at angle 0
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            targetAngle = 0;                  // Set target to 0
            armLockedAtZero = true;           // Lock arm at angle 0
        }

        // PID control for precise arm movement
        int currentAngle = encoder.get_value(); // Get the current arm position
        int error = targetAngle - currentAngle; // Calculate error
        integral += error;                      // Accumulate integral
        double derivative = error - lastError;  // Calculate derivative
        double output = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;
        arm.move_velocity(output);              // Move arm using PID output
        lastError = error;                      // Save error for next iteration

        // Print arm state to the LCD screen for debugging
        pros::lcd::print(6, "Target: %d, Current: %d", targetAngle, currentAngle);

        //////////////////////////////// INTAKE CONTROL ////////////////////////////////
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(600);         // Intake forward
            intake1.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(-600);        // Intake reverse
            intake1.move_velocity(-600);
        } else {
            intake.move_velocity(0);           // Stop intake
            intake1.move_velocity(0);
        }

        //////////////////////////////// MOGO CLAMP CONTROL ////////////////////////////////
        currentButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        // Manual toggle of mogo clamp
        if (currentButtonState && !lastButtonState) {
            manualOverride = true;             // Enable manual override
            mogoState = !mogoState;            // Toggle clamp state
            mogo.set_value(mogoState);         // Apply the toggle state

            if (!mogoState) {
                autoClamped = false;           // Reset auto-clamp flag if manually unclamped
                manualOverride = false;
            }
        }

        // Auto-clamp logic (if manual override is not engaged)
        int currentTime = pros::millis();
        if (!manualOverride && (currentTime - autoClampLastActivated >= AUTO_CLAMP_COOLDOWN)) {
            double distanceValue = distance.get(); // Get distance sensor value

            // Trigger auto-clamping if object is detected within threshold
            if (distanceValue < DISTANCE_THRESHOLD && !autoClamped) {
                mogoState = true;             // Close clamp
                mogo.set_value(mogoState);    // Activate pneumatic clamp
                autoClamped = true;           // Mark auto-clamp as active
                controller.rumble("--");      // Haptic feedback
                autoClampLastActivated = currentTime; // Record activation time
            }
        }
        lastButtonState = currentButtonState; // Update last button state

        //////////////////////////////// LOOP DELAY ////////////////////////////////
        pros::delay(25); // Delay to prevent overloading cpu resources
}

}






