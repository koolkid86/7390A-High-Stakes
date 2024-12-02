#include "main.h"
#include "auton.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <string>

#define ts std::to_string

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  encoder.reset();

  pros::lcd::set_text(0, "Auton Selected = " + autonNames[autonSelect]);

  pros::lcd::register_btn0_cb(previousAuton);
  pros::lcd::register_btn2_cb(nextAuton);

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

  // Define target angles for the arm
  const int ARM_ANGLE_ONE =
      120; // Replace with actual encoder value for position 1
  const double ARM_ANGLE_TWO =
      15.5; // Replace with actual encoder value for position 2

  // PID constants for arm control
  const double ARM_KP = 5;   // Proportional gain
  const double ARM_KI = 0.0; // Integral gain
  const double ARM_KD = 5;   // Derivative gain

  // Variables for arm control
  int targetAngle = 0;                // Target position for the arm
  int lastError = 0;                  // Previous error for PID calculation
  double integral = 0;                // Integral term for PID
  static bool armTargetState = false; // Arm toggle state (starts at 0)

  // Arm lock state
  bool armLockedAtZero = true; // Tracks if arm is locked at angle 0

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

    ///////////////// ARM CONTROL (TOGGLE + PID) /////////////////////////////
    static bool lastL2State =
        false; // Tracks the previous state of the L2 button
    bool currentL2State = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    // Reset arm lock state and toggle target when L2 is pressed
    if (currentL2State && !lastL2State) {
      armTargetState = !armTargetState; // Toggle arm state
      armLockedAtZero = false;          // Unlock arm from angle 0
    }
    lastL2State = currentL2State; // Update the last state

    // Set target angle based on toggle state or lock state
    if (armLockedAtZero) {
      targetAngle = 0; // Lock arm at angle 0
    } else {
      targetAngle = armTargetState ? ARM_ANGLE_TWO : ARM_ANGLE_ONE;
    }

    // Check if button A is pressed to reset and lock arm at angle 0
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      targetAngle = 0;        // Set target to 0
      armLockedAtZero = true; // Lock arm at angle 0
    }

    // PID control for precise arm movement
    int currentAngle = encoder.get_value(); // Get the current arm position
    int error = targetAngle - currentAngle; // Calculate error
    integral += error;                      // Accumulate integral
    double derivative = error - lastError;  // Calculate derivative
    double output = ARM_KP * error + ARM_KI * integral + ARM_KD * derivative;
    arm.move_velocity(output); // Move arm using PID output
    lastError = error;         // Save error for next iteration

    // Print arm state to the LCD screen for debugging
    pros::lcd::print(6, "Target: %d, Current: %d", targetAngle, currentAngle);

    //////////////////////// INTAKE CONTROL //////////////////////////////
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake1.move_velocity(600); // Intake forward
      intake2.move_velocity(600);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
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

    //////////////////////////////// LOOP DELAY ////////////////////////////////
    pros::delay(25); // Delay to prevent overloading cpu resources
  }
}
