#include "arm_control.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include <cmath>

// Global variables for arm control
int g_targetAngle = 0;
static bool g_armTaskRunning = false;
static pros::Task* g_armTask = nullptr;

// Function to set the target position for the arm
void setArmPosition(int targetDegrees) {
    g_targetAngle = targetDegrees;
}

// Slew rate control function
double slewRate(double current, double target, double rate) {
    double change = target - current;
    if (std::abs(change) > rate) {
        return current + (change > 0 ? rate : -rate);
    }
    return target;
}

// The main arm control task
void armControlTask(void*) {
    // Variables for PID control
    int lastError = 0;
    double integral = 0;
    double lastOutput = 0;
    const double INTEGRAL_LIMIT = 2000.0;  // Increased for smoother integration
    const int ERROR_THRESHOLD = 2;         // Tighter dead zone for more precision
    const double MAX_OUTPUT = 600.0;       // Maximum output
    const int INTEGRAL_THRESHOLD = 20;     // Increased integration window
    const double SLEW_RATE = 15.0;        // Maximum change in output per iteration
    const int MANUAL_SPEED = 127;         // Speed for manual control
    
    // Set the arm motor to hold its position when stopped
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (g_armTaskRunning) {
        // Check for manual control input
        bool buttonUp = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X);
        bool buttonDown = pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_B);

        if (buttonUp || buttonDown) {
            // Manual control mode
            int speed = buttonUp ? MANUAL_SPEED : (buttonDown ? -MANUAL_SPEED : 0);
            arm.move_velocity(speed);
            
            // Update target angle to current position to prevent jumping when switching back to PID
            g_targetAngle = encoder.get_value();
            
            // Reset PID variables
            integral = 0;
            lastError = 0;
            lastOutput = 0;
        } else {
            // PID control mode
            int currentAngle = encoder.get_value();
            int error = g_targetAngle - currentAngle;
            
            // Smooth integral accumulation
            if (abs(error) < INTEGRAL_THRESHOLD) {
                integral = integral * 0.95 + error; // Decay factor on integral
            } else {
                integral *= 0.5; // Rapidly decay integral when error is large
            }
            
            // Limit integral to prevent windup
            if (integral > INTEGRAL_LIMIT) {
                integral = INTEGRAL_LIMIT;
            } else if (integral < -INTEGRAL_LIMIT) {
                integral = -INTEGRAL_LIMIT;
            }
            
            // Calculate derivative with additional smoothing
            double derivative = (error - lastError) * 0.7; // Add derivative smoothing
            
            // Calculate PID output with improved scaling
            double output = (ARM_KP * error) + (ARM_KI * integral) + (ARM_KD * derivative);
            
            // Apply smooth acceleration curve
            double scale = std::min(1.0, std::abs(error) / 50.0); // Gradual scaling based on error
            output *= scale;
            
            // Apply slew rate limiting for smooth acceleration/deceleration
            output = slewRate(lastOutput, output, SLEW_RATE);
            
            // Limit maximum output
            if (output > MAX_OUTPUT) {
                output = MAX_OUTPUT;
            } else if (output < -MAX_OUTPUT) {
                output = -MAX_OUTPUT;
            }
            
            // Dead zone with smooth transition
            if (abs(error) < ERROR_THRESHOLD) {
                output *= std::max(0.0, (abs(error) - 1.0) / 1.0);
            }
            
            // Move arm using PID output
            arm.move_velocity(output);
            
            // Save values for next iteration
            lastError = error;
            lastOutput = output;
        }

        // Print debug info to LCD
        pros::lcd::print(6, "T:%d C:%d Mode:%s", 
            g_targetAngle, encoder.get_value(), 
            (buttonUp || buttonDown) ? "Manual" : "PID");

        // Delay to prevent CPU hogging
        pros::delay(10);  // Keep fast update rate for smooth control
    }
}

// Start the arm control task
void startArmTask() {
    if (!g_armTaskRunning) {
        g_armTaskRunning = true;
        g_armTask = new pros::Task(armControlTask);
    }
}

// Stop the arm control task
void stopArmTask() {
    if (g_armTaskRunning) {
        g_armTaskRunning = false;
        if (g_armTask != nullptr) {
            delete g_armTask;
            g_armTask = nullptr;
        }
    }
}
