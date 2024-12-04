#include "arm_control.hpp"
#include "main.h"
#include "pros/rtos.hpp"

// Global variables for arm control
static int g_targetAngle = 0;
static bool g_armTaskRunning = false;
static pros::Task* g_armTask = nullptr;

// Function to set the target position for the arm
void setArmPosition(int targetDegrees) {
    g_targetAngle = targetDegrees;
}

// The main arm control task
void armControlTask(void*) {
    // Variables for PID control
    int lastError = 0;
    double integral = 0;
    const double INTEGRAL_LIMIT = 1000.0;  // Increased integral limit
    const int ERROR_THRESHOLD = 3;         // Tighter dead zone
    const double MAX_OUTPUT = 600.0;       // Increased max output for more power
    const int INTEGRAL_THRESHOLD = 15;     // Only integrate when error is small
    
    // Set the arm motor to hold its position when stopped
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (g_armTaskRunning) {
        // PID control for precise arm movement
        int currentAngle = encoder.get_value();
        int error = g_targetAngle - currentAngle;
        
        // Only accumulate integral when close to target
        if (abs(error) < INTEGRAL_THRESHOLD) {
            integral += error;
        } else {
            integral = 0; // Reset integral when error is large
        }
        
        // Limit integral to prevent windup
        if (integral > INTEGRAL_LIMIT) {
            integral = INTEGRAL_LIMIT;
        } else if (integral < -INTEGRAL_LIMIT) {
            integral = -INTEGRAL_LIMIT;
        }
        
        // Calculate derivative (rate of change of error)
        double derivative = error - lastError;
        
        // Calculate PID output with improved scaling
        double output = (ARM_KP * error) + (ARM_KI * integral) + (ARM_KD * derivative);
        
        // Apply exponential scaling for smoother control
        output = (output > 0) ? 
                std::min(output * output / 100.0, MAX_OUTPUT) : 
                std::max(-output * output / 100.0, -MAX_OUTPUT);
        
        // Dead zone - if error is very small, maintain holding power
        if (abs(error) < ERROR_THRESHOLD) {
            output = 0;  // Let the motor brake mode handle holding
        }
        
        // Move arm using PID output
        arm.move_velocity(output);
        
        // Save error for next iteration
        lastError = error;

        // Print debug info to LCD
        pros::lcd::print(6, "T:%d C:%d E:%d I:%.1f O:%.1f", 
            g_targetAngle, currentAngle, error, integral, output);

        // Delay to prevent CPU hogging
        pros::delay(10);  // Faster update rate for more responsive control
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
