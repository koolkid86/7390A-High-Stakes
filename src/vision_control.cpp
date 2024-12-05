#include "vision_control.hpp"
#include "arm_control.hpp"
#include "main.h"
#include "pros/rtos.hpp"

// Define vision sensor port
#define VISION_PORT 15

// Create vision sensor signatures
pros::vision_signature_s_t REDBOX = pros::Vision::signature_from_utility(3, 6403, 10109, 8256, -1377, -381, -880, 2.400, 0);
pros::vision_signature_s_t BLUEBOX = pros::Vision::signature_from_utility(1, -4505, -3449, -3978, 3969, 6799, 5384, 2.500, 0);

// MOTORS

// Global variables
static bool g_visionTaskRunning = false;
static pros::Task* g_visionTask = nullptr;
static pros::Vision* vision_sensor = nullptr;

// Constants for control
const int DETECTION_THRESHOLD = 20;     // Reduced size threshold for quicker detection
const int INTAKE_STOP_TIME = 500;       // Time to stop intake (in ms)
const int ARM_LIFT_TIME = 300;          // Time to keep arm lifted (in ms)
const int ARM_LIFT_ANGLE = 90;          // Increased angle for more aggressive lift
const int ARM_REST_ANGLE = 1.5;         // Rest position for arm
const int COOLDOWN_TIME = 1000;         // Time before allowing next detection
const int32_t RED_LED_COLOR = 0xFF0000; // Red color for LED
const int MIN_OBJECT_WIDTH = 20;        // Minimum width for object detection
const int VISION_CENTER_X = 160;        // Center x-coordinate of vision sensor
const int CENTER_TOLERANCE = 20;        // Tolerance for object centering

// Global flags for control
volatile bool g_stopIntake = false;  // Made volatile for cross-task access
uint32_t g_lastDetectionTime = 0;

void visionControlTask(void*) {
    // Initialize vision sensor
    vision_sensor = new pros::Vision(VISION_PORT);
    vision_sensor->clear_led();

    // Set signatures
    vision_sensor->set_signature(3, &REDBOX);
    vision_sensor->set_signature(1, &BLUEBOX);
    
    while (g_visionTaskRunning) {
        uint32_t currentTime = pros::millis();
        
        // Only check for rings if we're not in cooldown
        if (currentTime - g_lastDetectionTime > COOLDOWN_TIME) {
            // Get the largest red object (signature 3)
            pros::vision_object_s_t redObject = vision_sensor->get_by_sig(0, REDBOX.id);
            
            // Print basic object info
            pros::lcd::print(3, "Obj: %d,%d w:%d", 
                redObject.x_middle_coord, 
                redObject.y_middle_coord,
                redObject.width);
            
            // Print detection criteria
            pros::lcd::print(4, "Sig:%d Min:%d Ctr:%d", 
                redObject.signature == REDBOX.id,
                redObject.width > MIN_OBJECT_WIDTH,
                abs(redObject.x_middle_coord - VISION_CENTER_X) < CENTER_TOLERANCE);

            // Check if a red ring is detected and centered
            if (redObject.signature == REDBOX.id && 
                redObject.width > MIN_OBJECT_WIDTH &&
                abs(redObject.x_middle_coord - VISION_CENTER_X) < CENTER_TOLERANCE) {
                
                vision_sensor->set_led(RED_LED_COLOR);
                pros::lcd::print(5, "DETECTED - Stop:%d", g_stopIntake);
                
                if (!g_stopIntake) {
                    g_stopIntake = true;
                    
                    // Debug state transitions
                    pros::lcd::print(6, "ARM UP");
                    setArmPosition(ARM_LIFT_ANGLE);
                    pros::delay(ARM_LIFT_TIME);
                    
                    pros::lcd::print(6, "ARM DOWN");
                    setArmPosition(ARM_REST_ANGLE);
                    pros::delay(INTAKE_STOP_TIME - ARM_LIFT_TIME);
                    
                    g_stopIntake = false;
                    vision_sensor->clear_led();
                    pros::lcd::print(6, "COMPLETE");
                }
            } else {
                vision_sensor->clear_led();
                if (g_stopIntake) {
                    g_stopIntake = false;
                    pros::lcd::print(5, "NO DETECTION");
                }
            }
        }
        
        // Very small delay to prevent CPU hogging but maintain responsiveness
        pros::delay(20);
    }

    // Cleanup
    if (vision_sensor != nullptr) {
        delete vision_sensor;
        vision_sensor = nullptr;
    }
}

void startVisionTask() {
    if (!g_visionTaskRunning) {
        g_visionTaskRunning = true;
        g_visionTask = new pros::Task(visionControlTask);
    }
}

void stopVisionTask() {
    if (g_visionTaskRunning) {
        g_visionTaskRunning = false;
        if (g_visionTask != nullptr) {
            delete g_visionTask;
            g_visionTask = nullptr;
        }
    }
}

bool shouldStopIntake() {
    return g_stopIntake;
}
