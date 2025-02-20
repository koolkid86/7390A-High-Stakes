#include "colorsort.hpp"
#include "globals.hpp"  

#include "constants.hpp" // Now you have access to optical, intake1, etc.

// Rest of your colorsort.cpp code...
// Contains definitions for optical, intake1, etc.

// Static variables to manage the task and mode
static pros::Task* colorSortTaskHandle = nullptr;
static ColorSortMode currentMode = ColorSortMode::NONE;
static bool shouldStop = false;



// The task function that continuously checks the sensor and controls the intake
static void colorSortTaskFn(void* param) {
   
   
    while (!shouldStop) {
        if (currentMode != ColorSortMode::NONE) {
            if (currentMode == ColorSortMode::RED) {
                // Red sorting logic: eject if red rings detected
                if ((optical.get_hue() <= 80 && optical.get_hue() >= 0) &&
                    optical.get_proximity() <= 100) {
                   
                    intake1.move_voltage(0);  // Stop intake
                    pros::delay(1000);        // Allow time for ring ejection
                } else {
                    intake1.move_velocity(200);
                }
            } else if (currentMode == ColorSortMode::BLUE) {
                // Blue sorting logic: eject if blue rings detected
                if ((optical.get_hue() <= 250 && optical.get_hue() >= 180) &&
                    optical.get_proximity() <= 100) {
                    intake1.move_voltage(0);
                    pros::delay(1000);
                } else {
                    intake1.move_velocity(200);
                }
            }
        } else {
            // When sorting is disabled, run intake normally
            intake1.move_velocity(200);
        }
        pros::delay(20);  // Small delay to prevent hogging the CPU
    }
}

void startColorSortTask(ColorSortMode mode) {
    currentMode = mode;
    shouldStop = false;
    if (colorSortTaskHandle == nullptr) {
        colorSortTaskHandle = new pros::Task(colorSortTaskFn, nullptr, "Color Sort Task");
    }
}

void stopColorSortTask() {
    shouldStop = true;
    if (colorSortTaskHandle != nullptr) {
        // Optionally, allow a short delay for the task loop to exit
        pros::delay(50);
        delete colorSortTaskHandle;
        colorSortTaskHandle = nullptr;
    }
}
