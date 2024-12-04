#pragma once
#include "main.h"

// PID constants for arm control
const double ARM_KP = 8.0;   // Increased proportional gain for better position holding
const double ARM_KI = 0.001; // Small integral gain to eliminate steady-state error
const double ARM_KD = 30.0;  // Increased derivative gain for better damping

// Function declarations
void setArmPosition(int targetDegrees);
void startArmTask();
void stopArmTask();
