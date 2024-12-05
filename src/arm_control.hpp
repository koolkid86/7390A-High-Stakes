#pragma once
#include "main.h"

// PID constants for arm control
const double ARM_KP = 4.0;    // Reduced for smoother motion
const double ARM_KI = 0.0008; // Slightly reduced integral
const double ARM_KD = 40.0;   // Increased derivative for better damping

// Global variables
extern int g_targetAngle;

// Function declarations
void setArmPosition(int targetDegrees);
void startArmTask();
void stopArmTask();
