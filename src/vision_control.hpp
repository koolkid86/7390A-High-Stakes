#pragma once
#include "main.h"

// Vision sensor signatures
extern pros::vision_signature_s_t REDBOX;
extern pros::vision_signature_s_t BLUEBOX;

// Function declarations
void startVisionTask();
void stopVisionTask();
bool shouldStopIntake();
