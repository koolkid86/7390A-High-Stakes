#ifndef COLORSORT_HPP
#define COLORSORT_HPP

#include "pros/rtos.hpp"

// Enum for selecting the sorting mode
enum class ColorSortMode {
    RED,
    BLUE,
    NONE
};

// Starts the colorsort task with the specified mode
void startColorSortTask(ColorSortMode mode);

// Stops the colorsort task if it is running
void stopColorSortTask();

#endif // COLORSORT_HPP
