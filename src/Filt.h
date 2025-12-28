#pragma once
#include "Arduino.h"




/** @brief Float constant for pi */
constexpr float F_PI = 3.14159265358979323846f;




/**
 * @enum FilterType
 * @brief Type of one-pole filter.
 */
enum class FilterType
{
    LowPass,  /**< Low-pass filter output */
    HighPass  /**< High-pass filter output */
};




#include "SinglePoleFilts.h"
#include "DoublePoleFilts.h"