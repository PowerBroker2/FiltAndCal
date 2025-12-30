#pragma once
#include "Arduino.h"

/**
 * @enum FilterType
 * @brief Type of one-pole filter.
 */
enum class FilterType
{
    LowPass,  /**< Low-pass filter output */
    BandPass, /**< Band-pass filter output */
    HighPass  /**< High-pass filter output */
};

/**
 * @enum FilterPoleNum
 * @brief Number of poles to use for filtering.
 *
 * Determines whether a first-order (one-pole) or second-order (two-pole)
 * band-pass filter is used internally.
 */
enum class FilterPoleNum
{
    One, /**< Use a first-order (one-pole) band-pass filter */
    Two  /**< Use a second-order (two-pole) band-pass filter */
};