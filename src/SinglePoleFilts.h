#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Dense>




using namespace Eigen;




/**
 * @class OnePoleFilt3f
 * @brief Single-pole filter with variable sample interval support for Vector3f inputs.
 *
 * Supports low-pass (LPF) and high-pass (HPF) filtering independently on each component.
 * Handles variable sample intervals and first-sample initialization with rollover-safe timestamps.
 */
class OnePoleFilt3f
{
public:
    /**
     * @brief Construct a 3-component one-pole filter.
     * @param cutoffHz Filter cutoff frequency in Hz. Must be > 0 for meaningful filtering.
     * @param filterType Type of filter (LowPass or HighPass). Defaults to LowPass.
     * @param filterMin_dt Minimum allowed sample interval in seconds. Defaults to 1e-6 s.
     * @param filterMax_dt Maximum allowed sample interval in seconds. Defaults to 0.1 s.
     */
    explicit OnePoleFilt3f(float      cutoffHz,
                           FilterType filterType   = FilterType::LowPass,
                           float      filterMin_dt = 1e-6f,
                           float      filterMax_dt = 0.1f)
        : tau(0.0f),
          y(Vector3f::Zero()),
          lastInput(Vector3f::Zero()),
          type(filterType),
          lastMicros(0),
          initialized(false),
          min_dt(filterMin_dt),
          max_dt(filterMax_dt)
    {
        setCutoff(cutoffHz);
    }




    /**
     * @brief Set the filter type (LowPass or HighPass).
     * @param newType New filter type.
     *
     * Switching filter type resets the internal state to avoid large spikes:
     * - LowPass: output starts at current value
     * - HighPass: output starts at 0
     */
    void setType(FilterType newType)
    {
        if (newType != type)
        {
            type = newType;
            if (type == FilterType::LowPass)
            {
                reset(y);
            }
            else
            {
                reset(Vector3f::Zero());
            }
        }
    }




    /**
     * @brief Set the filter cutoff frequency.
     * @param cutoffHz Cutoff frequency in Hz. Must be > 0 for normal operation.
     *
     * If the frequency is <= 0, the time constant is set very high (effectively no filtering).
     */
    void setCutoff(float cutoffHz)
    {
        if (cutoffHz <= 0.0f)
            tau = 1e9f; // effectively infinite
        else
            tau = 1.0f / (2.0f * F_PI * cutoffHz);
    }




    /**
     * @brief Set the allowed range of sample intervals.
     * @param filterMin_dt Minimum sample interval in seconds.
     * @param filterMax_dt Maximum sample interval in seconds.
     * @return True if the range is valid and applied, false otherwise.
     *
     * Ensures min_dt > 0, max_dt > 0, and min_dt < max_dt.
     */
    bool setDtRange(float filterMin_dt, float filterMax_dt)
    {
        if ((filterMin_dt > 0.0f) && (filterMin_dt < filterMax_dt) && (filterMax_dt > 0.0f))
        {
            min_dt = filterMin_dt;
            max_dt = filterMax_dt;
            return true;
        }
        return false;
    }




    /**
     * @brief Process a new Vector3f input sample and compute filtered output.
     * @param input Current Vector3f input sample.
     * @param nowMicros Current timestamp in microseconds.
     * @return Filtered Vector3f output sample.
     *
     * Automatically calculates delta time between samples, clamps it to [min_dt, max_dt],
     * and applies LPF or HPF filtering independently on each component.
     * Handles first-sample initialization and timer rollover safely.
     */
    Vector3f process(const Vector3f &input, uint32_t nowMicros)
    {
        // First call: initialize timebase
        if (!initialized)
        {
            lastInput = input;
            if (type == FilterType::LowPass)
                y = input;
            else
                y = Vector3f::Zero(); // HPF output starts at 0

            lastMicros = nowMicros;
            initialized = true;
            return y;
        }

        // Rollover-safe delta
        uint32_t dt_us = nowMicros - lastMicros;
        lastMicros = nowMicros;

        // Convert to seconds and clamp
        float dt = dt_us * 1e-6f;
        if (dt < min_dt) dt = min_dt;
        if (dt > max_dt) dt = max_dt;

        if (type == FilterType::LowPass)
        {
            Vector3f alpha = Vector3f::Constant(dt / (tau + dt));
            y += alpha.cwiseProduct(input - y);
        }
        else // High-pass
        {
            Vector3f alpha = Vector3f::Constant(tau / (tau + dt));
            Vector3f diff  = y + input - lastInput;
            y = alpha.cwiseProduct(diff);
        }

        lastInput = input;
        return y;
    }




    /**
     * @brief Reset the filter internal state.
     * @param value Optional initial state vector. Defaults to zero vector.
     *
     * For LPF: output starts at `value`.
     * For HPF: output starts at 0, last input set to `value`.
     */
    void reset(const Vector3f &value = Vector3f::Zero())
    {
        if (type == FilterType::LowPass)
            y = value;
        else
            y = Vector3f::Zero();

        lastInput   = value;
        initialized = false;
    }




private:
    float tau;                   /**< Filter time constant in seconds */
    Vector3f y;                  /**< Filter output/internal state */
    Vector3f lastInput;          /**< Last input value (used for HPF) */
    FilterType type;             /**< Filter type */

    uint32_t lastMicros;         /**< Last timestamp in microseconds */
    bool initialized;            /**< True if first sample has been processed */
    float min_dt;                /**< Minimum allowed sample interval (s) */
    float max_dt;                /**< Maximum allowed sample interval (s) */
};




/**
 * @class OnePoleFilt
 * @brief Single-pole filter with variable sample interval support.
 *
 * Supports low-pass (LPF) and high-pass (HPF) filtering with variable sample intervals.
 * Automatically handles first-sample initialization and timer rollover of 32-bit microsecond counters.
 */
class OnePoleFilt
{
public:
    /**
     * @brief Construct a one-pole filter with given cutoff frequency and type.
     * @param cutoffHz Filter cutoff frequency in Hz. Must be > 0 for meaningful filtering.
     * @param filterType Type of filter (LowPass or HighPass). Defaults to LowPass.
     * @param filterMin_dt Minimum allowed sample interval in seconds. Defaults to 1e-6 s.
     * @param filterMax_dt Maximum allowed sample interval in seconds. Defaults to 0.1 s.
     */
    explicit OnePoleFilt(float cutoffHz,
                         FilterType filterType = FilterType::LowPass,
                         float filterMin_dt = 1e-6f,
                         float filterMax_dt = 0.1f)
        : tau(0.0f),
          y(0.0f),
          lastInput(0.0f),
          type(filterType),
          lastMicros(0),
          initialized(false),
          min_dt(filterMin_dt),
          max_dt(filterMax_dt)
    {
        setCutoff(cutoffHz);
    }

    /**
     * @brief Set the filter type (LowPass or HighPass).
     * @param newType New filter type.
     *
     * Switching filter type resets the internal state to avoid large spikes:
     * - LowPass: output starts at current value
     * - HighPass: output starts at 0
     */
    void setType(FilterType newType)
    {
        if (newType != type)
        {
            type = newType;
            if (type == FilterType::LowPass)
            {
                reset(y);
            }
            else
            {
                reset(0.0f);
            }
        }
    }

    /**
     * @brief Set the filter cutoff frequency.
     * @param cutoffHz Cutoff frequency in Hz. Must be > 0 for normal operation.
     *
     * If the frequency is <= 0, the time constant is set very high (effectively no filtering).
     */
    void setCutoff(float cutoffHz)
    {
        if (cutoffHz <= 0.0f)
            tau = 1e9f; // effectively infinite
        else
            tau = 1.0f / (2.0f * F_PI * cutoffHz);
    }

    /**
     * @brief Set the allowed range of sample intervals.
     * @param filterMin_dt Minimum sample interval in seconds.
     * @param filterMax_dt Maximum sample interval in seconds.
     * @return True if the range is valid and applied, false otherwise.
     *
     * The function ensures min_dt > 0, max_dt > 0, and min_dt < max_dt.
     */
    bool setDtRange(float filterMin_dt, float filterMax_dt)
    {
        if ((filterMin_dt > 0.0f) && (filterMin_dt < filterMax_dt) && (filterMax_dt > 0.0f))
        {
            min_dt = filterMin_dt;
            max_dt = filterMax_dt;
            return true;
        }
        return false;
    }

    /**
     * @brief Process a new input sample and compute filtered output.
     * @param input Current input sample.
     * @param nowMicros Current timestamp in microseconds.
     * @return Filtered output sample.
     *
     * This function automatically calculates the delta time between samples,
     * clamps it to [min_dt, max_dt], and applies LPF or HPF filtering.
     * Handles first-sample initialization and timer rollover safely.
     */
    float process(float input, uint32_t nowMicros)
    {
        // First call: initialize timebase
        if (!initialized)
        {
            lastInput = input;
            if (type == FilterType::LowPass)
                y = input;
            else
                y = 0.0f; // HPF output starts at 0

            lastMicros = nowMicros;
            initialized = true;
            return y;
        }

        // Rollover-safe delta
        uint32_t dt_us = nowMicros - lastMicros;
        lastMicros = nowMicros;

        // Convert to seconds and clamp
        float dt = dt_us * 1e-6f;
        if (dt < min_dt) dt = min_dt;
        if (dt > max_dt) dt = max_dt;

        if (type == FilterType::LowPass)
        {
            float alpha = dt / (tau + dt);
            y += alpha * (input - y);
        }
        else // High-pass
        {
            float alpha = tau / (tau + dt);
            y = alpha * (y + input - lastInput);
        }

        lastInput = input;
        return y;
    }

    /**
     * @brief Reset the filter internal state.
     * @param value Optional initial state. Defaults to 0.0f.
     *
     * For LPF: output starts at `value`.
     * For HPF: output starts at 0, last input set to `value`.
     */
    void reset(float value = 0.0f)
    {
        if (type == FilterType::LowPass)
            y = value;
        else
            y = 0.0f;

        lastInput = value;
        initialized = false;
    }

private:
    float tau;           /**< Filter time constant in seconds */
    float y;             /**< Filter output/internal state */
    float lastInput;     /**< Last input value (used for HPF) */
    FilterType type;     /**< Filter type */

    uint32_t lastMicros; /**< Last timestamp in microseconds */
    bool initialized;    /**< True if first sample has been processed */
    float min_dt;        /**< Minimum allowed sample interval (s) */
    float max_dt;        /**< Maximum allowed sample interval (s) */
};