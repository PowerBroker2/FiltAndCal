#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

/**
 * @class TwoPoleFilt3f
 * @brief 2-pole Butterworth filter for Vector3f inputs.
 *
 * Each component (x, y, z) is filtered independently.
 * Coefficients are recomputed per sample using dt,
 * making it suitable for variable-rate sensor processing (e.g., magnetometers, accelerometers).
 */
class TwoPoleFilt3f
{
public:
    /**
     * @brief Construct a 2-pole Butterworth filter for Vector3f inputs.
     * @param cutoffHz Cutoff frequency in Hz
     * @param type Filter type (LowPass or HighPass)
     * @param min_dt Minimum allowed dt (seconds)
     * @param max_dt Maximum allowed dt (seconds)
     */
    explicit TwoPoleFilt3f(float      cutoffHz,
                           FilterType type   = FilterType::LowPass,
                           float      min_dt = 1e-6f,
                           float      max_dt = 0.1f)
        : cutoffHz(cutoffHz),
          type(type),
          lastMicros(0),
          initialized(false),
          min_dt(min_dt),
          max_dt(max_dt)
    {
        reset();
    }

    /**
     * @brief Set the filter cutoff frequency.
     * @param hz Cutoff frequency in Hz. Must be >0.
     */
    void setCutoff(float hz)
    {
        cutoffHz = (hz > 0.0f) ? hz : 0.001f;
    }

    /**
     * @brief Set the filter type.
     * @param newType Filter type (LowPass or HighPass)
     *
     * Changing the filter type resets the internal state.
     */
    void setType(FilterType newType)
    {
        if (newType != type)
        {
            type = newType;
            reset();
        }
    }

    /**
     * @brief Process a new Vector3f input sample.
     * @param input Vector3f input sample
     * @param nowMicros Current timestamp in microseconds
     * @return Filtered Vector3f output
     *
     * This function computes variable-dt 2-pole Butterworth filtering for each
     * component of the vector independently. Handles first-sample initialization
     * and clamps dt to [min_dt, max_dt].
     */
    Vector3f process(const Vector3f &input, uint32_t nowMicros)
    {
        if (!initialized)
        {
            x1 = x2 = input;
            y1 = y2 = Vector3f::Zero();

            lastMicros  = nowMicros;
            initialized = true;

            return Vector3f::Zero();
        }

        uint32_t dt_us = nowMicros - lastMicros;
        lastMicros     = nowMicros;

        float dt = dt_us * 1e-6f;

        if (dt < min_dt) dt = min_dt;
        if (dt > max_dt) dt = max_dt;

        computeCoefficients(dt);

        Vector3f y = b0.cwiseProduct(input) 
                     + b1.cwiseProduct(x1) 
                     + b2.cwiseProduct(x2)
                     - a1.cwiseProduct(y1) 
                     - a2.cwiseProduct(y2);

        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = y;

        return y;
    }

    /**
     * @brief Reset the filter internal state.
     *
     * Clears previous input/output history and marks the filter as uninitialized.
     */
    void reset()
    {
        x1 = x2 = Vector3f::Zero();
        y1 = y2 = Vector3f::Zero();

        initialized = false;
    }

private:
    float cutoffHz;        /**< Filter cutoff frequency in Hz */
    FilterType type;       /**< Filter type (LowPass or HighPass) */

    Vector3f x1, x2;      /**< Previous input vectors */
    Vector3f y1, y2;      /**< Previous output vectors */

    Vector3f b0, b1, b2;  /**< Feedforward coefficients */
    Vector3f a1, a2;      /**< Feedback coefficients */

    uint32_t lastMicros;  /**< Last timestamp in microseconds */
    bool initialized;     /**< True if first sample has been processed */
    float min_dt;         /**< Minimum allowed sample interval (s) */
    float max_dt;         /**< Maximum allowed sample interval (s) */

    /**
     * @brief Compute Butterworth filter coefficients for each component using dt.
     * @param dt Time interval in seconds since last sample
     *
     * Coefficients are computed independently for each component (x, y, z).
     * Uses standard 2-pole Butterworth design formulas. Handles both LPF and HPF.
     */
    void computeCoefficients(float dt)
    {
        float wc   = 2.0f * F_PI * cutoffHz;
        float k    = wc * dt * 0.5f;
        float k2   = k * k;
        float norm = 1.0f / (1.0f + sqrtf(2.0f) * k + k2);

        if (type == FilterType::LowPass)
        {
            float b0s = k2 * norm;

            b0 = Vector3f(b0s, b0s, b0s);
            b1 = 2.0f * b0;
            b2 = b0;
        }
        else // High-pass
        {
            float b0s = 1.0f * norm;

            b0 = Vector3f(b0s, b0s, b0s);
            b1 = Vector3f(-2.0f * b0s, -2.0f * b0s, -2.0f * b0s);
            b2 = b0;
        }

        float a1s = 2.0f * (k2 - 1.0f) * norm;
        float a2s = (1.0f - sqrtf(2.0f) * k + k2) * norm;

        a1 = Vector3f(a1s, a1s, a1s);
        a2 = Vector3f(a2s, a2s, a2s);
    }
};

/**
 * @class TwoPoleBandPass3f
 * @brief Fourth-order band-pass filter for Vector3f inputs using cascaded two-pole filters.
 *
 * The filter is implemented as:
 *     2-pole HPF (lowCutHz) -> 2-pole LPF (highCutHz)
 *
 * This design provides steeper roll-off than a one-pole band-pass while
 * maintaining numerical stability and variable sample interval support.
 */
class TwoPoleBandPass3f
{
public:
    /**
     * @brief Construct a two-pole band-pass filter.
     * @param lowCutHz  Lower cutoff frequency (HPF cutoff), Hz
     * @param highCutHz Upper cutoff frequency (LPF cutoff), Hz
     * @param min_dt    Minimum allowed sample interval (seconds)
     * @param max_dt    Maximum allowed sample interval (seconds)
     */
    explicit TwoPoleBandPass3f(float lowCutHz,
                               float highCutHz,
                               float min_dt = 1e-6f,
                               float max_dt = 0.1f)
        : hpf(1.0f, FilterType::HighPass, min_dt, max_dt),
          lpf(1.0f, FilterType::LowPass, min_dt, max_dt)
    {
        setCutoffs(lowCutHz, highCutHz);
    }

    /**
     * @brief Set band-pass cutoff frequencies.
     * @param lowCutHz  Lower cutoff frequency (HPF), Hz
     * @param highCutHz Upper cutoff frequency (LPF), Hz
     *
     * If lowCutHz > highCutHz, the values are automatically swapped.
     * Cutoff values <= 0 effectively bypass the corresponding stage.
     */
    void setCutoffs(float lowCutHz, float highCutHz)
    {
        // Ensure low <= high
        if (lowCutHz > highCutHz)
        {
            float tmp = lowCutHz;
            lowCutHz  = highCutHz;
            highCutHz = tmp;
        }

        hpf.setCutoff(lowCutHz);
        lpf.setCutoff(highCutHz);

        reset();
    }

    /**
     * @brief Process a new input sample.
     * @param input Input Vector3f sample
     * @param nowMicros Timestamp in microseconds
     * @return Band-pass filtered output as Vector3f
     */
    Vector3f process(const Vector3f &input, uint32_t nowMicros)
    {
        Vector3f hp = hpf.process(input, nowMicros);
        return lpf.process(hp, nowMicros);
    }

    /**
     * @brief Reset internal filter state.
     *
     * Resets both HPF and LPF stages to zero.
     */
    void reset()
    {
        hpf.reset();
        lpf.reset();
    }

private:
    TwoPoleFilt3f hpf; /**< Two-pole high-pass stage */
    TwoPoleFilt3f lpf; /**< Two-pole low-pass stage */
};

/**
 * @class TwoPoleFilt
 * @brief 2-pole Butterworth filter with variable sample interval support.
 *
 * Supports low-pass (LPF) and high-pass (HPF) modes. Coefficients are recomputed
 * per sample using the provided delta-time, making this suitable for variable-rate
 * sensor processing on Teensy or other microcontrollers.
 */
class TwoPoleFilt
{
public:
    /**
     * @brief Construct a 2-pole Butterworth filter.
     * @param cutoffHz Cutoff frequency in Hz.
     * @param type Filter type (LowPass or HighPass). Defaults to LowPass.
     * @param min_dt Minimum allowed sample interval (seconds). Defaults to 1e-6.
     * @param max_dt Maximum allowed sample interval (seconds). Defaults to 0.1.
     *
     * Initializes filter state to zero and sets the initial configuration.
     */
    explicit TwoPoleFilt(float      cutoffHz,
                         FilterType type   = FilterType::LowPass,
                         float      min_dt = 1e-6f,
                         float      max_dt = 0.1f)
        : cutoffHz(cutoffHz),
          type(type),
          lastMicros(0),
          initialized(false),
          min_dt(min_dt),
          max_dt(max_dt)
    {
        reset();
    }

    /**
     * @brief Set the filter cutoff frequency.
     * @param hz New cutoff frequency in Hz. Values <= 0 are clamped to 0.001 Hz.
     *
     * Adjusts the frequency used for coefficient calculation in the next process call.
     */
    void setCutoff(float hz)
    {
        cutoffHz = (hz > 0.0f) ? hz : 0.001f;
    }

    /**
     * @brief Set the filter type (LowPass or HighPass).
     * @param newType New filter type.
     *
     * Changing the type resets the internal filter state to avoid discontinuities.
     */
    void setType(FilterType newType)
    {
        if (newType != type)
        {
            type = newType;
            reset();
        }
    }

    /**
     * @brief Process a new input sample.
     * @param input Current input sample.
     * @param nowMicros Current timestamp in microseconds.
     * @return Filtered output sample.
     *
     * This function automatically:
     * - Computes the delta-time (dt) between samples.
     * - Clamps dt to the range [min_dt, max_dt].
     * - Computes 2-pole Butterworth coefficients based on dt.
     * - Applies the digital filter difference equation to produce output.
     */
    float process(float input, uint32_t nowMicros)
    {
        if (!initialized)
        {
            x1 = x2 = input;
            y1 = y2 = 0.0f;

            lastMicros  = nowMicros;
            initialized = true;

            return 0.0f;
        }

        uint32_t dt_us = nowMicros - lastMicros;
        lastMicros     = nowMicros;

        float dt = dt_us * 1e-6f;

        if (dt < min_dt) dt = min_dt;
        if (dt > max_dt) dt = max_dt;

        computeCoefficients(dt);

        float y = b0 * input + b1 * x1 + b2 * x2
                  - a1 * y1 - a2 * y2;

        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = y;

        return y;
    }

    /**
     * @brief Reset the internal filter state.
     *
     * Resets previous input and output samples to zero, and clears the initialization
     * flag. Should be called if the filter needs to restart cleanly.
     */
    void reset()
    {
        x1 = x2 = 0.0f;
        y1 = y2 = 0.0f;

        initialized = false;
    }

private:
    // Filter settings
    float cutoffHz;       /**< Cutoff frequency in Hz */
    FilterType type;      /**< Filter type (LowPass or HighPass) */

    // State variables
    float x1, x2;         /**< Previous two input samples */
    float y1, y2;         /**< Previous two output samples */

    // Filter coefficients
    float b0, b1, b2;     /**< Feedforward coefficients */
    float a1, a2;         /**< Feedback coefficients */

    uint32_t lastMicros;  /**< Timestamp of last sample (microseconds) */
    bool initialized;     /**< True if filter has been initialized */
    float min_dt;         /**< Minimum allowed sample interval (seconds) */
    float max_dt;         /**< Maximum allowed sample interval (seconds) */

    /**
     * @brief Compute 2-pole Butterworth filter coefficients for the given dt.
     * @param dt Sample interval in seconds.
     *
     * This function updates the feedforward (b0,b1,b2) and feedback (a1,a2) coefficients
     * for the current sample interval. Coefficients depend on:
     * - Filter type (LowPass or HighPass)
     * - Cutoff frequency
     * - Sample interval dt
     */
    void computeCoefficients(float dt)
    {
        float wc   = 2.0f * F_PI * cutoffHz;
        float k    = wc * dt * 0.5f;
        float k2   = k * k;
        float norm = 1.0f / (1.0f + sqrtf(2.0f) * k + k2);

        if (type == FilterType::LowPass)
        {
            b0 = k2 * norm;
            b1 = 2.0f * b0;
            b2 = b0;
        }
        else // HighPass
        {
            b0 = 1.0f * norm;
            b1 = -2.0f * b0;
            b2 = b0;
        }

        a1 = 2.0f * (k2 - 1.0f) * norm;
        a2 = (1.0f - sqrtf(2.0f) * k + k2) * norm;
    }
};

/**
 * @class TwoPoleBandPass
 * @brief Fourth-order band-pass filter using cascaded two-pole Butterworth filters.
 *
 * The filter is implemented as:
 *     2-pole HPF (lowCutHz) -> 2-pole LPF (highCutHz)
 *
 * This design provides steeper roll-off than a one-pole band-pass while
 * maintaining numerical stability and variable sample interval support.
 */
class TwoPoleBandPass
{
public:
    /**
     * @brief Construct a two-pole band-pass filter.
     * @param lowCutHz  Lower cutoff frequency (HPF cutoff), Hz
     * @param highCutHz Upper cutoff frequency (LPF cutoff), Hz
     * @param min_dt    Minimum allowed sample interval (seconds)
     * @param max_dt    Maximum allowed sample interval (seconds)
     */
    explicit TwoPoleBandPass(float lowCutHz,
                             float highCutHz,
                             float min_dt = 1e-6f,
                             float max_dt = 0.1f)
        : hpf(1.0f, FilterType::HighPass, min_dt, max_dt),
          lpf(1.0f, FilterType::LowPass,  min_dt, max_dt)
    {
        setCutoffs(lowCutHz, highCutHz);
    }

    /**
     * @brief Set band-pass cutoff frequencies.
     * @param lowCutHz  Lower cutoff frequency (HPF), Hz
     * @param highCutHz Upper cutoff frequency (LPF), Hz
     *
     * If lowCutHz > highCutHz, the values are automatically swapped.
     * Cutoff values <= 0 effectively bypass the corresponding stage.
     */
    void setCutoffs(float lowCutHz, float highCutHz)
    {
        /* Ensure valid ordering */
        if (lowCutHz > highCutHz)
        {
            float tmp = lowCutHz;
            lowCutHz  = highCutHz;
            highCutHz = tmp;
        }

        hpf.setCutoff(lowCutHz);
        lpf.setCutoff(highCutHz);

        reset();
    }

    /**
     * @brief Process a new input sample.
     * @param input Input sample
     * @param nowMicros Timestamp in microseconds
     * @return Band-pass filtered output
     */
    float process(float input, uint32_t nowMicros)
    {
        float hp = hpf.process(input, nowMicros);
        return lpf.process(hp, nowMicros);
    }

    /**
     * @brief Reset internal filter state.
     */
    void reset()
    {
        hpf.reset();
        lpf.reset();
    }

private:
    TwoPoleFilt hpf; /**< 2-pole high-pass stage */
    TwoPoleFilt lpf; /**< 2-pole low-pass stage */
};
