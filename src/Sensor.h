#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

/**
 * @class ScalarFiltAndCal
 * @brief Scalar signal calibration and band-pass filtering pipeline.
 *
 * This class implements a standard sensor-processing pipeline:
 *   1. Raw input capture
 *   2. Bias removal
 *   3. Scaling into physical units
 *   4. Band-pass filtering (1-pole or 2-pole selectable)
 */
class ScalarFiltAndCal
{
public:
    /**
     * @brief Construct a scalar calibration and filtering pipeline.
     *
     * @param lowCutHz  Lower cutoff frequency of the band-pass filter (HPF), Hz
     * @param highCutHz Upper cutoff frequency of the band-pass filter (LPF), Hz
     * @param scale     Multiplicative scale factor applied after bias removal
     * @param bias      Additive bias removed from raw input
     * @param numPoles  Number of filter poles (one-pole or two-pole)
     * @param min_dt    Minimum allowed sample interval (seconds)
     * @param max_dt    Maximum allowed sample interval (seconds)
     */
    explicit ScalarFiltAndCal(float lowCutHz,
                              float highCutHz,
                              float scale,
                              float bias,
                              FilterPoleNum numPoles = FilterPoleNum::Two,
                              float min_dt           = 1e-6f,
                              float max_dt           = 0.1f)
        : scale(scale),
          bias(bias),
          onePoleFilt(lowCutHz,
                      highCutHz,
                      min_dt,
                      max_dt),
          twoPoleFilt(lowCutHz,
                      highCutHz,
                      min_dt,
                      max_dt),
          numPoles(numPoles)
    {
        reset();
    }

    /**
     * @brief Set calibration parameters.
     *
     * @param scale New scale factor
     * @param bias  New bias offset
     */
    void setCal(float scale, float bias)
    {
        this->scale = scale;
        this->bias  = bias;
    }

    /**
     * @brief Set band-pass cutoff frequencies.
     *
     * @param lowCutHz  Lower cutoff frequency (HPF), Hz
     * @param highCutHz Upper cutoff frequency (LPF), Hz
     */
    void setCutoffs(float lowCutHz, float highCutHz)
    {
        if (numPoles == FilterPoleNum::One)
            onePoleFilt.setCutoffs(lowCutHz, highCutHz);
        else
            twoPoleFilt.setCutoffs(lowCutHz, highCutHz);
    }

    /**
     * @brief Process a new input sample.
     *
     * The processing order is:
     *   raw → bias removal → scaling → band-pass filtering
     *
     * @param input     Raw input sample
     * @param nowMicros Timestamp in microseconds
     * @return Filtered output value
     */
    float process(float input, uint32_t nowMicros)
    {
        rawOut      = input;
        unbiasedOut = rawOut - bias;
        scaledOut   = scale * unbiasedOut;

        if (numPoles == FilterPoleNum::One)
            filteredOut = onePoleFilt.process(scaledOut, nowMicros);
        else
            filteredOut = twoPoleFilt.process(scaledOut, nowMicros);
        
        return filteredOut;
    }

    /**
     * @brief Reset internal filter state.
     *
     * Does not modify calibration parameters.
     */
    void reset()
    {
        if (numPoles == FilterPoleNum::One)
            onePoleFilt.reset();
        else
            twoPoleFilt.reset();
    }

    /**
     * @brief Get the most recent raw input value.
     * @return Raw (unprocessed) input
     */
    float getRawOut() { return rawOut; }

    /**
     * @brief Get the most recent bias-corrected value.
     * @return Bias-removed signal
     */
    float getUnbiasedOut() { return unbiasedOut; }

    /**
     * @brief Get the most recent scaled value.
     * @return Scaled signal (physical units)
     */
    float getScaledOut() { return scaledOut; }

    /**
     * @brief Get the most recent filtered output.
     * @return Band-pass filtered signal
     */
    float getFilteredOut() { return filteredOut; }

private:
    float scale; /**< Scale factor applied after bias removal */
    float bias;  /**< Bias offset subtracted from raw input */

    float rawOut;      /**< Last raw input value */
    float unbiasedOut; /**< Last bias-corrected value */
    float scaledOut;   /**< Last scaled value */
    float filteredOut; /**< Last filtered output value */

    OnePoleBandPass onePoleFilt; /**< First-order band-pass filter */
    TwoPoleBandPass twoPoleFilt; /**< Second-order band-pass filter */

    FilterPoleNum numPoles; /**< Selected filter pole count */
};
