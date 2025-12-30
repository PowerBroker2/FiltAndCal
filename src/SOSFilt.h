#pragma once
#include <Arduino.h>

/**
 * @brief Second-Order-Section (SOS) filter for float data.
 * 
 * Supports an arbitrary number of stages up to MAX_SOS_STAGES.
 * Each stage implements a standard biquad:
 * 
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */
#ifndef MAX_SOS_STAGES
#define MAX_SOS_STAGES 4
#endif

/**
 * @struct SOSCoefficients
 * @brief Coefficient container for a cascaded second-order IIR filter (SOS).
 *
 * Stores the coefficients for a cascade of second-order sections (biquads).
 * Each stage implements the transfer function:
 *
 * @f[
 * H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}
 *              {1 + a_1 z^{-1} + a_2 z^{-2}}
 * @f]
 *
 * The overall filter response is the product of all stages.
 *
 * @note
 * - The denominator coefficient a0 is assumed to be 1 for all stages.
 * - Coefficients are stored in **array-of-stages** format.
 * - The number of active stages is specified by @ref SOSCoefficients::sstages.
 */
struct SOSCoefficients
{
    /**
     * @brief Number of active second-order sections.
     *
     * Must be less than or equal to MAX_SOS_STAGES.
     * Any additional entries in the coefficient arrays are ignored.
     */
    int stages;

    /**
     * @brief Numerator coefficient b0 for each SOS stage.
     *
     * Corresponds to the zero-order term of the numerator polynomial.
     */
    float b0[MAX_SOS_STAGES];

    /**
     * @brief Numerator coefficient b1 for each SOS stage.
     *
     * Corresponds to the first-order term of the numerator polynomial.
     */
    float b1[MAX_SOS_STAGES];

    /**
     * @brief Numerator coefficient b2 for each SOS stage.
     *
     * Corresponds to the second-order term of the numerator polynomial.
     */
    float b2[MAX_SOS_STAGES];

    /**
     * @brief Denominator coefficient a1 for each SOS stage.
     *
     * Corresponds to the first-order feedback term.
     * The a0 coefficient is assumed to be 1.
     */
    float a1[MAX_SOS_STAGES];

    /**
     * @brief Denominator coefficient a2 for each SOS stage.
     *
     * Corresponds to the second-order feedback term.
     */
    float a2[MAX_SOS_STAGES];
};

/**
 * @class SOSFilter
 * @brief Cascade of Second-Order IIR filter sections (SOS).
 *
 * Implements a cascaded IIR filter using **Direct Form II Transposed**
 * biquad sections. Each stage realizes the transfer function:
 *
 * @f[
 * H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}
 *              {1 + a_1 z^{-1} + a_2 z^{-2}}
 * @f]
 *
 * The full filter response is the product of all SOS stages.
 *
 * - Uses two state variables per stage
 * - Assumes a0 = 1 for all sections
 */
class SOSFilter
{
public:
    /**
     * @brief Construct a new SOSFilter object.
     *
     * The filter is constructed in an uninitialized state.
     * Call begin() before processing samples.
     */
    SOSFilter() {}

    /**
     * @brief Initialize the filter with SOS coefficients.
     *
     * Copies the provided SOS coefficient structure into the filter
     * and resets all internal state variables.
     *
     * If the number of stages exceeds MAX_SOS_STAGES, it is clamped.
     *
     * @param coeffs SOS coefficient structure containing filter stages
     */
    void begin(const SOSCoefficients& coeffs)
    {
        sos = coeffs;

        if (sos.stages > MAX_SOS_STAGES)
            sos.stages = MAX_SOS_STAGES;

        reset();
    }

    /**
     * @brief Process a single input sample through the SOS filter.
     *
     * The filter is implemented as a cascade of second-order sections
     * using the Direct Form II Transposed structure.
     *
     * @param x Input sample
     * @return float Filtered output sample
     */
    float process(float x)
    {
        float y = x;

        for (int i = 0; i < sos.stages; i++)
        {
            float out = sos.b0[i] * y + w1[i];

            w1[i] = sos.b1[i] * y - sos.a1[i] * out + w2[i];
            w2[i] = sos.b2[i] * y - sos.a2[i] * out;

            y = out;
        }

        return y;
    }

    /**
     * @brief Reset all internal filter states.
     *
     * Clears the delay elements of all SOS stages, returning the filter
     * to its zero-state condition.
     */
    void reset()
    {
        for (int i = 0; i < sos.stages; i++)
        {
            w1[i] = 0.0f;
            w2[i] = 0.0f;
        }
    }

private:
    /** SOS coefficient structure (filter configuration) */
    SOSCoefficients sos;

    /** First delay state per SOS stage (DF-II Transposed) */
    float w1[MAX_SOS_STAGES];

    /** Second delay state per SOS stage (DF-II Transposed) */
    float w2[MAX_SOS_STAGES];
};
