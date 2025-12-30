#pragma once
#include <Arduino.h>
#include "SOSFilt.h"

/**
 * @brief Design a 2nd-order Butterworth low-pass filter stage
 * @param coeffs Reference to SOSCoefficients struct
 * @param numStages Number of 2-pole SOS stages
 * @param fs Sampling frequency in Hz
 * @param fc Low-pass cutoff frequency in Hz
 */
void designButterworthLPF(SOSCoefficients& coeffs, int numStages, float fs, float fc)
{
    if (numStages > MAX_SOS_STAGES) numStages = MAX_SOS_STAGES;
    coeffs.stages = numStages;

    float K = tanf(M_PI * fc / fs);

    float norm = 1.0f / (1.0f + sqrtf(2.0f) * K + K * K);
    float b0 = K * K * norm;
    float b1 = 2.0f * b0;
    float b2 = b0;
    float a1 = 2.0f * (K * K - 1.0f) * norm;
    float a2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;

    for (int i = 0; i < numStages; i++) {
        coeffs.b0[i] = b0;
        coeffs.b1[i] = b1;
        coeffs.b2[i] = b2;
        coeffs.a1[i] = a1;
        coeffs.a2[i] = a2;
    }
}

/**
 * @brief Design a 2nd-order Butterworth high-pass filter stage
 * @param coeffs Reference to SOSCoefficients struct
 * @param numStages Number of 2-pole SOS stages
 * @param fs Sampling frequency in Hz
 * @param fc High-pass cutoff frequency in Hz
 */
void designButterworthHPF(SOSCoefficients& coeffs, int numStages, float fs, float fc)
{
    if (numStages > MAX_SOS_STAGES) numStages = MAX_SOS_STAGES;
    coeffs.stages = numStages;

    float K = tanf(M_PI * fc / fs);

    float norm = 1.0f / (1.0f + sqrtf(2.0f) * K + K * K);
    float b0 = 1.0f * norm;
    float b1 = -2.0f * norm;
    float b2 = 1.0f * norm;
    float a1 = 2.0f * (K * K - 1.0f) * norm;
    float a2 = (1.0f - sqrtf(2.0f) * K + K * K) * norm;

    for (int i = 0; i < numStages; i++) {
        coeffs.b0[i] = b0;
        coeffs.b1[i] = b1;
        coeffs.b2[i] = b2;
        coeffs.a1[i] = a1;
        coeffs.a2[i] = a2;
    }
}

/**
 * @brief Design a 2nd-order Butterworth band-pass filter stage
 * @param coeffs Reference to SOSCoefficients struct
 * @param numStages Number of 2-pole SOS stages
 * @param fs Sampling frequency in Hz
 * @param f1 Lower cutoff frequency in Hz
 * @param f2 Upper cutoff frequency in Hz
 */
void designButterworthBPF(SOSCoefficients& coeffs, int numStages, float fs, float f1, float f2)
{
    if (numStages > MAX_SOS_STAGES) numStages = MAX_SOS_STAGES;
    coeffs.stages = numStages;

    float w1 = tanf(M_PI * f1 / fs);
    float w2 = tanf(M_PI * f2 / fs);
    float Bw = w2 - w1;
    float W0 = sqrtf(w1 * w2);  // Center frequency using geometric mean

    float norm = 1.0f / (1.0f + Bw * W0 + W0 * W0);
    float b0 = Bw * norm;
    float b1 = 0.0f;
    float b2 = -Bw * norm;
    float a1 = 2.0f * (W0 * W0 - 1.0f) * norm;
    float a2 = (1.0f - Bw * W0 + W0 * W0) * norm;

    for (int i = 0; i < numStages; i++) {
        coeffs.b0[i] = b0;
        coeffs.b1[i] = b1;
        coeffs.b2[i] = b2;
        coeffs.a1[i] = a1;
        coeffs.a2[i] = a2;
    }
}

/**
 * @class ButterworthFilter
 * @brief Generic Butterworth filter wrapper combining SOSFilter and SOSCoefficients
 */
class ButterworthFilter {
public:
    /**
     * @brief Default constructor
     */
    ButterworthFilter() {}

    /**
     * @brief Configure as low-pass Butterworth filter
     * @param order Number of poles (filter order)
     * @param fs Sampling frequency in Hz
     * @param fc Cutoff frequency in Hz
     */
    void configureLPF(int order, float fs, float fc)
    {
        int stages = (order + 1) / 2;  // Each SOS stage has 2 poles
        designButterworthLPF(coeffs, stages, fs, fc);
        filt.begin(coeffs);
    }

    /**
     * @brief Configure as high-pass Butterworth filter
     * @param order Number of poles (filter order)
     * @param fs Sampling frequency in Hz
     * @param fc Cutoff frequency in Hz
     */
    void configureHPF(int order, float fs, float fc)
    {
        int stages = (order + 1) / 2;
        designButterworthHPF(coeffs, stages, fs, fc);
        filt.begin(coeffs);
    }

    /**
     * @brief Configure as band-pass Butterworth filter
     * @param order Number of poles (filter order)
     * @param fs Sampling frequency in Hz
     * @param f1 Lower cutoff frequency in Hz
     * @param f2 Upper cutoff frequency in Hz
     */
    void configureBPF(int order, float fs, float f1, float f2)
    {
        int stages = (order + 1) / 2;
        designButterworthBPF(coeffs, stages, fs, f1, f2);
        filt.begin(coeffs);
    }

    /**
     * @brief Process a single sample through the filter
     * @param x Input sample
     * @return Filtered output sample
     */
    float process(float x)
    {
        return filt.process(x);
    }

    /**
     * @brief Reset internal filter state
     */
    void reset()
    {
        filt.reset();
    }

private:
    SOSCoefficients coeffs;  ///< Internal SOS coefficient struct
    SOSFilter filt;          ///< Internal SOS filter object
};
