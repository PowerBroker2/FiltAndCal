#include "FiltAndCal.h"




const bool TEST_BANDPASS = true;

const float LPF_HZ = 5;
const float HPF_HZ = 15;




OnePoleFilt     lpf(LPF_HZ, FilterType::LowPass);
OnePoleFilt     hpf(HPF_HZ, FilterType::HighPass);
OnePoleBandPass bpf(LPF_HZ, HPF_HZ);




/**
 * @brief Generate a sine wave sample.
 * @param freqHz Frequency of the sine wave in Hz
 * @param amplitude Amplitude of the sine wave (0.0 to 1.0)
 * @param offset Optional offset added to output
 * @return Sine wave value at current time
 */
float sineWave(float freqHz, float amplitude = 1.0f, float offset = 0.0f)
{
    static uint32_t startMicros = 0;
    if (startMicros == 0)
        startMicros = micros();

    uint32_t now = micros();
    uint32_t dt = now - startMicros;

    // Convert microseconds to seconds
    float t = dt * 1e-6f;

    // Compute sine wave
    float value = amplitude * sinf(2.0f * F_PI * freqHz * t) + offset;

    return value;
}




/**
 * @brief Generate a square wave with bias.
 * @param freqHz Square wave frequency in Hz
 * @param amplitude   Peak deviation from bias
 * @param bias        DC offset added to the waveform
 * @return Square wave value
 */
float squareWave(float freqHz, float amplitude, float bias)
{
    if (freqHz <= 0.0f)
        return bias;

    float period_us = 1e6f / freqHz;
    float t = fmodf((float)micros(), period_us);

    return (t < (period_us * 0.5f))
           ? (bias + amplitude)
           : (bias - amplitude);
}




void setup()
{
    Serial.begin(115200);

    if (TEST_BANDPASS)
    {
        Serial.print("in");
        Serial.print(',');
        Serial.println("bpf_out");
    }
    else
    {
        Serial.print("in");
        Serial.print(',');
        Serial.print("lpf_out");
        Serial.print(',');
        Serial.println("hpf_out");
    }
}




void loop()
{
    if (TEST_BANDPASS)
    {
        float in1     = sineWave(2,  1, 0);
        float in2     = sineWave(10, 1, 0);
        float in3     = sineWave(100, 1, 0);
        float in      = in1 + in2 + in3;
        float bpf_out = bpf.process(in, micros());

        Serial.print(in);
        Serial.print(',');
        Serial.println(bpf_out);
    }
    else
    {
        float in      = squareWave(2, 1, 1);
        float lpf_out = lpf.process(in, micros());
        float hpf_out = hpf.process(in, micros());

        Serial.print(in);
        Serial.print(',');
        Serial.print(lpf_out);
        Serial.print(',');
        Serial.println(hpf_out);
    }

    delay(1);
}