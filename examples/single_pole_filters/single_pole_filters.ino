#include "FiltAndCal.h"




const float IN_HZ  = 10;
const float LPF_HZ = 5;
const float HPF_HZ = 15;




OnePoleFilt lpf(LPF_HZ, FilterType::LowPass);
OnePoleFilt hpf(HPF_HZ, FilterType::HighPass);




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
 * @brief Generate a falling sawtooth waveform.
 *
 * Produces a sawtooth signal that immediately jumps to 1.0 and then
 * linearly ramps down to 0.0 at the specified frequency. The waveform
 * frequency is independent of loop timing and is calculated using
 * the microsecond timer. Timer rollover is handled automatically.
 *
 * @param freqHz Sawtooth frequency in Hertz.
 * @return Current sawtooth output value in the range [0.0, 1.0].
 *
 * @note This function maintains internal state and should be called
 *       continuously at any rate.
 */
float sawDown(float freqHz)
{
    static uint32_t lastMicros = 0;
    static float phase = 0.0f;

    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f;  // rollover-safe
    lastMicros = now;

    phase += freqHz * dt;

    if (phase >= 1.0f)
        phase -= 1.0f;

    return 1.0f - phase;
}




void setup()
{
    Serial.begin(115200);

    Serial.print("in");
    Serial.print(',');
    Serial.print("lpf_out");
    Serial.print(',');
    Serial.println("hpf_out");
}




void loop()
{
    // float in      = sineWave(60, 1, 0);
    float in      = sawDown(10);
    float lpf_out = lpf.process(in, micros());
    float hpf_out = hpf.process(in, micros());

    Serial.print(in);
    Serial.print(',');
    Serial.print(lpf_out);
    Serial.print(',');
    Serial.println(hpf_out);

    delay(1);
}