#include "FiltAndCal.h"




float fs = 1000.0f; // 1 kHz sample rate




ButterworthFilter lpFilt, hpFilt, bpFilt;




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




void setup() {
    Serial.begin(115200);

    Serial.print("in");
    Serial.print(',');
    Serial.println("sos_out");

    lpFilt.configureLPF(2, fs, 5.0f);       // 2nd-order LPF, 5 Hz cutoff
    hpFilt.configureHPF(2, fs, 50.0f);      // 2nd-order HPF, 50 Hz cutoff
    bpFilt.configureBPF(2, fs, 20.0f, 40.0f); // 2nd-order BPF, 20-40 Hz
}

float sineWave(float freqHz, float amplitude = 1.0f)
{
    float t = micros() * 1e-6f;
    return amplitude * sinf(2.0f * M_PI * freqHz * t);
}

void loop() {
    float in    = squareWave(1, 1, 0);
    float lpOut = lpFilt.process(in);
    float hpOut = hpFilt.process(in);
    float bpOut = bpFilt.process(in);

    Serial.print(in);    Serial.print(',');
    Serial.print(lpOut); Serial.print(',');
    Serial.print(hpOut); Serial.print(',');
    Serial.println(bpOut);

    delay(1);
}
