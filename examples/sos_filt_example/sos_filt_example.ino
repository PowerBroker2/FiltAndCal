#include "FiltAndCal.h"




SOSCoefficients coeffs;
SOSFilter       filt;




float sineWave(float freqHz, float amplitude = 1.0f, float offset = 0.0f)
{
    return amplitude * sinf(2.0f * F_PI * freqHz * micros() * 1e-6f) + offset;
}




void setup()
{
    Serial.begin(115200);

    Serial.print("in");
    Serial.print(',');
    Serial.println("sos_out");

    coeffs.stages = 1,

    // Coefficients assume ~1KHz sample rate - otherwise filter will not work right
    coeffs.b0[0] = 0.0002413;
    coeffs.b1[0] = 0.0004826;
    coeffs.b2[0] = 0.0002413;

    coeffs.a1[0] = -1.9555;
    coeffs.a2[0] =  0.9565;

    filt.begin(coeffs);
}




void loop()
{
    float in1     = sineWave(1,   1, 0);
    float in2     = sineWave(100, 1, 0);
    float in      = in1 + in2;
    float sos_out = filt.process(in);

    Serial.println(in);
    Serial.print(',');
    Serial.println(sos_out);

    delay(1);
}