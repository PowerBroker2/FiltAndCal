#include "FiltAndCal.h"




ProcessingConfigScalar config;
ProcessScalar          processor;




float sineWave(float freqHz, float amplitude = 1.0f, float offset = 0.0f)
{
    return amplitude * sinf(2.0f * F_PI * freqHz * micros() * 1e-6f) + offset;
}




void setup()
{
    Serial.begin(115200);

    Serial.print("in");
    Serial.print(',');
    Serial.println("proc_out");

    config.scale     = 0.5;
    config.bias      = 1;
    config.fs        = 1000;
    config.fl        = 5;
    config.fh        = 0;
    config.filtOrder = 2;
    config.filtType  = ButterworthFilterType::LPF;

    processor.begin(config);
}




void loop()
{
    float in1     = sineWave(1,   2, 1);
    float in2     = sineWave(100, 2, 1);
    float in      = in1 + in2;
    float proc_out = processor.process(in);

    Serial.println(in);
    Serial.print(',');
    Serial.println(proc_out);

    delay(1);
}