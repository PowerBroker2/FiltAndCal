#include "FiltAndCal.h"




ProcessingConfigVector config;
ProcessVector          processor;




float sineWave(float freqHz, float amplitude = 1.0f, float offset = 0.0f)
{
    return amplitude * sinf(2.0f * F_PI * freqHz * micros() * 1e-6f) + offset;
}




void setup()
{
    Serial.begin(115200);

    Serial.print("in_x");
    Serial.print(',');
    Serial.print("in_y");
    Serial.print(',');
    Serial.print("in_z");
    Serial.print(',');
    Serial.print("proc_out_x");
    Serial.print(',');
    Serial.print("proc_out_y");
    Serial.print(',');
    Serial.println("proc_out_z");

    processor.begin(config);
}




void loop()
{
    float in1_x     = sineWave(1,   2, 1);
    float in2_x     = sineWave(100, 2, 1);
    float in_x      = in1_x + in2_x;

    float in1_y     = sineWave(1,   2, 1);
    float in2_y     = sineWave(100, 2, 1);
    float in_y      = in1_y + in2_y;
    
    float in1_z     = sineWave(1,   2, 1);
    float in2_z     = sineWave(100, 2, 1);
    float in_z      = in1_z + in2_z;

    Vector3f in;
    in << in_x,
          in_y,
          in_z;

    Vector3f proc_out = processor.process(in);

    Serial.print(in(0));
    Serial.print(',');
    Serial.print(in(1));
    Serial.print(',');
    Serial.print(in(2));
    Serial.print(',');
    Serial.print(proc_out(0));
    Serial.print(',');
    Serial.print(proc_out(1));
    Serial.print(',');
    Serial.println(proc_out(2));

    delay(1);
}
