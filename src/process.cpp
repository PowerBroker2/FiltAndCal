#include "FiltAndCal.h"




Vector3d vectorFiltAndCal(const Vector3d&      data,
                          const sensor_cal&    cal,
                                bool&          filtInit,
                                FilterOnePole& x_lpf,
                                FilterOnePole& y_lpf,
                                FilterOnePole& z_lpf,
                                FilterOnePole& x_hpf,
                                FilterOnePole& y_hpf,
                                FilterOnePole& z_hpf)
{
    return vectorCal(vectorFilt(data,
                                cal,
                                filtInit,
                                x_lpf,
                                y_lpf,
                                z_lpf,
                                x_hpf,
                                y_hpf,
                                z_hpf),
                     cal);
}




Vector3d vectorFilt(const Vector3d&      data,
                    const sensor_cal&    cal,
                          bool&          filtInit,
                          FilterOnePole& x_lpf,
                          FilterOnePole& y_lpf,
                          FilterOnePole& z_lpf,
                          FilterOnePole& x_hpf,
                          FilterOnePole& y_hpf,
                          FilterOnePole& z_hpf)
{
    if (!filtInit)
    {
        x_lpf.setToNewValue(data(0));
        y_lpf.setToNewValue(data(1));
        z_lpf.setToNewValue(data(2));

        x_hpf.setToNewValue(data(0));
        y_hpf.setToNewValue(data(1));
        z_hpf.setToNewValue(data(2));

        filtInit = true;
    }

    Vector3d output;
    output << x_lpf.input(data(0)),
              y_lpf.input(data(1)),
              z_lpf.input(data(2));

    if (cal.hpf_cutoff_hz > 0)
    {
        output << x_hpf.input(output(0)),
                  y_hpf.input(output(1)),
                  z_hpf.input(output(2));
    }

    return output;
}




Vector3d vectorCal(const Vector3d&   data,
                   const sensor_cal& cal)
{
    return cal.extrinsic_cal_mat * ((cal.intrinsic_cal_mat * (data - cal.intrinsic_bias_vec)) - cal.extrinsic_bias_vec);
}




double doubleFiltAndCal(const double&        data,
                        const sensor_cal&    cal,
                              bool&          filtInit,
                              FilterOnePole& lpf,
                              FilterOnePole& hpf)
{
    return doubleCal(doubleFilt(data,
                                cal,
                                filtInit,
                                lpf,
                                hpf),
                     cal);
}




double doubleFilt(const double&        data,
                  const sensor_cal&    cal,
                        bool&          filtInit,
                        FilterOnePole& lpf,
                        FilterOnePole& hpf)
{
    if (!filtInit)
    {
        lpf.setToNewValue(data);
        hpf.setToNewValue(data);

        filtInit = true;
    }

    double output = lpf.input(data);

    if (cal.hpf_cutoff_hz > 0)
        output = hpf.input(output);

    return output;
}




double doubleCal(const double&     data,
                 const sensor_cal& cal)
{
    return cal.scale * (data - cal.bias);
}