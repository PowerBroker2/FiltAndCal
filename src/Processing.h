#pragma once
#include "Arduino.h"
#include "SOSFilt.h"
#include "ButterworthFilt.h"
#include "EigenHelpers.h"
#include "ArduHelpers.h"
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

/**
 * @struct ProcessingConfigScalar
 * @brief Configuration parameters for scalar signal processing.
 *
 * This structure defines calibration, sampling, and filtering settings
 * used by @ref ProcessScalar. It includes scale, bias, sampling frequency,
 * cutoff frequencies, filter order, and Butterworth filter type.
 */
struct ProcessingConfigScalar
{
    float scale = 1;            ///< Scale factor applied during calibration.
    float bias  = 0;            ///< Bias offset removed during calibration.

    float fs = 1000;            ///< Sampling frequency in Hz.
    float fl = 10;              ///< Low cutoff frequency in Hz (LPF or BPF lower bound).
    float fh = 400;             ///< High cutoff frequency in Hz (HPF or BPF upper bound).

    unsigned int filtOrder = 2; ///< Butterworth filter order (number of poles).

    ButterworthFilterType filtType = ButterworthFilterType::LPF; ///< Butterworth filter type (LPF, HPF, or BPF).
};

/**
 * @struct ProcessingConfigVector
 * @brief Configuration parameters for processing a 3D vector.
 *
 * This struct contains scaling, bias, transformation matrix, and filtering
 * parameters for axis-wise processing of a 3D vector of floats.
 */
struct ProcessingConfigVector
{
    Vector3f scale = Vector3f::Ones();       ///< Scale factors for each axis (applied as diagonal matrix).
    Matrix3f A     = Matrix3f::Identity();   ///< Transformation matrix applied after scaling.
    Vector3f bias  = Vector3f::Zero();       ///< Bias vector subtracted from input before processing.

    float    fs = 1000;                      ///< Sampling frequency in Hz.
    Vector3f fl = Vector3f::Constant(10);    ///< Low cutoff frequencies for each axis.
    Vector3f fh = Vector3f::Constant(400);   ///< High cutoff frequencies for each axis.

    unsigned int filtOrder = 2;              ///< Butterworth filter order.

    ButterworthFilterType filtType = ButterworthFilterType::LPF; ///< Filter type (LPF, HPF, or BPF).
};

/**
 * @class ProcessScalar
 * @brief Scalar signal calibration and filtering pipeline.
 *
 * This class performs bias removal, scaling, and optional Butterworth filtering
 * (LPF, HPF, or BPF) on scalar signals. The filter is implemented using
 * cascaded second-order sections (SOS) for numerical stability.
 */
class ProcessScalar
{
public:
    /**
     * @brief Default constructor.
     */
    ProcessScalar() {}

    /**
     * @brief Initialize the processing pipeline from configuration.
     *
     * Copies the provided configuration and sets up the internal Butterworth filter
     * according to the filter type, cutoff frequencies, and order.
     *
     * @param procConfig Configuration parameters for scalar processing.
     */
    void begin(const ProcessingConfigScalar& procConfig)
    {
        config = procConfig;

        if (config.filtType == ButterworthFilterType::HPF)
            filter.configureHPF(config.filtOrder, config.fs, config.fh);
        else if (config.filtType == ButterworthFilterType::BPF)
            filter.configureBPF(config.filtOrder, config.fs, config.fl, config.fh);
        else
            filter.configureLPF(config.filtOrder, config.fs, config.fl);
    }

    /**
     * @brief Apply calibration to a single sample.
     *
     * Performs bias removal followed by scaling:
     * \f[
     * y = \text{scale} \cdot (x - \text{bias})
     * \f]
     *
     * @param x Raw input sample.
     * @return Calibrated output sample.
     */
    float cal(float x)
    {
        return config.scale * (x - config.bias);
    }

    /**
     * @brief Apply Butterworth filtering to a single sample.
     *
     * @param x Input sample.
     * @return Filtered output sample.
     */
    float filt(float x)
    {
        return filter.process(x);
    }

    /**
     * @brief Apply full processing (calibration + filtering) to a single sample.
     *
     * Combines `cal()` and `filt()` to provide fully processed output.
     *
     * @param x Raw input sample.
     * @return Calibrated and filtered output sample.
     */
    float process(float x)
    {
        return filt(cal(x));
    }

private:
    ButterworthFilter      filter; ///< Internal Butterworth filter.
    ProcessingConfigScalar config; ///< Cached processing configuration.
};

/**
 * @class ProcessVector
 * @brief 3D vector calibration and filtering pipeline.
 *
 * This class applies scaling, bias removal, linear transformation, and
 * axis-wise Butterworth filtering to a 3D vector.
 */
class ProcessVector
{
public:
    /**
     * @brief Default constructor.
     */
    ProcessVector() {}

    /**
     * @brief Initialize the processor with configuration parameters.
     *
     * Configures internal axis-wise Butterworth filters according to the
     * specified filter type, cutoff frequencies, and order.
     *
     * @param procConfig Configuration parameters for vector processing.
     */
    void begin(const ProcessingConfigVector& procConfig)
    {
        config = procConfig;

        if (config.filtType == ButterworthFilterType::HPF)
        {
            filters[X].configureHPF(config.filtOrder, config.fs, config.fh(X));
            filters[Y].configureHPF(config.filtOrder, config.fs, config.fh(Y));
            filters[Z].configureHPF(config.filtOrder, config.fs, config.fh(Z));
        }
        else if (config.filtType == ButterworthFilterType::BPF)
        {
            filters[X].configureBPF(config.filtOrder, config.fs, config.fl(X), config.fh(X));
            filters[Y].configureBPF(config.filtOrder, config.fs, config.fl(Y), config.fh(Y));
            filters[Z].configureBPF(config.filtOrder, config.fs, config.fl(Z), config.fh(Z));
        }
        else
        {
            filters[X].configureLPF(config.filtOrder, config.fs, config.fl(X));
            filters[Y].configureLPF(config.filtOrder, config.fs, config.fl(Y));
            filters[Z].configureLPF(config.filtOrder, config.fs, config.fl(Z));
        }
    }

    /**
     * @brief Apply calibration to a 3D input vector.
     *
     * Performs scaling, bias removal, and linear transformation using
     * the configured scale vector, bias vector, and transformation matrix.
     *
     * @param data Input 3D vector.
     * @return Calibrated 3D vector.
     */
    Vector3f cal(const Vector3f& data)
    {
        return config.scale.asDiagonal() * config.A * (data - config.bias);
    }

    /**
     * @brief Apply Butterworth filtering to a 3D vector.
     *
     * Filters each axis independently using the configured axis filters.
     *
     * @param data Input 3D vector.
     * @return Filtered 3D vector.
     */
    Vector3f filt(const Vector3f& data)
    {
        Vector3f out;

        out << filters[X].process(data(X)),
               filters[Y].process(data(Y)),
               filters[Z].process(data(Z));

        return out;
    }

    /**
     * @brief Apply full processing (calibration + filtering) to a 3D vector.
     *
     * Combines `cal()` and `filt()` to provide fully processed output.
     *
     * @param data Input 3D vector.
     * @return Calibrated and filtered 3D vector.
     */
    Vector3f process(const Vector3f& data)
    {
        return filt(cal(data));
    }

private:
    static const unsigned int X = 0; ///< Index for X axis in vectors and filters.
    static const unsigned int Y = 1; ///< Index for Y axis in vectors and filters.
    static const unsigned int Z = 2; ///< Index for Z axis in vectors and filters.

    ButterworthFilter filters[3];           ///< Axis-wise Butterworth filters.
    ProcessingConfigVector config;          ///< Cached configuration for vector processing.
};
