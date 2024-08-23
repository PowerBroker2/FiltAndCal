#include "Arduino.h"
#include "FiltAndCal.h"
#include "Filters.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>




using namespace Eigen;




void RotVecCal::begin(const Vector3d& _truthVec,
                      const int&      dataRows)
{
    setTruthVec(_truthVec);
    initDataMat(dataRows);
}




void RotVecCal::setTruthVec(const Vector3d& _truthVec)
{
    truthVec = _truthVec;
    truthMag = truthVec.norm();
}




Vector3d RotVecCal::getTruthVec()
{
    return truthVec;
}




void RotVecCal::setTruthMag(const double& _truthMag)
{
    truthMag = _truthMag;
}




double RotVecCal::getTruthMag()
{
    return truthMag;
}




void RotVecCal::setA(const Matrix3d& _A)
{
    A     = _A;
    A_inv = A.inverse();
}




void RotVecCal::setA_inv(const Matrix3d& _A_inv)
{
    A_inv = _A_inv;
    A     = A_inv.inverse();
}




void RotVecCal::setB(const Vector3d& _b)
{
    b = _b;
}




Matrix3d RotVecCal::getA()
{
    return A;
}




Matrix3d RotVecCal::getA_inv()
{
    return A_inv;
}




Vector3d RotVecCal::getB()
{
    return b;
}




void RotVecCal::initDataMat(const int& dataCols)
{
    numCols = dataCols;
    dataMat.resize(numRows, numCols);
}




void RotVecCal::setDataMat(const Matrix<double, 3, Dynamic>& _dataMat)
{
    numCols = _dataMat.cols();
    dataMat.reshaped(numRows, numCols);
    dataMat << _dataMat;
}




void RotVecCal::clearDataMat()
{
    dataMat.resize(numRows, 0);
}




bool RotVecCal::insertToDataMat(const Vector3d& col,
                                const int&      colNum)
{
    if (colNum < numCols)
    {
        dataMat.col(colNum) = col;
        return true;
    }
    
    return false;
}




void RotVecCal::updateB()
{
    dataMat = prune_gaussian_outliers(dataMat, 95);
    b       = dataMat.rowwise().mean();
}




void RotVecCal::updateA_inv()
{
    Matrix3d covar = cov(dataMat);
    
    setA_inv(sqrtm(covar.inverse()));
}




void RotVecCal::findCalParams()
{
    updateB();
    updateA_inv();
}




Vector3d RotVecCal::calPoint(const Vector3d& point,
                             const bool&     norm)
{
    if (norm)
        return (A_inv * (point - b)).normalized();
    return A_inv * (point - b);
}




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