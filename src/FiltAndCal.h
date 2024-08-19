#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/LU>
#include <Eigen/Geometry>
#include "outliers.h"




struct sensor_cal
{
    Matrix3d intrinsic_cal_mat  = Matrix3d::Identity();
    Vector3d intrinsic_bias_vec = Vector3d::Zero();

    Matrix3d extrinsic_cal_mat  = Matrix3d::Identity();
    Vector3d extrinsic_bias_vec = Vector3d::Zero();

    double scale = 1;
    double bias  = 0;

    double lpf_cutoff_hz = 1000;
    double hpf_cutoff_hz = 0;
};



// https://teslabs.com/articles/magnetometer-calibration/
class vectorCal
{
public:
    void     begin(const Vector3d& _truthVec,
                   const int&      dataRows = 0);
    void     setTruthVec(const Vector3d& _truthVec);
    Vector3d getTruthVec();
    void     setTruthMag(const double& _truthMag);
    double   getTruthMag();
    void     setA(const Matrix3d& _A);
    void     setA_inv(const Matrix3d& _A_inv);
    void     setB(const Vector3d& _b);
    Matrix3d getA();
    Matrix3d getA_inv();
    Vector3d getB();
    void     initDataMat(const int& dataCols);
    void     setDataMat(const Matrix<double, 3, Dynamic>& _dataMat);
    void     clearDataMat();
    bool     insertToDataMat(const Vector3d& col,
                             const int&      colNum);
    void     findCalParams();
    Vector3d calPoint(const Vector3d& point,
                      const bool&     norm = true);



protected:
    int numRows = 3;
    int numCols;

    Matrix<double, 3, Dynamic> dataMat;
    
    Vector3d truthVec;
    double   truthMag;

    Matrix3d A;
    Matrix3d A_inv;
    Vector3d b;



    void     updateA_inv();
    void     updateB();
    Matrix3d sqrtm(const Matrix3d& mat);
};