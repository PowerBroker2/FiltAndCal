#pragma once
#include "Arduino.h"
#include "Utils.h"
#include "Filters.h"
#include "Filt.h"
#include "Sensor.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>




using namespace Eigen;




// https://teslabs.com/articles/magnetometer-calibration/
/*
class imuCal
{
public:
    bool     calMagData(const bool& trueRotsGiven = false);
    void     setMagTruthVec(const Vector3f& _magTruthVec);
    Vector3f getMagTruthVec();
    void     setMagTruthMag(const double& _magTruthMag);
    double   getMagTruthMag();
    Matrix3f getMagA();
    Matrix3f getMagA_inv();
    Vector3f getMagB();

    bool     calAccelData();
    void     setAccelTruthVec(const Vector3f& _accelTruthVec);
    Vector3f getAccelTruthVec();
    void     setAccelTruthMag(const double& _accelTruthMag);
    double   getAccelTruthMag();
    Vector3f getAccelB();

    bool     calGryoData();
    Vector3f getGryoB();

    void     initAccelDataMat(const int& numSamps);
    void     setAccelDataMat(const Matrix<double, 3, Dynamic>& _accelDataMat);
    void     clearAccelDataMat();
    bool     insertToAccelDataMat(const Vector3f& samp,
                                  const int&      sampNum);
    
    Vector3f calPoint(const Vector3f& point,
                      const bool&     norm = true);
    
    double magRotMinFunc(const VectorXd& vec);
    double magScalesMinFunc(const VectorXd& scales);



protected:
    int  numSamples;
    bool calDataReady;

    Matrix<double, 3, Dynamic> accelDataMat;
    Matrix<double, 3, Dynamic> magDataMat;
    Matrix<double, 3, Dynamic> gyroDataMat;
    Matrix<double, 3, Dynamic> rotVecMat;
    VectorXd                   timestamps;
    
    Vector3f accelTruthVec;
    double   accelTruthMag;
    Vector3f magTruthVec;
    double   magTruthMag;

    Matrix3f mag_A;
    Matrix3f mag_A_inv;
    Vector3f mag_b;

    Matrix3f accel_A;
    Vector3f accel_A_inv;
    Vector3f accel_b;

    Vector3f gyro_b;
};
*/




MatrixXd cov(const MatrixXd& mat);
double   single_mahalanobis_dist2(const VectorXd& pt, const VectorXd& mean, const MatrixXd& cov);
VectorXd batch_mahalanobis_dist2(const MatrixXd& x, const MatrixXd& xs);
VectorXi outlier_locs(const MatrixXd& x, const int& thresh_pct);
MatrixXd rm_select_cols(const MatrixXd& x, const VectorXi& cols);
MatrixXd prune_gaussian_outliers(const MatrixXd& x, const int& thresh_pct);