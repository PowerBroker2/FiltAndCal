#pragma once
#include "Arduino.h"
#include "Filters.h"
#include "Filt.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>




using namespace Eigen;




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
/*
class imuCal
{
public:
    bool     calMagData(const bool& trueRotsGiven = false);
    void     setMagTruthVec(const Vector3d& _magTruthVec);
    Vector3d getMagTruthVec();
    void     setMagTruthMag(const double& _magTruthMag);
    double   getMagTruthMag();
    Matrix3d getMagA();
    Matrix3d getMagA_inv();
    Vector3d getMagB();

    bool     calAccelData();
    void     setAccelTruthVec(const Vector3d& _accelTruthVec);
    Vector3d getAccelTruthVec();
    void     setAccelTruthMag(const double& _accelTruthMag);
    double   getAccelTruthMag();
    Vector3d getAccelB();

    bool     calGryoData();
    Vector3d getGryoB();

    void     initAccelDataMat(const int& numSamps);
    void     setAccelDataMat(const Matrix<double, 3, Dynamic>& _accelDataMat);
    void     clearAccelDataMat();
    bool     insertToAccelDataMat(const Vector3d& samp,
                                  const int&      sampNum);
    
    Vector3d calPoint(const Vector3d& point,
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
    
    Vector3d accelTruthVec;
    double   accelTruthMag;
    Vector3d magTruthVec;
    double   magTruthMag;

    Matrix3d mag_A;
    Matrix3d mag_A_inv;
    Vector3d mag_b;

    Matrix3d accel_A;
    Vector3d accel_A_inv;
    Vector3d accel_b;

    Vector3d gyro_b;
};
*/




Vector3d vectorFiltAndCal(const Vector3d&      data,
                          const sensor_cal&    cal,
                                bool&          filtInit,
                                FilterOnePole& x_lpf,
                                FilterOnePole& y_lpf,
                                FilterOnePole& z_lpf,
                                FilterOnePole& x_hpf,
                                FilterOnePole& y_hpf,
                                FilterOnePole& z_hpf);
Vector3d vectorFilt(const Vector3d&      data,
                    const sensor_cal&    cal,
                          bool&          filtInit,
                          FilterOnePole& x_lpf,
                          FilterOnePole& y_lpf,
                          FilterOnePole& z_lpf,
                          FilterOnePole& x_hpf,
                          FilterOnePole& y_hpf,
                          FilterOnePole& z_hpf);
Vector3d vectorCal(const Vector3d&   data,
                   const sensor_cal& cal);
double doubleFiltAndCal(const double&        data,
                        const sensor_cal&    cal,
                              bool&          filtInit,
                              FilterOnePole& lpf,
                              FilterOnePole& hpf);
double doubleFilt(const double&        data,
                  const sensor_cal&    cal,
                        bool&          filtInit,
                        FilterOnePole& lpf,
                        FilterOnePole& hpf);
double doubleCal(const double&     data,
                 const sensor_cal& cal);




MatrixXd cov(const MatrixXd& mat);
double   single_mahalanobis_dist2(const VectorXd& pt, const VectorXd& mean, const MatrixXd& cov);
VectorXd batch_mahalanobis_dist2(const MatrixXd& x, const MatrixXd& xs);
VectorXi outlier_locs(const MatrixXd& x, const int& thresh_pct);
MatrixXd rm_select_cols(const MatrixXd& x, const VectorXi& cols);
MatrixXd prune_gaussian_outliers(const MatrixXd& x, const int& thresh_pct);




long     factorial(const int& n);
Matrix3d skew(const Vector3d& w);
Matrix3d rotVec_2_dcm(const Vector3d& vec);
Matrix3d sqrtm(const Matrix3d& mat);
void     printVec2d(const Vector2d& vec,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printVec3d(const Vector3d& vec,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printVec4d(const Vector4d& vec,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printVecXd(const VectorXd& vec,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printQuatd(const Quaterniond& quat,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printMat3d(const Matrix3d& mat,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printMat4d(const Matrix4d& mat,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
void     printMatXd(const MatrixXd& mat,
                    const int&      p      = 5,
                    Stream&         stream = Serial);
double   double_constrain(const double& input,
                          const double& min,
                          const double& max);
double   double_map(const double& x,
                    const double& in_min,
                    const double& in_max,
                    const double& out_min,
                    const double& out_max);