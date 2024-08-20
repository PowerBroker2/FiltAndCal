#pragma once
#include "Arduino.h"
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



MatrixXd cov(const MatrixXd& mat);
double single_mahalanobis_dist2(const Vector2d& pt, const Vector2d& mean, const Matrix2d& cov);
VectorXd batch_mahalanobis_dist2(const MatrixXd& x, const MatrixXd& xs);
MatrixXd prune_gaussian_outliers(const MatrixXd& x, const int& thresh_pct);
long factorial(const int& n);
Matrix3d skew(const Vector3d& w);
void printVec2d(const Vector2d& vec,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printVec3d(const Vector3d& vec,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printVec4d(const Vector4d& vec,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printVecXd(const VectorXd& vec,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printQuatd(const Quaterniond& quat,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printMat3d(const Matrix3d& mat,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printMat4d(const Matrix4d& mat,
                const int&      p      = 5,
                Stream&         stream = Serial);
void printMatXd(const MatrixXd& mat,
                const int&      p      = 5,
                Stream&         stream = Serial);
double double_constrain(const double& input,
                        const double& min,
                        const double& max);
double double_map(const double& x,
                  const double& in_min,
                  const double& in_max,
                  const double& out_min,
                  const double& out_max);