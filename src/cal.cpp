#include "Arduino.h"
#include "FiltAndCal.h"
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
    b = dataMat.rowwise().mean();
}




void RotVecCal::updateA_inv()
{
    MatrixXd centered = dataMat.colwise()  - dataMat.rowwise().mean();
    Matrix3d cov      = centered.adjoint() * centered;
    
    setA_inv(sqrtm(cov.inverse()));
}




void RotVecCal::findCalParams()
{
    dataMat = prune_gaussian_outliers(dataMat, 95);
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
