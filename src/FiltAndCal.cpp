#include "Arduino.h"
#include "FiltAndCal.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>




using namespace Eigen;




void vectorCal::begin(const Vector3d& _truthVec,
                      const int&      dataRows)
{
    setTruthVec(_truthVec);
    initDataMat(dataRows);
}




void vectorCal::setTruthVec(const Vector3d& _truthVec)
{
    truthVec = _truthVec;
    truthMag = truthVec.norm();
}




Vector3d vectorCal::getTruthVec()
{
    return truthVec;
}




void vectorCal::setTruthMag(const double& _truthMag)
{
    truthMag = _truthMag;
}




double vectorCal::getTruthMag()
{
    return truthMag;
}




void vectorCal::setA(const Matrix3d& _A)
{
    A     = _A;
    A_inv = A.inverse();
}




void vectorCal::setA_inv(const Matrix3d& _A_inv)
{
    A_inv = _A_inv;
    A     = A_inv.inverse();
}




void vectorCal::setB(const Vector3d& _b)
{
    b = _b;
}




Matrix3d vectorCal::getA()
{
    return A;
}




Matrix3d vectorCal::getA_inv()
{
    return A_inv;
}




Vector3d vectorCal::getB()
{
    return b;
}




void vectorCal::initDataMat(const int& dataCols)
{
    numCols = dataCols;
    dataMat.resize(numRows, numCols);
}




void vectorCal::setDataMat(const Matrix<double, 3, Dynamic>& _dataMat)
{
    numCols = _dataMat.cols();
    dataMat.reshaped(numRows, numCols);
    dataMat << _dataMat;
}




void vectorCal::clearDataMat()
{
    dataMat.resize(numRows, 0);
}




bool vectorCal::insertToDataMat(const Vector3d& col,
                                const int&      colNum)
{
    if (colNum < numCols)
    {
        dataMat.col(colNum) = col;
        return true;
    }
    
    return false;
}




void vectorCal::updateB()
{
    b = dataMat.rowwise().mean();
}




void vectorCal::updateA_inv()
{
    MatrixXd centered = dataMat.colwise()  - dataMat.rowwise().mean();
    Matrix3d cov      = centered.adjoint() * centered;
    
    setA_inv(sqrtm(cov.inverse()));
}




void vectorCal::findCalParams()
{
    updateB();
    updateA_inv();
}




Vector3d vectorCal::calPoint(const Vector3d& point,
                             const bool&     norm)
{
    if (norm)
        return (A_inv * (point - b)).normalized();
    return A_inv * (point - b);
}




// https://en.wikipedia.org/wiki/Square_root_of_a_matrix
Matrix3d vectorCal::sqrtm(const Matrix3d& mat)
{
    // Find eigenvectors and eigenvalues of mat
    EigenSolver<Matrix3d> es(mat);
    Vector3d vals = es.eigenvalues().real();
    Matrix3d vecs = es.eigenvectors().real();

    Matrix3d sqrtVals;
    sqrtVals.diagonal() << sqrt(vals(0)),
                           sqrt(vals(1)),
                           sqrt(vals(2));
    
    Matrix3d sqrtMat;
    sqrtMat = vecs * sqrtVals * vecs.inverse();

    return sqrtMat;
}
