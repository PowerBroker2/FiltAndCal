#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>




using namespace Eigen;




// https://stackoverflow.com/a/70890675/9860973
long factorial(const int& n)
{
    long f = 1;
    for (int i=1; i<=n; ++i)
        f *= i;
    return f;
}




/*
  Description:
  ------------
  Make a skew symmetric matrix from a 3 element vector. Skew
  symmetric matrices are often used to easily take cross products.

  https://en.wikipedia.org/wiki/Skew-symmetric_matrix

  Arguments:
  ----------
  * const Vector3d& w - 3 element vector

  Returns:
  --------
  * Matrix3d C - Skew symmetric matrix of vector w
*/
Matrix3d skew(const Vector3d& w)
{
    Matrix3d C;

    C <<  0.0,  -w(2),  w(1),
         w(2),    0.0, -w(0),
        -w(1),   w(0),   0.0;

    return C;
}




// https://en.wikipedia.org/wiki/Square_root_of_a_matrix
Matrix3d sqrtm(const Matrix3d& mat)
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




void printVec2d(const Vector2d& vec,
                const int&      p,
                Stream&         stream)
{
    stream.println(vec(0), p);
    stream.println(vec(1), p);
}




void printVec3d(const Vector3d& vec,
                const int&      p,
                Stream&         stream)
{
    stream.println(vec(0), p);
    stream.println(vec(1), p);
    stream.println(vec(2), p);
}




void printVec4d(const Vector4d& vec,
                const int&      p,
                Stream&         stream)
{
    stream.println(vec(0), p);
    stream.println(vec(1), p);
    stream.println(vec(2), p);
    stream.println(vec(3), p);
}




void printVecXd(const VectorXd& vec,
                const int&      p,
                Stream&         stream)
{
    for (int i=0; i<vec.rows(); i++)
    {
        if (vec(i) >= 0)
            Serial.print(' ');
        
        stream.println(vec(i), p);
    }
}




void printQuatd(const Quaterniond& quat,
                const int&      p,
                Stream&         stream)
{
    stream.print("x: "); stream.println(quat.x(), p);
    stream.print("y: "); stream.println(quat.y(), p);
    stream.print("z: "); stream.println(quat.z(), p);
    stream.print("w: "); stream.println(quat.w(), p);
}




void printMat3d(const Matrix3d& mat,
                const int&      p,
                Stream&         stream)
{
    stream.print(mat(0, 0), p); stream.print(", "); stream.print(mat(0, 1), p); stream.print(", "); stream.println(mat(0, 2), p);
    stream.print(mat(1, 0), p); stream.print(", "); stream.print(mat(1, 1), p); stream.print(", "); stream.println(mat(1, 2), p);
    stream.print(mat(2, 0), p); stream.print(", "); stream.print(mat(2, 1), p); stream.print(", "); stream.println(mat(2, 2), p);
}




void printMat4d(const Matrix4d& mat,
                const int&      p,
                Stream&         stream)
{
    stream.print(mat(0, 0), p); stream.print(", "); stream.print(mat(0, 1), p); stream.print(", "); stream.print(mat(0, 2), p); stream.print(", "); stream.println(mat(0, 3), p);
    stream.print(mat(1, 0), p); stream.print(", "); stream.print(mat(1, 1), p); stream.print(", "); stream.print(mat(1, 2), p); stream.print(", "); stream.println(mat(1, 3), p);
    stream.print(mat(2, 0), p); stream.print(", "); stream.print(mat(2, 1), p); stream.print(", "); stream.print(mat(2, 2), p); stream.print(", "); stream.println(mat(2, 3), p);
    stream.print(mat(3, 0), p); stream.print(", "); stream.print(mat(3, 1), p); stream.print(", "); stream.print(mat(3, 2), p); stream.print(", "); stream.println(mat(3, 3), p);
}




void printMatXd(const MatrixXd& mat,
                const int&      p,
                Stream&         stream)
{
    for (int i=0; i<mat.rows(); i++)
    {
        for (int j=0; j<mat.cols(); j++)
        {
            if (mat(i, j) >= 0)
                Serial.print(' ');
        
            stream.print(mat(i, j), p);

            if (j != (mat.cols() - 1))
                stream.print(", ");
        }

        stream.println();
    }
}




double double_constrain(const double& input,
                        const double& min,
                        const double& max)
{
    if (input > max)
        return max;
    else if (input < min)
        return min;
    else
        return input;
}




double double_map(const double& x,
                  const double& in_min,
                  const double& in_max,
                  const double& out_min,
                  const double& out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}