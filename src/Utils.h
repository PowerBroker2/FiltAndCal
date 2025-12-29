#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

/** @brief Float constant for pi */
constexpr float F_PI = 3.14159265358979323846f;

/**
 * @enum FilterType
 * @brief Type of one-pole filter.
 */
enum class FilterType
{
    LowPass,  /**< Low-pass filter output */
    HighPass  /**< High-pass filter output */
};

/**
 * @enum FilterPoleNum
 * @brief Number of poles to use for filtering.
 *
 * Determines whether a first-order (one-pole) or second-order (two-pole)
 * band-pass filter is used internally.
 */
enum class FilterPoleNum
{
    One, /**< Use a first-order (one-pole) band-pass filter */
    Two  /**< Use a second-order (two-pole) band-pass filter */
};

// https://stackoverflow.com/a/70890675/9860973
// long factorial(const int& n)
// {
//     long f = 1;
//     for (int i=1; i<=n; ++i)
//         f *= i;
//     return f;
// }




/*
  Description:
  ------------
  Make a skew symmetric matrix from a 3 element vector. Skew
  symmetric matrices are often used to easily take cross products.

  https://en.wikipedia.org/wiki/Skew-symmetric_matrix

  Arguments:
  ----------
  * const Vector3f& w - 3 element vector

  Returns:
  --------
  * Matrix3f C - Skew symmetric matrix of vector w
*/
// Matrix3f skew(const Vector3f& w)
// {
//     Matrix3f C;

//     C <<  0.0,  -w(2),  w(1),
//          w(2),    0.0, -w(0),
//         -w(1),   w(0),   0.0;

//     return C;
// }




// Matrix3f rotVec_2_dcm(const Vector3f& vec)
// {
//     double   theta = vec.norm();
//     Vector3f axis;

//     if (theta == 0)
//         axis << 1, 0, 0;
//     else
//         axis = vec / theta;

//     AngleAxisf angAxis(theta, axis);
//     Matrix3f   dcm = angAxis.toRotationMatrix();

//     return dcm;
// }




// https://stackoverflow.com/a/15142446/9860973
// Assumes NxM mat where N is degrees of freedom and M is number of datapoints
// MatrixXd cov(const MatrixXd& mat)
// {
//     VectorXd xs_mean = mat.rowwise().mean();
//     xs_mean.transposeInPlace();

//     MatrixXd centered = mat.colwise() - xs_mean;
//     MatrixXd covar    = (centered * centered.adjoint()) / double(mat.cols() - 1);

//     return covar;
// }




// https://en.wikipedia.org/wiki/Square_root_of_a_matrix
// Matrix3f sqrtm(const Matrix3f& mat)
// {
//     // Find eigenvectors and eigenvalues of mat
//     EigenSolver<Matrix3f> es(mat);
//     Vector3f vals = es.eigenvalues().real();
//     Matrix3f vecs = es.eigenvectors().real();

//     Matrix3f sqrtVals;
//     sqrtVals.diagonal() << sqrt(vals(0)),
//                            sqrt(vals(1)),
//                            sqrt(vals(2));
    
//     Matrix3f sqrtMat;
//     sqrtMat = vecs * sqrtVals * vecs.inverse();

//     return sqrtMat;
// }




// void printVec2d(const Vector2d& vec,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.println(vec(0), p);
//     stream.println(vec(1), p);
// }




// void printVec3d(const Vector3f& vec,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.println(vec(0), p);
//     stream.println(vec(1), p);
//     stream.println(vec(2), p);
// }




// void printVec4d(const Vector4d& vec,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.println(vec(0), p);
//     stream.println(vec(1), p);
//     stream.println(vec(2), p);
//     stream.println(vec(3), p);
// }




// void printVecXd(const VectorXd& vec,
//                 const int&      p,
//                 Stream&         stream)
// {
//     for (int i=0; i<vec.rows(); i++)
//     {
//         if (vec(i) >= 0)
//             Serial.print(' ');
        
//         stream.println(vec(i), p);
//     }
// }




// void printQuatd(const Quaterniond& quat,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.print("x: "); stream.println(quat.x(), p);
//     stream.print("y: "); stream.println(quat.y(), p);
//     stream.print("z: "); stream.println(quat.z(), p);
//     stream.print("w: "); stream.println(quat.w(), p);
// }




// void printMat3d(const Matrix3f& mat,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.print(mat(0, 0), p); stream.print(", "); stream.print(mat(0, 1), p); stream.print(", "); stream.println(mat(0, 2), p);
//     stream.print(mat(1, 0), p); stream.print(", "); stream.print(mat(1, 1), p); stream.print(", "); stream.println(mat(1, 2), p);
//     stream.print(mat(2, 0), p); stream.print(", "); stream.print(mat(2, 1), p); stream.print(", "); stream.println(mat(2, 2), p);
// }




// void printMat4d(const Matrix4d& mat,
//                 const int&      p,
//                 Stream&         stream)
// {
//     stream.print(mat(0, 0), p); stream.print(", "); stream.print(mat(0, 1), p); stream.print(", "); stream.print(mat(0, 2), p); stream.print(", "); stream.println(mat(0, 3), p);
//     stream.print(mat(1, 0), p); stream.print(", "); stream.print(mat(1, 1), p); stream.print(", "); stream.print(mat(1, 2), p); stream.print(", "); stream.println(mat(1, 3), p);
//     stream.print(mat(2, 0), p); stream.print(", "); stream.print(mat(2, 1), p); stream.print(", "); stream.print(mat(2, 2), p); stream.print(", "); stream.println(mat(2, 3), p);
//     stream.print(mat(3, 0), p); stream.print(", "); stream.print(mat(3, 1), p); stream.print(", "); stream.print(mat(3, 2), p); stream.print(", "); stream.println(mat(3, 3), p);
// }




inline void printMatXd(const MatrixXd& mat,
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




// double double_constrain(const double& input,
//                         const double& min,
//                         const double& max)
// {
//     if (input > max)
//         return max;
//     else if (input < min)
//         return min;
//     else
//         return input;
// }




// double double_map(const double& x,
//                   const double& in_min,
//                   const double& in_max,
//                   const double& out_min,
//                   const double& out_max)
// {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }