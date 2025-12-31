#pragma once
#include "Arduino.h"
#include "EigenHelpers.h"
#include "Chi2_PPF.h"
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

// https://en.wikipedia.org/wiki/Mahalanobis_distance
inline double single_mahalanobis_dist2(const VectorXf& pt, const VectorXf& mean, const MatrixXf& cov)
{
    auto zero_mean_pt = pt - mean;
    return zero_mean_pt.transpose() * cov.ldlt().solve(zero_mean_pt);
}

inline VectorXi outlier_locs(const MatrixXf& x, const int& thresh_pct)
{
    int   df           = x.rows(); // Degrees of freedom (dimensionality of the data)
    int   n            = x.cols(); // Num data points
    double thresh_dist = 1000;

    if (df == 1)
        thresh_dist = chi2_1df_ppf[thresh_pct];
    else if (df == 2)
        thresh_dist = chi2_2df_ppf[thresh_pct];
    else if (df == 3)
        thresh_dist = chi2_3df_ppf[thresh_pct];
    
    VectorXf x_mean = x.rowwise().mean();
    MatrixXf x_cov  = cov(x);
    VectorXf dists(n);

    int numGood = 0;

    for (int i=0; i<n; i++)
    {
        double dist = single_mahalanobis_dist2(x(all, i), x_mean, x_cov);

        dists(i) = dist;
        
        if (dist <= thresh_dist)
            numGood++;
    }

    int      numBad = n - numGood;
    VectorXi out(numBad);
    int      k = 0;

    for (int i = 0; i < n; i++)
    {
        if (dists(i) > thresh_dist)
        {
            out(k) = i;
            k++;
        }
    }

    return out;
}

// https://towardsdatascience.com/multivariate-outlier-detection-in-python-e946cfc843b3
inline MatrixXf prune_gaussian_outliers(const MatrixXf& x, const int& thresh_pct)
{
    int    df          = x.rows(); // Degrees of freedom (dimensionality of the data)
    int    n           = x.cols(); // Num data points
    double thresh_dist = 1000;

    if (df == 1)
        thresh_dist = chi2_1df_ppf[thresh_pct];
    else if (df == 2)
        thresh_dist = chi2_2df_ppf[thresh_pct];
    else if (df == 3)
        thresh_dist = chi2_3df_ppf[thresh_pct];
    
    VectorXf x_mean = x.rowwise().mean();
    MatrixXf x_cov  = cov(x);
    VectorXf dists(n);

    int numGood = 0;

    for (int i=0; i<n; i++)
    {
        double dist = single_mahalanobis_dist2(x(all, i), x_mean, x_cov);

        dists(i) = dist;
        
        if (dist <= thresh_dist)
            numGood++;
    }

    MatrixXf out(df, numGood);
    int k = 0;

    for (int i = 0; i < n; i++)
    {
        if (dists(i) <= thresh_dist)
        {
            out(all, k) = x(all, i);
            k++;
        }
    }

    return out;
}