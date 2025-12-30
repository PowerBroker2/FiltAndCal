#include "FiltAndCal.h"
#include "EigenHelpers.h"




void setup()
{
  Serial.begin(115200);
  
  MatrixXf good_data(3, 20);
  good_data << 3.91542538,  1.77830157, -0.36368999, -1.00087408,  1.65898023,  2.19121918,  3.07366372,  2.01862603,  0.19489022, -0.00679431,  2.69314953,  3.53368957,  2.97802946,  0.32067887,  1.19654501,  3.67355092,  0.06746349,  3.11263296,  2.47380276,  2.49187178,
              -2.44862684,  0.33743269,  3.00290253,  3.11391635,  1.53993217,  0.30860205, -0.71053578,  1.19378438,  2.96334254,  0.63117182,  0.25826027,  0.13700148, -0.51480574, -0.76107036,  1.12609548,  0.12014191,  1.86552258, -2.19648066,  1.52881818,  0.02502557,
               1.08207634,  1.91531964,  3.74677927,  3.54187115,  1.91153436,  2.09950204,  1.75185221,  2.111693,    3.02394471,  4.07948591,  1.91059451,  1.11800634,  1.49780593,  3.83003141,  2.29246864,  1.02383345,  3.19862434,  1.58702413,  1.6620874,   1.7290803;
  Serial.println("good_data");
  printMat(good_data);
  Serial.println();

  Vector3f bad_data_point;
  bad_data_point << -2, -2, -2;
  MatrixXf noised_data = good_data;
  noised_data.conservativeResize(noised_data.rows(), noised_data.cols() + 1);
  noised_data.col(noised_data.cols() - 1) = bad_data_point;
  Serial.println("bad_data_point");
  printVec(bad_data_point);
  Serial.println();
  Serial.println("noised_data");
  printMat(noised_data);
  Serial.println();

  VectorXf mean              =  good_data.rowwise().mean();
  MatrixXf good_data_no_mean = (good_data.colwise() - mean);
  Serial.println("mean");
  printVec(mean);
  Serial.println();
  
  MatrixXf covar = cov(good_data);
  Serial.println("covar");
  printMat(covar);
  Serial.println();
  
  Serial.println("bad_data_point mahalanobis dist2");
  Serial.println(single_mahalanobis_dist2(bad_data_point, mean, covar));
  Serial.println();
  
  MatrixXf pruned_data = prune_gaussian_outliers(noised_data, 95);
  Serial.println("pruned_data");
  printMat(pruned_data);
  Serial.println();
}




void loop()
{
  
}