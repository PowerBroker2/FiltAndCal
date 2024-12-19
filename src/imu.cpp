#include "FiltAndCal.h"
#include "Optimization.h"




double magRotMinFunc(const VectorXd& vec, Matrix<double, 3, Dynamic> magDataMat, Matrix<double, 3, Dynamic> rotVecMat, Vector3d magTruthVec)
{
    // Convert rotation vector to DCM
    Matrix3d dcm = rotVec_2_dcm(vec);

    // Calc guess values
    auto guess = dcm * magDataMat;

    // Calc expected truth vals if true rotation vectors are provided
    Matrix3d truth(magDataMat.rows(), magDataMat.cols());

    for (int i=0; i<truth.cols(); i++)
    {
        auto rotVec = rotVecMat(all, i);
        auto rotDCM = rotVec_2_dcm(rotVec);

        truth(all, i) = rotDCM * magTruthVec;
    }

    // Compare normalized data points and return the total error
    guess.colwise().normalize();
    truth.colwise().normalize();

    double error = (truth - guess).sum();

    return error;
}




double magScalesMinFunc(const VectorXd& scales)
{
    return 4.1;
}




bool imuCal::calMagData(const bool& trueRotsGiven)
{
    // Rm outliers
    VectorXi magOutlierLocs = outlier_locs(magDataMat, 95);

    accelDataMat = rm_select_cols(accelDataMat, magOutlierLocs);
    magDataMat   = rm_select_cols(magDataMat,   magOutlierLocs);
    timestamps   = rm_select_cols(timestamps,   magOutlierLocs);

    // Directly find mean
    Vector3d magMean = magDataMat.rowwise().mean();

    // Apply mean
    magDataMat = magDataMat.colwise() - magMean;

    // Only optimize mag data for rotation if true rotations are known and given
    if (trueRotsGiven)
    {
        // Optimize rot vec
        VectorXd initRotVec(3);
        initRotVec << 0, 0, 0;

        auto res = Nelder_Mead_Optimizer(magRotMinFunc, // Function to minimize
                                         initRotVec,    // Initial position
                                         0.1,           // Look-around radius in initial step
                                         10e-10);       // Threshold on improve classification

        // Apply rotation
    }


    // Optimize axis scales
    

    // Apply axis scales ?

    return true;
}




void imuCal::setAccelTruthVec(const Vector3d& _accelTruthVec)
{
    accelTruthVec = _accelTruthVec;
    accelTruthMag = accelTruthVec.norm();
}




Vector3d imuCal::getAccelTruthVec()
{
    return accelTruthVec;
}




void imuCal::setMagTruthVec(const Vector3d& _magTruthVec)
{
    magTruthVec = _magTruthVec;
    magTruthMag = magTruthVec.norm();
}




Vector3d imuCal::getMagTruthVec()
{
    return magTruthVec;
}




void imuCal::setAccelTruthMag(const double& _accelTruthMag)
{
    accelTruthMag = _accelTruthMag;
    accelTruthVec = truthVec.normalized() * accelTruthMag;
}




double imuCal::getAccelTruthMag()
{
    return accelTruthMag;
}




void imuCal::setMagTruthMag(const double& _magTruthMag)
{
    magTruthMag = _magTruthMag;
    magTruthVec = truthVec.normalized() * magTruthMag;
}




double imuCal::getMagTruthMag()
{
    return magTruthMag;
}




Matrix3d imuCal::getMagA()
{
    return mag_A;
}




Matrix3d imuCal::getMagA_inv()
{
    return mag_A_inv;
}




Vector3d imuCal::getMagB()
{
    return mag_b;
}




bool imuCal::calAccelData()
{
    // actually cal the accel data

    return true;
}




Vector3d imuCal::getAccelB()
{
    return accel_b;
}




bool imuCal::calGryoData()
{
    // actually cal the gyro data

    return true;
}




Vector3d imuCal::getGryoB()
{
    return gyro_b;
}




void imuCal::initAccelDataMat(const int& numSamps)
{

}




void imuCal::setAccelDataMat(const Matrix<double, 3, Dynamic>& _accelDataMat)
{

}




void imuCal::clearAccelDataMat()
{

}




bool imuCal::insertToAccelDataMat(const Vector3d& samp,
                                  const int&      sampNum)
{
    return true;
}




Vector3d imuCal::calPoint(const Vector3d& point,
                          const bool&     norm)
{
    if (norm)
        return (A_inv * (point - b)).normalized();
    return A_inv * (point - b);
}