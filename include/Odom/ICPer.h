#ifndef ICPER_H
#define ICPER_H

#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Dense>

#include "Odom/SophusType.h"
#include "Odom/PMtypes.h"
#include "Odom/NavState.h"

namespace ORB_SLAM2
{
using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class ICPer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ICPer();
    ICPer(const string &icpConfig,
	  const string &filterConfig);
    ICPer(const ICPer &other);
    
    // reset to initial state
    void reset();
    
    void process(shared_ptr<DP> refPointCloud,
		 shared_ptr<DP> newPointCloud);
    
    void process_map(shared_ptr<DP> mapPointCloud,
		     shared_ptr<DP> newPointCloud);
    
    // delta measurements, position/velocity/rotation(matrix)
    inline Eigen::Vector3d getDeltaP() const    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    {
        return _delta_P;
    }
    inline Eigen::Matrix3d getDeltaR() const   // R_k+1 = R_k*exp(w_k*dt).     NOTE: Rwc, Rwc'=Rwc*[w_body]x
    {
        return _delta_R;
    }
    // noise covariance propagation of delta measurements
    // note: the order is rotation-velocity-position here
    inline Matrix6d getCovPPhi() const 
    {
        return _cov_P_Phi;
    }
    
    Eigen::Matrix<double, 4, 4> extrinsicMat;

protected:
    
    PM::DataPointsFilters inputFilters;
    PM::ICPSequence icp;
    shared_ptr<PM::Transformation> transformation;
    int minPointsCount;

private:
    
    // delta measurements, position/velocity/rotation(matrix)
    Eigen::Vector3d _delta_P;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    Eigen::Matrix3d _delta_R;    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    Matrix6d _cov_P_Phi;
};

}

#endif