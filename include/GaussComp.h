#ifndef GAUSSCOMP_H
#define GAUSSCOMP_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;

class GaussianComponent {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //构造函数
    GaussianComponent(const float &weight, const Eigen::Vector3f &mean, const Eigen::Matrix3f &cov) ;
    
    ~GaussianComponent() = default;
    
    float gaussian3d(Eigen::Vector3f &x);

    const Eigen::Matrix3f &cov() const { return mCov; }
    const Eigen::Vector3f &mean() const { return mMean; }
    const float &weight() const { return mWeight; }
    

public:

int plane_id;

float mWeight;

Eigen::Vector3f mMean;

Eigen::Matrix3f mCov;

Eigen::Matrix3f mInvCov;

Eigen::Vector3f mValues;

Eigen::Matrix3f mVectors;

float mdet;

float mSqrtdet;

bool mbDegenerated = false;
};

#endif