#include "GaussComp.h"

using namespace std;
using namespace Eigen;

const double pi3_sqrt = sqrt((M_PI * 2.0) * (M_PI * 2.0) * (M_PI * 2.0));

const double pi3_sqrt_inv = 1.0 / pi3_sqrt;

GaussianComponent::GaussianComponent(const float &weight, const Eigen::Vector3f &mean, const Eigen::Matrix3f &cov) :
mWeight(weight),mMean(mean), mCov(cov) 
{
    plane_id = -1;
    
    mdet = mCov.determinant();
    mInvCov = mCov.inverse();

    mSqrtdet = std::sqrt(mdet);

    //进行特征值分解，计算eigenvalue，判断是否是palaner
    auto eigen_solver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>(mCov);

    // eigen value 是按照从小到大的顺序排列的
    mValues = eigen_solver.eigenvalues();
    mVectors = eigen_solver.eigenvectors();
    
    // 1e-2
    if (mValues.x() / mValues.sum() < 1e-2) {
        mbDegenerated = true;
    }
}   

float GaussianComponent::gaussian3d(Eigen::Vector3f &x)
{
    float val = (x-mMean).transpose() * mInvCov * (x-mMean);
    float gaussianval = 1 / ( std::pow(2*3.1415,1.5) * mSqrtdet) * std::exp(-0.5*val);
    return gaussianval;
}

