#ifndef REGISTER_H
#define REGISTER_H

#include "utils.h"
#include "GaussComp.h"

class Register{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Register(const std::string& GMM_path,const std::string& PlaneFlag_path,const std::string& Planes_path);

    ~Register() = default;

    void SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);

    void run();

    vMoments RegEstep(pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud);

    vMoments RegEstepStruct(pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud);
    
    Eigen::Matrix4f RegMstep(vMoments &moments);

    void accumulate(Eigen::Vector4f& moments, float gamma, const Eigen::Vector3f& z);

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mSource_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mRes_cloud;
    
    vector<GaussianComponent*> mGMM;

    Eigen::Matrix3f mMF0;
    
    //last pose?
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    
    Eigen::Matrix4f mPose;

    std::vector<int> mvNodesFlag;
    std::vector<Vector4f, Eigen::aligned_allocator<Vector4f> > mvPlanesPiror;

    int max_tree_level = 3;
    int n_total;
    int maxiter = 50;
    int plane_num = 0;

};   
#endif