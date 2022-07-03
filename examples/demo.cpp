#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <mutex>

#include "utils.h"
#include "GaussComp.h"
#include "Register.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

int main( int argc, char** argv ){

    if(argc != 2)
    {
        cerr << endl << "Usage: struct_reg path_to_config" << endl;
        return 0;
    }  

    YAML::Node node = YAML::LoadFile(argv[1]);
    std::string base = node["base_path"].as<string>();
    std::string TestNumber = node["TestNumber"].as<string>();

    std::string target_pcd, source_pcd;
    target_pcd = base + "target.pcd";
    source_pcd = base + TestNumber + ".pcd";

    string GMMPath, FlagPath, PlanePath, GroundTruthPath;
    GMMPath = base + "GMM.txt";
    FlagPath = base + "PlaneFlag.txt";
    PlanePath = base + "Planes.txt";
    GroundTruthPath = base + TestNumber + ".txt";
    
    Eigen::Matrix4f gt;
    LoadPoseGroundtruth(GroundTruthPath,gt);

    cout<<"ground truth pose is "<<endl;
    cout<<gt.inverse()<<endl;

    // target_cloud is only used for visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
        std::cerr << "failed to load " << target_pcd << std::endl;
        return 0;
    }
    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
        std::cerr << "failed to load " << source_pcd << std::endl;
        return 0;
    }

    // Eigen::Matrix4f initPose;
    // initPose.setIdentity();
    // pcl::transformPointCloud(*source_cloud,*source_cloud,initPose);

    Register StructRegister(GMMPath,FlagPath,PlanePath);
    StructRegister.SetInputCloud(source_cloud);
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  
    StructRegister.run();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();  
    chrono::duration<double> time_12 = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    std::cout << "Time cost: " << time_12.count() << "[sec]" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_clouds(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::transformPointCloud(*source_cloud,*tf_clouds,StructRegister.mPose);

    // Evaluate the accuracy
    Eigen::Matrix4f ErrT = StructRegister.mPose * gt;
    float Err_t = ErrT.block(0,3,3,1).norm();
    float Err_angle = acos((ErrT.block(0,0,3,3).trace()-1)/2) * 180.0 / 3.1415926;
    cout<<"Accuracy: \nErr_angle: "<<Err_angle<<endl<<"Err_t: "<<Err_t<<endl;

    // visualization
    pcl::visualization::PCLVisualizer vis("vis");
    vis.setBackgroundColor(255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> result_handler(tf_clouds, 0.0, 0.0, 255.0);

    vis.addPointCloud(target_cloud, target_handler, "target");
    vis.addPointCloud(source_cloud, source_handler, "source");
    vis.addPointCloud(tf_clouds, result_handler, "result");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
    vis.spin();

    return 0;
}