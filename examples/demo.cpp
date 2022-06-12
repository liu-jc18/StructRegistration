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

    if(argc != 3)
    {
        cerr << endl << "Usage: struct_reg path_to_target_pointcloud path_to_source_pointcloud" << endl;
        return 0;
    }  

    std::string target_pcd = argv[1];
    std::string source_pcd = argv[2];

    // target_cloud is only for visualization
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


    // Load GMM
    string GMMpath = "data/GMM_level_3.txt";
    std::string Flagpath = "data/PlaneFlag.txt";
    string Planepath = "data/planes.txt";
    Register StructRegister(GMMpath,Flagpath,Planepath);

    StructRegister.SetInputCloud(source_cloud);
    StructRegister.run();

    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_clouds(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::transformPointCloud(*source_cloud,*tf_clouds,StructRegister.mPose);

    // visualization
    pcl::visualization::PCLVisualizer vis("vis");
    vis.setBackgroundColor(255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(tf_clouds, 0.0, 255.0, 0.0);
    vis.addPointCloud(target_cloud, target_handler, "target");
    vis.addPointCloud(tf_clouds, source_handler, "source");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
    vis.spin();

    return 0;
}