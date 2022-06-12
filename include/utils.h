#ifndef UTILIS_H
#define UTILIS_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <tuple>

using namespace std;

#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/Geometry"
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Eigenvalues"
#include "/usr/include/eigen3/Eigen/StdVector"
using namespace Eigen;

#include "/usr/include/pcl-1.7/pcl/common/common_headers.h"
#include "/usr/include/pcl-1.7/pcl/point_types.h"
#include "/usr/include/pcl-1.7/pcl/filters/voxel_grid.h"
#include "/usr/include/pcl-1.7/pcl/filters/statistical_outlier_removal.h"
#include "/usr/include/pcl-1.7/pcl/visualization/cloud_viewer.h"
#include "/usr/include/pcl-1.7/pcl/visualization/pcl_visualizer.h"
#include "/usr/include/pcl-1.7/pcl/io/pcd_io.h"
#include "/usr/include/pcl-1.7/pcl/common/transforms.h"
// #include <pcl/common/transforms.h>

#include "GaussComp.h"

static const int N_NODE = 8 ;
static const float eps = 1.0e-15;

// typedef std::tuple<float, Eigen::Vector3f, Eigen::Matrix3f> NodeParam;
// typedef std::vector<NodeParam, Eigen::aligned_allocator<NodeParam> > NodeParamArray;
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > vMoments;

vector<GaussianComponent*> loadGMMModel(const std::string &filename);

void readNodesFlag(const std::string& filename, std::vector<int>& vflags);

void readplaneparam(const std::string& filename, 
std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &vPlanes, Eigen::Matrix3f& MF0);


# endif