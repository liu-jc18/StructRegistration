#ifndef PLANEFITTING_H
#define PLANEFITTING_H

#include "utils.h"

// use RANSAC to fit plane
inline double uniformRandom(void)
    {return (double)rand() / (double)RAND_MAX;}

bool getSample(std::vector<int> set, std::vector<int> &sset);

bool verify(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vPointclouds);

int fit_plane(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vPointclouds,
              float &dist_, Eigen::Vector3f &normal_, bool do_leastsquare=true);

#endif