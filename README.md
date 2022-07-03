# StructRegistration

StructRegistration is a 3D point cloud registration method utilizing the EM algorithm with structural regularity. 
In particular, we proposed an improved E step which leverages the structural regularity to refine the probabilistic data association. 
Besides, unlike the existing point cloud registration approaches using structural information, we proposed to cluster coplanar points by associating the point set with degraded GMM, such that the time-consuming processes of plane segmentation and correspondence searching are efficiently replaced.

## 1. Related Publication
J. Liu, Y. Liu and Z. Meng. **Point cloud registration leveraging structural regularity in Manhattan world**. *IEEE Robotics and Automation Letters (RA-L),* 2022, doi: 10.1109/LRA.2022.3185782.

## 2. Prerequisites
We have tested the library in **Ubuntu 16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

### C++ and OpenMP

### Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### PCL 
Download and install instructions can be found at: https://github.com/PointCloudLibrary/pcl. We use the version of 1.7

### YAML-cpp
Download and install instructions can be found at: https://github.com/jbeder/yaml-cpp.git

## 3. Building StructRegistration library and examples
Clone the repository:
```
git clone https://github.com/liu-jc18/StructRegistration.git
cd StructRegistration/
mkdir build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```
## 4. Registration Example

Execute the following command. Please change `GMMpath`, `Flagpath` and `Planepath` in demo.cpp and config file to the corresponding path.
```
./examples/structreg path_to_config_file
```

## 5. Pre-processing Example
Coming soon

## 6. Acknowledgements

This work is inspired by [GMMTree(ECCV2018)](https://arxiv.org/pdf/1807.02587.pdf) and the Probreg project [probreg](https://pepy.tech/badge/probreg).

## 7. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
We are still working on improving the code reliability.