# NDT 3D #

This repository contains an implementation of the Normal Distribution Transform. 

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.

* PCL (Point cloud library) - 1.7 version
* Linux Ubuntu 16.04 or 18.04
* Qt Creator 4.5.2
## Usage ##

* Create a folder named build (for example)
mkdir build
* Enter folder
cd build/
* Run "CmakeLists.txt" which is a path before build
cmake ..
* Run make
make
* When the program has been built, run: 
./ndt argv[1] argv[2] argv[3] argv[4] argv[5] 

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: Setting maximum step size for More-Thuente line search.
* argv[4]: Setting Resolution of NDT grid structure (VoxelGridCovariance).
* argv[5]: Setting max number of registration iterations
## Suggestion of parameters ##
According to the PCL documentation, we can use the following parameters. Sets outlier_ratio_ to 0.35, step_size_ to 0.05 and resolution_ to 1.0. For the dimensions of the VoxelGrid, use in the Indoor experiments leaf_size(0.02) on all axes and in the outdoor life_size(0.2).
## Reference ##

