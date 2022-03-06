# IterativeClosestPoint #

This repository contains an implementation of the Iterative closest point. 

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
./icp_p2pt argv[1] argv[2] argv[3]

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: Setting max number of registration iterations (30 by default)
## Reference ##

Besl, Paul J., and Neil D. McKay. "Method for registration of 3-D shapes." 
Sensor fusion IV: control paradigms and data structures. Vol. 1611. International Society for Optics and Photonics, 1992.
