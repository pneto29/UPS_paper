# Generalized-icp #

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
./gicp argv[1] argv[2] argv[3] argv[4]

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: Setting max number of registration iterations (30 by default)
* argv[4]: cloud final

## Reference ##

Segal, Aleksandr, Dirk Haehnel, and Sebastian Thrun. "Generalized-icp." Robotics: science and systems. Vol. 2. No. 4. 2009.
