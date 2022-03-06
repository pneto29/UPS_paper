# CP_icp#

This repository contains an implementation of the CP-ICP. 

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.

* PCL (Point cloud library) - 1.7 version
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
./cpicp argv[1] argv[2] argv[3] argv[4] argv[5] argv[6]

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: number of partitions -> 2
* argv[4]: stop criterium -> 0.0030
* argv[5]: Setting max number of registration iterations -> 30


## Reference ##

Neto, Polycarpo Souza Souza, Nicolas S. Pereira, and George AP Thé. 
"Improved Cloud Partitioning Sampling for Iterative Closest Point: Qualitative and Quantitative Comparison Study." ICINCO (2). 2018.
