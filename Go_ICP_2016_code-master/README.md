# Go-ICP for globally optimal 3D pointset registration


### Introduction

This repository contains the C++ code for the Go-ICP algorithm (with trimming strategy for outlier handling). It is free software under the terms of the GNU General Public License (GPL) v3. Details of the Go-ICP algorithm can be found in our papers:

* J. Yang, H. Li, Y. Jia, *Go-ICP: Solving 3D Registration Efficiently and
Globally Optimally*, International Conference on Computer Vision (__ICCV__), 2013. [PDF](http://jlyang.org/iccv13_go-icp.pdf)

* J. Yang, H. Li, D. Campbell, Y. Jia, *Go-ICP: A Globally Optimal Solution to 3D ICP Point-Set Registration*, IEEE Transactions on Pattern Analysis and Machine Intelligence (__TPAMI__), 2016. [PDF](http://jlyang.org/tpami16_go-icp_preprint.pdf)

Please read this file carefully prior to using the code. Some frequently-asked questions have answers here.

### Compiling

Use cmake to generate desired projects on different platforms.

A pre-built Windows exe file can be found in [this zip file](http://jlyang.org/go-icp/Go-ICP_V1.3.zip).

### Terminology

Data points: points of the source point set to be transformed.

Model points: points of the target point set.

### Running

Run the compiled binary with following parameters: \<MODEL FILENAME\> \<DATA FILENAME\> \<NUM DOWNSAMPLED DATA POINTS\> \<CONFIGURATION FILENAME\> \<OUTPUT FILENAME\>, e.g. “./GoICP model data 1000 config output”, “GoICP.exe model.txt data.txt
500 or 1000 config.txt output.txt”.

* \<MODEL FILENAME\> and \<DATA FILENAME\> are the point files of the model and data pointsets respectively. Each point file is in plain text format. It begins with a positive point number N in the first line, followed with N lines of X, Y, Z values of the N points.

* \<NUM DOWNSAMPLED DATA POINTS\> indicates the number of down-sampled data points. The code assumes the input data points are randomly ordered and uses the first \<NUM DOWNSAMPLED DATA POINTS\> data points for registration. ___Make sure you randomly permute your data points or change the code for some other sampling strategies.___

* \<CONFIGURATION FILENAME\> is the configuration file containing parameters for the algorithm, e.g. initial rotation and translation cubes, convergence threshold and trimming percentage. See “config_example.txt” for example.
  
* \<OUTPUT FILENAME\> is the output file containing registration results. By default it contains the obtained 3x3 rotation matrix and 3x1 translation vector only. You can adapt the code to output other results as you wish.

### Acknowledgments

This implementation uses the nanoflann library, and a simple matrix library written by Andreas Geiger. The distance transform implementation is adapted from the code of Alexander Vasilevskiy.


