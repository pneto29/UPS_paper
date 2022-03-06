# UPS registration #

In this repository we have the UPS registration algorithm executable.

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.

* Linux Ubuntu 16.04 or 18.04
* chmod a+x execute_poly 

## Parameters ##
* argv[1]: target cloud
* argv[2]: source cloud
* argv[3]: theta value on the x axis to determine micro misalignment
* argv[4]: theta value on the y axis to determine micro misalignment
* argv[5]: theta value on the z axis to determine micro misalignment
* argv[6]: k_max
* argv[7]: k_min
* argv[8]: tinternal ICP (point to point) iterations
* argv[9]: save file with results (.txt)


## Values of parameters (Suggestion)

* argv[1]: tbunny_0.pcd
* argv[2]: bunny_45.pcd
* argv[3]: 0.043
* argv[4]: 0.043
* argv[5]: 0.043
* argv[6]: 2000
* argv[7]: 1000
* argv[8]: 30
* argv[9]: bunny_results.txt
