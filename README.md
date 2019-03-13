## Camera-LIDAR Bimodal Lane Detection, Recognition and Tracking

This code is based on the paper by Binbin Li, Dezhen Song, Haifeng Li,  Adam Pike, and Paul Carlson,  "Lane Marking Quality Assessment for Autonomous Driving", IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, October, 1-5, 2018. 

It has been tested on Ubuntu 16.04LTS and should work on 16.04 or newer versions.

## Dependencies

Below is a list of all dependencies required, and links to relevant information such as downloads.  There are also instructions for installing the required packages on a Ubuntu system, specifically, but which should also work on most Debian-based systems.
* Install prerequisites/dependencies
    
    ```
    sudo apt-get install libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
    sudo apt-get install libatlas-base-dev  libopenblas-dev
    sudo apt-get install libpcap-dev
    ```
* OpenCV 2.4.x: http://opencv.org
    ```
    sudo apt-get install libopencv-dev
    ```
   On Ubuntu-based systems, OpenCV should be installed manually from source.
       
* Matio, an open-source C library for reading and writing binary MATLAB MAT files.
   
   ```
   git clone git://git.code.sf.net/p/matio/matio
   cd matio
   git submodule update --initVPs  # for datasets used in unit tests
   ./autogen.sh
   ./configure
   make
   make check
   make install
   ```

* Pugixml, a C++ XML processing library

  ```
  git clone git: https://github.com/zeux/pugixml.git
  ```
* ALGLIB, a cross-platform numerical analysis and data processing library (http://www.alglib.net/).

* GNU Scientific Library,  a software library for numerical computations in applied mathematics and science
    ```
    sudo apt-get install libgsl-dev
    ```
    It is recommended to build from source (https://www.gnu.org/software/gsl/).
* Eigen3: http://eigen.tuxfamily.org/
   ```
   sudo apt-get install libeigen3-dev
   ```
* Download protobuf-cpp-3.5.1.zip (https://github.com/google/protobuf/releases/download/v3.5.1/protobuf-cpp-3.5.1.zip) and unzip it.
  ```
  cd $PROTOBUF_ROOT  
  ./configure  
  make  
  make check
  sudo make install
  sudo ldconfig  #refresh shared library cache.
  ```
* ICNet, real-time semantic segmentation on high-resolution images (https://hszhao.github.io/projects/icnet/).
 
     ```
     git clone --recursive https://github.com/hszhao/ICNet.git
     cd $ICNET_ROOT/PSPNet
     cp Makefile.config.example Makefile.config
     vim Makefile.config #change the configuration file for Caffee if needed
     make all
     make test
     make runtest 
     ```
Build Instruction
-----------------
This project uses CMake (http://www.cmake.org), a cross-platform build system.
* mkdir build
* cd build
* cmake ..
* make

## To run the code

1. Under the ${laneAssess_root}, run command ```./lanevaluate Parameters.xml```, where ```Parameters.xml``` is the configuration file.
2. Change the setting in ```Parameters.xml``` to visualize the result. 

## Contact

1. Binbin Li <binbinli@tamu.edu>
2. Aaron Kingery <aaronmkingery@tamu.edu>
3. Aaron Angert <adangert@tamu.edu>
4. Ankit Ramchandani <ankit61@tamu.edu>
5. Dezhen Song <dzsong@cse.tamu.edu>
   
