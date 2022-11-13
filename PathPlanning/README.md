BIT*-based path planning 
=======================================  
The Path Planning the OUTDOOR environment. The code is based on the Batch infromed trees and OMPL(Open Motion Planning Library). This is the [OMPL  page](https://ompl.kavrakilab.org/core/installation.html)


## Based on BIT* (Batch informed tree)
**[BITstar](https://arxiv.org/pdf/1405.5848.pdf) Authors** : Jonathan D. Gammell
, Siddhartha S. Srinivasa
, and Timothy D. Barfoot


Dependencies:

* [Boost](https://www.boost.org) (version 1.58 or higher)
* [CMake](https://www.cmake.org) (version 3.5 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)


📤 Installation  

Once dependencies are installed, you can build this.  
Go to the top-level directory of this folder and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    make
    ./bin/demo_tbit