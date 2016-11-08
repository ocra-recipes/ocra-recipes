# ocra-recipes


#### Build Status
| master | dev |
|:------:|:---:|
| [![Build Status](https://travis-ci.org/ocra-recipes/ocra-recipes.svg?branch=master)](https://travis-ci.org/ocra-recipes/ocra-recipes) |  [![Build Status](https://travis-ci.org/ocra-recipes/ocra-recipes.svg?branch=dev)](https://travis-ci.org/ocra-recipes/ocra-recipes) |


[Installation instructions](#Installation)

## Code Structure
Give a description...

- ocra
- wocra
- gocra
- hocra (coming soon)
- solvers
  - quadprog
  - qpoases

### ocra

(O.C.R.A.) Optimization-based Control for Robotics Applications

### wocra

(W.O.C.R.A.) Weighted Optimization-based Control for Robotics Applications

### hocra

(H.O.C.R.A.) Hierarchical Optimization-based Control for Robotics Applications

### solvers

Ultimately our goal is to implement the most recent convex solvers so we can mix and match control problem formulations with different solver algorithms. Right now we have only implemented a slightly modified version of QuadProg.

#### quadprog

This library is a QP (Quadratic Program) based on the QuadProg++ project (http://quadprog.sourceforge.net/) which has been slightly modified. In this version, vector and matrix classes are replaced by Eigen classes, in order to use the same definitions as the ocra libraries.



#### qpoases

Exploits the open-source C++ library qpOASES, which is an implementation of the recently proposed online active set strategy, which was inspired by important observations from the field of parametric quadratic programming (QP). It has several theoretical features that make it particularly suited for model predictive control (MPC) applications but also as a QP solver. [(ref)](https://projects.coin-or.org/qpOASES)


## Dependencies
- Boost (`filesystem`)
- Eigen 3.2.0 (*note:* We have issues with later versions of eigen so please do not use the current build - install via apt-get.)
- [EigenLgsm](https://github.com/ocra-recipes/eigen_lgsm)
- TinyXML
- YARP

**Boost, Eigen 3.2.0 & TinyXML**
```bash
sudo apt-get install libboost-dev libtinyxml-dev libeigen3-dev
```
[**EigenLgsm**](https://github.com/ocra-recipes/eigen_lgsm)
```
git clone https://github.com/ocra-recipes/eigen_lgsm.git
cd eigen_lgsm
mkdir build
cd build
cmake ..
sudo make install
```

[**YARP**]( http://www.yarp.it)

Please follow the instructions here: http://www.yarp.it/install_yarp_linux.html


## Installation

**WARNING**
*This is an experimental set of libs and there are no guarantees that they will not do damage to your computer. We take no responsibility for what happens if you install them. That being said, if you follow these instructions you should be fine.*

Okay that's out of the way... phew!

**Install to /usr/local**
```
git clone https://github.com/ocra-recipes/ocra-recipes.git
cd ocra-recipes
mkdir build
cd build
cmake ..
sudo make install
```

**Install to custom location (example: /home/user/Install)**
```
git clone https://github.com/ocra-recipes/ocra-recipes.git
cd ocra-recipes
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=home/user/Install ..
make install
```
*Note:* If you do the install this way then you must update your environment variables in your `.bashrc` file.

**In `.bashrc`**
```shell
# ocra-recipes install
export OCRA_INSTALL=/home/user/Install
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OCRA_INSTALL}/lib
export LIBRARY_PATH=${LIBRARY_PATH}:${OCRA_INSTALL}/lib
export PATH=${PATH}:${OCRA_INSTALL}/bin
```

### Tested OS's

- [x] Ubuntu 12.04
- [x] Ubuntu 14.04
- [x] Debian 7
- [x] OS X


In theory any linux distro should work if the dependencies are met and don't conflict with any system libs/headers. If you manage to build, install and use OCRA in any other platform please let us know and we can add it to the list with any helpful notes you provide along with it.


### Enjoying your OCRA...
Well now that you have `ocra-core` up and running, you probably want to try it out n'est pas? Well mosey on over to the [ocra-wbi-plugins](https://github.com/ocra-recipes/ocra-wbi-plugins) repo and follow the instructions.

Want to contribute? Maybe build a plugin or two? Read the [Contributing  section](#Contributing) for details on how to interface with OCRA and use it for world domination.

### Notes about OS X
:warning: As a recent feature we added rpath support, therefore, by default the flag `OCRA_ICUB_ENABLE_RPATH` is `OFF`. Change it by configuring `ocra-wbi-plugins` as: `cmake -DOCRA_ICUB_ENABLE_PATH=ON ./`. Once we're sure this ''always'' works it will be `ON` by default. Therefore, if you encounter error messages such as:

```bash
dyld library not loaded @rpath/libYarpMath.dylib
```

Most likely this variable is still `OFF`.

:warning: When using XCode to debug your code, make sure you change your LLVM C++ Language settings manually to C++11, by heading to the `Build Settings` of your project, searching for `C++ Language Dialect` and changing it to `C++11 [-std=c++11]`


### Contributing

Give a description...

### Generating the documentation

To build the documentation for `orca-recipes` you will need [`doxygen`](http://www.stack.nl/~dimitri/doxygen/index.html). To install `doxygen` on linux run `sudo apt-get install doxygen`.

In your `build/` directory (if you have already run `cmake ..` simply run:
```
make doc
```
**
If you run this right now you must ignore the latex errors due to the missing `.sty` files by just holding down `enter` until they are past.
**

HTML and LaTeX files will be generated in the `build/docs/` directory. Open the file, `index.html`.

###Authors

####current developers

 - [Ryan Lober](https://github.com/rlober)
 - [Antoine Hoarau](https://github.com/ahoarau)
 - [Jorhabib Eljaik](https://github.com/jeljaik)
 - [Silvio Traversaro](https://github.com/traversaro)


####past developers

 - [Darwin Lau](https://github.com/darwinlau)
 - [Mingxing Liu](https://github.com/mingxing-liu)
 - [Joseph Salini](https://github.com/salini)
 - [Hak Sovannara](https://github.com/sovannara-hak)
