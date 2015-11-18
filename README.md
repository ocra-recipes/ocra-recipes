# ocra-recipes

Badass shit...

[Installation instructions](#Installation)

## Code Structure
Give a description...

- ocra
- wocra
- gocra
- hocra (coming soon)
- solvers
  - quadprog
  - qpoases (coming soon)

### ocra

(O.C.R.A.) Optimization-based Control for Robotics Applications

### wocra

(W.O.C.R.A.) Weighted Optimization-based Control for Robotics Applications

### hocra

(H.O.C.R.A.) Hierarchical Optimization-based Control for Robotics Applications

### solvers

#### quadprog
#### qpoases

## Dependencies
- Boost
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


In theory any linux distro should work if the dependencies are met and don't conflict with any system libs/headers. If you manage to build, install and use OCRA in any other platform please let us know and we can add it to the list with any helpful notes you provide along with it.


### Enjoying your OCRA...
Well now that you have `ocra-core` up and running, you probably want to try it out n'est pas? Well mosey on over to the [ocra-wbi-plugins](https://github.com/ocra-recipes/ocra-wbi-plugins) repo and follow the instructions.

Want to contribute? Maybe build a plugin or two? Read the [Contributing  section](#Contributing) for details on how to interface with OCRA and use it for world domination.

### Contributing

Give a description...
