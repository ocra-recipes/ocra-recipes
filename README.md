### O.C.R.A.
#### Optimization-based Control for Robotics Applications

Give a description...

[Installation instructions](#Build & Install)

# ocra-core

Give a description...

### Code Structure
Give a description...

- core-framework
- wLQP-Control
  - wocra
  - quadprog

### core-framework

Give a description...

### wLQP-Control

Give a description...

# Installation

Give a description...

### Tested OS's

- [x] Ubuntu 12.04
- [x] Ubuntu 14.04
- [x] Debian 7


In theory any linux distro should work if the dependencies are met and don't conflict with any system libs/headers. If you manage to build, install and use OCRA in any other platform please let us know and we can add it to the list with any helpful notes you provide along with it.

### Install directory structure
For the rest of these instructions we are going to install ocra-core (among other things) to a special directory called `ocra-install-dir` which will be in our home folder. We start in the home directory of user, `/home/{userName}`. Remember throughout to replace `{userName}`, with your user name and no curly braces.
```
cd
mkdir ocra-install-dir
cd ocra-install-dir
mkdir -p include/eigen3_0_5 lib/pkgconfig
```
We need to set up the environment variables to point to this install directory.
```
nano ~/.bashrc
```
We are now in the nano text editor. Scroll down to the bottom and write the following lines:
```
export OCRA_INSTALL=/home/{userName}/ocra-install-dir
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${OCRA_INSTALL}/lib/pkgconfig
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OCRA_INSTALL}/lib
```
Press `ctrl + x`, answer `Y` to accept the changes and `enter`.

Now we have to source the `.bashrc` file to update the environment variables.
```
source ~/.bashrc
```
Ok, on to the good stuff!


### Dependencies

`ocra-core` depends on Boost along with Eigen 3.0 and its unsupported template library LGSM.

#### Boost
If you are in linux and have `apt-get` you can install via:
```bash
sudo apt-get install libboost-dev
```
Easy-peasy.

#### Eigen

Unfortunately `ocra-core` is incompatible with the current version, 3.2+, of Eigen. Specifically, the biggest issue is the use of `<Ref>` for passing Eigen objects as function arguments. Usage of `<Ref>` in any code linking to the `ocra-core` libs will break your build. As you can imagine this is somewhat of a sore point for us since we can't use all of the neat toys in Eigen 3.2+ but we are working on upgrading `ocra-core`. However, this process will be long so have patience grasshopper.  

For now we must content ourselves with the following **hack:**

We have put a working copy of Eigen 3.0.5 in the git repository: https://github.com/ocra-recipes/eigen_3_0_5.git. Go ahead and clone it.

```git
git clone https://github.com/ocra-recipes/eigen_3_0_5.git
cd eigen_3_0_5
```

We need to copy the eigen headers to the `/ocra-install-dir/include/eigen3_0_5` directory.
```
cp Eigen/ unsupported/ ~/ocra-install-dir/include/eigen3_0_5
```
Finally we need to copy the pkg-config file.
```
cp eigen3.pc ~/ocra-install-dir/lib/pkgconfig
```
We need to customize this `.pc` file so open it in nano:
```
nano ~/ocra-install-dir/lib/pkgconfig/eigen3.pc
```
The file should look like this,
```pkgconfig
Name: Eigen3
Description: A C++ template library for linear algebra: vectors, matrices, and related algorithms
Requires:
Version: 3.0.5
Libs:
Cflags: -I$EIGEN_ROOT -I$EIGEN_ROOT/unsupported
```
Where you see the variables `$EIGEN_ROOT` replace them with the path to `eigen3_0_5`
```
Name: Eigen3
Description: A C++ template library for linear algebra: vectors, matrices, and related algorithms
Requires:
Version: 3.0.5
Libs:
Cflags: -I/home/{userName}/ocra-install-dir/include/eigen3_0_5 -I/home/{userName}/ocra-install-dir/include/eigen3_0_5/unsupported
```
Don't forget to replace `{userName}` with your user name.

Press `ctrl + x`, answer `Y` to accept the changes and `enter`.


To make sure everything went ok, run the following command in a terminal
```
pkg-config eigen3 --modversion --cflags
```
If you don't get the following output,
```
3.0.5
-I/home/{userName}/Install/include/eigen3_0_5 -I/home/{userName}/Install/include/eigen3_0_5/unsupported  
```
Then something has gone terribly wrong! However, if you get this then you are all set for the real deal: ocra!

### Build & Install
**WARNING**
*This is an experimental set of libs and there are no guarantees that they will not do damage to your computer. We take no responsibility for what happens if you install them. That being said, if you follow these instructions you should be fine.*

Okay that's out of the way... phew!

So you have your dependencies installed and your package config files ready to go. Let's run through the whole process step by step.

First we clone the repo.
```bash
git clone https://github.com/ocra-recipes/ocra-core.git
cd ocra-core/
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/ocra-install-dir ..
```

**NOTE:** If you do not set the install prefix then the default `/usr/local/` will be used. Please don't do this!

Okay, let's build/install the darn thing already!
```bash
make install
```

That's it! The core framework is now built, installed and ready to rock!


### Enjoying your OCRA...
Well now that you have `ocra-core` up and running, you probably want to try it out n'est pas? Well mosey on over to the [ocra-wbi-plugins](https://github.com/ocra-recipes/ocra-wbi-plugins) repo and follow the instructions.

Want to contribute? Maybe build a plugin or two? Read the [Contributing  section](#Contributing) for details on how to interface with OCRA and use it for world domination.

### Contributing

Give a description...
