
wLQP-Control
======================

This library is a c++ package to control robotic systems in dynamics, based on
the thesis of Joseph Salini (http://hal.archives-ouvertes.fr/tel-00710013/).
It lies on the ocra framework to define the control problem.

dependencies
------------

It depends on:

* Eigen3 (with unsupported element: Lgsm)
* quadprog
* orc_framework


installation
------------

To install this library, use cmake (or ccmake for gui)::

  mkdir build
  cd build
  cmake .. [-DCMAKE_BUILD_TYPE=Debug] [-DCMAKE_INSTALL_PREFIX=/home/user/folder/]
  make
  [sudo] make install


examples
--------

There are 2 examples which use this framework, a 3-Translations robot and a
kuka robot (with source code for the Models). To build the executables,
use cmake as shown above (without the installation line).


documentation
-------------

To generate the documentation, you must install the file "salini_commands.sty"
in the latex path, and then run the doxygen compilation (the working directory
from which doxygen must run is in the doc/ folder)::

  doxywizard wLQP-Control.doxygen

The documentation is generated in "build/doc/"


codeblock project
-----------------

To generate a codeblock project with cmake, run::

  cmake .. -G "CodeBlocks - Unix Makefiles"


contact
-------

Sovannara Hak (hak[at]isir.upmc.fr)
Joseph Salini (salini[at]isir.upmc.fr)


