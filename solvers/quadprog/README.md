
quadprog
========

This library is a LQP (Linear Quadratic Program) based on the QuadProg++ project
(http://quadprog.sourceforge.net/) which has been slightly modified. In this
version, vector and matrix classes are replaced by Eigen classes, in order to
use the same definitions as the orc framework libraries.


dependencies
------------

It depends on:

* Eigen3 (with unsupported element: Lgsm)


installation
------------

To install this library, use cmake (or ccmake for gui)::

  mkdir build
  cd build
  cmake .. [-DCMAKE_BUILD_TYPE=Debug] [-DCMAKE_INSTALL_PREFIX=/home/user/folder/]
  make
  [sudo] make install


contact
-------

Sovannara Hak (hak[at]isir.upmc.fr)
Joseph Salini (salini[at]isir.upmc.fr)
