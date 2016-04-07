/** @file MathTypes.h
  * @brief Declaration file of the OptimizationVariable class
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@version 0.0.0
  * @date 09.04.01
  */


#ifndef _OCRA_TYPES_H_
#define _OCRA_TYPES_H_

#include <Eigen/Dense>
namespace ocra{using namespace Eigen;}

#include <cmath>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  typedef double real;

  typedef Eigen::DenseBase<MatrixXd>::ConstRowXpr  MatrixXdRow;

#define OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(TYPE) \
  OCRA_STATIC_ASSERT(TYPE::IsVectorAtCompileTime || \
                   (TYPE::RowsAtCompileTime==Eigen::Dynamic && TYPE::ColsAtCompileTime==Eigen::Dynamic), \
                    TYPE_MUST_BE_A_VECTOR_OR_A_DYNAMIC_MATRIX)

  typedef Eigen::Matrix<double,6,1>      Vector6d;
  typedef Eigen::Matrix<double,6,6>      Matrix6d;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Jacobian3d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian6d;


#ifndef M_PI
# define M_PI 3.14159265358979
#endif

  inline double deg2rad(const double deg) { return deg * M_PI / 180.; }
  inline double rad2deg(const double rad) { return rad / M_PI * 180.; }
}

#endif  //_OCRA_TYPES_H_

