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

//#include "base/Types.h"    //TODO::: WARNING: we comment that, we should not!!!!



//#include "cml/BlasOperations.h"
//#include "cml/Rotation3D.h"
//#include "cml/TinyVector.h"
//#include "cml/Displacement.h"
//#include "cml/DenseVector.h"
//#include "cml/SubDenseVector.h"
//#include "cml/DenseVectorWrap.h"
//#include "cml/ColumnMatrix.h"
//#include "cml/TinyMatrix.h"
//#include "cml/DenseMatrix.h"
//#include "cml/SubDenseMatrix.h"
//#include "cml/DenseMatrixWrap.h"
//#include "cml/Twist.h"
//#include "cml/Wrench.h"
#include <Eigen/Eigen>
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

//#define OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(TYPE) \
//  OCRA_STATIC_ASSERT(TYPE::IsVectorAtCompileTime || (true), TYPE_MUST_BE_A_VECTOR_OR_A_DYNAMIC_MATRIX)
 

  //typedef VectorXd                   Vector;  
  //typedef MatrixXd                   Matrix;
  //typedef xde::cmlDenseVectorBase<real>  VectorBase;  
  //typedef xde::cmlDenseMatrixBase<real>  MatrixBase;
  //typedef xde::cmlSubDenseVector<real>   SubVector;
  //typedef xde::cmlSubDenseMatrix<real>   SubMatrix;
  //typedef xde::cmlDenseVectorWrap<real>  VectorWrap;
  //typedef xde::cmlDenseMatrixWrap<real>  MatrixWrap;
  //typedef xde::cmlColumnMatrixBase<real> ColumnMatrixBase;
  //typedef xde::cmlColumnMatrix<real>     ColumnMatrix;
  //typedef xde::cmlQuaternion             Quaternion;
  //typedef Vector3d                    Vector3;
  //typedef Matrix<real,3,3>               Matrix33;
  //typedef xde::cmlTinyMatrix<6,3,real>   Matrix63;
  typedef Eigen::Matrix<double,6,1>      Vector6d;
  typedef Eigen::Matrix<double,6,6>      Matrix6d;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Jacobian3d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian6d;
  //typedef xde::cmlDisplacementBase       DisplacementBase;
  //typedef xde::cmlDisplacement           Displacement;
  //typedef xde::cmlTwistBase              TwistBase;
  //typedef xde::cmlTwist                  Twist;
  //typedef xde::cmlWrench                 Wrench;

  
  
  //typedef xde::base::uint   cfl_size_t;   //TODO: we comment that, we should not!!!!




#ifndef M_PI
# define M_PI 3.14159265358979
#endif

  inline double deg2rad(const double deg) { return deg * M_PI / 180.; }
  inline double rad2deg(const double rad) { return rad / M_PI * 180.; }
}

#endif  //_OCRA_TYPES_H_

