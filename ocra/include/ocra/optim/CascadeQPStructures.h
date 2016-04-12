/** @file CascadeQPStructures.h
* @brief Declaration file of the structures needed in CascadeQP.
*
*   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
*
* @author Yacoubi Salim
* @author Escande Adrien
*	@date 09/07/24
*/

#ifndef _OCRABASE_CASCADE_QP_STRUCTURES_H_
#define _OCRABASE_CASCADE_QP_STRUCTURES_H_

#include "ocra/MathTypes.h"
#include "ocra/utilities.h"
#include <iostream>

/** @namespace ocra
* @brief Optimization-based Robot Controller namespace.
*  a library of classes to write and solve optimization problems dedicated to
*  the control of multi-body systems.
*/
namespace ocra
{
  struct HierarchyLevel
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(ocra::HierarchyLevel)
    Eigen::MatrixXd A ;
    Eigen::VectorXd b ;
    Eigen::MatrixXd C ;
    Eigen::VectorXd d ;
  };

  struct HierarchyLevel_barre
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(HierarchyLevel_barre)
    Eigen::MatrixXd A_barre ;
    Eigen::VectorXd b_barre ;
    Eigen::MatrixXd C_barre ;
    Eigen::VectorXd d_barre ;
  };


  struct MatrixPQ
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(MatrixPQ)
    Eigen::MatrixXd  P ;
    Eigen::VectorXd  q ;
  };

  struct EqualitiesConstraints
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(EqualitiesConstraints)
    Eigen::MatrixXd  M ;
    Eigen::VectorXd  n ;
  };

  struct InequalitiesConstraints
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(InequalitiesConstraints)
    Eigen::MatrixXd  R ;
    Eigen::VectorXd  s ;
  };

  struct Solution
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(Solution)
    Eigen::VectorXd  y ;
  };

  struct FinalSolution
  {
    DEFINE_CLASS_POINTER_TYPEDEFS(FinalSolution)
    Eigen::VectorXd  yf ;
    int r;
  };

  std::ostream & operator<<(std::ostream &out, const HierarchyLevel& h );
  std::ostream & operator<<(std::ostream &out, const HierarchyLevel_barre& h );
  std::ostream & operator<<(std::ostream &out, const MatrixPQ& m );
  std::ostream & operator<<(std::ostream &out, const EqualitiesConstraints& e );
  std::ostream & operator<<(std::ostream &out, const InequalitiesConstraints& ie );
  std::ostream & operator<<(std::ostream &out, const FinalSolution& sf );
}

#endif	//_OCRABASE_CASCADE_QP_STRUCTURES_H_
