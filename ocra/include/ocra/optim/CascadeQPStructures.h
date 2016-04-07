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

// includes
#include "ocra/MathTypes.h"
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
    Eigen::MatrixXd A ;
    Eigen::VectorXd b ;
    Eigen::MatrixXd C ;
    Eigen::VectorXd d ;
  };

  struct HierarchyLevel_barre
  {
    Eigen::MatrixXd A_barre ;
    Eigen::VectorXd b_barre ;
    Eigen::MatrixXd C_barre ;
    Eigen::VectorXd d_barre ;
  };


  struct MatrixPQ
  {
    Eigen::MatrixXd  P ;
    Eigen::VectorXd  q ;
  };

  struct EqualitiesConstraints
  {
    Eigen::MatrixXd  M ;
    Eigen::VectorXd  n ;
  };

  struct InequalitiesConstraints
  {
    Eigen::MatrixXd  R ;
    Eigen::VectorXd  s ;
  };

  struct Solution
  {
    Eigen::VectorXd  y ;
  };

  struct FinalSolution
  {
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


// cmake:sourcegroup=toBeUpdated
