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

/** @namespace ocra
* @brief Optimization-based Robot Controller namespace. 
*  a library of classes to write and solve optimization problems dedicated to
*  the control of multi-body systems. 
*/
namespace ocra
{
  struct HierarchyLevel
  {
    Matrix A ;
    Vector b ;
    Matrix C ;
    Vector d ;
  };

  struct HierarchyLevel_barre
  {
    Matrix A_barre ;
    Vector b_barre ;
    Matrix C_barre ;
    Vector d_barre ;
  };


  struct MatrixPQ
  {
    Matrix  P ;
    Vector  q ;
  };

  struct EqualitiesConstraints
  {
    Matrix  M ;
    Vector  n ;
  };

  struct InequalitiesConstraints
  {
    Matrix  R ;
    Vector  s ;
  };

  struct Solution
  {
    Vector  y ;
  };

  struct FinalSolution
  {
    Vector  yf ;
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
