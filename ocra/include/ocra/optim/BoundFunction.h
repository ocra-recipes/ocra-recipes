/** @file BoundFunction.h
  * @brief Declaration file of the BoundFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/07/20
  *
  * File history:
  *  - 10/06/23: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_BOUND_FUNCTION_H_
#define _OCRABASE_BOUND_FUNCTION_H_

#ifdef WIN32
# pragma once
#endif

// includes
#include "ocra/optim/DiagonalLinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** Enumeration of the type of bounds. Value in the enum are important, they are used to perform computations.*/
  enum eBoundType
  {
    BOUND_TYPE_INFERIOR = -1,
    BOUND_TYPE_SUPERIOR = 1
  };


  /** @class BoundFunction
    *	@brief %BoundFunction class.
    *	@warning None
    *  
    * Function to describe bound constraints \f$ x<x^+ \f$ or \f$ x>x^- \f$ so that the constraint can be written
    * Ax+b<=0
    */
  class BoundFunction : public DiagonalLinearFunction
  {

    // ------------------------ structures --------------------------------------
  public:
    typedef DiagonalLinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of BoundFunction instances is forbidden*/
    //@{ 
    BoundFunction(const BoundFunction&);
    BoundFunction& operator= (const BoundFunction&);
    //@}

  public:
    /** Constructors
      *
      * \param [in] x The variable on which to build the function.
      * \param [in] bound A vector or a double used to build a constant vector, representing bounds on the variable
      * \param [in] the type of the bound
      */
    //@{
    BoundFunction(Variable& x, const VectorXd& bound, eBoundType type);
    BoundFunction(Variable& x, const double bound, eBoundType type);
    //@}

    // ------------------------ public interface --------------------------------
  public:
    /** Change all the bounds to the given value(s)*/
    //@{
    void changeBounds(const double bound);
    void changeBounds(const VectorXd& bounds);
    //@}

    /** Change the ith bound to the given value*/
    void changeIthBound(const int i, const double bound);


    // ------------------------ protected methods -------------------------------
  protected:
    /** Overloads of DiagonalLinearFunction methods. All throw a runtime_error: diagonal value are fixed for a
      * BoundFunction and default value cannot be changed.
      */
    //@{
    void doChangeDiagonal(const VectorXd& d);
    void doChangeDiagonal(const double diagonalElementValue, const bool changeDefault = true);
    void doChangeDefaultDiagonalValue(const double v);
    void doChangeDefaultbValue(const double v);
    //@}
  };
}

#endif	//_OCRABASE_BOUND_FUNCTION_H_

// cmake:sourcegroup=Function
