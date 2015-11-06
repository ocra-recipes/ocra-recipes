/** @file SquaredLinearFunction.h
  * @brief Declaration file of the SquaredLinearFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  * @author Brisset Julien
  *	@date 09/06/16
  *
  * File history:
  *  - 10/07/05: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_SQUARED_LINEAR_FUNCTION_H_
#define _OCRABASE_SQUARED_LINEAR_FUNCTION_H_

// includes
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/QuadraticFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class SquaredLinearFunction
    *	@brief %SquaredLinearFunction class.
    *	@warning None
    *  
    * Quadratic function of the form \f& 1/2 \left\|f(x)\right\|^2 \f& with f(x) = Ax+b
    * If A and b are constant, better use QuadraticFunction directly.
    *
    * //TODO [todo] : optimize computations
    */
  class SquaredLinearFunction : public QuadraticFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef QuadraticFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    SquaredLinearFunction(const SquaredLinearFunction&);
    SquaredLinearFunction& operator= (const SquaredLinearFunction&);
  protected:
  public:
    SquaredLinearFunction(LinearFunction* f);
    ~SquaredLinearFunction();

    void changeWeight(const VectorXd& weight);
    //void setWeight(const VectorXd& weight);

    LinearFunction& getFunction();
    const LinearFunction& getFunction() const;

    // ------------------------ protected methods -------------------------------
  protected:
    void updateHessian() const;

    void updateq() const;
    void updater() const;

    void doUpdateInputSizeBegin();

    void onResize(int);

    // ------------------------ protected members -------------------------------
  protected:
    LinearFunction* _f;
    VectorXd _weight;
  };


  void testSquaredLinearFunction();
}

#endif	//_OCRABASE_SQUARED_LINEAR_FUNCTION_H_

// cmake:sourcegroup=Function

