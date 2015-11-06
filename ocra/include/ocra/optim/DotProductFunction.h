/** @file DotProductFunction.h
  * @brief Declaration file of the DotProductFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/05/26
  */

#ifndef _OCRABASE_SCALAR_PRODUCT_FUNCTION_H_
#define _OCRABASE_SCALAR_PRODUCT_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class DotProductFunction
    *	@brief %DotProductFunction class.
    *	@warning None
    *  
    * scalar product between two vectors that can either be Function output or contant
    */
  class DotProductFunction: public Function
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Function  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    DotProductFunction();
    DotProductFunction(DotProductFunction&);
  protected:
  public:
    DotProductFunction(Function* f1, Function* f2);
    DotProductFunction(Function* f, const Vector& v);

    // ------------------------ public interface --------------------------------
  public:
    void changeV(const Vector& v);

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void computeValue(void)  const;
    virtual void computeGradient(void)  const;
    virtual void computeHessian(void)  const;
    virtual void computeJdot(void) const;
    virtual void computeJdotXdot(void) const;

    virtual void doUpdateSize(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    void  initHessian();

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    bool      _twoFunctions;
    Function* _f1;
    Function* _f2;
    Vector    _v;
    mutable Vector    _tmp;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRABASE_SCALAR_PRODUCT_FUNCTION_H_

// cmake:sourcegroup=toBeUpdated

