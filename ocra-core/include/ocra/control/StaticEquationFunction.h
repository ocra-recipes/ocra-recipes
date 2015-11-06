/** @file StaticEquationFunction.h
  * @brief Declaration file of the StaticEquationFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/22
  */

#ifndef _OCRACONTROL_STATIC_EQUATION_FUNCTION_H_
#define _OCRACONTROL_STATIC_EQUATION_FUNCTION_H_

// includes
#include "ocra/optim/LinearFunction.h"
#include "ocra/control/Robot.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class StaticEquationFunction
    *	@brief %StaticEquationFunction class.
    *	@warning None
    *  
    * Classical static equation with the form 
    */
  class StaticEquationFunction: public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    StaticEquationFunction();
    StaticEquationFunction(StaticEquationFunction&);
  protected:
  public:
    StaticEquationFunction(Robot* robot);

    // ------------------------ public interface --------------------------------
  public:
    virtual const Matrix& getA(void) const;
    virtual const Vector& getb(void) const;

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void computeValue(void)  const;
    virtual void computeGradient(void)  const;
 //   virtual void computeHessian(void)  const;

    virtual void doUpdateSize(void);
    virtual void doUpdateSizeEnd(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:
    static Variable& createSEVariable(Robot* robot);

    // ------------------------ protected members -------------------------------
  protected:
    Robot*  _robot;
    Variable* _q_dot;
    Variable* _tau;
    Variable* _f;
    //Matrix    _KL;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRACONTROL_STATIC_EQUATION_FUNCTION_H_

// cmake:sourcegroup=Functions
