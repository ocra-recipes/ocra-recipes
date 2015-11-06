/** @file CoMFunction.h
  * @brief Declaration file of the CoMFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Brisset Julien
  * @author Escande Adrien
  *	@date 09/05/26
  */

#ifndef _OCRACONTROL_COM_FUNCTION_H_
#define _OCRACONTROL_COM_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"
#include "ocra/control/Robot.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class CoMFunction
    *	@brief %CoMFunction class.
    *	@warning None
    *  
    * Description
    */
  class CoMFunction: public Function
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
    CoMFunction(CoMFunction&);
    CoMFunction& operator=(CoMFunction&);
  protected:
  public:
    CoMFunction(Robot* robot);

    // ------------------------ public interface --------------------------------
  public:

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

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    Robot* _robot;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRACONTROL_COM_FUNCTION_H_

// cmake:sourcegroup=Functions
