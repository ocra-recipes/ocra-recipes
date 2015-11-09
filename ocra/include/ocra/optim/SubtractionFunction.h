/** @file SubtractionFunction.h
  * @brief Declaration file of the SubtractionFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/27
  */

#ifndef _OCRABASE_SUBTRACTION_FUNCTION_H_
#define _OCRABASE_SUBTRACTION_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"


/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class SubtractionFunction
    *	@brief %SubtractionFunction class.
    *	@warning None
    *  
    * Subtraction between two functions, or a function and a constant
    */
  class SubtractionFunction: public Function
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
    SubtractionFunction();
    SubtractionFunction(SubtractionFunction&);
  protected:
  public:
	//TODO: implement
    //SubtractionFunction(Function* f1, Function* f2);
    SubtractionFunction(Function* f, const Vector& v);
    SubtractionFunction(const Vector& v, Function* f);
    virtual ~SubtractionFunction();

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
    int       _case;
    Function* _f1;
    Function* _f2;
    Vector    _v1;
    Vector    _v2;  //< -v2 in fact

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRABASE_SUBTRACTION_FUNCTION_H_

// cmake:sourcegroup=toBeUpdated
