/** @file ControlEquationFunction.h
  * @brief Declaration file of the ControlEquationFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/07/28
  */

#ifndef _OCRACONTROL_CONTROL_EQUATION_FUNCTION_H_
#define _OCRACONTROL_CONTROL_EQUATION_FUNCTION_H_

#ifdef WIN32
# pragma once
#endif


#include "ocra/optim/LinearFunction.h"
#include "ocra/control/Model.h"

namespace ocra
{
  /** @class ControlEquationFunction
    *	@brief %ControlEquationFunction class.
    *	@warning None
    *  
    * For a given robot model with generalized speed T, inertia matrix M and non-linear terms N, and a given function
    * f, the ControlEquationFunction g is the linear function of T_dot g(T_dot) = MT_dot + N - f(x) with f(x) the value 
    * of f at the instant g is evaluated. (x is not considered as a variable of g).
    * N can be ignored by disabling its use.
    */
  class ControlEquationFunction : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  
    // ------------------------ constructors ------------------------------------
  private:
    /** copy of ControlEquationFunction instances is forbidden*/
    //@{
    ControlEquationFunction(const ControlEquationFunction&);
    ControlEquationFunction& operator= (const ControlEquationFunction&);
    //@}

  public:
    ControlEquationFunction(Model& model, const Function& f, bool useNLTerms=true);
    ~ControlEquationFunction();

    // ------------------------ public interface --------------------------------
  public:
    bool isUsingNLTerms() const;

    const Model&    getModel()    const;
    Model&          getModel();
    const Function& getFunction() const;

    // ------------------------ protected methods -------------------------------
  protected:
    /** Overloads of the LinearFunction methods*/
    //@{
    void updateJacobian() const;
    void updateb() const;
    void doChangeA(const MatrixXd& A);
    void doChangeb(const VectorXd& b);
    //@}

    // ------------------------ private members ---------------------------------
  private:
    const bool      _useNLTerms;
    Model&          _model;
    const Function& _f;
  };
}


#endif //_OCRACONTROL_CONTROL_EQUATION_FUNCTION_H_

// cmake:sourcegroup=Functions
