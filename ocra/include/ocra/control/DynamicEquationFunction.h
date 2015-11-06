/** @file DynamicEquationFunction.h
  * @brief Declaration file of the DynamicEquationFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/02/16
  *
  * File history:
  *  - 10/08/12: Escande Adrien - Adaptation to the new Function interface, the new Model, and switch to Eigen.
  *  - 10/12/02: Evrard Paul - Forward declaration of Model, fix memory leak.
  */

#ifndef _OCRA_DYNAMIC_EQUATION_FUNCTION_H_
#define _OCRA_DYNAMIC_EQUATION_FUNCTION_H_

#include "ocra/optim/LinearFunction.h"

namespace ocra
{
  class Model;
}

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class DynamicEquationFunction
    *	@brief %DynamicEquationFunction class.
    *	@warning None
    *  
    * M Tdot + N T + G - L tau + Jt f, where f is the force applied by the model on the environment.
    * The variable is the concatenation of Tdot, tau and f.
    */
  class DynamicEquationFunction
    : public LinearFunction
  {
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

  private: // Forbid copy
    DynamicEquationFunction(DynamicEquationFunction&);
    DynamicEquationFunction& operator= (const DynamicEquationFunction&);

  public:
    DynamicEquationFunction(const Model& model);
    ~DynamicEquationFunction();

  protected:
    void updateJacobian() const;
    void updateb()        const;

    void doUpdateInputSizeBegin();
    void doUpdateInputSizeEnd();

  private:
    void   buildA();
    static Variable& createDEVariable(const Model& model);

  protected:
    const Model&  _model;
    Variable& _q_ddot;
    Variable& _tau;
    Variable& _f;
  };
}

#endif	//_OCRA_DYNAMIC_EQUATION_FUNCTION_H_

// cmake:sourcegroup=Functions
