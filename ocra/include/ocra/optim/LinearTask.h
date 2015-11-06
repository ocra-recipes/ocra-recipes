/** @file LinearTask.h
  * @brief Declaration file of the LinearTask class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/24
  *
  * File history:
  *  - 10/07/01: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_LINEAR_TASK_H_
#define _OCRABASE_LINEAR_TASK_H_

//ocra includes
#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/DiagonalLinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  /** @class LinearTask
    *	@brief %LinearTask class.
    *	@warning None
    *  
    * Derives a derivable error function \f$ f \f$ on \f$ x \f$ into a linear function
    * on \f$ x \f$, \f$ \frac{\partial f}{\partial x} \dot{x} + L(f(x)) \f$ with \f$ L \f$ 
    * a function.
    * Classical function \f$ L \f$ is \f$ L(y) = \Lambda y \f$ with \f$ \Lambda \f$ a
    * diagonal matrix of gains.
    */
  class LinearTask : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    LinearTask(const LinearTask&);
    LinearTask& operator= (const LinearTask&);
  protected:
  public:
    LinearTask(Function& f, Function& L);
    ~LinearTask();

    // ------------------------ public interface --------------------------------
  public:
    Function& getL();
    const Function& getL() const;
    Function& getf();
    const Function& getf() const;

    // ------------------------ public static methods ---------------------------
  public:
    static LinearTask* createEqualityTaskWithLinearLaw(Function* f, double weight);
    static LinearTask* createEqualityTaskWithLinearLaw(Function* f, const VectorXd& weight);
    //generate the task function for f <= 0
    //for component j the task value is f_dot-weight*f/fi. weight is thus the maximum speed
    //of f when f==fi when the task is used for an inequality constraint. fi should be negative.
    static LinearTask* createInequalityTaskWithLinearLaw(Function* f, double weight, double fi);
    static LinearTask* createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weight, real fi);
    static LinearTask* createInequalityTaskWithLinearLaw(Function* f, real weight, const VectorXd& fi);
    static LinearTask* createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weight, const VectorXd& fi);
    static LinearTask* createInequalityTaskWithLinearLaw(Function* f, const VectorXd& weightDividedByFi);


    // ------------------------ protected methods -------------------------------
  protected:
    void updateJacobian() const;
    void updateb() const;

    void doUpdateInputSizeBegin();

    void doUpdateDimensionBegin(int newDimension);
    void doUpdateDimensionEnd(int oldDimension);

    // ------------------------ private methods ---------------------------------
  private:
    void updateDimension(int timestamp);

    // ------------------------ protected members -------------------------------
  protected:
    Function* _f;
    Function* _L;
  };


  inline Function& LinearTask::getL()
  {
    return *_L;
  }

  inline const Function& LinearTask::getL() const
  {
    return *_L;
  }

  inline const Function& LinearTask::getf() const
  {
    return *_f;
  }

  inline Function& LinearTask::getf() 
  {
    return *_f;
  }
  




  void testLinearTask();
}

#endif	//_OCRABASE_LINEAR_TASK_H_

// cmake:sourcegroup=Function

