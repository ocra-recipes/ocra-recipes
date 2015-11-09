/** @file SecondOrderLinearTask.h
  * @brief Declaration file of the SecondOrderLinearTask class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/02/17
  *
  * File history:
  *  - 10/07/05: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_SECOND_ORDER_LINEAR_TASK_H_
#define _OCRABASE_SECOND_ORDER_LINEAR_TASK_H_

// includes
#include "ocra/optim/LinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  /** @class SecondOrderLinearTask
    *	@brief %SecondOrderLinearTask class.
    *	@warning None
    *  
    * f" = a_des where a_des = L(f,f').
    * L must then be a function over a composite variable whose two components' size is dim(f)
    */
  class SecondOrderLinearTask : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    SecondOrderLinearTask(const SecondOrderLinearTask&);
    SecondOrderLinearTask& operator= (const SecondOrderLinearTask&);
  protected:
  public:
    SecondOrderLinearTask(Function& f, Function& L);
    ~SecondOrderLinearTask();

    // ------------------------ public interface --------------------------------
  public:
    Function& getL();
    const Function& getL() const;
    Function& getf();
    const Function& getf() const;

    void   saturate(double accelerationMax) {_saturate = true; _accelerationMax = accelerationMax;}
    void   dontSaturate()  {_saturate = false;}
    double getSaturationValue(void) const {return _accelerationMax;}
    bool   isSaturated() const {return _saturate;}

    // ------------------------ public static methods ---------------------------
  public:
    //create the task J*x_ddot + J_dot*x_dot + diag(weight1)*J*x_dot + diag(weight0)*f(x)
    //weight0 and weight1 should be positive
    static SecondOrderLinearTask* createSecondOrderLinearTaskWithLinearLaw(Function* f, double weight0, double weight1);
    static SecondOrderLinearTask* createSecondOrderLinearTaskWithLinearLaw(Function* f, const VectorXd& weight0, const VectorXd& weight1);


    // ------------------------ private methods ---------------------------------
  private:
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
    bool      _saturate;
    double    _accelerationMax;

  };



  inline Function& SecondOrderLinearTask::getL(void) {return *_L;}

  inline const Function& SecondOrderLinearTask::getf(void) const {return *_f;}
}

#endif	//_OCRABASE_SECOND_ORDER_LINEAR_TASK_H_

// cmake:sourcegroup=Function

