/** @file WeightedSquaredSumOfControlEquationsFunction.h
  * @brief Declaration file of the WeightedSquaredSumOfControlEquationsFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/07/28
  */

#ifndef _OCRACONTROL_WEIGHTED_SQUARED_SUM_OF_CONTROL_EQUATIONS_FUNCTION_H_
#define _OCRACONTROL_WEIGHTED_SQUARED_SUM_OF_CONTROL_EQUATIONS_FUNCTION_H_

// includes
#include "ocra/optim/QuadraticFunction.h"
#include "ocra/control/ControlEquationFunction.h"


namespace ocra
{
  /** @class WeightedSquaredSumOfControlEquationsFunction
    *	@brief %WeightedSquaredSumOfControlEquationsFunction class.
    *	@warning None
    *  
    * This class implements the function \f& \sum_i{\left|f_i \right|^2_{W_i}} \f& where the \f& f_i \f& are
    * ControlEquationFunction instances and the \f& W_i \f& are diagonal matrices weighting the norms.
    * Computation are optimized so as to avoid performing several times the same matrix-matrix multiplications.
    */
  class WeightedSquaredSumOfControlEquationsFunction : public QuadraticFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef QuadraticFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  protected:
  private:
    class WeightAndFunction
    {
    public:
      WeightAndFunction(const ControlEquationFunction& f, const VectorXd& weight)
        :_weight(weight), _function(const_cast<ControlEquationFunction*>(&f))
      {}

      const ControlEquationFunction& getFunction() const {return *_function;}
      const VectorXd&                getWeight()   const {return _weight;}
      void  changeWeight(const VectorXd& weight) {_weight = weight;}
      bool  functionEquals(const ControlEquationFunction& f) const {return &f==_function;}
    private:
      VectorXd _weight;
      ControlEquationFunction* _function;
    };

    typedef std::vector<WeightAndFunction>::const_iterator const_fun_iterator;
    typedef std::vector<WeightAndFunction>::iterator fun_iterator;

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    WeightedSquaredSumOfControlEquationsFunction(const WeightedSquaredSumOfControlEquationsFunction&);
    WeightedSquaredSumOfControlEquationsFunction operator= (const WeightedSquaredSumOfControlEquationsFunction&);

  public:
    WeightedSquaredSumOfControlEquationsFunction(Model& model);
    WeightedSquaredSumOfControlEquationsFunction(const ControlEquationFunction& f, const VectorXd& weight);
    WeightedSquaredSumOfControlEquationsFunction(const ControlEquationFunction& f1, const VectorXd& weight1,
                                                 const ControlEquationFunction& f2, const VectorXd& weight2);

    ~WeightedSquaredSumOfControlEquationsFunction();

    // ------------------------ public interface --------------------------------
  public:
    WeightedSquaredSumOfControlEquationsFunction& add(const ControlEquationFunction& f, const VectorXd& weight);
    WeightedSquaredSumOfControlEquationsFunction& remove(const ControlEquationFunction& f);
    void changeFunctionWeight(const ControlEquationFunction& f, const VectorXd& weight);

    const Model& getModel() const;
    bool         contains(const ControlEquationFunction& f) const {return find(f) != _functions.end();}

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    void updateHessian()  const;

    void updateq()  const;
    void updater()  const;

    void doChangePi(const MatrixXd& Pi, int index);
    void doChangeqi(const VectorXd& qi, int index);
    void doChangeri(double ri, int index);


    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:
    fun_iterator        find(const ControlEquationFunction& f);
    const_fun_iterator  find(const ControlEquationFunction& f) const;
    void                invalidateTmp(int timestamp);
    void                updateTmp() const;

    void                connect(const ControlEquationFunction& f);
    void                disconnect(const ControlEquationFunction& f);

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:
    const Model&                    _model;
    std::vector<WeightAndFunction>  _functions;
    mutable VectorXd                _W;   //sum of the weights
    mutable VectorXd                _Wf;  //sum of the weighted value of the functions.
    mutable VectorXd                _WN;  //W*N.
    mutable bool                    _tempAreUpToDate;

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRACONTROL_WEIGHTED_SQUARED_SUM_OF_CONTROL_EQUATIONS_FUNCTION_H_

// cmake:sourcegroup=Functions
