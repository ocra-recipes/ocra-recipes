/** @file QuadraticFunction.h
  * @brief Declaration file of the QuadraticFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/30
  */

#ifndef _OCRA_QUADRATIC_FUNCTION_H_
#define _OCRA_QUADRATIC_FUNCTION_H_

// includes
#include "ocra/optim/Function.h"
#include "ocra/optim/ocra_assert.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class QuadraticFunction
    *	@brief %QuadraticFunction class.
    *	@warning None
    *  
    * A quadratic function with \f& f_i(x) = 1/2 x^T P_i x + q_i^T x + r_i \f& for each dimension of the function, with
    * \f$ P_i \f$ a symmetric, non null matrix of size nxn, \f$ q_i \f$ a vector of size n and \f$ r_i \f$ a real.
    */
  class QuadraticFunction : public Function
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Function  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of QuadraticFunction instances is forbidden*/
    //@{
    QuadraticFunction();
    QuadraticFunction(QuadraticFunction&);
    //@}

  protected:
    /** Light constructor to be used by derived classes.*/
    QuadraticFunction(Variable& x, int dimension = 1);

  public:
    /** Constructor for a unidimensional function*/
    template<class Derived, class VectorBase>
    QuadraticFunction(Variable& x, const MatrixBase<Derived>& P, const VectorBase& q, double r) //unidimensional constant form
      : NamedInstance("QuadraticFunction")
      ,AbilitySet(PARTIAL_X, PARTIAL_XX)
      ,CoupledInputOutputSize(false)
      ,Function(x,1, LINEARITY_QUADRATIC, CONVEXITY_UNDEFINED, CONTINUITY_CINF
      , false //not time dependant
      , true) //thus time separable
      ,_qIsUpToDate(true)
      ,_rIsUpToDate(true)
      ,_inhibitPropagationFrom_q_or_r(false)
    {
      _q.push_back(new VectorXd);
      _r.resize(1);
      changePi(P);
      changeqi(q);
      changeri(r);
    }

    virtual ~QuadraticFunction();

    // ------------------------ public interface --------------------------------
  public:
    const MatrixXd&  getPi(int index = 0) const;
    const VectorXd&  getqi(int index = 0) const;
    double           getri(int index = 0) const;
    const VectorXd&  getr()               const;

    void    changePi(const MatrixXd& Pi, int index = 0);
    void    changeqi(const VectorXd& qi, int index = 0);
    void    changeri(double ri, int index = 0);

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void updateValue() const;
    virtual void updateJacobian() const;
    virtual void updateHessian() const;

    virtual void updateq() const;
    virtual void updater() const;

    virtual void doUpdateInputSizeEnd();

    virtual void doUpdateDimensionEnd(int oldDimension);

    virtual void doChangePi(const MatrixXd& Pi, int index);
    virtual void doChangeqi(const VectorXd& qi, int index);
    virtual void doChangeri(double ri, int index);

    void invalidateq(int timestamp);
    void invalidater(int timestamp);

    /** Inhibit/desinhibit the fact that a call to invalidateq or invalidater triggers a EVT_CHANGE_VALUE event, which 
      * is done by default. This is meant to prevent multiple propagation of EVT_CHANGE_VALUE in case of complex 
      * update dependencies.
      */
    //@{
    void inhibitPropagationFrom_q_or_r() const;
    void desinhibitPropagationFrom_q_or_r() const;
    //@}

    // ------------------------ protected members -------------------------------
  protected:
    mutable std::vector<VectorXd*>  _q;
    mutable VectorXd                _r;
    mutable bool                    _qIsUpToDate;
    mutable bool                    _rIsUpToDate;

  private:
    mutable bool                    _inhibitPropagationFrom_q_or_r;
  };


  inline const MatrixXd& QuadraticFunction::getPi(int index) const
  {
    ocra_assert(index <_dim);
    return get<PARTIAL_XX>(index);
  }

  inline const VectorXd& QuadraticFunction::getqi(int index) const
  {
    ocra_assert(index <_dim);
    if (!_qIsUpToDate)
    {
      updateq();
      _qIsUpToDate = true;
    }
    return *_q[index];
  }

  inline double QuadraticFunction::getri(int index) const
  {
    return getr()[index];
  }

  inline const VectorXd& QuadraticFunction::getr() const
  {
    if (!_rIsUpToDate)
    {
      updater();
      _rIsUpToDate = true;
    }
    return _r;
  }


  void testQuadraticFunction();
}

#endif	//_OCRA_QUADRATIC_FUNCTION_H_

// cmake:sourcegroup=Function
