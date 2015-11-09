/** @file LinearFunction.h
  * @brief Declaration file of the LinearFunction class.
  *
  *        Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien, Brisset Julien
  *	@date 09.04.06
  *
  * File history:
  *  - 10/06/21: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_LINEAR_FUNCTION_H_
#define _OCRABASE_LINEAR_FUNCTION_H_

// ocra includes
#include "ocra/optim/Function.h"

//#define LINEAR_FUNCTION_PROVIDES_HESSIANS

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class LinearFunction
    *	@brief %LinearFunction class.
    *	@author Escande Adrien
    *	@warning None
    * 
    * A linear function Ax+b. The class is implemented for A et b are constant, but can be overloaded for A and b being
    * functions of other parameters. The updateValue ensure to take A and b up-to-date (provided the methods to update
    * are correct).
    */
  class LinearFunction : public Function
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Function  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of LinearFunction instances is forbidden */
    //@{
    LinearFunction(const LinearFunction&);
    LinearFunction& operator=(const LinearFunction&);
    //@}

  protected:
    /** A light constructor for derived class */
    LinearFunction(Variable& x, int dimension);
  public:
    /** Constructor for A and b constant*/
    template<class Derived, class VectorBase>
    LinearFunction(Variable& x, const MatrixBase<Derived>& A, const VectorBase& b);
    ~LinearFunction();

    // ------------------------ public interface---------------------------------
  public:
    /** getters/setters for A and b
      * \pre A and b must have the size of the data they replace.
      */
    //@{
    const MatrixXd& getA() const;
    const VectorXd& getb() const;
    void changeA(const MatrixXd& A);
    void changeb(const VectorXd& b);
    //@}

    /** Invalidate the value of b. Meant to be used as a callback.*/
    void invalidateb(int timestamp);

    // ------------------------ protected methods -------------------------------
  protected:
    /** Overloads of the Function methods*/
    //@{
    virtual void updateValue() const;
    virtual void updateJacobian() const;
    //@}

    /** Update the value of b. Does nothing in LinearFunction. It is provided so that derived class may overload it.*/
    virtual void updateb() const;

    /** Methods to be called in ChangeA and Change b*/
    //@{
    virtual void doChangeA(const MatrixXd& A);
    virtual void doChangeb(const VectorXd& b);
    //@}

    /** This methods ensures _b has the good size at the end of a resize, yet it will not be called in LinearFunction
      * itself because the class does not overload doUpdateInputSizeBegin() which throws an runtime_error by default.
      */
    virtual void doUpdateInputSizeEnd();

    /** Inhibit/Desinhibit the fact that a call to invalidateb triggers a EVT_CHANGE_VALUE event, which is done by 
      * default. This is meant to prevent multiple propagation of EVT_CHANGE_VALUE in case of complex update 
      * dependencies.
      */
    //@{
    void inhibitPropagationFromb() const;
    void desinhibitPropagationFromb() const;
    //@}

    // ------------------------ protected members -------------------------------
  protected:
    mutable VectorXd  _b;           //< the vecor b in Ax+b
    mutable bool      _bIsUpToDate; //< a boolean indicating the validity of _b

  private:
    mutable bool      _inhibitPropagationFromb; //< if true, invalidateb will not trigger an EVT_CHANGE_VALUE event
  };


  template<class Derived, class VectorBase>
  inline LinearFunction::LinearFunction(Variable& x, const MatrixBase<Derived>& A, const VectorBase& b)
    :NamedInstance("linear function")
    ,AbilitySet(PARTIAL_X)
    ,CoupledInputOutputSize(false)
    ,Function(x,static_cast<int>(A.rows()), LINEARITY_LINEAR, CONVEXITY_CONVEX_AND_CONCAVE, CONTINUITY_CINF)
    ,_bIsUpToDate(true)
    ,_inhibitPropagationFromb(false)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase);
    ocra_assert(b.cols()==1);

    _b.resize(_dim);

    changeA(A); //non-forwarded virtual call ok here
    changeb(b); //non-forwarded virtual call ok here
  }


  void testLinearFunction();
}

#endif //_OCRABASE_LINEAR_FUNCTION_H_

// cmake:sourcegroup=Function
