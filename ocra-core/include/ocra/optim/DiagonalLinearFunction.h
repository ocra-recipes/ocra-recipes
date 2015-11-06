/** @file DiagonalLinearFunction.h
  * @brief Declaration file of the DiagonalLinearFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/07/20
  *
  * File history:
  *  - 10/06/21: Escande Adrien, Evrard Paul - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_DIAGONAL_LINEAR_FUNCTION_H_
#define _OCRABASE_DIAGONAL_LINEAR_FUNCTION_H_

#ifdef WIN32
# pragma once
#endif

// includes
#include "ocra/optim/LinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class DiagonalLinearFunction
    *	@brief %DiagonalLinearFunction class.
    *	@warning None
    *  
    * Implements a function of the type \f$ Dx+b \f$ with \f$ D=diag(d) \f$ a diagonal matrix whose diagonal is given
    * by the vector \f$ d \f$. It is possible to specify a default value for the new diagonal elements in case the 
    * size of the variable, and thus the dimension of the function, change. 
    * Note that the values of the last diagonal elements are lost when the dimension decreases. Thus going from
    * dimension n to dimension n'<n then back to n will set the last n-n' diagonal elements to the default value.
    */
  class DiagonalLinearFunction : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    /** copy of DiagonalLinearFunction instances is forbidden*/
    //@{ 
    DiagonalLinearFunction(const DiagonalLinearFunction&);
    DiagonalLinearFunction& operator= (const DiagonalLinearFunction&);
    //@}

  public:
    /** A set of constructors where \a d and \a b are specified either by a vector or by a double.
      *
      * \param [in] x The variable on which to build the function.
      * \param [in] d The value of d.
      * \param [in] b The value of b.
      * \param [in] diagonalElementValue The value of each element of d.
      * \param [in] vectorElementValue The value of each element of b.
      * \param [in] useDefaultValue true if the function can use a default value when resizing to a bigger dimension.
      *             Note that if useDefaultValue is false, the function can still be downsized.
      * \param [in] defaultDiagValue The default value for the last value of d when resizing to a bigger dimension.
      * \param [in] defaultbValue The default value for the last value of b when resizing to a bigger dimension.
      *
      * \warning You need to put \a real double for diagonalElementValue and vectorElementValue. All other type will be
      *   considered as VectorBase and will trigger the static assertion on th VectorBase type. For example,
      * DiagonalLinearFunction(x, 1, b), where x is a variable and b is a vector, is not valid. The correct way to 
      * write it is DiagonalLinearFunction(x, 1., b), because 1. is a double, but 1 is an int so the compiler will not
      * try to cast it into a double and call the wrong constructor.
      */
    //@{
    template<class VectorBase1, class VectorBase2>
    DiagonalLinearFunction(Variable& x, const VectorBase1& d, const VectorBase2& b,
                           const bool useDefaultValue = false, const double defaultDiagValue = 1.,
                           const double defaultbValue = 0.);

    template<class VectorBase>
    DiagonalLinearFunction(Variable& x, const double diagonalElementValue, const VectorBase& b,
                           const bool useDefaultValue = false, const double defaultbValue = 0.);

    template<class VectorBase>
    DiagonalLinearFunction(Variable& x, const VectorBase& d, const double vectorElementValue,
                           const bool useDefaultValue = false, const double defaultDiagValue = 1.);

    DiagonalLinearFunction(Variable& x, const double diagonalElementValue, const double vectorElementValue,
                           const bool useDefaultValue = false);
    //@}

  protected:
    /** A light constructor for derived class*/
    DiagonalLinearFunction(Variable& x);


    // ------------------------ public interface --------------------------------
  public:
    /** Trivial getters.*/
    //@{
    inline const  VectorXd& getDiagonal()   const;
    inline double getDefaultDiagonalValue() const;
    inline double getDefaultbValue()        const;
    inline bool   isUsingDefaultValue()     const;
    //@}

    /** Set the diagonal to \a d 
      * \pre d.size() == _dim
      * Calls buildA which triggers a EVT_CHANGE_VALUE event.
      */
    void  changeDiagonal(const VectorXd& d);

    /** Set the diagonal to \a diagonalElementValue and the default diagonal value to the same value if changeDefault
      * is true.
      * Calls buildA which triggers a EVT_CHANGE_VALUE event.
      */
    void  changeDiagonal(const double diagonalElementValue, const bool changeDefault = true);

    /** Change only the diagonal default value */
    void  changeDefaultDiagonalValue(const double v);

    /** Set the default value for b to \a v. (To change b, a method is directly accessible on LinearFunction)*/
    void  changeDefaultbValue(const double v);

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void updateValue() const;

    virtual void doUpdateInputSizeBegin();
    virtual void doUpdateInputSizeEnd();

    virtual void doChangeDiagonal(const VectorXd& d);
    virtual void doChangeDiagonal(const double diagonalElementValue, const bool changeDefault = true);
    virtual void doChangeDefaultDiagonalValue(const double v);
    virtual void doChangeDefaultbValue(const double v);

    virtual void doChangeA(const MatrixXd& A);

    int computeDimensionFromInputSize() const;
    void doUpdateDimensionBegin(int newDimension);

    // ------------------------ private methods ---------------------------------
  private:
    /** Build the jacobian matrix from the current _d value then triggers a EVT_CHANGE_VALUE event*/
    void buildA();

    // ------------------------ protected members -------------------------------
  protected:
    VectorXd  _d;                       //< diagonal value
    double    _defaultDiagonalValue;    //< default diagonal value
    double    _defaultbValue;           //< default value to fill b
    bool      _useDefaultValue;         //< true if the default value can be used
  };


  template <class VectorBase1, class VectorBase2>
  inline DiagonalLinearFunction::DiagonalLinearFunction(Variable& x, const VectorBase1& d, const VectorBase2& b,
                                                 const bool useDefaultValue, const double defaultDiagValue,
                                                 const double defaultbValue)
    :NamedInstance("diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()), _useDefaultValue(useDefaultValue), _defaultDiagonalValue(defaultDiagValue)
    , _defaultbValue(defaultbValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase1);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d.rows()==1 || d.cols()==1);
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase2);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(b.cols()==1);

    if (d.size() != _dim)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::DiagonalLinearFunction] Size of diagonal d doesn't match the variable size");

    if (b.size() != _dim)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::DiagonalLinearFunction] Size of b doesn't match the variable size");

    _d = d;
    buildA();
    changeb(b);
  }


  template <class VectorBase>
  inline DiagonalLinearFunction::DiagonalLinearFunction(Variable& x, const double diagonalElementValue, const VectorBase& b,
                                                 const bool useDefaultValue, const double defaultbValue)
    :NamedInstance("diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()), _useDefaultValue(useDefaultValue), _defaultDiagonalValue(diagonalElementValue)
    , _defaultbValue(defaultbValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase); //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(b.cols()==1);

    if (b.size() != _dim)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::DiagonalLinearFunction] Size of b doesn't match the variable size");

    _d.resize(_dim);
    _d.setConstant(diagonalElementValue);
    buildA();
    changeb(b);
  }


  template <class VectorBase>
  inline DiagonalLinearFunction::DiagonalLinearFunction(Variable& x, const VectorBase& d, const double vectorElementValue,
                                                 const bool useDefaultValue, const double defaultDiagValue)
    :NamedInstance("diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()), _useDefaultValue(useDefaultValue), _defaultDiagonalValue(defaultDiagValue), _d(d)
    ,_defaultbValue(vectorElementValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase); //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d.rows()==1 || d.cols()==1);

    if (d.size() != _dim)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::DiagonalLinearFunction] Size of diagonal d doesn't match the variable size");
  
    buildA();
    _b.resize(_dim);
    _b.setConstant(vectorElementValue);

  }



  inline const VectorXd& DiagonalLinearFunction::getDiagonal() const
  {
    return _d;
  }


  inline double DiagonalLinearFunction::getDefaultDiagonalValue() const
  {
    return _defaultDiagonalValue;
  }


  inline double DiagonalLinearFunction::getDefaultbValue() const
  {
    return _defaultbValue;
  }


  inline bool DiagonalLinearFunction::isUsingDefaultValue() const
  {
    return _useDefaultValue;
  }


  void testDiagonalLinearFunction();
}

#endif	//_OCRABASE_DIAGONAL_LINEAR_FUNCTION_H_

// cmake:sourcegroup=Function

