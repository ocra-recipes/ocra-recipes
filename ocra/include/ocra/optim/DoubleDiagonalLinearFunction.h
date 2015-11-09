/** @file DoubleDiagonalLinearFunction.h
  * @brief Declaration file of the DoubleDiagonalLinearFunction class.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/02/18
  *
  * File history:
  *  - 10/07/05: Escande Adrien - Adaptation to the new Function interface and switch to Eigen.
  */

#ifndef _OCRABASE_DOUBLE_DIAGONAL_LINEAR_FUNCTION_H_
#define _OCRABASE_DOUBLE_DIAGONAL_LINEAR_FUNCTION_H_

// includes
#include "ocra/optim/LinearFunction.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class DoubleDiagonalLinearFunction
    *	@brief %DoubleDiagonalLinearFunction class.
    *	@warning None
    *  
    * Equivalent as two DiagonalLinearFunction side by side
    */
  class DoubleDiagonalLinearFunction : public LinearFunction
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    // ------------------------ constructors ------------------------------------
  private:
    DoubleDiagonalLinearFunction(const DoubleDiagonalLinearFunction&);
    DoubleDiagonalLinearFunction& operator= (const DoubleDiagonalLinearFunction&);

  public:
    template<class VectorBase1, class VectorBase2, class VectorBase3>
    DoubleDiagonalLinearFunction(Variable& x, const VectorBase1& d1, const VectorBase2& d2, const VectorBase3& b,
                           const bool useDefaultValue = false, const double defaultDiagValue1 = 1., const double defaultDiagValue2 = 1.,
                           const double defaultbValue = 0.);

    template<class VectorBase>
    DoubleDiagonalLinearFunction(Variable& x, const double diagonalElementValue1, const double diagonalElementValue2, const VectorBase& b,
                           const bool useDefaultValue = false, const double defaultbValue = 0.);

    template<class VectorBase1, class VectorBase2>
    DoubleDiagonalLinearFunction(Variable& x, const VectorBase1& d1 , const VectorBase2& d2, const double vectorElementValue,
                           const bool useDefaultValue = false, const double defaultValue1 = 1., const double defaultValue2 = 1.);

    DoubleDiagonalLinearFunction(Variable& x, const double diagonalElementValue1, const double diagonalElementValue2, const double vectorElementValue,
                           const bool useDefaultValue = false);


    // ------------------------ public interface --------------------------------
  public:
    inline const  VectorXd& getDiagonal1()      const;
    inline const  VectorXd& getDiagonal2()      const;
    inline double getDefaultDiagonalValue1()    const;
    inline double getDefaultDiagonalValue2()    const;
    inline double getDefaultbValue()            const;
    inline bool   isUsingDefaultValue()         const;
    virtual void  changeDiagonal1(const VectorXd& d);
    virtual void  changeDiagonal2(const VectorXd& d);
    virtual void  changeDiagonal1(const double diagonalElementValue, const bool changeDefault = true);
    virtual void  changeDiagonal2(const double diagonalElementValue, const bool changeDefault = true);
    virtual void  changeDefaultDiagonalValue1(const double v);
    virtual void  changeDefaultDiagonalValue2(const double v);
    virtual void  changeDefaultbValue(const double v);


    // ------------------------ protected methods -------------------------------
  protected:
    virtual void updateValue() const;

    virtual void doUpdateInputSizeBegin();
    virtual void doUpdateInputSizeEnd();

    virtual void  doChangeA(const MatrixXd& A);

    int computeDimensionFromInputSize() const;
    void doUpdateDimensionBegin(int newDimension);

    // ------------------------ private methods ---------------------------------
  private:
    void buildA();
  
    // ------------------------ protected members -------------------------------
  protected:
    VectorXd  _d1;
    VectorXd  _d2;
    double    _defaultDiagonalValue1;
    double    _defaultDiagonalValue2;
    double    _defaultbValue;           //default value to fill b
    bool      _useDefaultValue;
  };



  template<class VectorBase1, class VectorBase2, class VectorBase3>
  inline DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction(Variable& x, 
                            const VectorBase1& d1, const VectorBase2& d2, const VectorBase3& b,
                            const bool useDefaultValue, const double defaultDiagValue1, const double defaultDiagValue2,
                            const double defaultbValue)
    :NamedInstance("double diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()/2)
    , _useDefaultValue(useDefaultValue)
    , _defaultDiagonalValue1(defaultDiagValue1)
    , _defaultDiagonalValue2(defaultDiagValue2)
    , _defaultbValue(defaultbValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase1);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d1.rows()==1 || d1.cols()==1);
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase2);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d2.rows()==1 || d2.cols()==1);
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase3);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(b.cols()==1);

    if (d1.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction] Size of diagonal d1 doesn't match the function dimension");

    if (d2.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction] Size of diagonal d2 doesn't match the function dimension");

    if (b.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction] Size of b doesn't match the function dimension");

    _d1 = d1;
    _d2 = d2;
    buildA();
    changeb(b);
  }


  template<class VectorBase>
  inline DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction(Variable& x, 
                            const double diagonalElementValue1, const double diagonalElementValue2, const VectorBase& b,
                            const bool useDefaultValue, const double defaultbValue)
    :NamedInstance("double diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()/2)
    , _useDefaultValue(useDefaultValue)
    , _defaultDiagonalValue1(diagonalElementValue1)
    , _defaultDiagonalValue2(diagonalElementValue2)
    , _defaultbValue(defaultbValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(b.cols()==1);

    if (b.size() != _dim)
      throw std::runtime_error("[ocra::DiagonalLinearFunction::DiagonalLinearFunction] Size of b doesn't match the function dimension");

    _d1.resize(_dim);
    _d1.setConstant(diagonalElementValue1);
    _d2.resize(_dim);
    _d2.setConstant(diagonalElementValue2);
    buildA();
    changeb(b);
  }


  template<class VectorBase1, class VectorBase2>
  inline DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction(Variable& x, 
                            const VectorBase1& d1 , const VectorBase2& d2, const double vectorElementValue,
                            const bool useDefaultValue, const double defaultValue1, const double defaultValue2)
    :NamedInstance("double diagonal linear function")
    , AbilitySet(PARTIAL_X)
    , CoupledInputOutputSize(true)
    , LinearFunction(x, x.getSize()/2)
    , _useDefaultValue(useDefaultValue)
    , _defaultDiagonalValue1(defaultValue1)
    , _defaultDiagonalValue2(defaultValue2)
    ,_defaultbValue(vectorElementValue)
  {
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase1);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d1.rows()==1 || d1.cols()==1);
    OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase2);  //Did you put an int instead of a double ? That would explain you arrived in the wrong ctor
    ocra_assert(d2.rows()==1 || d2.cols()==1);

    if (d1.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction] Size of diagonal d1 doesn't match the function dimension");

    if (d2.size() != _dim)
      throw std::runtime_error("[ocra::DoubleDiagonalLinearFunction::DoubleDiagonalLinearFunction] Size of diagonal d2 doesn't match the function dimension");
  
    _d1 = d1;
    _d2 = d2;
    buildA();
    _b.resize(_dim);
    _b.setConstant(vectorElementValue);
  }

  inline const VectorXd& DoubleDiagonalLinearFunction::getDiagonal1()    const
  {
    return _d1;
  }

  inline const VectorXd& DoubleDiagonalLinearFunction::getDiagonal2()    const
  {
    return _d2;
  }

  inline double DoubleDiagonalLinearFunction::getDefaultDiagonalValue1() const
  {
    return _defaultDiagonalValue1;
  }

  inline double DoubleDiagonalLinearFunction::getDefaultDiagonalValue2() const
  {
    return _defaultDiagonalValue2;
  }

  inline double DoubleDiagonalLinearFunction::getDefaultbValue()        const
  {
    return _defaultbValue;
  }

  inline bool DoubleDiagonalLinearFunction::isUsingDefaultValue()     const
  {
    return _useDefaultValue;
  }
}

#endif	//_OCRABASE_DOUBLE_DIAGONAL_LINEAR_FUNCTION_H_

// cmake:sourcegroup=Function

