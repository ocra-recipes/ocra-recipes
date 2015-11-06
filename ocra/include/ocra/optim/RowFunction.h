/** @file RowFunction.h
  * @brief Declaration file of the RowFunction class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/06/09
  */

#ifndef _OCRABASE_ROW_H_
#define _OCRABASE_ROW_H_

// includes
#include "ocra/optim/CompileTimeChecks.h"
#include "ocra/optim/Function.h"

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  /** @class RowFunction
    *	@brief %RowFunction class.
    *	@warning None
    *  
    * A single row of an other Function.
    * Be careful with this class, it still implies the full computation of the original
    * function, and it involves a copy.
    * TODO [mineur] : modify this comment in case this row functionality is implemented directly in 
    * the Function class
    */
  template<class T>
  class RowFunction : public T::functionType_t, private IsDerivedFrom<T, Function>
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef T  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
    RowFunction();
    RowFunction(RowFunction&);
  protected:
  public:
    RowFunction(T* function, const cfl_size_t index);
    RowFunction(T* function, const std::vector<int>& indices);

    // ------------------------ public interface --------------------------------
  public:

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    virtual void computeValue(void) const;
    virtual void computeGradient(void) const;
    virtual void computeHessian(void) const;
    virtual void computeJdot(void) const;
    virtual void computeJdotXdot(void) const;

    virtual void doUpdateSize(void);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:

    // ------------------------ protected static members ------------------------
  protected:
    Function*           _f;
    std::vector<int> _indices;

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };


  template <class T>
  inline RowFunction<T>::RowFunction(T* function, const cfl_size_t index)
    : Function(function->getVariable(), 1, function->getType(), function->getConvexityProperty(), 
                function->getContinuityProperty(),function->canComputeHessian(), function->canComputeGradient())
    , _indices(1), _f(function)
  {
    _indices[0] = index;
    function->attach(*this);
    this->_name = "rowFunction";
  }

  template <class T>
  inline RowFunction<T>::RowFunction(T* function, const std::vector<int>& indices)
    : Function(function->getVariable(), (cfl_size_t)indices.size(), function->getType(), function->getConvexityProperty(), 
                function->getContinuityProperty(),function->canComputeHessian(), function->canComputeGradient())
    , _indices(indices), _f(function)
  {
    assert(indices.size() <= function->getDimension());
    function->attach(*this);
    this->_name = "rowFunction";
  }

  template <class T>
  inline void RowFunction<T>::computeValue(void) const
  {
    const Vector& v = _f->getValues();
    for (cfl_size_t i=0; i<this->_dimension; ++i)
      this->_value[i] = v[_indices[i]];
  }

  template <class T>
  inline void RowFunction<T>::computeGradient(void) const
  {
    const Matrix& g = _f->getGradients();
    for (cfl_size_t i=0; i<this->_dimension; ++i)
    {
      for (cfl_size_t j=0; j<this->_x->getSize(); ++j)
      {
        this->_gradient(i,j) = g(_indices[i], j);
      }
    }
  }

  template <class T>
  inline void RowFunction<T>::computeHessian(void) const
  {
    throw std::runtime_error("RowFunction<T>::computeHessian: method non implemented.");
    for (cfl_size_t i=0; i<this->_dimension; ++i)
    {
      //TODO [todo] : write properly
//      _hessians[i]->copyValuesFrom(_f->getHessian(_indices[i]));
    }
  }

  template <class T>
  inline void RowFunction<T>::computeJdot(void) const
  {
    const Matrix& g = _f->getJdot();
    for (cfl_size_t i=0; i<this->_dimension; ++i)
    {
      for (cfl_size_t j=0; j<this->_x->getSize(); ++j)
      {
        this->_Jdot(i,j) = g(_indices[i], j);
      }
    }
  }

  template <class T>
  inline void RowFunction<T>::computeJdotXdot(void) const
  {
    const Vector& v = _f->getJdotXdot();
    for (cfl_size_t i=0; i<this->_dimension; ++i)
    {
      this->_JdotXdot[i] = v[_indices[i]];
    }
  }

  template <class T>
  inline void RowFunction<T>::doUpdateSize(void)
  {
  }

}

#endif	//_OCRABASE_ROW_H_

// cmake:sourcegroup=toBeUpdated

