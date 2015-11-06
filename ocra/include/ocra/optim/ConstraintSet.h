/** @file ConstraintSet.h
  * @brief Declaration file of the ConstraintSet class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Brisset Julien
  * @author Escande Adrien
  *	@date 09/06/04
  *
  * File history:
  *   - 10/06/28 Escande Adrien: minor update (GeneralConstraint->GenericConstraint, cfl_size_t->size_t), cleanup and doc
  */

#ifndef _OCRABASE_CONSTRAINT_SET_H_
#define _OCRABASE_CONSTRAINT_SET_H_

#ifdef WIN32
# pragma once
#endif

// ocra includes
#include "ocra/optim/Constraint.h"

// std includes
#include <vector>
#include <algorithm>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *   a library of classes to write and solve optimization problems dedicated to
  *   the control of multi-body systems. 
  */
namespace ocra
{
  /** @class ConstraintSet
    *	@brief %ConstraintSet class.
    *	@warning None
    *  
    * A set of constraint. This class is meant to be derived.
    * ConstraintSet mimics the hierarchy of Function, in the same way Constraint does.
    */
  template<class T>
  class ConstraintSet: protected ConstraintSet<typename T::functionType_t>
  {
    // ------------------------ constructors ------------------------------------
  private:
    ConstraintSet(const ConstraintSet&);
    ConstraintSet& operator= (const ConstraintSet&);
  protected:
  public:
    ConstraintSet();

    // ------------------------ public interface --------------------------------
  public:
    /** Does what it says.*/
    //@{
    Constraint<T>* getIthEqualityConstraint(size_t i);
    Constraint<T>* getIthInequalityConstraint(size_t i);
    ConstraintSet<T>& add(Constraint<T>* constraint);
    ConstraintSet<T>& add(ConstraintSet<T>& constraintSet);
    ConstraintSet<T>& remove(Constraint<T>* constraint);
    ConstraintSet<T>& remove(ConstraintSet<T>& constraintSet);

    size_t getNumberOfEqualityConstraints() const;
    size_t getNumberOfInequalityConstraints() const;
    size_t getEqualityDimension() const;
    size_t getInequalityDimension() const;
    //@}

  private:
    typedef boost::is_base_of<Function, T> T_must_derived_from_Function;
    BOOST_STATIC_ASSERT(T_must_derived_from_Function           ::value);
  };



  template<>
  class ConstraintSet<Function>
  {
    // ------------------------ constructors ------------------------------------
  private:
    ConstraintSet(const ConstraintSet&);
    ConstraintSet& operator= (const ConstraintSet&);
  protected:
  public:
    ConstraintSet();

    // ------------------------ public interface --------------------------------
  public:
    GenericConstraint* getIthEqualityConstraint(size_t i);
    GenericConstraint* getIthInequalityConstraint(size_t i);
    ConstraintSet<Function>& add(GenericConstraint* constraint);
    ConstraintSet<Function>& add(ConstraintSet<Function>& constraintSet);
    ConstraintSet<Function>& remove(GenericConstraint* constraint);
    ConstraintSet<Function>& remove(ConstraintSet<Function>& constraintSet);

    size_t getNumberOfEqualityConstraints() const;
    size_t getNumberOfInequalityConstraints() const;
    size_t getEqualityDimension() const;
    size_t getInequalityDimension() const;

    // ------------------------ private members ---------------------------------
  private:
    size_t  _dime;       //< sum of the dimension of each equality constraint
    size_t  _dimi;       //< sum of the dimension of each inequality constraint
    std::vector<GenericConstraint*> _equalityConstraints;
    std::vector<GenericConstraint*> _inequalityConstraints;
  };



  template<class T>
  inline ConstraintSet<T>::ConstraintSet()
    :ConstraintSet<typename T::functionType_t>()
  {
  }


  inline ConstraintSet<Function>::ConstraintSet()
    :_dime(0), _dimi(0)
  {
  }


  template<class T>
  inline Constraint<T>* ConstraintSet<T>::getIthEqualityConstraint(size_t i)
  {
    return static_cast<Constraint<T>*>(ConstraintSet<typename T::functionType_t>::getIthEqualityConstraint(i));
  }


  inline GenericConstraint* ConstraintSet<Function>::getIthEqualityConstraint(size_t i)
  {
    return _equalityConstraints[i];
  }


  template<class T>
  inline Constraint<T>* ConstraintSet<T>::getIthInequalityConstraint(size_t i)
  {
    return static_cast<Constraint<T>*>(ConstraintSet<typename T::functionType_t>::getIthInequalityConstraint(i));
  }


  inline GenericConstraint* ConstraintSet<Function>::getIthInequalityConstraint(size_t i)
  {
    return _inequalityConstraints[i];
  }


  template<class T>
  inline ConstraintSet<T>& ConstraintSet<T>::add(Constraint<T>* constraint)
  {
    ConstraintSet<typename T::functionType_t>::add(constraint);
    return *this;
  }


  inline ConstraintSet<Function>& ConstraintSet<Function>::add(GenericConstraint* constraint)
  {
    if (constraint->isEquality())
    {
      _dime += constraint->getDimension();
      _equalityConstraints.push_back(constraint);
    }
    else
    {
      _dimi += constraint->getDimension();
      _inequalityConstraints.push_back(constraint);
    }
    return *this;
  }


  template<class T>
  inline ConstraintSet<T>& ConstraintSet<T>::add(ConstraintSet<T>& constraintSet)
  {
    ConstraintSet<typename T::functionType_t>::add(constraintSet);
    return *this;
  }


  inline ConstraintSet<Function>& ConstraintSet<Function>::add(ConstraintSet<Function>& constraintSet)
  {
    for (size_t i=0; i<constraintSet._equalityConstraints.size(); ++i)
      add(constraintSet._equalityConstraints[i]);
    for (size_t i=0; i<constraintSet._inequalityConstraints.size(); ++i)
      add(constraintSet._inequalityConstraints[i]);
    return *this;
  }


  template<class T>
  inline ConstraintSet<T>& ConstraintSet<T>::remove(Constraint<T>* constraint)
  {
    ConstraintSet<typename T::functionType_t>::remove(constraint);
    return *this;
  }

  inline ConstraintSet<Function>& ConstraintSet<Function>::remove(GenericConstraint* constraint)
  {
    std::vector<GenericConstraint*>::iterator it;
    if (constraint->isEquality())
    {
      it = std::find(_equalityConstraints.begin(), _equalityConstraints.end(), constraint);
      if (it != _equalityConstraints.end())
      {
        _dime -= constraint->getDimension();
        _equalityConstraints.erase(it);
      }
    }
    else
    {
      it = std::find(_inequalityConstraints.begin(), _inequalityConstraints.end(), constraint);
      if (it != _inequalityConstraints.end())
      {
        _dimi -= constraint->getDimension();
        _inequalityConstraints.erase(it);
      }
    }
    return *this;
  }

  template<class T>
  inline ConstraintSet<T>& ConstraintSet<T>::remove(ConstraintSet<T>& constraintSet)
  {
    ConstraintSet<typename T::functionType_t>::remove(constraintSet);
    return *this;
  }

  inline ConstraintSet<Function>& ConstraintSet<Function>::remove(ConstraintSet<Function>& constraintSet)
  {
    for (size_t i=0; i<constraintSet._equalityConstraints.size(); ++i)
      remove(constraintSet._equalityConstraints[i]);
    for (size_t i=0; i<constraintSet._inequalityConstraints.size(); ++i)
      remove(constraintSet._inequalityConstraints[i]);
    return *this;
  }


  template<class T>
  inline size_t ConstraintSet<T>::getNumberOfEqualityConstraints() const
  {
    return ConstraintSet<Function>::getNumberOfEqualityConstraints();
  }


  inline size_t ConstraintSet<Function>::getNumberOfEqualityConstraints() const
  {
    return _equalityConstraints.size();
  }


  template<class T>
  inline size_t ConstraintSet<T>::getNumberOfInequalityConstraints() const
  {
    return ConstraintSet<Function>::getNumberOfInequalityConstraints();
  }


  inline size_t ConstraintSet<Function>::getNumberOfInequalityConstraints() const
  {
    return _inequalityConstraints.size();
  }


  template<class T>
  inline size_t ConstraintSet<T>::getEqualityDimension() const
  {
    return ConstraintSet<Function>::getEqualityDimension();
  }


  inline size_t ConstraintSet<Function>::getEqualityDimension() const
  {
    return _dime;
  }


  template<class T>
  inline size_t ConstraintSet<T>::getInequalityDimension() const
  {
    return ConstraintSet<Function>::getInequalityDimension();
  }


  inline size_t ConstraintSet<Function>::getInequalityDimension() const
  {
    return _dimi;
  }

}

#endif	//_OCRABASE_CONSTRAINT_SET_H_

// cmake:sourcegroup=Constraint

