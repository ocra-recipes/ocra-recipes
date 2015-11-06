#ifndef _OCRA_FUNCTION_HELPERS_H_
#define _OCRA_FUNCTION_HELPERS_H_

#include "ocra/optim/Objective.h"
#include "ocra/optim/Constraint.h"

#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace ocra
{
  // --- OBJECTIVE ----------------------------------------------

  template<class T>
  class ObjectivePtr
  {
  public:
    ObjectivePtr()
      : function()
      , objective()
    {
    }

    ObjectivePtr(T* f, double weight=1.)
      : function(f)
      , objective(new Objective<T>(f, weight))
    {
    }

    operator Objective<T>&() { return getObjective(); }
    operator const Objective<T>&() const { return getObjective(); }
    
    operator T&() { return getFunction(); }
    operator const T&() const { return getFunction(); }

    void set(T* f, double weight=1.)
    {
      function.reset(f);
      objective.reset(new Objective<T>(f, weight));
    }

    T& getFunction()
    {
      if(!function)
        throw std::runtime_error("[ObjectivePtr::getFunction] objective was not set");
      return *function;
    }
    
    const T& getFunction() const
    {
      if(!function)
        throw std::runtime_error("[ObjectivePtr::getFunction] objective was not set");
      return *function;
    }

    Objective<T>& getObjective()
    {
      if(!objective)
        throw std::runtime_error("[ObjectivePtr::getFunction] objective was not set");
      return *objective;
    }

    const Objective<T>& getObjective() const
    {
      if(!objective)
        throw std::runtime_error("[ObjectivePtr::getFunction] objective was not set");
      return *objective;
    }

  private:
    boost::shared_ptr<T> function;
    boost::shared_ptr<Objective<T> > objective;
  };


  // --- EQUAL ZERO CONSTRAINT ----------------------------------
  
  template<class T>
  class EqualZeroConstraintPtr
  {
  public:
    EqualZeroConstraintPtr()
      : function()
      , constraint()
    {
    }

    EqualZeroConstraintPtr(T* f)
      : function(f)
      , constraint(new Constraint<T>(f, true))
    {
    }

    operator Constraint<T>&() { return getConstraint(); }
    operator const Constraint<T>&() const { return getConstraint(); }
    
    operator T&() { return getFunction(); }
    operator const T&() const { return getFunction(); }

    void set(T* f)
    {
      function.reset(f);
      constraint.reset(new Constraint<T>(f, true));
    }

    T& getFunction()
    {
      if(!function)
        throw std::runtime_error("[EqualZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }
    
    const T& getFunction() const
    {
      if(!function)
        throw std::runtime_error("[EqualZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }

    Constraint<T>& getConstraint()
    {
      if(!constraint)
        throw std::runtime_error("[EqualZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

    const Constraint<T>& getConstraint() const
    {
      if(!constraint)
        throw std::runtime_error("[EqualZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

  private:
    boost::shared_ptr<T> function;
    boost::shared_ptr<Constraint<T> > constraint;
  };


  // --- LOWER THAN ZERO CONSTRAINT -----------------------------
  
  template<class T>
  class LessThanZeroConstraintPtr
  {
  public:
    LessThanZeroConstraintPtr()
      : function()
      , constraint()
    {
    }

    LessThanZeroConstraintPtr(T* f)
      : function(f)
      , constraint(new Constraint<T>(f, false))
    {
    }

    operator Constraint<T>&() { return getConstraint(); }
    operator const Constraint<T>&() const { return getConstraint(); }
    
    operator T&() { return getFunction(); }
    operator const T&() const { return getFunction(); }

    void set(T* f)
    {
      function.reset(f);
      constraint.reset(new Constraint<T>(f, false));
    }

    T& getFunction()
    {
      if(!function)
        throw std::runtime_error("[LessThanZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }
    
    const T& getFunction() const
    {
      if(!function)
        throw std::runtime_error("[LessThanZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }

    Constraint<T>& getConstraint()
    {
      if(!constraint)
        throw std::runtime_error("[LessThanZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

    const Constraint<T>& getConstraint() const
    {
      if(!constraint)
        throw std::runtime_error("[LessThanZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

  private:
    boost::shared_ptr<T> function;
    boost::shared_ptr<Constraint<T> > constraint;
  };


  // --- GREATER THAN ZERO CONSTRAINT ---------------------------
  
  template<class T>
  class GreaterThanZeroConstraintPtr
  {
  public:
    GreaterThanZeroConstraintPtr()
      : function()
      , constraint()
    {
    }

    GreaterThanZeroConstraintPtr(T* f)
      : function(f)
      , constraint(new Constraint<T>(f))
    {
    }

    operator Constraint<T>&() { return getConstraint(); }
    operator const Constraint<T>&() const { return getConstraint(); }
    
    operator T&() { return getFunction(); }
    operator const T&() const { return getFunction(); }

    void set(T* f)
    {
      function.reset(f);
      constraint.reset(new Constraint<T>(f));
    }

    T& getFunction()
    {
      if(!function)
        throw std::runtime_error("[GreaterThanZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }
    
    const T& getFunction() const
    {
      if(!function)
        throw std::runtime_error("[GreaterThanZeroConstraintPtr::getFunction] constraint was not set");
      return *function;
    }

    Constraint<T>& getConstraint()
    {
      if(!constraint)
        throw std::runtime_error("[GreaterThanZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

    const Constraint<T>& getConstraint() const
    {
      if(!constraint)
        throw std::runtime_error("[GreaterThanZeroConstraintPtr::getFunction] constraint was not set");
      return *constraint;
    }

  private:
    boost::shared_ptr<T> function;
    boost::shared_ptr<Constraint<T> > constraint;
  };
}

#endif

// cmake:sourcegroup=Utils
