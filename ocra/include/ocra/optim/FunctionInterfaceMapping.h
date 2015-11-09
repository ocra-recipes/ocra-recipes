/** @file FunctionInterfaceMapping.h
  * @brief Declaration file of the FunctionInterfaceMapping struct.
  *
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Escande Adrien
  *	@date 10/06/10
  *
  */

#ifndef _OCRABASE_FUNCTION_INTERFACE_MAPPING_H_
#define _OCRABASE_FUNCTION_INTERFACE_MAPPING_H_

#include "ocra/optim/Function.h"

namespace ocra
{//TODO update documentation
  /** This structure is meant to be use jointly with the Constraint class through a CRTP mechanism to access the
    * protected members of Constraint or Objective and replicate their interfaces. The generic structure is completly
    * empty so as to induce no memory cost when used as a base class in the common case. It must be specialized for
    * values of the template parameter Derived.
    */
  template<class Derived>
  struct FunctionInterfaceMapping {};

  template<class T> class Constraint;

/** \internal Two aliases to get const and no const version of the function in X<Function> */
#define GET_FUNCTION(X) static_cast<X<Function>*>(this)->getFunction()
#define GET_FUNCTION_CONST(X) static_cast<const X<Function>* const>(this)->getFunction()

  /** \internal Specialization of the class for Derived = X<Function>
    * X is meant to be a class having a method getFunction() returning and instance of Function.
    * X was thought to be Constraint or Objective
    */
  template<template<class> class X >
  struct FunctionInterfaceMapping<X<Function> >
  {
    int                 getDimension() const                    {return GET_FUNCTION_CONST(X).getDimension();}
    void                changeDimension(int newDimension)       {GET_FUNCTION(X).changeDimension(newDimension);}
    const std::string&  getName() const                         {return GET_FUNCTION_CONST(X).getName();}
    const Variable&     getVariable() const                     {return GET_FUNCTION_CONST(X).getVariable();}
    Variable&           getVariable()                           {return GET_FUNCTION(X).getVariable();}
    template<eFunctionAbility Ability>
    void                invalidate()                            {GET_FUNCTION(X).invalidate<Ability>();}
    void                invalidateAll()                         {GET_FUNCTION(X).invalidateAll();}
    template<eFunctionAbility Ability>
    bool                isValid() const                         {return GET_FUNCTION_CONST(X).isValid<Ability>();}
    template<eFunctionAbility Ability>
    bool                canCompute() const                      {return GET_FUNCTION_CONST(X).canCompute<Ability>();}
    template<eFunctionAbility Ability> const typename IFunction<Ability>::return_type&
                        get() const                             {return GET_FUNCTION_CONST(X).get<Ability>();} //index
    template<eFunctionAbility Ability> typename IFunction<Ability>::return_sub_type
                        get(int index) const                    {return GET_FUNCTION_CONST(X).get<Ability>(index);}
    const VectorXd&     getValue() const                        {return GET_FUNCTION_CONST(X).getValue();}
    double              getValue(int index) const               {return GET_FUNCTION_CONST(X).getValue(index);}
    const MatrixXd&     getJacobian() const                     {return GET_FUNCTION_CONST(X).getJacobian();}
    MatrixXdRow         getJacobian(int index) const            {return GET_FUNCTION_CONST(X).getJacobian(index);}


    eFunctionLinearity  getType(void) const                     {return GET_FUNCTION_CONST(X).getType();}
    eFunctionConvexity  getConvexityProperty(void) const        {return GET_FUNCTION_CONST(X).getConvexityProperty();}
    int                 getContinuityProperty(void) const       {return GET_FUNCTION_CONST(X).getContinuityProperty();}
    const std::string&  getProperty(int i) const                {return GET_FUNCTION_CONST(X).getProperty(i);}
    int                 getNumberOfProperties(void) const       {return GET_FUNCTION_CONST(X).getNumberOfProperties();}
    bool hasProperty(const std::string& functionProperty) const {return GET_FUNCTION_CONST(X).hasProperty(functionProperty);}
    bool                isExplicitlyTimeDependant(void) const   {return GET_FUNCTION_CONST(X).isExplicitlyTimeDependant();}
    bool                hasSeparableTimeDependancy(void) const  {return GET_FUNCTION_CONST(X).hasSeparableTimeDependancy();}
  };


#undef GET_FUNCTION
#undef GET_FUNCTION_CONST
}

#endif //_OCRABASE_FUNCTION_INTERFACE_MAPPING_H_

// cmake:sourcegroup=Constraint
