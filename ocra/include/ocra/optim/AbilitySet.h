#ifndef _OCRA_OPTIM_ABILITY_SET_H_
#define _OCRA_OPTIM_ABILITY_SET_H_

#ifdef WIN32
# pragma once
#endif

#include "ocra/optim/IFunction.h"

#include <vector>

namespace ocra
{
  /** This class simply encapsulates a vector of boolean, one for each element of eFunctionAbility (except 
    * PROP_NUMBER). The ith element of this vector is true when the corresponding ability has to be used.
    * 
    * This class is only meant to be a helper class to enumerate the abilities of a function at construction time.
    */
  class AbilitySet
  {
  protected:
    /** Constructor with a given vector a bool, which needs to be of size PROP_NUMBER*/
    AbilitySet(const std::vector<bool>& usageSet)
      :_usageSet(usageSet)
    {
      ocra_assert(usageSet.size() == PROP_NUMBER && "given usage vector has not the proper size");
      _usageSet[FUN_VALUE] = true;
    }


    /** A set of constructors to take 1 to 10 eFunctionAbility elements. Giving one element indicates the
      * corresponding ability is to be used.
      * For example, AbilitySet(FUN_VALUE, PARTIAL_X, PARTIAL_T, FUN_DOT) build an AbilitySet with the booleans for
      * the abilities FUN_VALUE, PARTIAL_X, PARTIAL_T and FUN_DOT set to true, and the others to false.
      */
    //@{
    AbilitySet(
      eFunctionAbility prop0 = FUN_VALUE,
      eFunctionAbility prop1 = FUN_VALUE
      )
      : _usageSet(PROP_NUMBER, false)
    {
      _usageSet[FUN_VALUE] = true;
      _usageSet[prop0] = true;
      _usageSet[prop1] = true;
    }


    AbilitySet(
      eFunctionAbility prop0,
      eFunctionAbility prop1,
      eFunctionAbility prop2,
      eFunctionAbility prop3 = FUN_VALUE,
      eFunctionAbility prop4 = FUN_VALUE
      )
      : _usageSet(PROP_NUMBER, false)
    {
      _usageSet[FUN_VALUE] = true;

      const eFunctionAbility props[] = { prop0, prop1, prop2, prop3, prop4 };
      for (int i=0; i<5; ++i)
        _usageSet[props[i]] = true;
    }

    AbilitySet(
      eFunctionAbility prop0,
      eFunctionAbility prop1,
      eFunctionAbility prop2,
      eFunctionAbility prop3,
      eFunctionAbility prop4,
      eFunctionAbility prop5,
      eFunctionAbility prop6 = FUN_VALUE,
      eFunctionAbility prop7 = FUN_VALUE,
      eFunctionAbility prop8 = FUN_VALUE,
      eFunctionAbility prop9 = FUN_VALUE
      )
      : _usageSet(PROP_NUMBER, false)
    {
      _usageSet[FUN_VALUE] = true;

      const eFunctionAbility props[] = { prop0, prop1, prop2, prop3, prop4, prop5, prop6, prop7, prop8, prop9 };
      for (int i=0; i<10; ++i)
        _usageSet[props[i]] = true;
    }
    //@}


    /** Get the vector.*/
    const std::vector<bool>& getUsageSet() const
    {
      return _usageSet;
    }

  public:
    /** Add an Ability to the set.*/
    AbilitySet& add(eFunctionAbility prop)
    {
      _usageSet[prop] = true;
      return *this;
    }

    /** Remove the ability from the set.*/
    AbilitySet& remove(eFunctionAbility prop)
    {
      _usageSet[prop] = false;
      return *this;
    }

  private:
    std::vector<bool> _usageSet;

    /** Element by element \a and and \or of two AbilitySet*/
    //@{
    friend AbilitySet operator& (const AbilitySet a1, const AbilitySet a2);
    friend AbilitySet operator| (const AbilitySet a1, const AbilitySet a2);
    //@}
  };

  inline AbilitySet operator& (const AbilitySet a1, const AbilitySet a2)
  {
    size_t n = a1.getUsageSet().size();
    ocra_assert(n == a2.getUsageSet().size() && "the sets must have the same size");
    std::vector<bool> props;
    props.reserve(n);
    for (size_t i=0; i<n; ++i)
      props.push_back(a1.getUsageSet()[i] && a2.getUsageSet()[i]);
    return AbilitySet(props);
  }

  inline AbilitySet operator| (const AbilitySet a1, const AbilitySet a2)
  {
    size_t n = a1.getUsageSet().size();
    ocra_assert(n == a2.getUsageSet().size() && "the sets must have the same size");
    std::vector<bool> props;
    props.reserve(n);
    for (size_t i=0; i<n; ++i)
      props.push_back(a1.getUsageSet()[i] || a2.getUsageSet()[i]);
    return AbilitySet(props);
  }
}



#endif //_OCRA_OPTIM_ABILITY_SET_H_

// cmake:sourcegroup=Function
