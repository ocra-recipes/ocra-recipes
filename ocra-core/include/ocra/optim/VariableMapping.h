/** @file VariableMapping.h
 *  @brief Declaration file of the VariableMapping class.
 *
 *         Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
 *
 *  @author Escande Adrien
 *	@date 10.06.16
 */



#ifndef _OCRABASE_VARIABLE_MAPPING_H_
#define _OCRABASE_VARIABLE_MAPPING_H_

#ifdef WIN32
# pragma once
#endif

namespace ocra
{
  class Variable;
}

#include <vector>

namespace ocra
{
  /** \brief A class to manage the relative mapping of a variable with respect to another one.
    *
    * The role of VariableMapping is to provide the relative mapping of one variable v1 wrt. another v2 as it could be
    * directly obtained by invoking v2.getRelativeMappingOf(v1, returnVector). The mapping is updated whenever it is 
    * needed (i.e. after an event EVT_RESIZE or EVT_CHANGE_DEPENDENCIES was triggered by one of the two variable).
    */
  class VariableMapping
  {
  public:
    VariableMapping(const Variable& relative, const Variable& base);
    ~VariableMapping();

  private:
    VariableMapping(const VariableMapping&);
    VariableMapping& operator= (const VariableMapping&);

  public:
    const std::vector<int>& getMapping() const;
    const Variable& getBaseVariable() const;
    const Variable& getRelativeVariable() const;

  private:
    void updateMapping() const;
    void invalidate(int timestamp);

  private:
    mutable std::vector<int>  _mapping;
    const Variable&           _relative;
    const Variable&           _base;
    mutable bool              _validated;
  };
}

#endif //_OCRABASE_VARIABLE_MAPPING_H_

// cmake:sourcegroup=Solvers
