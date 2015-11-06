#ifndef _OCRA_MERGED_VARIABLE_H_
#define _OCRA_MERGED_VARIABLE_H_

#include "ocra/optim/Variable.h"

#include <string>
#include <map>

namespace ocra
{
  class VariableMapping;
}

namespace ocra
{
  /** \brief Maintains a CompositeVariable by merging sub-variables.
    *
    * A MergedVariable stores a composite variables allows to attach and detach variables from it by merging them.
    * It also stores the mappings of these variables with respect to the parent variable. A subvariable is actually
    * added only once to the merged variable, however, you have to call 'remove' as many times as you have called
    * 'insert' to actually detach it.
    * \example
    * BaseVariable b("b");
    * MergedVariable m("m");
    * m.insert(&b); // attaches b to m
    * m.insert(&b); // nothing is done from the point of view of the client code
    * m.remove(&b); // nothing is done from the point of view of the client code
    * m.remove(&b); // detaches b from m
    */
  class MergedVariable
    : public ObserverSubject
  {
  public:
    MergedVariable(const std::string& name);
    ~MergedVariable();

    void insert(Variable* var);
    void remove(Variable* var);
    VariableMapping* find(Variable* var) const;

    const CompositeVariable& getVariable() const;
    void setValue(const VectorXd& val) const;
    void recomputeVariable() const;
    bool isVariableUpToDate() const;

  private:
    void invalidateVariable();
    void invalidateVariable(int timestamp);

  private:
    typedef std::map<Variable*, std::pair<int, VariableMapping*> > map_t;

    map_t _varToMapping;
    mutable CompositeVariable _base;
    mutable bool _isVariableUpToDate;
  };
}

#endif

// cmake:sourcegroup=Variable
