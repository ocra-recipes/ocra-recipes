#include "MergedVariable.h"
#include "ocra/optim/VariableMapping.h"
#include <boost/foreach.hpp>

namespace ocra
{
  MergedVariable::MergedVariable(const std::string& name)
    : _base(name)
    , _isVariableUpToDate(false)
  {
  }

  MergedVariable::~MergedVariable()
  {
    BOOST_FOREACH(map_t::value_type& element, _varToMapping)
      delete element.second.second;
  }

  void MergedVariable::insert(Variable* var)
  {
    var->connect<EVT_CHANGE_DEPENDENCIES>(*this, &MergedVariable::invalidateVariable);
    invalidateVariable();

    map_t::iterator it = _varToMapping.find(var);
    if(it == _varToMapping.end())
      _varToMapping[var] = std::make_pair(1, new VariableMapping(*var, _base));
    else
      ++it->second.first;
  }

  void MergedVariable::remove(Variable* var)
  {
    map_t::iterator it = _varToMapping.find(var);
    ocra_assert(it != _varToMapping.end() && "Precondition violation: nesting class should ensure this never happens");
    ocra_assert(it->second.first > 0 && "Invariant violation: there should always be at least one variable attached to a mapping");

    var->disconnect<EVT_CHANGE_DEPENDENCIES>(*this, &MergedVariable::invalidateVariable);
    invalidateVariable();

    --it->second.first;
    if(it->second.first == 0)
    {
      delete it->second.second; // talk about readability...
      _varToMapping.erase(it);
    }
  }

  VariableMapping* MergedVariable::find(Variable* var) const
  {
    if (!_isVariableUpToDate)
      recomputeVariable();

    map_t::const_iterator it = _varToMapping.find(var);
    return it != _varToMapping.end() ? it->second.second : 0x0;
  }

  const CompositeVariable& MergedVariable::getVariable() const
  {
    if (!_isVariableUpToDate)
      recomputeVariable();

    return _base;
  }

  void MergedVariable::setValue(const VectorXd& val) const
  {
    if (!_isVariableUpToDate)
      recomputeVariable();

    _base.setValue(val);
  }

  void MergedVariable::recomputeVariable() const
  {
    _base.clear();

    BOOST_FOREACH(const map_t::value_type& mapping, _varToMapping)
      _base.addByMerge(*mapping.first);

    _isVariableUpToDate = true;
  }

  void MergedVariable::invalidateVariable()
  {
    _isVariableUpToDate = false;
  }

  void MergedVariable::invalidateVariable(int timestamp)
  {
    invalidateVariable();
  }

  bool MergedVariable::isVariableUpToDate() const
  {
    return _isVariableUpToDate;
  }
}

// cmake:sourcegroup=Variable
