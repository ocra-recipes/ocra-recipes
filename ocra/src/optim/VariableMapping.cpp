#include "VariableMapping.h"
#include "ocra/optim/Variable.h"

namespace ocra
{
  // ------------------------------------------------------------
  // --- VARIABLE MAPPING ---------------------------------------
  // ------------------------------------------------------------

  VariableMapping::VariableMapping(const Variable& relative, const Variable& base)
    :_relative(relative), _base(base), _validated(false)
  {
    _base.connect<EVT_CHANGE_DEPENDENCIES>(*this, &VariableMapping::invalidate);
    _base.connect<EVT_RESIZE>(*this, &VariableMapping::invalidate);
    _relative.connect<EVT_CHANGE_DEPENDENCIES>(*this, &VariableMapping::invalidate);
    _relative.connect<EVT_RESIZE>(*this, &VariableMapping::invalidate);
  }

  VariableMapping::~VariableMapping()
  {
    _relative.disconnect<EVT_RESIZE>(*this, &VariableMapping::invalidate);
    _relative.disconnect<EVT_CHANGE_DEPENDENCIES>(*this, &VariableMapping::invalidate);
    _base.disconnect<EVT_RESIZE>(*this, &VariableMapping::invalidate);
    _base.disconnect<EVT_CHANGE_DEPENDENCIES>(*this, &VariableMapping::invalidate);
  }

  const std::vector<int>& VariableMapping::getMapping() const
  {
    if (!_validated)
      updateMapping();
    return _mapping;
  }

  const Variable& VariableMapping::getBaseVariable() const
  {
    return _base;
  }

  const Variable& VariableMapping::getRelativeVariable() const
  {
    return _relative;
  }

  void VariableMapping::updateMapping() const
  {
    _base.getRelativeMappingOf(_relative, _mapping);
    _validated = true;
  }

  void VariableMapping::invalidate(int)
  {
    _validated = false;
  }
}


// cmake:sourcegroup=Solvers
