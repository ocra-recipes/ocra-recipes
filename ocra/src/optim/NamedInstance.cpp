#include "ocra/optim/NamedInstance.h"

namespace ocra
{
  NamedInstance::NamedInstance(const std::string& name)
    : name_(name)
  {
  }

  const std::string& NamedInstance::getName() const
  {
    return name_;
  }

  NamedInstance::~NamedInstance()
  {
  }
}

// cmake:sourcegroup=Utils