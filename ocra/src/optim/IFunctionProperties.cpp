#include "IFunctionProperties.h"
#include "ocra/optim/ocra_assert.h"
#include <algorithm>


namespace ocra
{
  IFunctionProperties::IFunctionProperties(eFunctionLinearity linearity, eFunctionConvexity convexity, int continuity,
                                           bool explicitlyTimeDependant, bool separableTimeDependancy)
    : _type(linearity)
    , _convexityProperty(convexity)
    , _continuityProperty(continuity)
    , _timeDependant(explicitlyTimeDependant)
    , _separableTimeDependancy(separableTimeDependancy)
  {
    ocra_assert((continuity==CONTINUITY_UNKNOWN) || (continuity>0 && continuity<=CONTINUITY_CINF));
  }


  eFunctionLinearity IFunctionProperties::getType(void) const
  {
    return _type;
  }

  eFunctionConvexity IFunctionProperties::getConvexityProperty(void) const
  {
    return _convexityProperty;
  }

  int IFunctionProperties::getContinuityProperty(void) const
  {
    return _continuityProperty;
  }
    
  void IFunctionProperties::changeType(eFunctionLinearity newType)
  {
    _type = newType;
  }

  void IFunctionProperties::changeConvexityProperty(eFunctionConvexity newProperty)
  {
    _convexityProperty = newProperty;
  }

  void IFunctionProperties::changeContinuityProperty(int newProperty)
  {
    ocra_assert((newProperty==CONTINUITY_UNKNOWN) || (newProperty>0 && newProperty<=CONTINUITY_CINF));
    _continuityProperty = newProperty;
  }

  void IFunctionProperties::addProperty(const std::string& functionProperty)
  {
    std::vector<std::string>::iterator it = std::find(_properties.begin(), _properties.end(), functionProperty);
    if (it == _properties.end())
      _properties.push_back(functionProperty);
  }

  void IFunctionProperties::removeProperty(const std::string& functionProperty)
  {
    std::vector<std::string>::iterator it = std::find(_properties.begin(), _properties.end(), functionProperty);
    if (it != _properties.end())
      _properties.erase(it);

    ocra_assert(std::find(_properties.begin(), _properties.end(), functionProperty) == _properties.end());
  }

  const std::string& IFunctionProperties::getProperty(int i) const
  {
    return _properties[i];
  }

  int IFunctionProperties::getNumberOfProperties(void) const
  {
    return (int)_properties.size();
  }

  bool IFunctionProperties::hasProperty(const std::string& functionProperty) const
  {
    std::vector<std::string>::const_iterator it = std::find(_properties.begin(), _properties.end(), functionProperty);
    return (it != _properties.end());
  }

  bool IFunctionProperties::isExplicitlyTimeDependant(void) const
  {
    return _timeDependant;
  }

  bool IFunctionProperties::hasSeparableTimeDependancy(void) const
  {
    return !_timeDependant || _separableTimeDependancy;
  }

  void IFunctionProperties::changeExplicitTimeDependancy(bool b)
  {
    _timeDependant = b;
  }

  void IFunctionProperties::changeSeparableTimeDependancy(bool b)
  {
    _separableTimeDependancy = b;
  }
}

// cmake:sourcegroup=Function
