#include <map>
#include <vector>
#include <iostream>

#include <wocra/Models/wOcraModel.h>

namespace wocra
{

//=================================  Class methods  =================================//
wOcraModel::wOcraModel(const std::string& name, int ndofs, bool freeRoot) :ocra::Model(name, ndofs, freeRoot)
{
}

wOcraModel::~wOcraModel()
{
    
}

int wOcraModel::getDofIndex(const std::string& name) const
{
  return doGetDofIndex(name);
}

const std::string& wOcraModel::getDofName(int index) const
{
  assert(index>=0 && index <ndofs);
  return doGetDofName(index);
}

const std::string wOcraModel::SegmentName(const std::string& name) const
{
    return doSegmentName(name);
}

const std::string wOcraModel::DofName(const std::string& name) const
{
    return doDofName(name);
}

int wOcraModel::doGetDofIndex(const std::string& name) const
{
  throw std::runtime_error("[wOcraModel::doGetDofIndex] This function was not overriden for a specific model");
}

const std::string& wOcraModel::doGetDofName(int index) const
{
  throw std::runtime_error("[wOcraModel::doGetDofName] This function was not overriden for a specific model");
}

// Translates segment name to the format of the model (for example orcXdeModel should be robot.name)
const std::string wOcraModel::doSegmentName(const std::string& name) const
{
  throw std::runtime_error("[wOcraModel::doSegmentName] This function was not overriden for a specific model");
}

// Translates DOF name to the format of the model (for example orcXdeModel should be robot.name)
const std::string wOcraModel::doDofName(const std::string& name) const
{
  throw std::runtime_error("[wOcraModel::doDofName] This function was not overriden for a specific model");
}

}
