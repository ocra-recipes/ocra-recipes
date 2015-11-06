#include <map>
#include <vector>
#include <iostream>

#include <gocra/Models/gOcraModel.h>

namespace gocra
{

//=================================  Class methods  =================================//
gOcraModel::gOcraModel(const std::string& name, int ndofs, bool freeRoot) :ocra::Model(name, ndofs, freeRoot)
{
}

gOcraModel::~gOcraModel()
{
    
}

int gOcraModel::getDofIndex(const std::string& name) const
{
  return doGetDofIndex(name);
}

const std::string& gOcraModel::getDofName(int index) const
{
  assert(index>=0 && index <ndofs);
  return doGetDofName(index);
}

const std::string gOcraModel::SegmentName(const std::string& name) const
{
    return doSegmentName(name);
}

const std::string gOcraModel::DofName(const std::string& name) const
{
    return doDofName(name);
}

int gOcraModel::doGetDofIndex(const std::string& name) const
{
  throw std::runtime_error("[gOcraModel::doGetDofIndex] This function was not overriden for a specific model");
}

const std::string& gOcraModel::doGetDofName(int index) const
{
  throw std::runtime_error("[gOcraModel::doGetDofName] This function was not overriden for a specific model");
}

// Translates segment name to the format of the model (for example orcXdeModel should be robot.name)
const std::string gOcraModel::doSegmentName(const std::string& name) const
{
  throw std::runtime_error("[gOcraModel::doSegmentName] This function was not overriden for a specific model");
}

// Translates DOF name to the format of the model (for example orcXdeModel should be robot.name)
const std::string gOcraModel::doDofName(const std::string& name) const
{
  throw std::runtime_error("[gOcraModel::doDofName] This function was not overriden for a specific model");
}

}
