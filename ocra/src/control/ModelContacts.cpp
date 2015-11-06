#include "ModelContacts.h"
#include "ocra/control/Model.h"
#include "ocra/control/Feature.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/ocra_assert.h"
#include <algorithm>
#include <vector>
#include <utility>

namespace ocra
{
  struct ModelContacts::Pimpl
  {
    Model& model;
    CompositeVariable contactForcesVariable;
    MatrixXd Jct;
    bool Jct_isUpToDate;
    std::vector<const Feature*>  contactFeatures;
    std::vector<Variable*> forceVariables;

    Pimpl(Model& m)
      : model(m)
      , contactForcesVariable(m.getName()+"_fc")
      , Jct_isUpToDate(false)
    {}
  };

  ModelContacts::ModelContacts(Model& model)
    : pimpl(new Pimpl(model))
  {
    pimpl->model.connect<EVT_CHANGE_VALUE>(*this, &ModelContacts::invalidate);
  }

  ModelContacts::~ModelContacts()
  {
    pimpl->model.disconnect<EVT_CHANGE_VALUE>(*this, &ModelContacts::invalidate);
  }

  ModelContacts& ModelContacts::addContactPoint(Variable& f, const Feature& contactFeature)
  {
    ocra_assert(std::find(pimpl->contactFeatures.begin(), pimpl->contactFeatures.end(), &contactFeature) == pimpl->contactFeatures.end() 
      && "This contact point was already added");
    ocra_assert(std::find(pimpl->forceVariables.begin(), pimpl->forceVariables.end(), &f) == pimpl->forceVariables.end() 
      && "This force variable was already added");
    ocra_assert(contactFeature.getDimension()==3 && "Only 3D features can be used for contact points");

    // add the variable
    pimpl->contactForcesVariable.add(f);
    pimpl->forceVariables.push_back(&f);

    // add contact in vector
    pimpl->contactFeatures.push_back(&contactFeature);

    // invalidate jacobian
    invalidate();

    return *this;
  }

  ModelContacts& ModelContacts::removeContactPoint(Variable& f)
  {
    std::vector<Variable*>::iterator it = std::find(pimpl->forceVariables.begin(), pimpl->forceVariables.end(), &f);
    
    if (it != pimpl->forceVariables.end())
    {
      size_t i = std::distance(pimpl->forceVariables.begin(), it);
      pimpl->forceVariables.erase(it);
      pimpl->contactForcesVariable.remove(f);
      pimpl->contactFeatures.erase(pimpl->contactFeatures.begin()+i);
      invalidate();
    }

    return *this;
  }

  void ModelContacts::removeAllContacts()
  {
    while (pimpl->forceVariables.size()>0)
    {
      pimpl->contactFeatures.pop_back();
      pimpl->contactForcesVariable.remove(*pimpl->forceVariables.back());
      pimpl->forceVariables.pop_back();
    }

    invalidate();
  }

  Variable& ModelContacts::getContactForcesVariable() const
  {
    return pimpl->contactForcesVariable;
  }

  const MatrixXd& ModelContacts::getJct() const
  {
    if(!pimpl->Jct_isUpToDate)
    {
      pimpl->Jct.resize(pimpl->model.nbDofs(), 3*nbContactPoints());
      for(int i = 0; i < nbContactPoints(); ++i)
        pimpl->Jct.block(0,3*i,pimpl->Jct.rows(),3) = getContactFeature(i).computeJacobian().transpose();
      pimpl->Jct_isUpToDate = true;
    }

    return pimpl->Jct;
  }

  int ModelContacts::nbContactPoints() const
  {
    return static_cast<int>(pimpl->contactFeatures.size());
  }

  std::pair<const Variable*, const Feature*> ModelContacts::getContactPoint(int index) const
  {
    return std::make_pair(pimpl->forceVariables[index], pimpl->contactFeatures[index]);
  }

  const Variable& ModelContacts::getContactForceVariable(int index) const
  {
    return *pimpl->forceVariables[index];
  }

  const Feature& ModelContacts::getContactFeature(int index) const
  {
    return *pimpl->contactFeatures[index];
  }

  const Model& ModelContacts::getModel() const
  {
    return pimpl->model;
  }

  void ModelContacts::invalidate(int timestamp)
  {
    pimpl->Jct_isUpToDate = false;
  }
}

// cmake:sourcegroup=Api
