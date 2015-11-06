/** @file ModelContacts.h
  * @brief Declaration file of the ModelContacts class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *   Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
  *
  * @author Brisset Julien
  * @author Escande Adrien
  *	@date 2009/04/28
  *
  * File history:
  *  - 10/07/29: Escande Adrien - Adaptation to Model and switch to Eigen.
  *  - 10/12/03: Evrard Paul - adaptation to new Api.
  */

#ifndef _OCRACONTROL_ROBOT_CONTACTS_H_
#define _OCRACONTROL_ROBOT_CONTACTS_H_

#ifdef WIN32
# pragma once
#endif

#include "ocra/optim/ObserverSubject.h"
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace ocra
{
  class Model;
  class Variable;
  class Feature;
}

namespace ocra
{
  /** @class ModelContacts
    *	@brief %ModelContacts class.
    *	@warning None
    *  
    * Concatenates the contact variables and jacobians for a model. This class
    * is not necessary to handle contacts, but an be useful if a component of
    * your controller handles all contact points as a whole.
    * For instance, Task objects will generally manipulate forces independently.
    * But if you need to use a DynamicEquationFunction in your control, then
    * you have to update the ModelContacts of your Model so that they are
    * included in the dynamic equation of your model.
    *
    * NB: the forces considered here are applied by the manikin on the environment.
    *
    * TODO: review update mechanism.
    */
  class ModelContacts
    : public ObserverSubject
  {
  public:
    ModelContacts(Model& model);
    ~ModelContacts();

  public:
    ModelContacts& addContactPoint(Variable& f, const Feature& contactFeature);
    ModelContacts& removeContactPoint(Variable& f);
    void removeAllContacts();

    Variable& getContactForcesVariable() const;
    const Eigen::MatrixXd& getJct() const;

    int nbContactPoints() const;
    std::pair<const Variable*, const Feature*> getContactPoint(int index) const; 
    const Variable& getContactForceVariable(int index) const;
    const Feature& getContactFeature(int index) const;

    const Model& getModel() const;

  private:
    void invalidate(int timestamp = 0);

  private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
  };
}

#endif	//_OCRACONTROL_ROBOT_CONTACTS_H_

// cmake:sourcegroup=Api
