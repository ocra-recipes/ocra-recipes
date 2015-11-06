#include "ContactSet.h"

#include "ocra/optim/Constraint.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/optim/Objective.h"

#include "ocra/control/Model.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/control/ControlEnum.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Task.h"
#include "ocra/control/Controller.h"

#include <stdexcept>
#include <sstream>

namespace ocra
{
  ContactSet::ContactSet(const std::string& name, const Controller& factory, const SegmentFrame& body, double mu, double margin)
    : NamedInstance(name)
    , _factory(factory)
    , _body(body)
    , _bodyFeature(new ContactConstraintFeature(body.getName()+".feature", body))
    , _bodyTask(&factory.createTask(body.getName()+".task", *_bodyFeature))
    , _points()
    , _mu(mu)
    , _margin(margin)
    , _nFacets(4)
  {
    _bodyTask->activateContactMode();
  }

  ContactSet::~ContactSet()
  {
  }

  void ContactSet::addContactFrame(const Eigen::Displacementd& frame)
  {
    std::stringstream ss;
    ss << getName() << "." << _points.size();

    SegmentFrame* sf = new SegmentFrame(ss.str(), _body.getModel(), _body.getSegmentIndex(), frame);
    _points.push_back(sf);

    PointContactFeature* pf = new PointContactFeature(sf->getName()+".feature", *sf);
    _features.push_back(pf);

    Task* task = &_factory.createContactTask(sf->getName()+".task", *pf, _mu, _margin);
    _tasks.push_back(task);
  }

  const SegmentFrame& ContactSet::getBodyFrame() const
  {
    return _body;
  }

  ContactConstraintFeature& ContactSet::getBodyFeature() const
  {
    return *_bodyFeature;
  }

  Task& ContactSet::getBodyTask() const
  {
    return *_bodyTask;
  }

  const std::vector<SegmentFrame*>& ContactSet::getPoints() const
  {
    return _points;
  }

  const std::vector<PointContactFeature*>& ContactSet::getFeatures() const
  {
    return _features;
  }

  const std::vector<Task*>& ContactSet::getTasks() const
  {
    return _tasks;
  }

  void ContactSet::setMu(double mu)
  {
    _mu = mu;
  }

  double ContactSet::getMu() const
  {
    return _mu;
  }

  void ContactSet::setMargin(double margin)
  {
    _margin = margin;
  }

  double ContactSet::getMargin() const
  {
    return _margin;
  }

  void ContactSet::setNFacets(int nFacets)
  {
    _nFacets = nFacets;
  }

  int ContactSet::getNFacets() const
  {
    return _nFacets;
  }
}

// cmake:sourcegroup=Api
