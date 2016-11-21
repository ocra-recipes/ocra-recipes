#include "ocra/control/ContactSet.h"



namespace ocra
{
  ContactSet::ContactSet(const std::string& name, const Controller& factory, SegmentFrame::Ptr body, double mu, double margin)
    : NamedInstance(name)
    , _factory(factory)
    , _body(body)
    , _bodyFeature(std::make_shared<ContactConstraintFeature>(body->getName()+".feature", body))
    , _bodyTask(factory.createTask(body->getName()+".task", _bodyFeature))
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

    SegmentFrame::Ptr sf = std::make_shared<SegmentFrame>(ss.str(), _body->getModel(), _body->getSegmentIndex(), frame);
    _points.push_back(sf);

    PointContactFeature::Ptr pf = std::make_shared<PointContactFeature>(sf->getName()+".feature", sf);
    _features.push_back(pf);

    std::shared_ptr<Task> task = _factory.createContactTask(sf->getName()+".task", pf, _mu, _margin);
    _tasks.push_back(task);
  }

  SegmentFrame::Ptr ContactSet::getBodyFrame() const
  {
    return _body;
  }

  ContactConstraintFeature& ContactSet::getBodyFeature() const
  {
    return *_bodyFeature;
  }

  std::shared_ptr<Task> ContactSet::getBodyTask() const
  {
    return _bodyTask;
  }

  const std::vector<SegmentFrame::Ptr>& ContactSet::getPoints() const
  {
    return _points;
  }

  const std::vector<PointContactFeature::Ptr>& ContactSet::getFeatures() const
  {
    return _features;
  }

  const std::vector<std::shared_ptr<Task>>& ContactSet::getTasks() const
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
