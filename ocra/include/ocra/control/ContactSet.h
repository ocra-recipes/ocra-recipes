/*!
\file ContactSet.h
\brief Factory classes and functions to build sets of contact frames.

Copyright (C) 2010 CEA/DRT/LIST/DTSI/SRCI

\author Evrard Paul
\date 2010/10/03

File history:
*/

#ifndef _OCRA_CONTACT_SET_H_
#define _OCRA_CONTACT_SET_H_

#include "ocra/optim/NamedInstance.h"
#include <Eigen/Lgsm>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <memory>
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
  class ContactSetBuilder;
  class SegmentFrame;
  class ContactConstraintFeature;
  class PointContactFeature;
  class Task;
  class Model;
  class Controller;
}

namespace ocra
{
  // ------------------------------------------------------------
  // --- BASE CLASS ---------------------------------------------
  // ------------------------------------------------------------

  //! Contact task factory.
  /*!
  Creates a set of contact
  This object must be used as a factory. It does not manage its memory, so you have to delete the
  created frames yourself.
  */
  class ContactSet
    : public NamedInstance
  {
  public:
    ContactSet(const std::string& name, const Controller& factory, SegmentFrame::Ptr body, double mu, double margin);
    virtual ~ContactSet();

    void addContactFrame(const Eigen::Displacementd& frame);

    ContactConstraintFeature& getBodyFeature() const;
    std::shared_ptr<Task> getBodyTask() const;
    const std::vector<SegmentFrame::Ptr>& getPoints() const;
    const std::vector<PointContactFeature::Ptr>& getFeatures() const;
    const std::vector<std::shared_ptr<Task>>& getTasks() const;

    SegmentFrame::Ptr getBodyFrame() const;
    double getMu() const;
    double getMargin() const;
    int getNFacets() const;

    void setMu(double mu);
    void setMargin(double margin);
    void setNFacets(int nFacets);

  private:
    const Controller& _factory;
    const SegmentFrame::Ptr _body;
    ContactConstraintFeature::Ptr _bodyFeature;
    std::shared_ptr<Task> _bodyTask;
    std::vector<SegmentFrame::Ptr> _points;
    std::vector<PointContactFeature::Ptr> _features;
    std::vector<std::shared_ptr<Task>> _tasks;
    double _mu;
    double _margin;
    int _nFacets;
  };
}

#endif

// cmake:sourcegroup=Api
