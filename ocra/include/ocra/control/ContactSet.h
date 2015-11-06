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
    ContactSet(const std::string& name, const Controller& factory, const SegmentFrame& body, double mu, double margin);
    virtual ~ContactSet();

    void addContactFrame(const Eigen::Displacementd& frame);

    ContactConstraintFeature& getBodyFeature() const;
    Task& getBodyTask() const;
    const std::vector<SegmentFrame*>& getPoints() const;
    const std::vector<PointContactFeature*>& getFeatures() const;
    const std::vector<Task*>& getTasks() const;

    const SegmentFrame& getBodyFrame() const;
    double getMu() const;
    double getMargin() const;
    int getNFacets() const;

    void setMu(double mu);
    void setMargin(double margin);
    void setNFacets(int nFacets);

  private:
    const Controller& _factory;
    const SegmentFrame& _body;
    ContactConstraintFeature* _bodyFeature;
    Task* _bodyTask;
    std::vector<SegmentFrame*> _points;
    std::vector<PointContactFeature*> _features;
    std::vector<Task*> _tasks;
    double _mu;
    double _margin;
    int _nFacets;
  };
}

#endif

// cmake:sourcegroup=Api
