#include "TrackingFunction.h"
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"



namespace ocra
{
  TrackingFunction::TrackingFunction(const SegmentFrame& segmentFrame, ECartesianDof fixedPosition, ECartesianDof fixedOrientation)
    :NamedInstance("tracking function")
    ,AbilitySet(FUN_VALUE, PARTIAL_X, FUN_DOT)
    ,CoupledInputOutputSize(false)
    ,Function(segmentFrame.getModel().getConfigurationVariable(), utils::computeDimensionFor(fixedPosition, fixedOrientation))
    , _fixedPosition(fixedPosition)
    , _fixedOrientation(fixedOrientation)
    , _target(segmentFrame.getName()+"_tracking_target", segmentFrame.getModel())
  {
    if (fixedOrientation != NONE && fixedOrientation != XYZ)
      throw std::runtime_error("[TrackingFunction::TrackingFunction]: the only options supported for fixedOrientation are NONE and XYZ.");

    const std::string name = segmentFrame.getName()+"_tracking";
    if (fixedPosition)
    {
      if (fixedOrientation)
      {
        _robotFeature = new DisplacementFeature(name+"_robotFeature", segmentFrame, fixedPosition);
        _targetFeature = new DisplacementFeature(name+"_targetFeature", _target, fixedPosition);
      }
      else
      {
        _robotFeature = new PositionFeature(name+"_robotFeature", segmentFrame, fixedPosition);
        _targetFeature = new PositionFeature(name+"_targetFeature", _target, fixedPosition);
      }
    }
    else
    {
      if (fixedOrientation)
      {
        _robotFeature = new OrientationFeature(name+"_robotFeature", segmentFrame);
        _targetFeature = new OrientationFeature(name+"_targetFeature", _target);
      }
      else
        throw std::runtime_error("[TrackingFunction::TrackingFunction]: At least one of the dimension must be fixed");
    }

    segmentFrame.getModel().connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
  }


  TrackingFunction::TrackingFunction(const CoMFrame& comFrame, ECartesianDof fixedPosition)
    :NamedInstance("tracking function")
    ,AbilitySet(FUN_VALUE, PARTIAL_X, FUN_DOT)
    ,CoupledInputOutputSize(false)
    ,Function(comFrame.getModel().getConfigurationVariable(), utils::computeDimensionFor(fixedPosition, NONE))
    , _fixedPosition(fixedPosition)
    , _fixedOrientation(NONE)
    , _target(comFrame.getName()+"_tracking_target", comFrame.getModel())
  {
    const std::string name = comFrame.getName()+"_tracking";
    if (fixedPosition)
    {
      _robotFeature = new PositionFeature(name+"_robotFeature", comFrame, fixedPosition);
      _targetFeature = new PositionFeature(name+"_targetFeature", _target, fixedPosition);
    }
    else
    {
      throw std::runtime_error("[TrackingFunction::TrackingFunction]: At least one of the dimension must be fixed");
    }

    comFrame.getModel().connect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
  }


  TrackingFunction::~TrackingFunction()
  {
    delete _robotFeature;
    delete _targetFeature;
    //the following line is valid as long as _target is built with the same model as segmentFrame
    _target.getModel().disconnect<EVT_CHANGE_VALUE>(*this, &Function::invalidateAll);
  }


  void TrackingFunction::setDesiredDisplacement(const Eigen::Displacementd& Hdes)
  {
    _target.setPosition(Hdes);
    invalidateAll();
    propagate<EVT_CHANGE_VALUE>();
  }

  void TrackingFunction::setDesiredVelocity(const Eigen::Twistd& Tdes)
  {
    _target.setVelocity(Tdes);
    invalidateAll();
    propagate<EVT_CHANGE_VALUE>();
  }

  void TrackingFunction::updateValue() const
  {
    _value = _robotFeature->computeError(*_targetFeature);
  }


  void TrackingFunction::updateJacobian() const
  {
    _jacobian = _robotFeature->computeJacobian(*_targetFeature);
  }


  void TrackingFunction::updateFdot() const
  {
    IFunction<FUN_DOT>::_val = _robotFeature->computeErrorDot(*_targetFeature);
  }
}

// cmake:sourcegroup=Functions
