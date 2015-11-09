#if 0

#include "ocra/control/CoMFunction.h"



namespace ocra
{
  CoMFunction::CoMFunction(Robot* robot)
    :Function (*robot->getVariableJointPosition(), 3, L_UNDEFINED, C_UNDEFINED, C_INF, false, true)
    ,_robot(robot)
  {
  }


  void CoMFunction::computeValue(void)  const
  {
    _value = _robot->getModel().getCenterOfMass();
    //std::cout << "CoM: (" << _value[0] << ", " << _value[1] << ", " << _value[2] << std::endl;
  }

  void CoMFunction::computeGradient(void)  const
  {
    _gradient.copyValuesFrom(_robot->getModel().getCtMCscene());
  }

  void CoMFunction::computeHessian(void)  const
  {
    throw std::runtime_error("[CoMFunction::computeHessian] No hessian computation");
  }

  void CoMFunction::computeJdot(void) const
  {
    throw std::runtime_error("[CoMFunction::computeJdot] Function is not implemented yet");
  }

  void CoMFunction::computeJdotXdot(void) const
  {
    throw std::runtime_error("[CoMFunction::computeJdotXdot] Function is not implemented yet");
  }

  void CoMFunction::doUpdateSize(void)
  {
  }

}

#endif

// cmake:sourcegroup=Functions
