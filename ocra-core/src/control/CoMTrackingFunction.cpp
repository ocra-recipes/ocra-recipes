#if 0

#include "CoMTrackingFunction.h"

//#define DUMP_COM_TRACKING

namespace ocra
{
  CoMTrackingFunction::CoMTrackingFunction(Robot* robot, const Target& target, ECartesianDof fixedPosition)
    :Function(*robot->getVariableJointPosition(), computeDimensionFor(fixedPosition)
              , L_UNDEFINED, C_UNDEFINED, C_INF, false, true)
    ,_target(target), _fixedPosition(fixedPosition), _robot(robot)
  {
#ifdef DUMP_COM_TRACKING
    std::ofstream aof("com.sci");
      aof << "CoM = [";
    aof.close();
    std::ofstream aof2("comTarget.sci");
      aof2 << "target = [";
    aof2.close();
#endif
  }

  void CoMTrackingFunction::changeTarget(const Target& target)
  {
    _target = target;
  }

  void CoMTrackingFunction::computeValue(void) const
  {
    int r=0;
    const Vector3& com = _robot->getModel().getCenterOfMass();
    const double mass = _robot->getModel().getMass();
    /*const Matrix& massMatrix = _robot->getModel().getMassRef();
    Vector3 p;
    p[0] = -massMatrix(1,5);
    p[1] = massMatrix(0,5);
    p[2] = -massMatrix(0,4);
    p /= _robot->getModel().getMass();
    Displacement dcom;
    dcom.getTranslation() = p;
    std::cout << "p   = " << p << std::endl;
    std::cout << "p2  = " << _robot->get_H_root_bm()*p << std::endl;*/
    //std::cout << "com = " << com << std::endl;
    //std::cout << "t_com = " << _target.frame.getTranslation() << std::endl;
#ifdef DUMP_COM_TRACKING
    std::ofstream aof("com.sci",std::ios_base::app);
    aof << "[" << com[0] << ", " << com[1] << ", " << com[2] <<" ];" << std::endl;
    aof.close();
    std::ofstream aof2("comTarget.sci",std::ios_base::app);
    aof2 << "[" << _target.frame.getTranslation()[0] << ", " << _target.frame.getTranslation()[1] << ", " << _target.frame.getTranslation()[2] <<" ];" << std::endl;
    aof2.close();
#endif
    for (int i=0; i<3; ++i)
    {
      if (_fixedPosition & (1 << i))  //if coordinate i is used 
        _value[r++] = mass*(com[i] - _target.frame.getTranslation()[i]);         //copy the corresponding value
    }
    //std::cout << "deltaCoM = " << _value[0]/mass << ", " << _value[1]/mass << ", " <<_value[2]/mass << std::endl;
  }

  void CoMTrackingFunction::computeGradient(void) const
  {
    const Matrix& g = _robot->getModel().getCtMCscene();

    //std::cout << _robot->getModel().getCtMCscene() << std::endl << std::endl;
    //std::cout << _robot->getModel().getCtMCroot() << std::endl << std::endl;
    //std::cout << _robot->getModel().getMassRef() << std::endl;
    int r = 0;
    for (int i=0; i<3; ++i)
    {
      if (_fixedPosition & (1 << i))
      {
        for (int j=0; j<(int)_x->getSize(); ++j)
          _gradient(r,j) = g(i,j);
        ++r;
      }
    }
  }

  void CoMTrackingFunction::computeHessian(void) const
  {
    throw std::runtime_error("[CoMTrackingFunction::computeHessian] No hessian computation");
  }

  void CoMTrackingFunction::computeJXdot(void) const
  {
    Vector3 v;
    v.copyValuesFrom(_target.twist.getPointVelocity());
    //std::cout << "v_des_com = " << v << std::endl;
    //std::cout << (Vector)(*_x->getTimeDerivative()) << std::endl;
    xde::CML_gemv<'n'>(1., _robot->getModel().getCtMCscene(), (*_x->getTimeDerivative()), -1., v); //TODO : directly get the COM speed
    int r=0;
    for (int i=0; i<3; ++i)
    {
      if (_fixedPosition & (1 << i))  //if coordinate i is used 
        _JXdot[r++] = v[i];           //copy the corresponding value
    }
    //std::cout << "deltaV = " << _JXdot << std::endl;

    /*const Skeleton& skeleton = _robot->getModel().getSkeleton();
    for(size_t i=0; i<skeleton.getNrOfSegments(); ++i)
    {
      gvmRigidBody* body = skeleton.getSegment(i)->getBodyModel();
      const real mi = body->getMass();*/
  }

  void CoMTrackingFunction::computeJdot(void) const
  {
    _Jdot.setToZero();
  }

  void CoMTrackingFunction::computeJdotXdot(void) const
  {
    _JdotXdot.setToZero(); //TODO : initialize to 0: size does not change
  }

  void CoMTrackingFunction::doUpdateSize(void)
  {
  }
}

#endif

// cmake:sourcegroup=Functions
