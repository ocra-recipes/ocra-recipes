#if 0

#include "ocra/control/StaticEquationFunction.h"
#include "ocra/optim/CompositeVariable.h"
#include <sstream>

using namespace xde;

namespace ocra
{
  StaticEquationFunction::StaticEquationFunction(Robot* robot)
    :LinearFunction(createSEVariable(robot), robot->getVariableJointPosition()->getSize())
    , _robot(robot), _q_dot(robot->getVariableJointPosition()->getTimeDerivative()), _tau(robot->getVariableJointEffort())
    , _f(robot->getVariableContactForce())
  {
    SubMatrix A(_gradient);
    A.rescope(_dimension, _tau->getSize(), 0, _q_dot->getSize());
    //CML_gemm<'n', 'n'>(1., _robot->getK(), _robot->getModel().getL(), 0., A);
    //tmp KL
    A.rescope(6, _tau->getSize(), 0, _q_dot->getSize());
    A.setToZero();
    A.rescope(_dimension-6, _tau->getSize(), 6, _q_dot->getSize());
    A.setToIdentity();

    _name = "static equation function";
  }


  Variable& StaticEquationFunction::createSEVariable(Robot* robot)
  {
    VariableManager& vm = robot->getVariableManager();
    static int cpt = 0;
    std::stringstream name;
    name << robot->getName() << "_se" << cpt++;
    CompositeVariable* var = new CompositeVariable(name.str(), 
                                                   *robot->getVariableJointPosition()->getTimeDerivative());

    var->add(*robot->getVariableJointEffort());
    var->add(*robot->getVariableContactForce());
    return *var;
  }

  const Matrix& StaticEquationFunction::getA(void) const
  {
    getGradients();
    return _gradient;
  }

  const Vector& StaticEquationFunction::getb(void) const
  {
    return _robot->getModel().getCtMCG();
  }


  void StaticEquationFunction::computeValue(void)  const
  {
    //force evaluation of A
    getGradients();

    _value.copyValuesFrom(_robot->getModel().getCtMCG());
    CML_gemv<'n'>(1, _gradient, (*_x), 1, _value);
  }


  void StaticEquationFunction::computeGradient(void)  const
  {
    SubMatrix A(_gradient);
    //B
    A.rescope(_dimension, _q_dot->getSize(), 0, 0);
//    A.copyValuesFrom(_robot->getModel().getCtBC());
    const Matrix& B = _robot->getModel().getCtBC();
    for (cfl_size_t i=0; i<_dimension; ++i)
    {
      for (cfl_size_t j=0; j<_q_dot->getSize(); ++j)
      {
        A(i,j) = -1 * B(i,j);
      }
    }
    //no need to do KL
    //Cct
    A.rescope(_dimension, _f->getSize(), 0, _q_dot->getSize() + _tau->getSize());
    A.copyValuesFrom(_robot->getRobotContacts().getCct());
  }


  void StaticEquationFunction::doUpdateSize(void)
  {
  }

  void StaticEquationFunction::doUpdateSizeEnd(void)
  {
    SubMatrix A(_gradient);
    //A.rescope(_dimension, _tau->getSize(), 0, _q_dot->getSize());
    //CML_gemm<'n', 'n'>(1., _robot->getK(), _robot->getModel().getL(), 0., A);
    A.rescope(6, _tau->getSize(), 0, _q_dot->getSize());
    A.setToZero();
    A.rescope(_dimension-6, _tau->getSize(), 6, _q_dot->getSize());
    A.setToIdentity();
  }

}

#endif

// cmake:sourcegroup=Functions
