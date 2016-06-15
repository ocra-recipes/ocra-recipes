// #pragma warning(disable: 4244) // XXX Eigen 3 JacobiSVD
#include "ocra/control/Task.h"


// #pragma warning(default: 4244)

using namespace Eigen;

namespace
{
    enum
    {
        TASK_DEACTIVATED,
        TASK_AS_OBJECTIVE,
        TASK_AS_CONSTRAINT
    };

    struct GainsWorkspace
    {
        JacobiSVD<MatrixXd> svd;
        VectorXd s;
        VectorXd kps;
        VectorXd kds;

        GainsWorkspace(int n) : svd(n, n), s(n), kps(n), kds(n) {}
    };
}

namespace ocra
{
  struct Task::Pimpl
  {
    Feature::Ptr feature;
    Feature::Ptr featureDes;
    int mode;
    GainsWorkspace gainsWorkspace;
    MatrixXd M;
    MatrixXd M_inverse;
    MatrixXd B;
    MatrixXd K;
    VectorXd weight;
    VectorXd output;
    VectorXd error;
    VectorXd errorDot;
    VectorXd errorDdot;
    VectorXd effort;
    MatrixXd jacobian;
    Vector3d frictionOffset;
    double frictionCoeff;
    double margin;
    bool useActualMass;
    bool contactActive;
    TYPETASK innerTaskType;
    int hierarchyLevel;

    std::shared_ptr<Model> innerModel;
    std::shared_ptr<OneLevelSolver> solver;
    const FullDynamicEquationFunction* dynamicEquation;
    bool useReducedProblem;
    BaseVariable fcVar;
    bool taskHasBeenInitialized;
    LessThanZeroConstraintPtr<LinearizedCoulombFunction> frictionConstraint; // if contact task
    bool contactForceConstraintHasBeenSavedInSolver;
    bool contactPointHasBeenSavedInModel;
    bool frictionConstraintIsRegisteredInConstraint;
    EqualZeroConstraintPtr<LinearFunction>  ContactForceConstraint;

    bool isRegisteredAsObjective;
    bool isRegisteredAsConstraint;


    LinearFunction*                         innerObjectiveFunction;
    Objective<SquaredLinearFunction>*       innerTaskAsObjective;
    EqualZeroConstraintPtr<LinearFunction>  innerTaskAsConstraint;

    Pimpl(const std::string& name, std::shared_ptr<Model> m, Feature::Ptr s, Feature::Ptr sdes)
      : feature(s)
      , featureDes(sdes)
      , mode(TASK_DEACTIVATED)
      , gainsWorkspace(s->getDimension())
      , M( MatrixXd::Identity(s->getDimension(), s->getDimension()) )
      , M_inverse( MatrixXd::Identity(s->getDimension(), s->getDimension()) )
      , B( MatrixXd::Zero(s->getDimension(), s->getDimension()) )
      , K( MatrixXd::Zero(s->getDimension(), s->getDimension()) )
      , weight( VectorXd::Ones(s->getDimension()) )
	  , frictionOffset(Vector3d::Zero())
      , frictionCoeff(1.)
      , margin(0.)
      , useActualMass(true)
      , contactActive(false)
      , innerTaskType(UNKNOWNTASK)
      , hierarchyLevel(-1)
      , innerModel(m)
      , solver(0x0)
      , dynamicEquation(0x0)
      , useReducedProblem(false)
      , fcVar(name+".var", s->getDimension())
//        , weight(1.)
      , taskHasBeenInitialized(false)
      , contactForceConstraintHasBeenSavedInSolver(false)
      , contactPointHasBeenSavedInModel(false)
      , frictionConstraintIsRegisteredInConstraint(false)
      , isRegisteredAsObjective(false)
      , isRegisteredAsConstraint(false)
      , innerObjectiveFunction(NULL)
      , innerTaskAsObjective(NULL)
    {
        innerTaskAsConstraint.set(NULL);

        if(fcVar.getSize() == 3)
        {
//            std::cout<<"CAN BE A CONTACT POINT!!! register friction and contact constraints\n";
//            registerFrictionConstraint = true;
            frictionConstraint.set(  new LinearizedCoulombFunction(fcVar, 1., 6, 0.) );
            ContactForceConstraint.set( new LinearFunction( fcVar, Eigen::MatrixXd::Identity(3,3), VectorXd::Zero(3) ) );
        }
        else
        {
//            registerFrictionConstraint = false;
            frictionConstraint.set(NULL);
            ContactForceConstraint.set(NULL);
        }
    }
    ~Pimpl()
    {
        if (innerTaskAsObjective)
        {
            delete &innerTaskAsObjective->getFunction();
            delete innerTaskAsObjective;
        }
    }
    void setAsAccelerationTask()
    {

        int featn = feature->getDimension();
        if (useReducedProblem)
        {
            innerObjectiveFunction = new VariableChiFunction(dynamicEquation->getActionVariable(), featn);
        }
        else
        {
            innerObjectiveFunction = new LinearFunction (innerModel->getAccelerationVariable(), Eigen::MatrixXd::Zero(featn, innerModel->nbDofs()), Eigen::VectorXd::Zero(featn));
        }
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsTorqueTask()
    {
        int featn = feature->getDimension();
        innerObjectiveFunction = new LinearFunction(innerModel->getJointTorqueVariable(), Eigen::MatrixXd::Zero(featn, innerModel->nbInternalDofs()), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void setAsForceTask()
    {
        int featn = feature->getDimension();
        innerObjectiveFunction = new LinearFunction(fcVar, Eigen::MatrixXd::Identity(featn, featn), Eigen::VectorXd::Zero(featn));
        connectFunctionnWithObjectiveAndConstraint();
    }

    void connectFunctionnWithObjectiveAndConstraint()
    {
        innerTaskAsObjective = new Objective<SquaredLinearFunction>(new SquaredLinearFunction(innerObjectiveFunction));//, weight); // Here, it will manage the SquaredLinearFunction build on a pointer of the function.
        innerTaskAsConstraint.set(innerObjectiveFunction);      // As as ConstraintPtr, it will manage the new created function innerObjectiveFunction
    }

  };

  Task::Task(const std::string& name, std::shared_ptr<Model> model, Feature::Ptr feature, Feature::Ptr featureDes)
    : NamedInstance(name)
    , pimpl(new Pimpl(name, model, feature, featureDes))
  {
  }

  Task::Task(const std::string& name, std::shared_ptr<Model> model, Feature::Ptr feature)
    : NamedInstance(name)
    , pimpl(new Pimpl(name, model, feature, 0x0))
  {
  }

  Task::~Task()
  {
  }

  void Task::setTaskType(Task::TYPETASK newTaskType)
  {
      if (getTaskType()==UNKNOWNTASK) {
          pimpl->innerTaskType = newTaskType;
      }else{
          std::cout << "[warning] can't change a task's type once it has been set." << std::endl;
      }
  }
  int Task::getHierarchyLevel()
  {
    return pimpl->hierarchyLevel;
  }
  void Task::setHierarchyLevel(int level)
  {
      std::cout << "\033[1;31m["<<getName()<<"]\033[0m Setting hierarchy level to "<<level<< std::endl;
      if( level < 0 )
      {
          std::cout << "[warning] Level should be >0 , but you provided "<<level << std::endl;
          return;
      }
      if(getHierarchyLevel() == -1) // TODO: find out why this crashes
      {
          pimpl->hierarchyLevel = level;
      }else{
          std::cout << "[warning] Can't change a task's level once it has been set." << std::endl;
      }
      return;
  }
  Task::TYPETASK Task::getTaskType()
  {
      return pimpl->innerTaskType;
  }

  void Task::update()
  {
      switch(getTaskType())
      {

          case(ACCELERATIONTASK):
          {
              updateAccelerationTask();
              break;
          }
          case(TORQUETASK):
          {
              updateTorqueTask();
              break;
          }
          case(FORCETASK):
          {
              updateForceTask();
              break;
          }
          case(COMMOMENTUMTASK):
          {
              updateCoMMomentumTask();
              break;
          }
          case(UNKNOWNTASK):
          {
              throw std::runtime_error(std::string("[Task::update]: The task type has not been set during creation."));
              break;
          }
          default:
          {
              throw std::runtime_error(std::string("[Task::update]: Unhandle case of TYPETASK."));
              break;
          }
      }
  }


  void Task::activateAsObjective()
  {
    if(pimpl->mode == TASK_AS_OBJECTIVE)
      return;
    else if(pimpl->mode == TASK_AS_CONSTRAINT)
      deactivate();

    doActivateAsObjective();
    if(pimpl->contactActive)
      doActivateContactMode();

    pimpl->mode = TASK_AS_OBJECTIVE;
  }

  void Task::activateAsConstraint()
  {
    if(pimpl->mode == TASK_AS_CONSTRAINT)
      return;
    else if(pimpl->mode == TASK_AS_OBJECTIVE)
      deactivate();

    doActivateAsConstraint();
    if(pimpl->contactActive)
      doActivateContactMode();

    pimpl->mode = TASK_AS_CONSTRAINT;
  }

  void Task::deactivate()
  {
    if(pimpl->mode == TASK_DEACTIVATED)
      return;

    if(pimpl->mode == TASK_AS_OBJECTIVE)
      doDeactivateAsObjective();
    else if(pimpl->mode == TASK_AS_CONSTRAINT)
      doDeactivateAsConstraint();

    if(pimpl->contactActive)
      doDeactivateContactMode();

    pimpl->mode = TASK_DEACTIVATED;
  }

  bool Task::isActiveAsObjective() const
  {
    return (pimpl->mode == TASK_AS_OBJECTIVE);
  }

  bool Task::isActiveAsConstraint() const
  {
    return (pimpl->mode == TASK_AS_CONSTRAINT);
  }

  void Task::setDesiredMassToActualOne()
  {
    pimpl->useActualMass = true;
  }

  void Task::setDesiredMass(double Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->M.diagonal().setConstant(Md);
  }

  void Task::setDesiredMass(const VectorXd& Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->M.diagonal() = Md;
  }

  void Task::setDesiredMass(const MatrixXd& Md)
  {
    pimpl->useActualMass = false;
    pimpl->M = Md;
  }

  void Task::setDamping(double B)
  {
    pimpl->B = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->B.diagonal().setConstant(B);
  }

  void Task::setDamping(const VectorXd& B)
  {
    pimpl->B = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->B.diagonal() = B;
  }

  void Task::setDamping(const MatrixXd& B)
  {
    pimpl->B = B;
  }

  void Task::setStiffness(double K)
  {
    pimpl->K = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->K.diagonal().setConstant(K);
  }

  void Task::setStiffness(const VectorXd& K)
  {
    pimpl->K = MatrixXd::Zero(pimpl->feature->getDimension(), pimpl->feature->getDimension());
    pimpl->K.diagonal() = K;
  }

  void Task::setStiffness(const MatrixXd& K)
  {
    pimpl->K = K;
  }

  void Task::setAutoGains(double freq)
  {
    setAutoGains(freq, 100.);
  }

  void Task::setAutoGains(double freq, double massSaturation)
  {
    JacobiSVD<MatrixXd>& svd = pimpl->gainsWorkspace.svd;
    svd.compute(getDesiredMass(), ComputeFullU | ComputeFullV);

    const VectorXd& s = svd.singularValues();
    pimpl->gainsWorkspace.s = (s.array() < massSaturation).select(s, massSaturation);

    const double w = 2. * M_PI * freq;
    VectorXd& kps = pimpl->gainsWorkspace.kps;
    VectorXd& kds = pimpl->gainsWorkspace.kds;

    kps = s * w*w; // kp[i] = s[i] * w^2
    kds = (kps.array()*s.array()).sqrt() * 2.; // kd[i] = 2 * sqrt(kp[i]*s[i])

    pimpl->K = svd.matrixU() * kps.asDiagonal() * svd.matrixV().transpose();
    pimpl->B = svd.matrixU() * kds.asDiagonal() * svd.matrixV().transpose();
  }

  bool Task::isDesiredMassTheActualOne() const
  {
    return pimpl->useActualMass;
  }

  const MatrixXd& Task::getDesiredMass() const
  {
    if(pimpl->useActualMass)
    {
      if(pimpl->featureDes)
        return pimpl->feature->computeProjectedMass(*pimpl->featureDes);
      else
        return pimpl->feature->computeProjectedMass();
    }
    return pimpl->M;
  }

  const MatrixXd& Task::getDesiredMassInverse() const
  {
    if(pimpl->useActualMass)
    {
      if(pimpl->featureDes)
        return pimpl->feature->computeProjectedMassInverse(*pimpl->featureDes);
      else
        return pimpl->feature->computeProjectedMassInverse();
    }
    pimpl->M_inverse = pimpl->M.inverse();
    return pimpl->M_inverse;
  }

  const MatrixXd& Task::getDamping() const
  {
    return pimpl->B;
  }

  const MatrixXd& Task::getStiffness() const
  {
    return pimpl->K;
  }

  void Task::activateContactMode()
  {
    if(pimpl->contactActive)
      return;

    if((pimpl->feature->getDimension() != 3) && (pimpl->feature->getDimension() != 6))
      throw std::runtime_error("[Task::activateContactMode] Contact mode is available only for features with dimension 3 or 6");

    if(isActiveAsConstraint() || isActiveAsObjective())
      doActivateContactMode();

    pimpl->contactActive = true;
  }

  void Task::deactivateContactMode()
  {
    if(!pimpl->contactActive)
      return;

    if(isActiveAsConstraint() || isActiveAsObjective())
      doDeactivateContactMode();

    pimpl->contactActive = false;
  }

  void Task::setWeight(double weight)
  {
    pimpl->weight.setConstant(weight);
    doSetWeight();
  }

  bool Task::isContactModeActive() const
  {
    return pimpl->contactActive;
  }

  bool Task::isBodyContactConstraint() const
  {
    return isContactModeActive() && getDimension() != 3;
  }

  bool Task::isPointContactTask() const
  {
    return isContactModeActive() && getDimension() == 3;
  }

  double Task::getFrictionCoeff() const
  {
    return pimpl->frictionCoeff;
  }

  double Task::getMargin() const
  {
    return pimpl->margin;
  }

	const Vector3d& Task::getFrictionConstraintOffset() const
	{
		return pimpl->frictionOffset;
	}

  void Task::setFrictionCoeff(double coeff)
  {
    pimpl->frictionCoeff = coeff;
    doSetFrictionCoeff();
  }

  void Task::setMargin(double margin)
  {
    pimpl->margin = margin;
    doSetMargin();
  }

  void Task::setWeight(const VectorXd& weight)
  {
    pimpl->weight = weight;
    doSetWeight();
  }

	void Task::setFrictionConstraintOffset(const Vector3d& offset)
	{
		pimpl->frictionOffset = offset;
	}

  const VectorXd& Task::getWeight() const
  {
    return pimpl->weight;
  }

  int Task::getDimension() const
  {
    return getFeature()->getDimension();
  }

  const VectorXd& Task::getOutput() const
  {
    doGetOutput(pimpl->output);
    return pimpl->output;
  }

  const VectorXd& Task::getError() const
  {
    pimpl->error = pimpl->featureDes ? pimpl->feature->computeError(*pimpl->featureDes) : pimpl->feature->computeError();
    return pimpl->error;
  }

  const VectorXd& Task::getErrorDot() const
  {
    pimpl->errorDot = pimpl->featureDes ? pimpl->feature->computeErrorDot(*pimpl->featureDes) : pimpl->feature->computeErrorDot();
    return pimpl->errorDot;
  }

  const VectorXd& Task::getErrorDdot() const
  {
    pimpl->errorDdot = pimpl->featureDes ? pimpl->feature->computeAcceleration(*pimpl->featureDes) : pimpl->feature->computeAcceleration();
    return pimpl->errorDdot;
  }

  const VectorXd& Task::getEffort() const
  {
    pimpl->effort = pimpl->featureDes ? pimpl->feature->computeEffort(*pimpl->featureDes) : pimpl->feature->computeEffort();
    return pimpl->effort;
  }

  const MatrixXd& Task::getJacobian() const
  {
    pimpl->jacobian = pimpl->featureDes ? pimpl->feature->computeJacobian(*pimpl->featureDes) : pimpl->feature->computeJacobian();
    return pimpl->jacobian;
  }

  Feature::Ptr Task::getFeature() const
  {
    return pimpl->feature;
  }

  Feature::Ptr Task::getFeatureDes() const
  {
    return pimpl->featureDes;
  }

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////



/** I don't really know, I suppose it is for a direct output.
 *
 * \param output  The vector instance where to write the output.
 *
 * This output is set to zero.
 */
void Task::doGetOutput(Eigen::VectorXd& output) const
{
    output = Eigen::VectorXd::Zero(getDimension());
}


const Eigen::VectorXd& Task::getComputedForce() const
{
    return pimpl->fcVar.getValue();
}

void Task::disconnectFromController()
{
    if (pimpl->isRegisteredAsObjective)
    {
        if(pimpl->innerTaskAsObjective != NULL){
            pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
        }
    }
    if (pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
    }

    if (pimpl->frictionConstraintIsRegisteredInConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    }
    if (pimpl->contactForceConstraintHasBeenSavedInSolver)
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
    }

}

void Task::connectToController(std::shared_ptr<OneLevelSolver> solver, const FullDynamicEquationFunction& dynamicEquation, bool useReducedProblem)
{
    pimpl->solver            = solver;
    pimpl->dynamicEquation   = &dynamicEquation;
    pimpl->useReducedProblem =  useReducedProblem;

    switch(getTaskType())
    {

        case(ACCELERATIONTASK):
        {
            pimpl->setAsAccelerationTask();

            break;
        }
        case(TORQUETASK):
        {
            pimpl->setAsTorqueTask();
            break;
        }
        case(FORCETASK):
        {
            pimpl->setAsForceTask();
            break;
        }
        case(COMMOMENTUMTASK):
        {
            pimpl->setAsAccelerationTask();
            break;
        }
        case(UNKNOWNTASK):
        {
            std::string errmsg = std::string("[Task::connectToController]: The task type of '") + getName() + std::string("' has not been set during creation.\nCall prior that 'initAsAccelerationTask', 'initAsTorqueTask' or 'initAsForceTask'\n"); //
            throw std::runtime_error(std::string(errmsg));
            break;
        }
        default:
        {
            throw std::runtime_error(std::string("[Task::connectToController]: Unhandle case of TYPETASK for task ")+getName() );
            break;
        }
    }
}


void Task::addContactPointInModel()
{
    //THIS SHOULD BE DONE ONLY ONCE!!!
    if ( ! pimpl->contactPointHasBeenSavedInModel )
    {
        pimpl->innerModel->getModelContacts().addContactPoint(pimpl->fcVar, *getFeature());
        pimpl->contactPointHasBeenSavedInModel = true;
    }

    if ( pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->removeConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = false;
    }
}

void Task::removeContactPointInModel()
{
    //    if ( pimpl->contactPointHasBeenSavedInModel )
//    {
//        pimpl->model.getModelContacts().removeContactPoint(pimpl->fcVar);
//        pimpl->contactPointHasBeenSavedInModel = false;
//    }
    if ( ! pimpl->contactForceConstraintHasBeenSavedInSolver )
    {
        pimpl->solver->addConstraint(pimpl->ContactForceConstraint);
        pimpl->contactForceConstraintHasBeenSavedInSolver = true;
    }
}



/** Update linear function of the task for the full formalism.
 *
 * It computes a desired acceleration \f$ \vec{a}^{des} = - \left( \vec{a}_{ref} + K_p (\vec{p}^{des} - vec{p}) +  K_d (\vec{v}^{des} - vec{v}) \right) \f$ .
 * Then The linear function is set as follows:
 *
 * - if it use the reduced problem:
 *
 * \f{align*}{
 *       \A &= J_{task}  .  \left(  \M^{-1} \J_{\tav}\tp  \right)
 *     & \b &= \vec{a}^{des} - \left(  J_{task} \M^{-1} ( \g - \n)  \right)
 * \f}
 *
 * see \ref sec_transform_formalism for more information.
 *
 * - else:
 *
 * \f{align*}{
 *       \A &= J_{task}
 *     & \b &= \vec{a}^{des}
 * \f}
 */
void Task::updateAccelerationTask()
{
    const MatrixXd& J  = getJacobian();
    const MatrixXd& Kp = getStiffness();
    const MatrixXd& Kd = getDamping();

    const VectorXd  accDes = - ( getErrorDdot() + Kp * getError() + Kd * getErrorDot() );

    // std::cout << "\n----\ngetError() = " << getError() << std::endl;
    // std::cout << "getErrorDot() = " << getErrorDot() << std::endl;
    // std::cout << "getErrorDdot() = " << getErrorDdot() << std::endl;

    if (pimpl->useReducedProblem)
    {
        const Eigen::MatrixXd E2 =        - J * pimpl->dynamicEquation->getInertiaMatrixInverseJchiT();
        const Eigen::VectorXd f2 = accDes + J * pimpl->dynamicEquation->getInertiaMatrixInverseLinNonLinGrav();

        pimpl->innerObjectiveFunction->changeA(E2);
        pimpl->innerObjectiveFunction->changeb(f2);
    }
    else
    {
        pimpl->innerObjectiveFunction->changeA(J);
        pimpl->innerObjectiveFunction->changeb(accDes);
    }
}




void Task::updateTorqueTask()
{
    const MatrixXd& J    =   getJacobian();
    const VectorXd  eff  = - getEffort();

    pimpl->innerObjectiveFunction->changeA(J);
    pimpl->innerObjectiveFunction->changeb(eff);
}


void Task::updateForceTask()
{
    //innerObjectiveFunction->changeA(); //already set in initForceTask

    const VectorXd  eff  = - getEffort();

    pimpl->innerObjectiveFunction->changeb(eff);

}

void Task::updateCoMMomentumTask()
{
    const MatrixXd& J  = pimpl->innerModel->getCoMAngularJacobian();
    const MatrixXd& Kd = getDamping();

    const VectorXd  accDes = - Kd * pimpl->innerModel->getCoMAngularVelocity();


    if (pimpl->useReducedProblem)
    {
        const Eigen::MatrixXd E2 =        - J * pimpl->dynamicEquation->getInertiaMatrixInverseJchiT();
        const Eigen::VectorXd f2 = accDes + J * pimpl->dynamicEquation->getInertiaMatrixInverseLinNonLinGrav();

        pimpl->innerObjectiveFunction->changeA(E2);
        pimpl->innerObjectiveFunction->changeb(f2);
    }
    else
    {
        pimpl->innerObjectiveFunction->changeA(J);
        pimpl->innerObjectiveFunction->changeb(accDes);
    }
}


void Task::checkIfConnectedToController() const
{
    if (!pimpl->solver)
    {
        std::string errmsg = std::string("[Task::doActivateAsObjective]: task '") + getName() + std::string("' not connected to any solver; Call prior that 'Controller::addTask' to connect to the solver inside the controller.\n"); //
        throw std::runtime_error(std::string(errmsg));
    }
}



/** Do task activation when it is a contact task.
 *
 * When this function is called, it adds a contact point in the model of contact contained in the Model instance,
 * and it adds in the solver an inequality constraint that represents the limitation of the contact force that must remain inside the cone of friction.
 */
void Task::doActivateContactMode()
{
    checkIfConnectedToController();

    addContactPointInModel();

    // add friction cone in constraint
    pimpl->solver->addConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = true;
}


/** Do task deactivation when it is a contact task.
 *
 * When this function is called, it removes the contact point in the model of contact,
 * and it removes from the solver the friction cone inequality constraint.
 */
void Task::doDeactivateContactMode()
{
    checkIfConnectedToController();

    removeContactPointInModel();

    // remove friction cone from constraint set
    pimpl->solver->removeConstraint(pimpl->frictionConstraint);
    pimpl->frictionConstraintIsRegisteredInConstraint = false;
}


/** For contact task, do the setting of the coefficient of friction.
 *
 * The cone of friction constraint is modified to represent a cone with this new coefficient of friction.
 */
void Task::doSetFrictionCoeff()
{
    pimpl->frictionConstraint.getFunction().setFrictionCoeff(getFrictionCoeff());
}

/** For contact task, do the setting of the friction margin.
 *
 * The cone of friction constraint is modified to represent a friction cone with this new margin.
 */
void Task::doSetMargin()
{
    pimpl->frictionConstraint.getFunction().setMargin(getMargin());
}









/** Do activation of task as an objective.
 *
 * It means that the task is not fully completed and a little error may occur.
 */
void Task::doActivateAsObjective()
{
    checkIfConnectedToController();
    if(!pimpl->isRegisteredAsObjective)
    {
        pimpl->solver->addObjective(*pimpl->innerTaskAsObjective);
        pimpl->isRegisteredAsObjective = true;
    }

    if (getTaskType() == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as an objective.
 *
 * objective is no more considered.
 */
void Task::doDeactivateAsObjective()
{
    checkIfConnectedToController();
    if(pimpl->isRegisteredAsObjective)
    {
        pimpl->solver->removeObjective(*pimpl->innerTaskAsObjective);
        pimpl->isRegisteredAsObjective = false;
    }
    if (getTaskType() == FORCETASK)
    {
        removeContactPointInModel();
    }
}

/** Do activation of task as a constraint.
 *
 * It means that the task should be full completed and no error may occur.
 * Be aware that stong constraints may lead to system instability (very "sharp" solution that requires lot of energy).
 */
void Task::doActivateAsConstraint()
{
    checkIfConnectedToController();
    if(!pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->addConstraint(pimpl->innerTaskAsConstraint);
        pimpl->isRegisteredAsConstraint = true;
    }
    if (getTaskType() == FORCETASK)
    {
        addContactPointInModel();
    }
}

/** Do deactivation of task as a constraint.
 *
 * objective is no more considered.
 */
void Task::doDeactivateAsConstraint()
{
    checkIfConnectedToController();
    if(pimpl->isRegisteredAsConstraint)
    {
        pimpl->solver->removeConstraint(pimpl->innerTaskAsConstraint);
        pimpl->isRegisteredAsConstraint = false;
    }
    if (getTaskType() == FORCETASK)
    {
        removeContactPointInModel();
    }
}



/** Do set the weight of the task.
 *
 * The weight in the objective function is modified.
 *
 */
void Task::doSetWeight()
{
    if (pimpl->innerTaskAsObjective)
    {
         pimpl->innerTaskAsObjective->getFunction().changeWeight(getWeight());
    }


}



}















// cmake:sourcegroup=Api
