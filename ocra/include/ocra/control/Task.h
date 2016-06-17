#ifndef _OCRA_TASK_API_H_
#define _OCRA_TASK_API_H_

#include "ocra/optim/NamedInstance.h"
#include <Eigen/Core>
#include <ocra/util/Macros.h>
#include "ocra/control/Feature.h"
#include "ocra/control/Model.h"
//



#include "ocra/optim/FunctionHelpers.h"
#include "ocra/optim/SquaredLinearFunction.h"
#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include "ocra/control/Task.h"
#include "ocra/control/TaskState.h"
#include "ocra/control/ControlFrame.h"
#include "ocra/optim/OneLevelSolver.h"
#include "ocra/control/ControlConstraint.h"
#include "ocra/optim/LinearizedCoulombFunction.h"
#include "ocra/optim/VariableChiFunction.h"
#include "ocra/optim/FcQuadraticFunction.h"





namespace ocra
{

class Task : public NamedInstance
{
DEFINE_CLASS_POINTER_TYPEDEFS(Task)

public:
    Task(const std::string& name, std::shared_ptr<Model> model, Feature::Ptr feature, Feature::Ptr featureDes);
    Task(const std::string& name, std::shared_ptr<Model> model, Feature::Ptr feature);
    virtual ~Task();


    enum TYPETASK { UNKNOWNTASK, ACCELERATIONTASK, TORQUETASK, FORCETASK, COMMOMENTUMTASK };

    TaskState getTaskState();
    void setDesiredTaskState(const TaskState& newDesiredTaskState);
    void setDesiredTaskStateDirect(const TaskState& newDesiredTaskState);

    int getHierarchyLevel();
    void setHierarchyLevel(int level);
    void update();
    void setTaskType(Task::TYPETASK newTaskType);
    Task::TYPETASK getTaskType();
    void activateAsObjective();
    void activateAsConstraint();
    void deactivate();
    bool isActiveAsObjective() const;
    bool isActiveAsConstraint() const;

    void setDesiredMassToActualOne();
    void setDesiredMass(double Md);
    void setDesiredMass(const Eigen::VectorXd& Md);
    void setDesiredMass(const Eigen::MatrixXd& Md);

    void setDamping(double B);
    void setDamping(const Eigen::VectorXd& B);
    void setDamping(const Eigen::MatrixXd& B);

    void setStiffness(double K);
    void setStiffness(const Eigen::VectorXd& K);
    void setStiffness(const Eigen::MatrixXd& K);

    void setAutoGains(double freq);
    void setAutoGains(double freq, double massSaturation);

    bool isDesiredMassTheActualOne() const;
    const Eigen::MatrixXd& getDesiredMass() const;
    const Eigen::MatrixXd& getDesiredMassInverse() const;
    const Eigen::MatrixXd& getDamping() const;
    const Eigen::MatrixXd& getStiffness() const;

    void activateContactMode();
    void deactivateContactMode();
    bool isContactModeActive() const;

    bool isBodyContactConstraint() const;
    bool isPointContactTask() const;

    double getFrictionCoeff() const;
    double getMargin() const;
    const Eigen::Vector3d& getFrictionConstraintOffset() const;
    void setFrictionCoeff(double coeff);
    void setMargin(double margin);
    void setFrictionConstraintOffset(const Eigen::Vector3d& offset);

    void setWeight(double weight);
    void setWeight(const Eigen::VectorXd& weight);
    const Eigen::VectorXd& getWeight() const;

    int getDimension() const;
    const Eigen::VectorXd& getOutput() const;
    const Eigen::VectorXd& getError() const;
    const Eigen::VectorXd& getErrorDot() const;
    const Eigen::VectorXd& getErrorDdot() const;
    const Eigen::VectorXd& getEffort() const;
    const Eigen::MatrixXd& getJacobian() const;

protected:
    Feature::Ptr getFeature() const;
    Feature::Ptr getFeatureDes() const;

protected:
    void doActivateAsObjective();
    void doActivateAsConstraint();
    void doActivateContactMode();
    void doDeactivateAsObjective();
    void doDeactivateAsConstraint();
    void doDeactivateContactMode();
    void doSetFrictionCoeff();
    void doSetMargin();
    void doSetWeight();
    void doGetOutput(Eigen::VectorXd& output) const;


private:
    struct Pimpl;
    std::shared_ptr<Pimpl> pimpl;


public:
    const Eigen::VectorXd& getComputedForce() const;
    void disconnectFromController();
    void connectToController(std::shared_ptr<OneLevelSolver> _solver, const FullDynamicEquationFunction& dynamicEquation, bool useReducedProblem);

protected:
    void addContactPointInModel();
    void removeContactPointInModel();

    void updateAccelerationTask();
    void updateTorqueTask();
    void updateForceTask();
    void updateCoMMomentumTask();

    void checkIfConnectedToController() const;




};
}
#endif

// cmake:sourcegroup=Api
