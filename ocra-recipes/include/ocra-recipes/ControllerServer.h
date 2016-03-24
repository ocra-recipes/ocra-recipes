#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <iostream>
#include <memory>

#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskManagers/TaskManagerSet.h>
#include <ocra/control/TaskManagers/TaskManagerFactory.h>
#include <ocra/control/TaskManagers/TaskManagerOptions.h>

#include <ocra/optim/OneLevelSolver.h>
#include <wocra/WocraController.h>

#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>

#include <ocra-recipes/ServerCommunications.h>
#include <ocra-recipes/RobotState.h>


namespace ocra_recipes
{

enum CONTROLLER_TYPE
{
    WOCRA_CONTROLLER,
    HOCRA_CONTROLLER,
    GOCRA_CONTROLLER
};

enum SOLVER_TYPE
{
    QUADPROG,
    QPOASES
};

class ControllerServer
{
protected:
    virtual std::shared_ptr<Model> loadRobotModel() = 0;
    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root) = 0;

public:
    ControllerServer(const CONTROLLER_TYPE ctrlType=WOCRA_CONTROLLER, const SOLVER_TYPE solver=QUADPROG, const bool usingInterprocessCommunication=true);
    virtual ~ControllerServer();

    bool initialize();
    const Eigen::VectorXd& computeTorques();
    void computeTorques(Eigen::VectorXd& torques);


    const std::shared_ptr<ocra::Controller> getController(){return controller;}
    const std::shared_ptr<ocra::Model> getRobotModel(){return model;}

    bool addTaskManagersFromXmlFile(const std::string& filePath);
    bool addTaskManagers(ocra::TaskManagerOptions& tmOpts);

private:
    void updateModel();

    std::shared_ptr<ocra::Model>                     model;
    std::shared_ptr<ocra::Controller>           controller;
    std::shared_ptr<ocra::OneLevelSolver>   internalSolver;
    std::shared_ptr<ocra::TaskManagerSet>   taskManagerSet;

    std::shared_ptr<ServerCommunications>       serverComs;

    Eigen::VectorXd            tau;
    RobotState              rState;
    // Eigen::VectorXd              q;
    // Eigen::VectorXd             qd;
    // Eigen::Displacementd    H_root;
    // Eigen::Twistd           T_root;

    CONTROLLER_TYPE    controllerType;
    SOLVER_TYPE        solverType;
    bool                    usingComs;

    yarp::os::Bottle statesBottle;
    yarp::os::Port statesPort;
};

} // namespace ocra_recipes
#endif // CONTROLLER_SERVER_H
