#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <iostream>
#include <memory>

#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskManagers/TaskManagerSet.h>
#include <ocra/control/TaskManagers/TaskParser.h>

#include <ocra/optim/OneLevelSolver.h>
#include <wocra/WocraController.h>

#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>

#include <ocra-recipes/ServerCommunications.h>


namespace ocra_recipes
{

enum CONTROLLER_TYPE
{
    WOCRA_CONTROLLER,
    HOCRA_CONTROLLER,
    GOCRA_CONTROLLER
};

class ControllerServer
{
protected:
    virtual std::shared_ptr<Model> setRobotModel() = 0;
    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root) = 0;

public:
    ControllerServer(const CONTROLLER_TYPE ctrlType, const bool usingInterprocessCommunication=true);
    virtual ~ControllerServer();

    bool initialize();
    const Eigen::VectorXd& computeTorques();


private:

    std::shared_ptr<ocra::Model>                     model;
    std::shared_ptr<ocra::Controller>           controller;
    std::shared_ptr<ocra::OneLevelSolver>   internalSolver;
    std::shared_ptr<ocra::TaskManagerSet>   taskManagerSet;

    ServerCommunications                        serverComs;


    Eigen::VectorXd              q;
    Eigen::VectorXd             qd;
    Eigen::VectorXd            tau;
    Eigen::Displacementd    H_root;
    Eigen::Twistd           T_root;

    CONTROLLER_TYPE    controllerType;
    bool                    usingComs;
};

} // namespace ocra_recipes
#endif // CONTROLLER_SERVER_H
