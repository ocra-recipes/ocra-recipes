#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <iostream>
#include <memory>

#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskManagers/TaskManagerSet.h>

#include <ocra/optim/OneLevelSolver.h>
#include <wocra/WocraController.h>


namespace ocra_recipes
{
class ControllerServer
{
protected:
    virtual std::shared_ptr<Model> void setRobotModel() = 0;
    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::VectorXd& H_root, Eigen::VectorXd& T_root) = 0;

public:
    ControllerServer(const OCRA_CONTROLLER_TYPE ctrlType, const bool using_interprocess_coms=true);
    virtual ~ControllerServer();
    const Eigen::VectorXd& computeTorques();


private:
    void parseControllerMessage(input, reply);

    // yarp port open and bind to parse methods.



private:

    std::shared_ptr<ocra::Model>                     model;
    std::shared_ptr<ocra::Controller>           controller;
    std::shared_ptr<ocra::OneLevelSolver>   internalSolver;
    std::shared_ptr<ocra::TaskManagerSet>   taskManagerSet;


    Eigen::VectorXd              q;
    Eigen::VectorXd             qd;
    Eigen::VectorXd            tau;
    Eigen::Displacementd    H_root;
    Eigen::Twist            T_root;
}

} // namespace ocra_recipes
#endif // CONTROLLER_SERVER_H
