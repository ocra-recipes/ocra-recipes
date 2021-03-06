#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <iostream>
#include <memory>

#include <ocra/util/Macros.h>
#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskBuilders/TaskConstructionManager.h>

#include <ocra/optim/OneLevelSolver.h>
#include <wocra/WocraController.h>
#include <hocra/HocraController.h>

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
    WOCRA_CONTROLLER = 1,
    HOCRA_CONTROLLER,
    GOCRA_CONTROLLER
};

enum SOLVER_TYPE
{
    QUADPROG = 1,
    QPOASES
};

class ControllerServer
{
    DEFINE_CLASS_POINTER_TYPEDEFS(ControllerServer)
protected:
    virtual std::shared_ptr<Model> loadRobotModel() = 0;
    virtual void getRobotState(Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root) = 0;

public:
    ControllerServer(CONTROLLER_TYPE ctrlType/*=WOCRA_CONTROLLER*/,
                     SOLVER_TYPE solver/*=QUADPROG*/,
                     bool usingInterprocessCommunication=true,
                     bool useOdometry=false);

    virtual ~ControllerServer();

    bool initialize();
    const Eigen::VectorXd& computeTorques();
    void computeTorques(Eigen::VectorXd& torques);


    const std::shared_ptr<ocra::Controller> getController(){return controller;}
    const std::shared_ptr<ocra::Model> getRobotModel(){return model;}

    bool addTasksFromXmlFile(const std::string& filePath);
    bool addTasks(std::vector<ocra::TaskBuilderOptions>& tmOpts);

    /**
     If the useOdometry flag is passed to the server, odometry is computed.

     - returns: True if properly initialized, false otherwise.
     */
    bool initializeOdometry();
    void updateModel();

    void setRegularizationTermWeights(double wDdq, double wTau, double wFc); 

protected:

    ocra::Model::Ptr                     model;
    ocra::Controller::Ptr           controller;
    ocra::Solver::Ptr           internalSolver;

    bool firstRun;
    
    ServerCommunications::Ptr       serverComs;

    Eigen::VectorXd            tau;
    RobotState              rState;
    Eigen::VectorXd         qdPrevious;
    Eigen::VectorXd         qPrevious;
    // Eigen::VectorXd              q;
    // Eigen::VectorXd             qd;
    // Eigen::Displacementd    H_root;
    // Eigen::Twistd           T_root;

    CONTROLLER_TYPE    controllerType;
    SOLVER_TYPE            solverType;
    bool                    usingComs;
    bool                  usingOdometry;

    yarp::os::Bottle statesBottle;
    yarp::os::Port statesPort;

    //Odometry options
    std::string modelFile;
    std::string initialFixedFrame;
};

} // namespace ocra_recipes
#endif // CONTROLLER_SERVER_H
