#include <ocra-recipes/ControllerServer.h>

using namespace ocra_recipes;

ControllerServer::ControllerServer(CONTROLLER_TYPE ctrlType,
                                   SOLVER_TYPE solver,
                                   bool usingInterprocessCommunication,
                                   bool useOdometry)
: controllerType(ctrlType)
, solverType(solver)
, usingComs(usingInterprocessCommunication)
, usingOdometry(useOdometry)
{
}

ControllerServer::~ControllerServer()
{
    // if(taskManagerSet)
    //     taskManagerSet->clearSet();
}

bool ControllerServer::initialize()
{
    bool res = true;
    model = loadRobotModel();

    rState = RobotState(model->nbDofs());
    if(model)
    {
        // Set the solver.
        switch (solverType)
        {
            case QUADPROG:
            {
                std::cout << "Using QuadProg++ "<<std::endl;
                internalSolver = std::make_shared<ocra::OneLevelSolverWithQuadProg>();
            }break;

            case QPOASES:
            {
                std::cout << "Using qpOASES"<<std::endl;
                internalSolver = std::make_shared<ocra::OneLevelSolverWithQPOASES>();
            }break;

            default:
            {
                std::cout << "Using qpOASES [Default]"<<std::endl;
                internalSolver = std::make_shared<ocra::OneLevelSolverWithQPOASES>();
            }break;
        }

        // Construct the desired controller.
        switch (controllerType)
        {
            case WOCRA_CONTROLLER:
            {
                std::cout << "Constructing a WOCRA Controller "<<std::endl;
                bool useReducedProblem = false;
                controller = std::make_shared<wocra::WocraController>("WocraController", model, std::static_pointer_cast<ocra::OneLevelSolver>(internalSolver), useReducedProblem);
            }break;

            case HOCRA_CONTROLLER:
            {
                std::cout << "Constructing a HOCRA Controller "<<std::endl;
                bool useReducedProblem = false;
                controller = std::make_shared<hocra::HocraController>("HocraController", model, std::static_pointer_cast<ocra::OneLevelSolver>(internalSolver), useReducedProblem);
            }break;

            default:
            {
                std::cout << "Constructing a WOCRA Controller [Default]"<<std::endl;
                bool useReducedProblem = false;
                controller = std::make_shared<wocra::WocraController>("WocraController", model, std::static_pointer_cast<ocra::OneLevelSolver>(internalSolver), useReducedProblem);
            }break;
        }
    }

    if(usingComs)
    {
        serverComs = std::make_shared<ServerCommunications>(controller, model);
        res &= serverComs->open();
        res &= statesPort.open("/ControllerServer/states:o");
    }

    res &= bool(model);
    res &= bool(controller);
    
    // WARNING! If useOdometry is true we must call updateModel during initialization explicitly after ControllerServer::initialize()
    std::cout << "[DEBUG-ODOMETRY] useOdometry is: " << usingOdometry << std::endl;
    std::cout << "[DEBUG-ODOMETRY] this->useOdometry is: " << this->usingOdometry << std::endl;
    if (!this->usingOdometry) {
        updateModel();
    }
    
    return res;
}

const Eigen::VectorXd& ControllerServer::computeTorques()
{
    updateModel();
    computeTorques(tau);
    return tau;
}

void ControllerServer::computeTorques(Eigen::VectorXd& torques)
{
    updateModel();
    controller->computeOutput(torques);
}

void ControllerServer::updateModel()
{
    getRobotState(rState.q, rState.qd, rState.H_root, rState.T_root);
    if (model->hasFixedRoot()){
        model->setState(rState.q, rState.qd);
    }else{
        model->setState(rState.H_root, rState.q, rState.T_root, rState.qd);
    }
    statesPort.write(rState);
}

bool ControllerServer::addTasksFromXmlFile(const std::string& filePath)
{
    ocra::TaskConstructionManager factory(model, controller, filePath);
    return true;
}

bool ControllerServer::addTasks(std::vector<ocra::TaskBuilderOptions>& taskOptions)
{
    ocra::TaskConstructionManager factory(model, controller, taskOptions);
    return true;
}
