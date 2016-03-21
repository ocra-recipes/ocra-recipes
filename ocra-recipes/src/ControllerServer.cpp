#include <ocra-recipes/ControllerServer.h>

using namespace ocra_recipes;

ControllerServer::ControllerServer(const CONTROLLER_TYPE ctrlType, const bool usingInterprocessCommunication)
: controllerType(ctrlType)
, usingComs(usingInterprocessCommunication)
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
    if(model)
    {
        switch (controllerType)
        {
            case WOCRA_CONTROLLER:
            {
                internalSolver = std::make_shared<ocra::OneLevelSolverWithQuadProg>();
                bool useReducedProblem = false;
                controller = std::make_shared<wocra::WocraController>("WocraController", *model, *internalSolver, useReducedProblem);
            }break;

            default:
            {
                internalSolver = std::make_shared<ocra::OneLevelSolverWithQuadProg>();
                bool useReducedProblem = false;
                controller = std::make_shared<wocra::WocraController>("WocraController", *model, *internalSolver, useReducedProblem);
            }break;
        }
        if(controller){
            taskManagerSet = std::make_shared<ocra::TaskManagerSet>(controller, model);
        }
    }

    if(usingComs)
    {
        serverComs = std::make_shared<ServerCommunications>(controller, model, taskManagerSet);
        res &= serverComs->open();
    }

    res &= bool(model);
    res &= bool(controller);
    res &= bool(taskManagerSet);

    updateModel();
    return res;
}

const Eigen::VectorXd& ControllerServer::computeTorques()
{
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
    getRobotState(q, qd, H_root, T_root);
    if (model->hasFixedRoot()){
        model->setState(q, qd);
    }else{
        model->setState(H_root, q, T_root, qd);
    }
}

bool ControllerServer::addTaskManagersFromXmlFile(const std::string& filePath)
{
    ocra::TaskManagerFactory factory;
    if(factory.parseTasksXML(filePath))
        return factory.addTaskManagersToSet(controller, model, taskManagerSet);

    else
        return false;
}

bool ControllerServer::addTaskManagers(ocra::TaskManagerOptions& tmOpts)
{
    ocra::TaskManagerFactory factory;
    if(factory.addTaskManagerOptions(tmOpts))
        return factory.addTaskManagersToSet(controller, model, taskManagerSet);

    else
        return false;
}
