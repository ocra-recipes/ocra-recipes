#include <ocra-recipes/ControllerServer.h>

using namespace ocra_recipes;

ControllerServer::ControllerServer(const OCRA_CONTROLLER_TYPE ctrlType, const bool usingInterprocessCommunication)
: controllerType(ctrlType)
, usingComs(usingInterprocessCommunication)
{
}

ControllerServer::~ControllerServer()
{
}

bool ControllerServer::initialize()
{
    model = setRobotModel();
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
        // port.open

    }
}

const Eigen::VectorXd& ControllerServer::computeTorques()
{
    getRobotState(q, qd, H_root, T_root);
    if (model->hasFixedRoot()){
        model->setState(q, qd);
    }else{
        model->setState(H_root, q, T_root, qd);
    }
    controller->computeOutput(tau);
    return tau;
}

void ControllerServer::parseControllerMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{

}
