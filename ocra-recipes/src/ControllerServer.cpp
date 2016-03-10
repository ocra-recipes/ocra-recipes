#include <ocra-recipes/ControllerServer.h>

using namespace ocra_recipes;

ControllerServer::ControllerServer(const OCRA_CONTROLLER_TYPE ctrlType, const bool using_interprocess_coms)
{
    model = setModel();
    if(model)
    {
        switch (ctrlType)
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

    if(using_interprocess_coms)
    {
        // port.open

    }

};

virtual ControllerServer::~ControllerServer()

const Eigen::VectorXd& ControllerServer::computeTorques()
{
    getStates(q, qd, H_root, T_root);
    if (model->hasFixedRoot()){
        model->setState(q, qd);
    }else{
        model->setState(q, qd, H_root, T_root);
    }
    controller->computeOutput(tau);
    return tau;
}

void ControllerServer::parseControllerMessage(input, reply)
{

}
