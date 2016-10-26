#include "ocra/control/TaskYarpInterface.h"

using namespace ocra;


/** base constructor
 *
 * \param ctrl                  ocra::Controller to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
TaskYarpInterface::TaskYarpInterface(Task::Ptr taskPtr)
: task(taskPtr)
, controlPortsOpen(false)
, taskMode(TASK_NOT_DEFINED)
, logMessages(false)
{
    if(task) {
        portName = "/Task/"+task->getName()+"/rpc:i";
        rpcCallback =  std::make_shared<RpcMessageCallback>(*this);
        rpcPort.open(portName.c_str());
        rpcPort.setReader(*rpcCallback);
    }
}


TaskYarpInterface::~TaskYarpInterface()
{
    std::cout << "\t--> Closing ports" << std::endl;
    rpcPort.close();
    if(stateThread){
        closeControlPorts();
    }
}

int TaskYarpInterface::getTaskHierarchyLevel()
{
    return task->getHierarchyLevel();
}

void TaskYarpInterface::setTaskHierarchyLevel(int level)
{
    task->setHierarchyLevel(level);
    return;
}

bool TaskYarpInterface::activate()
{
    if(task){
        if(isActivated()){taskMode = getTaskMode();}
        return activate(taskMode);
    }
    else{
        yLog.error() << "No valid task(s) to activate.";
        return false;
    }
}

bool TaskYarpInterface::activate(const TASK_MODE tmode)
{
    bool allActivated;
    if(task)
    {
        switch (tmode)
        {
            case TASK_AS_OBJECTIVE:
            {
                task->activateAsObjective();
                allActivated = true;
            }break;

            case TASK_AS_CONSTRAINT:
            {
                task->activateAsConstraint();
                allActivated = true;
            }break;

            case TASK_NOT_DEFINED:
            {
                yLog.error() << "Task, " << task->getName() << ", has not been defined as an objective or constraint therefore I can't activate it.";
                allActivated = false;
            }break;

            default:
            {
                allActivated = false;
            }break;
        }
    }
    return allActivated;
}

bool TaskYarpInterface::deactivate()
{
    if(task){
        if(isActivated()){taskMode = getTaskMode();}
        task->deactivate();
        return true;
    }
    else{
        taskMode = TASK_NOT_DEFINED;
        yLog.error() << "No valid task(s) to activate.";
        return false;
    }
}

TASK_MODE TaskYarpInterface::getTaskMode()
{
    if (task->isActiveAsObjective()) {
        return TASK_AS_OBJECTIVE;
    } else if (task->isActiveAsConstraint()) {
        return TASK_AS_CONSTRAINT;
    } else {
        return TASK_NOT_DEFINED;
    }
}

bool TaskYarpInterface::isActivated()
{
    if(task){
        return task->isActiveAsObjective() || task->isActiveAsConstraint();
    } else {
        return false;
    }
}


Eigen::VectorXd TaskYarpInterface::getTaskError()
{
    if(task) {
        return task->getError();
    } else {
        return Eigen::VectorXd::Zero(0);
    }
}

double TaskYarpInterface::getTaskErrorNorm()
{
    return getTaskError().norm();
}

void TaskYarpInterface::setStiffness(double K)
{
    if(task) {
        task->setStiffness(K);
    } else {
        std::cout << "\n\n\n FUUUUUUCK" << std::endl;
    }
}

void TaskYarpInterface::setStiffness(const VectorXd& K)
{
    if(task) {
        task->setStiffness(K);
    }
}

void TaskYarpInterface::setStiffness(const MatrixXd& K)
{
    if(task) {
        task->setStiffness(K);
    }
}

double TaskYarpInterface::getStiffness()
{
    if(task) {
        // Get the first entry in the stiffness matrix.
        return getStiffnessMatrix()(0,0);
    } else {
        yLog.error() << "Unable to get the stiffness of the task because it does not exist. Returning 0.0";
        return 0.0;
    }
}

Eigen::MatrixXd TaskYarpInterface::getStiffnessMatrix()
{
    if(task) {
        return task->getStiffness();
    } else {
        yLog.error() << "Unable to get the stiffness of the task because it does not exist. Returning empty matrix";
        return Eigen::MatrixXd::Zero(0,0);
    }
}

void TaskYarpInterface::setDamping(double B)
{
    if(task) {
        task->setDamping(B);
    }
}

void TaskYarpInterface::setDamping(const VectorXd& B)
{
    if(task) {
        task->setDamping(B);
    }
}

void TaskYarpInterface::setDamping(const MatrixXd& B)
{
    if(task) {
        task->setDamping(B);
    }
}

double TaskYarpInterface::getDamping()
{
    if(task) {
        // Get the first entry in the stiffness matrix.
        return getDampingMatrix()(0,0);
    } else {
        yLog.error() << "Unable to get the damping coeff of the task because it does not exist. Returning 0.0";
        return 0.0;
    }
}

Eigen::MatrixXd TaskYarpInterface::getDampingMatrix()
{
    if(task) {
        return task->getDamping();
    } else {
        yLog.error() << "Unable to get the damping coeffs of the task because it does not exist. Returning empty matrix";
        return Eigen::MatrixXd::Zero(0,0);
    }
}

void TaskYarpInterface::setWeight(double weight)
{
    if(task) {
        return task->setWeight(weight);
    }
}

void TaskYarpInterface::setWeight(const Eigen::VectorXd& weights)
{
    if(task) {
        return task->setWeight(weights);
    }
}

Eigen::VectorXd TaskYarpInterface::getWeight()
{
    if(task) {
        return task->getWeight();
    } else {
        yLog.error() << "Unable to get the weights of the task because it does not exist. Returning empty vector";
        return Eigen::VectorXd::Zero(0);
    }
}


TaskState TaskYarpInterface::getTaskState()
{
    return this->task->getTaskState();
}

TaskState TaskYarpInterface::getDesiredTaskState()
{
    return this->task->getDesiredTaskState();
}

void TaskYarpInterface::setDesiredTaskState(const TaskState& newDesiredTaskState)
{
    return this->task->setDesiredTaskState(newDesiredTaskState);
}

void TaskYarpInterface::setDesiredTaskStateDirect(const TaskState& newDesiredTaskState)
{
    return this->task->setDesiredTaskStateDirect(newDesiredTaskState);
}

void TaskYarpInterface::publishTaskState()
{
    stateOutBottle.clear();
    this->getTaskState().putIntoBottle(stateOutBottle);
    outputControlPort.write(stateOutBottle);
}

bool TaskYarpInterface::openControlPorts()
{
    bool res = true;
    if (!controlPortsOpen)
    {
        inputControlPortName = "/Task/"+task->getName()+"/state:i";
        outputControlPortName = "/Task/"+task->getName()+"/state:o";

        res = res && inputControlPort.open(inputControlPortName.c_str());
        res = res && outputControlPort.open(outputControlPortName.c_str());

        controlPortsOpen = res;
    }

    if (!controlCallback) {
        controlCallback = std::make_shared<ControlInputCallback>(*this);
        inputControlPort.setReader(*controlCallback);
    }
    if(!stateThread) {
        stateThread = std::make_shared<StateUpdateThread>(10, *this);
    }
    if (!stateThread->isRunning()) {
        stateThread->start();
    }

    return res;
}

bool TaskYarpInterface::closeControlPorts()
{
    if(stateThread) {
        if (stateThread->isRunning()) {
            stateThread->stop();
        }
    }
    if (controlPortsOpen) {
        inputControlPort.close();
        outputControlPort.close();
    }

    bool controlPortsClosed = !inputControlPort.isOpen() && !outputControlPort.isOpen();

    controlPortsOpen = !controlPortsClosed;

    return controlPortsClosed;
}

bool TaskYarpInterface::parseControlInput(yarp::os::Bottle& input)
{
    TaskState state;
    int dummyInt;
    state.extractFromBottle(input, dummyInt);
    this->setDesiredTaskStateDirect(state);
}



//////////////////////////////////////////////////////////////////////////////
//                  Segment Frame Tasks
//////////////////////////////////////////////////////////////////////////////

Eigen::Displacementd TaskYarpInterface::getTaskFrameDisplacement()
{
    return getTaskState().getPosition();
}

Eigen::Twistd TaskYarpInterface::getTaskFrameVelocity()
{
    return getTaskState().getVelocity();
}

Eigen::Twistd TaskYarpInterface::getTaskFrameAcceleration()
{
    return getTaskState().getAcceleration();
}

Eigen::Vector3d TaskYarpInterface::getTaskFramePosition()
{
    return getTaskFrameDisplacement().getTranslation();
}

Eigen::Rotation3d TaskYarpInterface::getTaskFrameOrientation()
{
    return getTaskFrameDisplacement().getRotation();
}

Eigen::Vector3d TaskYarpInterface::getTaskFrameLinearVelocity()
{
    return getTaskFrameVelocity().getLinearVelocity();
}

Eigen::Vector3d TaskYarpInterface::getTaskFrameAngularVelocity()
{
    return getTaskFrameVelocity().getAngularVelocity();
}

Eigen::Vector3d TaskYarpInterface::getTaskFrameLinearAcceleration()
{
    return getTaskFrameAcceleration().getLinearVelocity();
}

Eigen::Vector3d TaskYarpInterface::getTaskFrameAngularAcceleration()
{
    return getTaskFrameAcceleration().getAngularVelocity();
}

//////////////////////////////////////////////////////////////////////////////



std::string TaskYarpInterface::getPortName()
{
    return portName;
}


/**************************************************************************************************
                                    Nested RpcMessageCallback Class
**************************************************************************************************/
TaskYarpInterface::RpcMessageCallback::RpcMessageCallback(TaskYarpInterface& tmBaseRef)
: tmBase(tmBaseRef)
{
    //do nothing
}

bool TaskYarpInterface::RpcMessageCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;
    yarp::os::Bottle reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        tmBase.parseIncomingMessage(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/



/**************************************************************************************************
                                    Nested ControlInputCallback Class
**************************************************************************************************/
TaskYarpInterface::ControlInputCallback::ControlInputCallback(TaskYarpInterface& tmBaseRef)
: tmBase(tmBaseRef)
{
    //do nothing
}

bool TaskYarpInterface::ControlInputCallback::read(yarp::os::ConnectionReader& connection)
{
    input.clear();
    if (input.read(connection)) {
        return tmBase.parseControlInput(input);
    } else {
        return false;
    }
}
/**************************************************************************************************
**************************************************************************************************/


/**************************************************************************************************
                                    Nested StateUpdateThread Class
**************************************************************************************************/
TaskYarpInterface::StateUpdateThread::StateUpdateThread(int period, TaskYarpInterface& tmBaseRef):
RateThread(period),
tmBase(tmBaseRef)
{
}
bool TaskYarpInterface::StateUpdateThread::threadInit()
{
    std::cout << "StateUpdateThread: Opening.\n";
    return true;
}
void TaskYarpInterface::StateUpdateThread::run()
{
    tmBase.publishTaskState();
}
void TaskYarpInterface::StateUpdateThread::threadRelease()
{
    std::cout << "StateUpdateThread: Closing.\n";
}
/**************************************************************************************************
**************************************************************************************************/

void TaskYarpInterface::parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (auto i=0; i<btlSize; ++i) {
        int msg;
        int testInt = input.get(0).asInt();
        std::string testStr = input.get(0).asString();
        if ((testStr=="") && (testInt!=0)) {
            msg = TASK_MESSAGE(testInt);
        } else {
            msg = TaskMessageHandler::stringToTaskManagerMessageTag(testStr);
        }
        switch (msg) {

            case GET_TASK_STATE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TASK_STATE";
                }
                TaskState state = this->task->getTaskState();
                state.putIntoBottle(reply);
            }break;

            case GET_STIFFNESS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_STIFFNESS";
                }
                util::pourEigenMatrixIntoBottle(this->getStiffnessMatrix(), reply);
            }break;

            case GET_DAMPING:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_DAMPING";
                }
                util::pourEigenMatrixIntoBottle(this->getDampingMatrix(), reply);
            }break;

            case GET_WEIGHTS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_WEIGHTS";
                }
                util::pourEigenVectorIntoBottle(this->getWeight(), reply);
            }break;

            case GET_DESIRED_TASK_STATE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_DESIRED_TASK_STATE";
                }
                TaskState state = this->task->getDesiredTaskState();
                state.putIntoBottle(reply);
            }break;

            case GET_TASK_POSITION:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TASK_POSITION";
                }
                if ((this->task->getMetaTaskType()==Task::META_TASK_TYPE::POSITION) || (this->task->getMetaTaskType()==Task::META_TASK_TYPE::COM)) {
                    Eigen::Vector3d pos = this->task->getTaskState().getPosition().getTranslation();
                    util::pourEigenVectorIntoBottle(pos, reply);
                }
            }break;

            case GET_DESIRED_TASK_POSITION:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_DESIRED_TASK_POSITION";
                }
                if ((this->task->getMetaTaskType()==Task::META_TASK_TYPE::POSITION) || (this->task->getMetaTaskType()==Task::META_TASK_TYPE::COM)) {
                    Eigen::Vector3d pos = this->task->getDesiredTaskState().getPosition().getTranslation();
                    util::pourEigenVectorIntoBottle(pos, reply);
                }
            }break;

            case GET_ACTIVITY_STATUS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_ACTIVITY_STATUS";
                }
                if (this->isActivated()) {
                    reply.addInt(TASK_IS_ACTIVATED);
                } else {
                    reply.addInt(TASK_IS_DEACTIVATED);
                }

            }break;

            case GET_DIMENSION:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_DIMENSION";
                }
                reply.addInt(this->getWeight().size());
            }break;

            case GET_TYPE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TYPE";
                }
                reply.addInt(this->task->getMetaTaskType());
            }break;

            case GET_TYPE_AS_STRING:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TYPE";
                }
                reply.addString(this->task->getMetaTaskTypeAsString());
            }break;

            case GET_NAME:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_NAME";
                }
                reply.addString(this->task->getName());
            }break;

            case GET_CONTROL_PORT_NAMES:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_CONTROL_PORT_NAMES";
                }
                reply.addString(this->inputControlPortName);
                reply.addString(this->outputControlPortName);
            }break;

            case GET_TASK_PORT_NAME:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TASK_PORT_NAME";
                }
                reply.addString(this->getPortName());
            }break;

            case GET_TASK_ERROR:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: GET_TASK_ERROR";
                }
                util::pourEigenVectorIntoBottle(this->getTaskError(), reply);
            }break;

            case SET_STIFFNESS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_STIFFNESS";
                }
                ++i;
                setStiffness(input.get(i).asDouble());
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_STIFFNESS_VECTOR:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_STIFFNESS_VECTOR";
                }
                int indexesToSkip;
                this->setStiffness(util::pourBottleIntoEigenVector(util::trimBottle(input, i+1), indexesToSkip) );
                i += indexesToSkip;
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_STIFFNESS_MATRIX:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_STIFFNESS_MATRIX";
                }
                int indexesToSkip;
                this->setStiffness(util::pourBottleIntoEigenMatrix(util::trimBottle(input, i+1), indexesToSkip) );
                i += indexesToSkip;
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_DAMPING:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_DAMPING";
                }
                ++i;
                this->setDamping(input.get(i).asDouble());
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_DAMPING_VECTOR:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_DAMPING_VECTOR";
                }
                int indexesToSkip;
                this->setDamping(util::pourBottleIntoEigenVector(util::trimBottle(input, i+1), indexesToSkip) );
                i += indexesToSkip;
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_DAMPING_MATRIX:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_DAMPING_MATRIX";
                }
                int indexesToSkip;
                this->setDamping(util::pourBottleIntoEigenMatrix(util::trimBottle(input, i+1), indexesToSkip) );
                i += indexesToSkip;
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_WEIGHT:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_WEIGHT";
                }
                ++i;
                this->setWeight(input.get(i).asDouble());
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_WEIGHT_VECTOR:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_WEIGHT_VECTOR";
                }
                int indexesToSkip;
                this->setWeight(util::pourBottleIntoEigenVector(util::trimBottle(input, i+1), indexesToSkip) );
                i += indexesToSkip;
                reply.addInt(OCRA_SUCCESS);
            }break;

            case SET_DESIRED_TASK_STATE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_DESIRED_TASK_STATE";
                }

                int indexesToSkip;
                TaskState state;
                bool extractSuccess = state.extractFromBottle(util::trimBottle(input, i+1), indexesToSkip);
                i += indexesToSkip;
                if (extractSuccess) {
                    // TODO: Set to setDesiredTaskState (needs internal traj gen.) See Task::setDesiredTaskState()
                    this->setDesiredTaskStateDirect(state);
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }

            }break;

            case SET_DESIRED_TASK_POSITION:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: SET_DESIRED_TASK_POSITION";
                }
                if (((this->task->getMetaTaskType()==Task::META_TASK_TYPE::POSITION) || (this->task->getMetaTaskType()==Task::META_TASK_TYPE::COM)) && (input.size()>=4)){
                    Eigen::Vector3d pos(input.get(i+1).asDouble(), input.get(i+2).asDouble(), input.get(i+3).asDouble());
                    TaskState state;
                    state.setPosition(Eigen::Displacementd(pos, Eigen::Rotation3d::Identity()));
                    // TODO: Set to setDesiredTaskState (needs internal traj gen.) See Task::setDesiredTaskState()
                    this->setDesiredTaskStateDirect(state);
                    i += 3;
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }
            }break;

            case ACTIVATE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: ACTIVATE";
                }
                if(this->activate()) {
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }
            }break;

            case DEACTIVATE:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: DEACTIVATE";
                }
                if(this->deactivate()) {
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }
            }break;

            case OPEN_CONTROL_PORTS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: OPEN_CONTROL_PORTS";
                }
                if(openControlPorts()) {
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }
            }break;

            case CLOSE_CONTROL_PORTS:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: CLOSE_CONTROL_PORTS";
                }
                if(closeControlPorts()) {
                    reply.addInt(OCRA_SUCCESS);
                } else {
                    reply.addInt(OCRA_FAILURE);
                }
            }break;

            case HELP:
            {
                if (logMessages) {
                    yLog.info() << " ["<< this->task->getName() <<"]: " << "Processing request: HELP";
                }

            }break;

            default:
            {
                yLog.warning() << "Unrecognized request tag.";

            }break;


        }
    }
}


std::string TaskYarpInterface::printValidMessageTags()
{
    std::string helpString  = "\n=== Valid message tags are: ===\n";
    helpString += "setStiffness: Allows you to set the Kp gain. Expects 1 double value.\n";
    helpString += "setDamping: Allows you to set the Kd gain. Expects 1 double value.\n";
    helpString += "setWeight: Allows you to set the task weight. Expects 1 double value.\n";
    helpString += "setDesired: Allows you to set the desired task reference. Expects nDoF double values where nDoF is the task dimension.\n";
    helpString += "getCurrentState: Allows you to get the current state of the task. No arguments expected.\n";
    helpString += "getStiffness: Allows you to get the Kp gain. No arguments expected.\n";
    helpString += "getDamping: Allows you to get the Kd gain. No arguments expected.\n";
    helpString += "getWeight: Allows you to get the task weight. No arguments expected.\n";
    helpString += "getDesired: Allows you to get the desired task reference. No arguments expected.\n";
    helpString += "getActivityStatus: Queries the activity status of the task. No arguments expected.\n";
    helpString += "activate: Allows you to activate the task. No arguments expected.\n";
    helpString += "deactivate: Allows you to deactivate the task. No arguments expected.\n";
    helpString += "getDimension: Prints the state dimension. No arguments expected.\n";
    helpString += "getType: Retrieve the Type of the task. No arguments expected.\n";
    helpString += "getName: Retrieve the Name of the task. No arguments expected.\n";
    helpString += "openControlPorts: Open up high speed yarp control ports.\n";
    helpString += "closeControlPorts: Close high speed yarp control ports.\n";
    helpString += "getControlPortNames: Get the names of the high speed yarp control ports.\n";
    helpString += "help: Prints all the valid commands. No arguments expected.\n";

    helpString += "\nTypical usage: [message tag] [message value(s)]\ne.g.  >> stiffness 20 damping 10 desired_state 1.0 2.0 2.0\n";

    return helpString;
}
