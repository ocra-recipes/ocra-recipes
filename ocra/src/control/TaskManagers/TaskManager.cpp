#include "ocra/control/TaskManagers/TaskManager.h"


// #include "ocra/control/Trajectory/MinimumJerkTrajectory.h"
// #include "ocra/control/Trajectory/LinearInterpolationTrajectory.h"

using namespace ocra;


/** base constructor
 *
 * \param ctrl                  ocra::Controller to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
TaskManager::TaskManager(ocra::Controller& _ctrl, const ocra::Model& _model, const std::string& _taskName, bool _usesYarpPorts)
: ctrl(_ctrl)
, model(_model)
, name(_taskName)
, controlPortsOpen(false)
, stateDimension(0)
, taskMode(TASK_NOT_DEFINED)
, usesYarp(_usesYarpPorts)
{
    stableName = name;

    if(usesYarp)
    {
        portName = "/TM/"+name+"/rpc:i";
        rpcCallback =  std::make_shared<RpcMessageCallback>(*this);
        rpcPort.open(portName.c_str());
        rpcPort.setReader(*rpcCallback);
    }
}


TaskManager::~TaskManager()
{
    if(usesYarp)
    {
        std::cout << "\t--> Closing ports" << std::endl;
        rpcPort.close();
    }
    // if(task){
    //     std::dynamic_pointer_cast<OneLevelTask>(task)->disconnectFromController();
    // }
    if(stateThread){
        closeControlPorts();
    }


    std::cout << "\t--> Destroying " << stableName << std::endl;
}

bool TaskManager::activate()
{
    if(task){
        if(isActivated()){taskMode = getTaskMode(task);}
        return activate(task, taskMode);
    }
    else if (!taskVector.empty()){
        bool retVal = true;
        if(isActivated()){taskMode = getTaskMode(taskVector[0]);}
        for (auto tsk : taskVector)
        {
            retVal &= activate(tsk, taskMode);
        }
        return retVal;
    }
    else{
        yLog.error() << "No valid task(s) to activate.";
        return false;
    }
}

bool TaskManager::activate(std::shared_ptr<Task> tsk, const TASK_MODE tmode)
{
    bool allActivated;
    if(tsk)
    {
        switch (tmode)
        {
            case TASK_AS_OBJECTIVE:
            {
                tsk->activateAsObjective();
                allActivated = true;
            }break;

            case TASK_AS_CONSTRAINT:
            {
                tsk->activateAsConstraint();
                allActivated = true;
            }break;

            case TASK_NOT_DEFINED:
            {
                yLog.error() << "Task, " << stableName << ", has not been defined as an objective or constraint therefore I can't activate it.";
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

bool TaskManager::deactivate()
{
    if(task){
        if(isActivated()){taskMode = getTaskMode(task);}

        taskMode = getTaskMode(task);
        task->deactivate();
        return true;
    }
    else if (!taskVector.empty()){
        if(taskVector[0]){
            if(isActivated()){taskMode = getTaskMode(taskVector[0]);}
        }
        for (auto tsk : taskVector)
        {
            if(tsk){tsk->deactivate();}
        }
        return true;
    }
    else{
        taskMode = TASK_NOT_DEFINED;
        yLog.error() << "No valid task(s) to activate.";
        return false;
    }
}

TASK_MODE TaskManager::getTaskMode(std::shared_ptr<Task> tsk)
{
    if (tsk->isActiveAsObjective()) {
        return TASK_AS_OBJECTIVE;
    } else if (tsk->isActiveAsConstraint()) {
        return TASK_AS_CONSTRAINT;
    } else {
        return TASK_NOT_DEFINED;
    }
}

bool TaskManager::isActivated()
{
    if(task){
        return task->isActiveAsObjective() || task->isActiveAsConstraint();
    }
    else if (!taskVector.empty()) {
        if(taskVector[0]) {
            return taskVector[0]->isActiveAsObjective() || taskVector[0]->isActiveAsConstraint();
        }
    } else {
        return false;
    }
}


Eigen::VectorXd TaskManager::getTaskError()
{
    if(task) {
        return task->getError();
    } else {
        return Eigen::VectorXd::Zero(0);
    }
}

double TaskManager::getTaskErrorNorm()
{
    return getTaskError().norm();
}

void TaskManager::setStiffness(double K)
{
    if(task) {
        task->setStiffness(K);
    }
}

void TaskManager::setStiffness(const VectorXd& K)
{
    if(task) {
        task->setStiffness(K);
    }
}

void TaskManager::setStiffness(const MatrixXd& K)
{
    if(task) {
        task->setStiffness(K);
    }
}

double TaskManager::getStiffness()
{
    if(task) {
        // Get the first entry in the stiffness matrix.
        return getStiffnessMatrix()(0,0);
    } else {
        yLog.error() << "Unable to get the stiffness of the task because it does not exist. Returning 0.0";
        return 0.0;
    }
}

Eigen::MatrixXd TaskManager::getStiffnessMatrix()
{
    if(task) {
        return task->getStiffness();
    } else {
        yLog.error() << "Unable to get the stiffness of the task because it does not exist. Returning empty matrix";
        return Eigen::MatrixXd::Zero(0,0);
    }
}

void TaskManager::setDamping(double B)
{
    if(task) {
        task->setDamping(B);
    }
}

void TaskManager::setDamping(const VectorXd& B)
{
    if(task) {
        task->setDamping(B);
    }
}

void TaskManager::setDamping(const MatrixXd& B)
{
    if(task) {
        task->setDamping(B);
    }
}

double TaskManager::getDamping()
{
    if(task) {
        // Get the first entry in the stiffness matrix.
        return getDampingMatrix()(0,0);
    } else {
        yLog.error() << "Unable to get the damping coeff of the task because it does not exist. Returning 0.0";
        return 0.0;
    }
}

Eigen::MatrixXd TaskManager::getDampingMatrix()
{
    if(task) {
        return task->getDamping();
    } else {
        yLog.error() << "Unable to get the damping coeffs of the task because it does not exist. Returning empty matrix";
        return Eigen::MatrixXd::Zero(0,0);
    }
}

void TaskManager::setWeight(double weight)
{
    if(task) {
        return task->setWeight(weight);
    }
}

void TaskManager::setWeight(Eigen::VectorXd& weights)
{
    if(task) {
        return task->setWeight(weights);
    }
}

Eigen::VectorXd TaskManager::getWeight()
{
    if(task) {
        return task->getWeight();
    } else {
        yLog.error() << "Unable to get the weights of the task because it does not exist. Returning empty vector";
        return Eigen::VectorXd::Zero(0);
    }
}


bool TaskManager::openControlPorts()
{
    bool res = true;
    if (!controlPortsOpen)
    {
        inputControlPortName = "/TM/"+stableName+"/state:i";
        outputControlPortName = "/TM/"+stableName+"/state:o";

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

bool TaskManager::closeControlPorts()
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

bool TaskManager::parseControlInput(yarp::os::Bottle& input)
{
    if (input.size() >= stateDimension)
    {
        for(int i=0; i<stateDimension; ++i)
        {
            newDesiredStateVector[i] = input.get(i).asDouble(); //make sure there are no NULL entries
        }
        Eigen::VectorXd newWeights = getWeight();
        if (input.size()==(stateDimension + newWeights.size())) {
            int j = 0;
            for(int i = stateDimension; i<(stateDimension + newWeights.size()); ++i)
            {
                newWeights(j) = input.get(i).asDouble(); //make sure there are no NULL entries
                ++j;
            }
            setWeight(newWeights);
        }

        setDesiredState(); // constructs the appropropriate state inputs
        return true;
    }
    else{return false;}
}

std::string TaskManager::getTaskManagerType()
{
    return "[TaskManager::getTaskManagerType()]: getTaskManagerType has not been implemented for this task manager.";
}

void TaskManager::setStateDimension(int taskDimension)
{
    stateDimension = taskDimension;
    currentStateVector.resize(stateDimension);
    desiredStateVector.resize(stateDimension);
    newDesiredStateVector.resize(stateDimension);

    for(int i=0; i<stateDimension; i++){
      newDesiredStateVector[i] = 0.0; //make sure there are no NULL entries
    }

    eigenCurrentStateVector = Eigen::VectorXd::Zero(stateDimension);
    eigenDesiredStateVector = Eigen::VectorXd::Zero(stateDimension);
}

void TaskManager::updateCurrentStateVector(const double* ptrToFirstIndex)
{
    if (controlPortsOpen){stateOutBottle.clear();}
    for(int i=0; i<stateDimension; i++){
        currentStateVector[i] = *ptrToFirstIndex;
        if (controlPortsOpen){stateOutBottle.addDouble(*ptrToFirstIndex);}
        ptrToFirstIndex++;
    }
    if (controlPortsOpen){outputControlPort.write(stateOutBottle);}
}

void TaskManager::updateDesiredStateVector(const double* ptrToFirstIndex)
{
    for(int i=0; i<stateDimension; i++){
        desiredStateVector[i] = *ptrToFirstIndex;
        ptrToFirstIndex++;
    }
}

const double* TaskManager::getCurrentState()
{
    Eigen::VectorXd emptyVector = Eigen::VectorXd::Zero(stateDimension);
    return emptyVector.data();
}


std::string TaskManager::getPortName()
{
    return portName;
}


/**************************************************************************************************
                                    Nested RpcMessageCallback Class
**************************************************************************************************/
TaskManager::RpcMessageCallback::RpcMessageCallback(TaskManager& tmBaseRef)
: tmBase(tmBaseRef)
{
    //do nothing
}

bool TaskManager::RpcMessageCallback::read(yarp::os::ConnectionReader& connection)
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
TaskManager::ControlInputCallback::ControlInputCallback(TaskManager& tmBaseRef)
: tmBase(tmBaseRef)
{
    //do nothing
}

bool TaskManager::ControlInputCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;
    if (input.read(connection)){
        return tmBase.parseControlInput(input);
    }
    else{
        return false;
    }
}
/**************************************************************************************************
**************************************************************************************************/


/**************************************************************************************************
                                    Nested StateUpdateThread Class
**************************************************************************************************/
TaskManager::StateUpdateThread::StateUpdateThread(int period, TaskManager& tmBaseRef):
RateThread(period),
tmBase(tmBaseRef)
{
    // Do nothing
}
bool TaskManager::StateUpdateThread::threadInit()
{
    return true;
}
void TaskManager::StateUpdateThread::run()
{
    tmBase.updateCurrentStateVector(tmBase.getCurrentState());
}
void TaskManager::StateUpdateThread::threadRelease()
{
    std::cout << "StateUpdateThread: Closing.\n";
}
/**************************************************************************************************
**************************************************************************************************/


void TaskManager::parseIncomingMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        std::string msgTag = input.get(i).asString();

        if(msgTag == "getCurrentState")
        {
            updateCurrentStateVector(getCurrentState());

            reply.addString("currentState:");
            for (int j=0; j < stateDimension; j++){
                reply.addDouble(currentStateVector[j]);
            }

            i++;
        }

        // Stiffness
        else if(msgTag == "setStiffness")
        {
            i++;
            setStiffness(input.get(i).asDouble());
            reply.addString("Kp:");
            reply.addDouble(getStiffness());
            i++;
        }
        else if(msgTag == "getStiffness")
        {
            reply.addString("Kp:");
            reply.addDouble(getStiffness());
            i++;
        }

        // Damping
        else if (msgTag == "setDamping")
        {
            i++;
            setDamping(input.get(i).asDouble());
            reply.addString("Kd:");
            reply.addDouble(getDamping());
            i++;
        }
        else if(msgTag == "getDamping")
        {
            reply.addString("Kd:");
            reply.addDouble(getDamping());
            i++;
        }

       // Weight
       else if (msgTag == "setWeight")
       {
           i++;
           Eigen::VectorXd currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               currentWeights(j) = input.get(i).asDouble();
               i++;
           }
           setWeight(currentWeights);
           reply.addString("Weight:");
           currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               reply.addDouble(currentWeights(j));
           }
       }
       else if(msgTag == "getWeight")
       {
           reply.addString("Weight:");
           Eigen::VectorXd currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               reply.addDouble(currentWeights(j));
           }
           i++;
       }
        // Desired State
        else if (msgTag == "setDesired")
        {
            i++;
            int startIndex = i;
            int k = 0;
            while(i<(startIndex + stateDimension))
            {
                newDesiredStateVector[k] = input.get(i).asDouble(); //make sure there are no NULL entries
                i++; k++;
            }
            setDesiredState(); // constructs the appropropriate state inputs
            reply.addString("Desired:");
            for (int j=0; j < stateDimension; j++){
                reply.addDouble(desiredStateVector[j]);
            }
            i++;
        }
        else if(msgTag == "getDesired")
        {
            reply.addString("Desired:");
            for (int j=0; j < stateDimension; j++){
                reply.addDouble(desiredStateVector[j]);
            }
            i++;
        }

        // Activity Status
        else if (msgTag == "getActivityStatus")
        {
            reply.addString("activated");
            reply.addInt(isActivated());
            i++;
        }

        // Activate
        else if (msgTag == "activate")
        {
            activate();
            if (isActivated()) {
                reply.addString("activated");
            }else{reply.addString("failed");}

            i++;
        }

        // Deactivate
        else if (msgTag == "deactivate")
        {
            deactivate();
            if (!isActivated()) {
                reply.addString("deactivated");
            }else{reply.addString("failed");}
            i++;
        }

        // State Dimension
        else if (msgTag == "getDimension")
        {
            reply.addString("Dimension:");
            reply.addInt(stateDimension);
            i++;
        }

        // Help
        else if (msgTag == "help")
        {
            // TODO: Properly print help message to rpc reply
            // reply.addString(printValidMessageTags());
            std::cout << printValidMessageTags();
            i++;
        }

        // Task Manager Type
        else if (msgTag == "getType")
        {
            reply.addString("Type:");
            reply.addString(getTaskManagerType());
            i++;
        }

        // Task Manager Name
        else if (msgTag == "getName")
        {
            reply.addString("Name:");
            reply.addString(stableName);
            i++;
        }

        // Open high speed control ports
        else if (msgTag == "openControlPorts")
        {
            if(openControlPorts()){
                reply.addInt(1);
            }
            else{
                reply.addInt(0);
            }
            i++;
        }
        // Close high speed control ports
        else if (msgTag == "closeControlPorts")
        {
            if(closeControlPorts()){
                reply.addInt(1);
            }
            else{
                reply.addInt(0);
            }
            i++;
        }

        // Get control port names
        else if (msgTag == "getControlPortNames")
        {
            if (controlPortsOpen) {
                reply.addString("Control port names:");
                reply.addString(inputControlPortName);
                reply.addString(outputControlPortName);
            }else{
                reply.addString("[ERROR] Control ports haven't been opened. Use command: openControlPorts");
            }
            i++;
        }

        // Fallback
        else
        {
            std::cout << "[WARNING] (TaskManager::parseIncomingMessage): The message tag, " << msgTag << " doesn't exist. Skipping. Use help to see availible options." << std::endl;

            reply.addString("invalid_input");
            i++;
        }
    }
}

std::string TaskManager::printValidMessageTags()
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
