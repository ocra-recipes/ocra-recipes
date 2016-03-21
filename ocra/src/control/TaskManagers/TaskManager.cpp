#include "ocra/control/TaskManagers/TaskManager.h"


// #include "ocra/control/Trajectory/MinimumJerkTrajectory.h"
// #include "ocra/control/Trajectory/LinearInterpolationTrajectory.h"

namespace ocra
{

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
, task(NULL)
, usesTrajectory(false)
, followingTrajectory(false)
, controlPortsOpen(false)
, stateDimension(0)
{
    stableName = name;

    setTrajectoryType();

    portName = "/TM/"+name+"/rpc:i";
    rpcCallback =  std::unique_ptr<RpcMessageCallback>(new RpcMessageCallback(*this));
    rpcPort.open(portName.c_str());
    rpcPort.setReader(*rpcCallback);
}


TaskManager::~TaskManager()
{
    std::cout << "\t--> Closing ports" << std::endl;
    rpcPort.close();
    // if(task){
    //     std::dynamic_pointer_cast<OneLevelTask>(task)->disconnectFromController();
    // }
    if(stateThread){
        closeControlPorts();
    }


    std::cout << "\t--> Destroying " << stableName << std::endl;
}

/** Returns the error vector of the task
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
Eigen::VectorXd TaskManager::getTaskError()
{
    throw std::runtime_error(std::string("[TaskManager::getTaskError()]: getTaskError has not been implemented or is not supported"));
}

/** Returns the norm of the error vector
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
double TaskManager::getTaskErrorNorm()
{
    return getTaskError().norm();
}


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

        else if (msgTag == "useTrajectory")
        {
            i++;
            usesTrajectory = true;
            if(!input.get(i).asString().empty()){
                setTrajectoryType(input.get(i).asString());
                i++;
            }else{
                setTrajectoryType();
            }
            reply.addString("Using Trajectory");
        }

        else if (msgTag == "setMaxVelocity")
        {
            i++;
            taskTrajectory->setMaxVelocity(input.get(i).asDouble());
            reply.addString("maxVel:");
            reply.addDouble(taskTrajectory->getMaxVelocity());
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
            if (usesTrajectory) {
                updateCurrentStateVector(getCurrentState());
                taskTrajectory->setWaypoints(currentStateVector, newDesiredStateVector, waypointSelector);
                followingTrajectory = true;
                reply.addString("Starting Trajectory...");
            }

            else{
                setDesiredState(); // constructs the appropropriate state inputs
                reply.addString("Desired:");
                for (int j=0; j < stateDimension; j++){
                    reply.addDouble(desiredStateVector[j]);
                }
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
            reply.addInt(checkIfActivated());
            i++;
        }

        // Activate
        else if (msgTag == "activate")
        {
            activate();
            if (checkIfActivated()) {
                reply.addString("activated");
            }else{reply.addString("failed");}

            i++;
        }

        // Deactivate
        else if (msgTag == "deactivate")
        {
            deactivate();
            if (!checkIfActivated()) {
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
    helpString += "useTrajectory: Automatically generate a trajectory to follow for new desired states. You can optionally specify the type of trajectory: MinJerk, LinInterp, Experimental.\n";
    helpString += "openControlPorts: Open up high speed yarp control ports.\n";
    helpString += "closeControlPorts: Close high speed yarp control ports.\n";
    helpString += "getControlPortNames: Get the names of the high speed yarp control ports.\n";
    helpString += "help: Prints all the valid commands. No arguments expected.\n";

    helpString += "\nTypical usage: [message tag] [message value(s)]\ne.g.  >> stiffness 20 damping 10 desired_state 1.0 2.0 2.0\n";

    return helpString;
}

bool TaskManager::openControlPorts()
{
    bool res = true;
    inputControlPortName = "/TM/"+stableName+"/state:i";
    outputControlPortName = "/TM/"+stableName+"/state:o";

    res = res && inputControlPort.open(inputControlPortName.c_str());
    res = res && outputControlPort.open(outputControlPortName.c_str());

    controlCallback = std::unique_ptr<ControlInputCallback>(new ControlInputCallback(*this));
    inputControlPort.setReader(*controlCallback);

    stateThread = std::unique_ptr<StateUpdateThread>(new StateUpdateThread(10, *this));
    stateThread->start();

    controlPortsOpen = res;

    return res;
}

bool TaskManager::closeControlPorts()
{
    if (stateThread->isRunning()) {
        stateThread->stop();
    }
    if (controlPortsOpen) {
        inputControlPort.close();
        outputControlPort.close();
    }

    bool res = !inputControlPort.isOpen() && !outputControlPort.isOpen();

    controlPortsOpen = !res;

    return res;
}

bool TaskManager::parseControlInput(yarp::os::Bottle& input)
{
    if (input.size() >= stateDimension)
    {
        for(int i=0; i<stateDimension; i++)
        {
            newDesiredStateVector[i] = input.get(i).asDouble(); //make sure there are no NULL entries
        }
        Eigen::VectorXd newWeights = getWeight();
        if (input.size()==(stateDimension + newWeights.size())) {
            int j = 0;
            for(int i = stateDimension; i<(stateDimension + newWeights.size()); i++)
            {
                newWeights(j) = input.get(i).asDouble(); //make sure there are no NULL entries
                j++;
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

void TaskManager::setStateDimension(int taskDimension, int waypointDimension)
{
    waypointSelector = waypointDimension;
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


bool TaskManager::checkIfActivated()
{
    return false;
}


std::string TaskManager::getPortName()
{
    return portName;
}

void TaskManager::setTrajectoryType(std::string trajType)
{
    if (trajType=="MinJerk") {
        taskTrajectory = std::unique_ptr<MinimumJerkTrajectory>(new MinimumJerkTrajectory());
    }
    else if (trajType=="LinInterp") {
        taskTrajectory = std::unique_ptr<LinearInterpolationTrajectory>(new LinearInterpolationTrajectory());
    }
    // else if (trajType == "Experimental"){
    //     taskTrajectory = std::unique_ptr<ExperimentalTrajectory>(new ExperimentalTrajectory());
    // }
    else {
        taskTrajectory = std::unique_ptr<MinimumJerkTrajectory>(new MinimumJerkTrajectory());
    }
}

void TaskManager::updateTrajectory(double time)
{
    taskTrajectory->getDesiredValues(time, newDesiredStateVector);
    setDesiredState();
    followingTrajectory = !taskTrajectory->isFinished();
}


/**************************************************************************************************
                                    Nested PortReader Class
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
                                    Nested PortReader Class
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
                                    StateUpdateThread Class
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

}
