#include "wocra/Tasks/wOcraTaskManagerBase.h"


#include "wocra/Trajectory/wOcraMinimumJerkTrajectory.h"
#include "wocra/Trajectory/wOcraLinearInterpolationTrajectory.h"

namespace wocra
{

/** base constructor
 *
 * \param ctrl                  wOcraController to connect to
 * \param model                 ocra model to setup the task
 * \param taskName              Name of the task
 */
wOcraTaskManagerBase::wOcraTaskManagerBase(wOcraController& _ctrl, const ocra::Model& _model, const std::string& _taskName, bool _usesYarpPorts):
ctrl(_ctrl),
model(_model),
name(_taskName),
usesYARP(_usesYarpPorts),
processor(NULL),
controlCallback(NULL),
stateThread(NULL),
taskTrajectory(NULL),
task(NULL)
{
    stableName = name;

    //TODO: Make these args default to true in the individual task managers and add them to the task parser
    // usesYARP = true;
    usesTrajectory = false;
    setTrajectoryType();
    followingTrajectory = false;

    if (usesYARP) {
        portName = "/TM/"+name+"/rpc:i";

        processor = new rpcMessageCallback(*this);
        rpcPort.open(portName.c_str());
        rpcPort.setReader(*processor);

        std::cout << "\n";
    }
    stateDimension = 0; // should be overwritten by derived classes who have implemented the necessary functions.

    controlPortsOpen = false;
}


wOcraTaskManagerBase::~wOcraTaskManagerBase()
{
    if (usesYARP) {
        std::cout << "\t--> Closing ports" << std::endl;
        rpcPort.close();
    }
    if(task!=NULL){
        task->disconnectFromController();
    }
    if(processor!=NULL){delete processor; processor = NULL;}
    if(controlCallback!=NULL){delete controlCallback; controlCallback = NULL;}
    if(stateThread!=NULL){delete stateThread; stateThread = NULL;}
    if(taskTrajectory!=NULL){delete taskTrajectory; taskTrajectory = NULL;}



    std::cout << "\t--> Destroying " << stableName << std::endl;
}

/** Returns the error vector of the task
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
Eigen::VectorXd wOcraTaskManagerBase::getTaskError()
{
    throw std::runtime_error(std::string("[wOcraTaskManagerBase::getTaskError()]: getTaskError has not been implemented or is not supported"));
}

/** Returns the norm of the error vector
 *
 *  If the derived child class does not have a meaningful error, it should override this function to throw an error
 */
double wOcraTaskManagerBase::getTaskErrorNorm()
{
    return getTaskError().norm();
}


void wOcraTaskManagerBase::parseIncomingMessage(yarp::os::Bottle *input, yarp::os::Bottle *reply)
{
    int btlSize = input->size();
    for (int i=0; i<btlSize;)
    {
        std::string msgTag = input->get(i).asString();

        if(msgTag == "getCurrentState")
        {
            updateCurrentStateVector(getCurrentState());

            reply->addString("currentState:");
            for (int j=0; j < stateDimension; j++){
                reply->addDouble(currentStateVector[j]);
            }

            i++;
        }

        // Stiffness
        else if(msgTag == "setStiffness")
        {
            i++;
            setStiffness(input->get(i).asDouble());
            reply->addString("Kp:");
            reply->addDouble(getStiffness());
            i++;
        }
        else if(msgTag == "getStiffness")
        {
            reply->addString("Kp:");
            reply->addDouble(getStiffness());
            i++;
        }

        // Damping
        else if (msgTag == "setDamping")
        {
            i++;
            setDamping(input->get(i).asDouble());
            reply->addString("Kd:");
            reply->addDouble(getDamping());
            i++;
        }
        else if(msgTag == "getDamping")
        {
            reply->addString("Kd:");
            reply->addDouble(getDamping());
            i++;
        }

       // Weight
       else if (msgTag == "setWeight")
       {
           i++;
           Eigen::VectorXd currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               currentWeights(j) = input->get(i).asDouble();
               i++;
           }
           setWeight(currentWeights);
           reply->addString("Weight:");
           currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               reply->addDouble(currentWeights(j));
           }
       }
       else if(msgTag == "getWeight")
       {
           reply->addString("Weight:");
           Eigen::VectorXd currentWeights = getWeight();
           for(int j=0; j<currentWeights.size(); j++)
           {
               reply->addDouble(currentWeights(j));
           }
           i++;
       }

        else if (msgTag == "useTrajectory")
        {
            i++;
            usesTrajectory = true;
            if(!input->get(i).asString().empty()){
                setTrajectoryType(input->get(i).asString());
                i++;
            }else{
                setTrajectoryType();
            }
            reply->addString("Using Trajectory");
        }

        else if (msgTag == "setMaxVelocity")
        {
            i++;
            taskTrajectory->setMaxVelocity(input->get(i).asDouble());
            reply->addString("maxVel:");
            reply->addDouble(taskTrajectory->getMaxVelocity());
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
                newDesiredStateVector[k] = input->get(i).asDouble(); //make sure there are no NULL entries
                i++; k++;
            }
            if (usesTrajectory) {
                updateCurrentStateVector(getCurrentState());
                taskTrajectory->setWaypoints(currentStateVector, newDesiredStateVector, waypointSelector);
                followingTrajectory = true;
                reply->addString("Starting Trajectory...");
            }

            else{
                setDesiredState(); // constructs the appropropriate state inputs
                reply->addString("Desired:");
                for (int j=0; j < stateDimension; j++){
                    reply->addDouble(desiredStateVector[j]);
                }
            }
            i++;
        }
        else if(msgTag == "getDesired")
        {
            reply->addString("Desired:");
            for (int j=0; j < stateDimension; j++){
                reply->addDouble(desiredStateVector[j]);
            }
            i++;
        }

        // Activity Status
        else if (msgTag == "getActivityStatus")
        {
            reply->addString("activated");
            reply->addInt(checkIfActivated());
            i++;
        }

        // Activate
        else if (msgTag == "activate")
        {
            activate();
            if (checkIfActivated()) {
                reply->addString("activated");
            }else{reply->addString("failed");}

            i++;
        }

        // Deactivate
        else if (msgTag == "deactivate")
        {
            deactivate();
            if (!checkIfActivated()) {
                reply->addString("deactivated");
            }else{reply->addString("failed");}
            i++;
        }

        // State Dimension
        else if (msgTag == "getDimension")
        {
            reply->addString("Dimension:");
            reply->addInt(stateDimension);
            i++;
        }

        // Help
        else if (msgTag == "help")
        {
            // TODO: Properly print help message to rpc reply
            // reply->addString(printValidMessageTags());
            std::cout << printValidMessageTags();
            i++;
        }

        // Task Manager Type
        else if (msgTag == "getType")
        {
            reply->addString("Type:");
            reply->addString(getTaskManagerType());
            i++;
        }

        // Task Manager Name
        else if (msgTag == "getName")
        {
            reply->addString("Name:");
            reply->addString(stableName);
            i++;
        }

        // Open high speed control ports
        else if (msgTag == "openControlPorts")
        {
            if(openControlPorts()){
                reply->addInt(1);
            }
            else{
                reply->addInt(0);
            }
            i++;
        }
        // Close high speed control ports
        else if (msgTag == "closeControlPorts")
        {
            if(closeControlPorts()){
                reply->addInt(1);
            }
            else{
                reply->addInt(0);
            }
            i++;
        }

        // Get control port names
        else if (msgTag == "getControlPortNames")
        {
            if (controlPortsOpen) {
                reply->addString("Control port names:");
                reply->addString(inputControlPortName);
                reply->addString(outputControlPortName);
            }else{
                reply->addString("[ERROR] Control ports haven't been opened. Use command: openControlPorts");
            }
            i++;
        }

        // Fallback
        else
        {
            std::cout << "[WARNING] (wOcraTaskManagerBase::parseIncomingMessage): The message tag, " << msgTag << " doesn't exist. Skipping. Use help to see availible options." << std::endl;

            reply->addString("invalid_input");
            i++;
        }
    }
}

std::string wOcraTaskManagerBase::printValidMessageTags()
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

bool wOcraTaskManagerBase::openControlPorts()
{
    bool res = true;
    inputControlPortName = "/TM/"+stableName+"/state:i";
    outputControlPortName = "/TM/"+stableName+"/state:o";

    res = res && inputControlPort.open(inputControlPortName.c_str());
    res = res && outputControlPort.open(outputControlPortName.c_str());

    controlCallback = new controlInputCallback(*this);
    inputControlPort.setReader(*controlCallback);

    // stateCallback = new controlOutputCallback(*this);
    // outputControlPort.setReader(*stateCallback);

    stateThread = new stateUpdateThread(10, *this);
    stateThread->start();

    controlPortsOpen = res;

    return res;
}

bool wOcraTaskManagerBase::closeControlPorts()
{
    if (controlPortsOpen) {
        stateThread->stop();
        delete stateThread;
        stateThread = NULL;
        inputControlPort.close();
        outputControlPort.close();
    }

    bool res = !inputControlPort.isOpen() && !outputControlPort.isOpen();

    controlPortsOpen = !res;

    return res;
}

bool wOcraTaskManagerBase::parseControlInput(yarp::os::Bottle *input)
{
    if (input->size() >= stateDimension)
    {
        for(int i=0; i<stateDimension; i++)
        {
            newDesiredStateVector[i] = input->get(i).asDouble(); //make sure there are no NULL entries
        }
        Eigen::VectorXd newWeights = getWeight();
        if (input->size()==(stateDimension + newWeights.size())) {
            int j = 0;
            for(int i = stateDimension; i<(stateDimension + newWeights.size()); i++)
            {
                newWeights(j) = input->get(i).asDouble(); //make sure there are no NULL entries
                j++;
            }
            setWeight(newWeights);
        }

        setDesiredState(); // constructs the appropropriate state inputs
        return true;
    }
    else{return false;}
}

std::string wOcraTaskManagerBase::getTaskManagerType()
{
    return "[wOcraTaskManagerBase::getTaskManagerType()]: getTaskManagerType has not been implemented for this task manager.";
}

void wOcraTaskManagerBase::setStateDimension(int taskDimension, int waypointDimension)
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

void wOcraTaskManagerBase::updateCurrentStateVector(const double* ptrToFirstIndex)
{
    if (controlPortsOpen){stateOutBottle.clear();}
    for(int i=0; i<stateDimension; i++){
        currentStateVector[i] = *ptrToFirstIndex;
        if (controlPortsOpen){stateOutBottle.addDouble(*ptrToFirstIndex);}
        ptrToFirstIndex++;
    }
    if (controlPortsOpen){outputControlPort.write(stateOutBottle);}
}

void wOcraTaskManagerBase::updateDesiredStateVector(const double* ptrToFirstIndex)
{
    for(int i=0; i<stateDimension; i++){
        desiredStateVector[i] = *ptrToFirstIndex;
        ptrToFirstIndex++;
    }
}

const double* wOcraTaskManagerBase::getCurrentState()
{
    Eigen::VectorXd emptyVector = Eigen::VectorXd::Zero(stateDimension);
    return emptyVector.data();
}


bool wOcraTaskManagerBase::checkIfActivated()
{
    return false;
}


std::string wOcraTaskManagerBase::getPortName()
{
    return portName;
}

void wOcraTaskManagerBase::setTrajectoryType(std::string trajType)
{
    if (trajType=="MinJerk") {
        taskTrajectory = new wOcraMinimumJerkTrajectory();
    }
    else if (trajType=="LinInterp") {
        taskTrajectory = new wOcraLinearInterpolationTrajectory();
    }
    // else if (trajType == "Experimental"){
    //     taskTrajectory = new wOcraExperimentalTrajectory();
    // }
    else {
        taskTrajectory = new wOcraMinimumJerkTrajectory();
    }
}

void wOcraTaskManagerBase::updateTrajectory(double time)
{
    taskTrajectory->getDesiredValues(time, newDesiredStateVector);
    setDesiredState();
    followingTrajectory = !taskTrajectory->isFinished();
}


/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
wOcraTaskManagerBase::rpcMessageCallback::rpcMessageCallback(wOcraTaskManagerBase& tmBaseRef):tmBase(tmBaseRef)
{
    //do nothing
}

bool wOcraTaskManagerBase::rpcMessageCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;
    bool ok = input.read(connection);
    if (!ok)
        return false;

    else{
        tmBase.parseIncomingMessage(&input, &reply);
        yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
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
wOcraTaskManagerBase::controlInputCallback::controlInputCallback(wOcraTaskManagerBase& tmBaseRef):tmBase(tmBaseRef)
{
    //do nothing
}

bool wOcraTaskManagerBase::controlInputCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;
    if (input.read(connection)){
        return tmBase.parseControlInput(&input);
    }
    else{
        return false;
    }
}
/**************************************************************************************************
**************************************************************************************************/


/**************************************************************************************************
                                    stateUpdateThread Class
**************************************************************************************************/
wOcraTaskManagerBase::stateUpdateThread::stateUpdateThread(int period, wOcraTaskManagerBase& tmBaseRef):
RateThread(period),
tmBase(tmBaseRef)
{
    // Do nothing
}
bool wOcraTaskManagerBase::stateUpdateThread::threadInit()
{
    return true;
}
void wOcraTaskManagerBase::stateUpdateThread::run()
{
    tmBase.updateCurrentStateVector(tmBase.getCurrentState());
}
void wOcraTaskManagerBase::stateUpdateThread::threadRelease()
{
    std::cout << "stateUpdateThread: Closing.\n";
}
/**************************************************************************************************
**************************************************************************************************/

}
