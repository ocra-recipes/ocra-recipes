#include <ocra-recipes/ControllerClient.h>

using namespace ocra_recipes;

int ControllerClient::CONTROLLER_CLIENT_COUNT = 0;

ControllerClient::ControllerClient()
: yarp::os::RateThread(DEFAULT_LOOP_PERIOD)
, isReady(false)
, expectedPeriod(DEFAULT_LOOP_PERIOD)
{

}

ControllerClient::ControllerClient(ocra::Model::Ptr derivedModelPtr, const int loopPeriod)
: yarp::os::RateThread(loopPeriod)
, model(derivedModelPtr)
, expectedPeriod(loopPeriod)
{
    clientNumber = ++ControllerClient::CONTROLLER_CLIENT_COUNT;

    clientComs = std::make_shared<ClientCommunications>();
    clientComs->open();

    std::string statesPort_Name = "/ControllerClient/"+ std::to_string(clientNumber) +"/states:i";

    statesPort.open(statesPort_Name.c_str());

    stateCallback = StateListener(model);

    statesPort.setReader(stateCallback);

    yarp.connect("/ControllerServer/states:o", statesPort_Name.c_str());

    isReady = true;
}

ControllerClient::~ControllerClient()
{

}

bool ControllerClient::threadInit()
{
    clientThreadHasBeenReleased = false;
    if(isReady)
        return initialize();
    else
        return false;
}

void ControllerClient::run()
{
    loop();
}

void ControllerClient::threadRelease()
{
    release();
    clientThreadHasBeenReleased = true;
}

bool ControllerClient::removeTask(const std::string& taskName)
{
    yarp::os::Bottle request;
    request.addInt(REMOVE_TASK);
    request.addString(taskName);

    return (clientComs->queryController(request).get(0).asInt() == SUCCESS);
}

bool ControllerClient::removeTasks(const std::vector<std::string>& taskNameVector)
{
    return false;
}


void ControllerClient::addTasks(const std::string& pathToXmlFile, bool overwrite)
{
    ocra::TaskConstructionManager factory = ocra::TaskConstructionManager();

    std::vector<ocra::TaskBuilderOptions> taskOptsVec = factory.parseTaskOptionsFromXml(pathToXmlFile);
    std::vector<ocra::TaskBuilderOptions> addTaskOptsVec;
    std::vector<ocra::TaskBuilderOptions> overwriteTaskOptsVec;
    for (auto taskOpts : taskOptsVec) {
        if(!checkIfTaskExists(taskOpts)) {
            addTaskOptsVec.push_back(taskOpts);
        } else {
            if (overwrite) {
                overwriteTaskOptsVec.push_back(taskOpts);
            }
        }
    }

    yarp::os::Bottle request;
    request.addInt(ADD_TASKS);
    request.addInt(addTaskOptsVec.size());
    for (auto taskOpts : addTaskOptsVec) {
        taskOpts.putIntoBottle(request);
    }
    if(clientComs->queryController(request).get(0).asInt() != SUCCESS)
    {
        std::cout << "Didn't work." << std::endl;
    }
    // Code to send the tm opts to the server.
}

bool ControllerClient::checkIfTaskExists(ocra::TaskBuilderOptions& tmOpts)
{
    std::vector<std::string> tmNames = getTaskNames();
    for (auto name : tmNames) {
        if (tmOpts.taskName == name) {
            TaskConnection tCon(tmOpts.taskName);
            if (tmOpts.taskType == tCon.getTaskTypeAsString()) {
                return true;
            }
            else {
                yLog.warning() << "You are trying to add a task with the same name as another task. I am gonna rename it for you big dummy.";
                tmOpts.taskName += "_" + std::to_string(clientNumber);
                return false;
            }
        }
    }
    return false;
}


std::vector<std::string> ControllerClient::getTaskTypes()
{
    return clientComs->getTaskTypes();
}

std::vector<std::string> ControllerClient::getTaskNames()
{
    return clientComs->getTaskNames();
}

bool ControllerClient::changeFixedLink(std::string newFixedLink)
{
    yarp::os::Bottle request;
    if (!newFixedLink.compare("r_sole") || !newFixedLink.compare("right")) {
        request.addInt(CHANGE_FIXED_LINK_RIGHT);
        std::cout << "[DEBUG-JORH] ControllerClient::changeFixedLink: Changed fixed link to right sole" << std::endl;
        if(clientComs->queryController(request).get(0).asInt() != SUCCESS)
        {
            std::cout << "[ERROR] Communication with ocra-icub-server didn't work. Requested change to right foot link" << std::endl;
            return false;
        }
        return true;
    } else {
        if (!newFixedLink.compare("l_sole") || !newFixedLink.compare("left")) {
            request.addInt(CHANGE_FIXED_LINK_LEFT);
            std::cout << "[DEBUG-JORH] ControllerClient::changeFixedLink: Changed fixed link to left sole" << std::endl;           
            if(clientComs->queryController(request).get(0).asInt() != SUCCESS)
            {
                std::cout << "[ERROR] Communication with ocra-icub-server didn't work. Requested change to left foot link" << std::endl;
                return false;
            }
            return true;
        } else {
            std::cout << "[ERROR] ControllerClient::changeFixedLink - The new fixed link you specify is not suppoert yet. Please try r_sole, right, l_sole or left." << std::endl;
        }
    }
    return false;
}
