#include <ocra-recipes/ControllerClient.h>

using namespace ocra_recipes;

int ControllerClient::CONTROLLER_CLIENT_COUNT = 0;

ControllerClient::ControllerClient()
: yarp::os::RateThread(DEFAULT_LOOP_PERIOD)
, isReady(false)
, expectedPeriod(DEFAULT_LOOP_PERIOD)
{

}

ControllerClient::ControllerClient(std::shared_ptr<ocra::Model> derivedModelPtr, const int loopPeriod)
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

}


void ControllerClient::addTasks(const std::string& pathToXmlFile, bool overwrite)
{
    ocra::TaskManagerFactory factory = ocra::TaskManagerFactory();
    factory.parseTasksXML(pathToXmlFile);
    std::vector<ocra::TaskManagerOptions> tmOptsVec = factory.getParsedOptionsVector();
    std::vector<ocra::TaskManagerOptions> addTmOptsVec;
    std::vector<ocra::TaskManagerOptions> overwriteTmOptsVec;
    for (auto tmOpts : tmOptsVec) {
        if(!checkIfTaskExists(tmOpts)) {
            addTmOptsVec.push_back(tmOpts);
        } else {
            if (overwrite) {
                overwriteTmOptsVec.push_back(tmOpts);
            }
        }
    }

    yarp::os::Bottle request;
    request.addInt(ADD_TASKS);
    request.addInt(addTmOptsVec.size());
    for (auto tmOpts : addTmOptsVec) {
        tmOpts.putIntoBottle(request);
    }
    if(clientComs->queryController(request).get(0).asInt() != SUCCESS)
    {
        std::cout << "Didn't work." << std::endl;
    }
    // Code to send the tm opts to the server.
}

bool ControllerClient::checkIfTaskExists(ocra::TaskManagerOptions& tmOpts)
{
    std::vector<std::string> tmNames = getTaskNames();
    for (auto name : tmNames) {
        if (tmOpts.taskName == name) {
            TaskConnection tCon(tmOpts.taskName);
            if (tmOpts.taskType == tCon.getTaskType()) {
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
