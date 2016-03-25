#include <ocra-recipes/ServerCommunications.h>


using namespace ocra_recipes;

ServerCommunications::ServerCommunications()
{
}

ServerCommunications::ServerCommunications(std::shared_ptr<ocra::Controller> ctrl, std::shared_ptr<ocra::Model> mdl, std::shared_ptr<TaskManagerSet> tms)
: controller(ctrl)
, model(mdl)
, taskManagerSet(tms)
{
    rpcServerPort_Name = "/ControllerServer/rpc:i";
    outputPort_Name = "/ControllerServer:o";
}

ServerCommunications::~ServerCommunications()
{
    close();
}

bool ServerCommunications::open()
{
    rpcServerPort.open(rpcServerPort_Name.c_str());
    rpcServerPort.setReader(*this);
    outputPort.open(outputPort_Name.c_str());
}
bool ServerCommunications::close()
{
    rpcServerPort.close();
    outputPort.close();
}

bool ServerCommunications::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input, reply;

    if (!input.read(connection)){
        return false;
    }
    else{
        parseMessage(input, reply);
        yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}

void ServerCommunications::parseMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        switch (input.get(i).asInt()) {
            case GET_CONTROLLER_STATUS:
                {
                    std::cout << "Got message: GET_CONTROLLER_STATUS." << std::endl;
                    // reply.addInt(controllerStatus);
                    ++i;
                }break;

            case GET_WBI_CONFIG_FILE_PATH:
                {
                    std::cout << "Got message: GET_WBI_CONFIG_FILE_PATH." << std::endl;
                    // reply.addString(ctrlOptions.wbiConfigFilePath);
                    ++i;
                }break;

            case GET_ROBOT_NAME:
                {
                    std::cout << "Got message: GET_ROBOT_NAME." << std::endl;
                    // reply.addString(ctrlOptions.robotName);
                    ++i;
                }break;

            case GET_IS_FLOATING_BASE:
                {
                    std::cout << "Got message: GET_IS_FLOATING_BASE." << std::endl;
                    reply.addInt(!model->hasFixedRoot());
                    ++i;
                }break;

            case START_CONTROLLER:
                {
                    ++i;
                    std::cout << "Got message: START_CONTROLLER." << std::endl;
                    // this->start();
                    // TODO: make a switch case for if the controller is suspended then resume but if it is stopped then start.
                }break;

            case STOP_CONTROLLER:
                {
                    ++i;
                    std::cout << "Got message: STOP_CONTROLLER." << std::endl;
                    // this->stop();
                }break;

            case PAUSE_CONTROLLER:
                {
                    ++i;
                    std::cout << "Got message: PAUSE_CONTROLLER." << std::endl;
                    // this->suspend();
                    // TODO: Make a custom function that puts the robot in pos mode before suspend.
                }break;

            case ADD_TASK:
                {
                    ++i;
                    std::cout << "Got message: ADD_TASK." << std::endl;
                }break;

            case ADD_TASK_FROM_FILE:
                {
                    ++i;
                    std::cout << "Got message: ADD_TASK_FROM_FILE." << std::endl;
                }break;

            case REMOVE_TASK:
                {
                    ++i;
                    std::cout << "Got message: REMOVE_TASK." << std::endl;
                    std::string taskToRemove = input.get(i).asString();
                    bool taskRemoved = taskManagerSet->removeTaskManager(taskToRemove);
                    if (taskRemoved) {
                        reply.addInt(SERVER_COMMUNICATIONS_MESSAGE::SUCCESS);
                        yarp::os::Bottle outputMessage;
                        outputMessage.addInt(SERVER_COMMUNICATIONS_MESSAGE::REMOVE_TASK_PORT);
                        outputMessage.addString(taskToRemove);
                        outputPort.write(outputMessage);
                    }else{
                        reply.addInt(SERVER_COMMUNICATIONS_MESSAGE::FAILURE);
                    }
                    ++i;
                }break;

            case REMOVE_TASKS:
                {
                    ++i;
                    std::cout << "Got message: REMOVE_TASKS." << std::endl;
                }break;

            case GET_TASK_LIST:
                {
                    ++i;
                    std::cout << "Got message: GET_TASK_LIST." << std::endl;
                    for(auto taskName : taskManagerSet->getTaskList()){
                        reply.addString(taskName);
                    }
                }break;

            case GET_TASK_PORT_LIST:
                {
                    ++i;
                    std::cout << "Got message: GET_TASK_PORT_LIST." << std::endl;
                    for(auto taskPort : taskManagerSet->getTaskPorts()){
                        reply.addString(taskPort);
                    }
                }break;

            case GET_TASK_PORT_NAME:
                {
                    ++i;
                    std::string taskName = input.get(i).asString();
                    ++i;
                    std::cout << "Got message: GET_TASK_PORT_NAME." << std::endl;
                    int taskCounter = 0;
                    int taskFoundIndex = -1;
                    for(auto tName : taskManagerSet->getTaskList()){
                        if (taskName == tName) {
                            taskFoundIndex = taskCounter;
                        } else {
                            ++taskCounter;
                        }
                    }
                    if (taskFoundIndex >= 0)
                    {
                        reply.addString(taskManagerSet->getTaskPorts()[taskFoundIndex]);
                    }

                }break;

            case HELP:
                {
                    ++i;
                    std::cout << "Got message: HELP." << std::endl;
                }break;

            default:
                {
                    ++i;
                    std::cout << "Got message: UNKNOWN." << std::endl;
                }break;

        }
    }
}
