#include <ocra-recipes/ServerCommunications.h>


using namespace ocra_recipes;

ServerCommunications::ServerCommunications()
{
}

ServerCommunications::ServerCommunications(ocra::Controller::Ptr ctrl, ocra::Model::Ptr mdl)
: controller(ctrl)
, model(mdl)
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
    bool res = true;
    res &= rpcServerPort.open(rpcServerPort_Name.c_str());
    rpcServerPort.setReader(*this);
    res &= outputPort.open(outputPort_Name.c_str());
    return res;
}
void ServerCommunications::close()
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
    parseMessage(input, reply);
    yarp::os::ConnectionWriter* returnToSender = connection.getWriter();
    if (returnToSender!=NULL) {
        reply.write(*returnToSender);
    }
    return true;
}

void ServerCommunications::parseMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize; ++i)
    {
        switch (input.get(i).asInt()) {
            case GET_CONTROLLER_STATUS:
            {
                std::cout << "Got message: GET_CONTROLLER_STATUS." << std::endl;
                // reply.addInt(controllerStatus);
            }break;

            case GET_WBI_CONFIG_FILE_PATH:
            {
                std::cout << "Got message: GET_WBI_CONFIG_FILE_PATH." << std::endl;
                // reply.addString(ctrlOptions.wbiConfigFilePath);
            }break;

            case GET_ROBOT_NAME:
            {
                std::cout << "Got message: GET_ROBOT_NAME." << std::endl;
                // reply.addString(ctrlOptions.robotName);
            }break;

            case GET_IS_FLOATING_BASE:
            {
                std::cout << "Got message: GET_IS_FLOATING_BASE." << std::endl;
                reply.addInt(!model->hasFixedRoot());
            }break;
                
            case START_CONTROLLER:
            {
                std::cout << "Got message: START_CONTROLLER." << std::endl;
                // this->start();
                // TODO: make a switch case for if the controller is suspended then resume but if it is stopped then start.
            }break;

            case STOP_CONTROLLER:
            {
                std::cout << "Got message: STOP_CONTROLLER." << std::endl;
                // this->stop();
            }break;

            case PAUSE_CONTROLLER:
            {
                std::cout << "Got message: PAUSE_CONTROLLER." << std::endl;
                // this->suspend();
                // TODO: Make a custom function that puts the robot in pos mode before suspend.
            }break;
                
            case CHANGE_FIXED_LINK_RIGHT:
            {
                std::cout << "Got message: CHANGE_FIXED_LINK_RIGHT." << std::endl;
                //How to I know which
                this->controller->setFixedLinkForOdometry("r_sole");
            }
            case CHANGE_FIXED_LINK_LEFT:
            {
                std::cout << "Got message: CHANGE_FIXED_LINK_LEFT." << std::endl;
                //How to I know which
                this->controller->setFixedLinkForOdometry("l_sole");
            }


            case ADD_TASKS:
            {
                int numberOfTasks = input.get(++i).asInt();
                ++i;

                std::vector<ocra::TaskBuilderOptions> taskOptionsVector;
                for (int j=0; j<numberOfTasks; ++j)
                {
                    int sizeOfOptions = 0;
                    yarp::os::Bottle trimmedBottle = ocra::util::trimBottle(input, i);
                    ocra::TaskBuilderOptions taskOptions;
                    if (taskOptions.extractFromBottle(trimmedBottle, sizeOfOptions)) {
                        taskOptionsVector.push_back(taskOptions);
                    }
                    i += sizeOfOptions;
                }
                ocra::TaskConstructionManager factory(model, controller, taskOptionsVector);
                reply.addInt(SUCCESS);
            }break;

            case ADD_TASKS_FROM_FILE:
            {
                std::cout << "Got message: ADD_TASK_FROM_FILE." << std::endl;
            }break;

            case REMOVE_TASK:
            {
                ++i;
                std::cout << "Got message: REMOVE_TASK." << std::endl;
                std::string taskToRemove = input.get(i).asString();
                controller->removeTask(taskToRemove);
                // if (taskRemoved) {
                    reply.addInt(SERVER_COMMUNICATIONS_MESSAGE::SUCCESS);
                    yarp::os::Bottle outputMessage;
                    outputMessage.addInt(SERVER_COMMUNICATIONS_MESSAGE::REMOVE_TASK_PORT);
                    outputMessage.addString(taskToRemove);
                    outputPort.write(outputMessage);
                // }else{
                //     reply.addInt(SERVER_COMMUNICATIONS_MESSAGE::FAILURE);
                // }
                ++i;
            }break;

            case REMOVE_TASKS:
            {
                std::cout << "Got message: REMOVE_TASKS." << std::endl;
            }break;

            case GET_TASK_LIST:
            {
                std::cout << "Got message: GET_TASK_LIST." << std::endl;
                for(auto taskName : controller->getTaskNames()) {
                    reply.addString(taskName);
                }
            }break;

            case GET_TASK_PORT_LIST:
            {
                std::cout << "Got message: GET_TASK_PORT_LIST." << std::endl;
                for(auto taskPort : controller->getTaskPortNames()) {
                    reply.addString(taskPort);
                }
            }break;

            case GET_TASK_PORT_NAME:
            {
                std::string taskName = input.get(++i).asString();
                std::cout << "Got message: GET_TASK_PORT_NAME." << std::endl;
                reply.addString(controller->getTaskPortName(taskName));
            }break;

            case HELP:
            {
                std::cout << "Got message: HELP." << std::endl;
            }break;

            default:
            {
                std::cout << "Got message: UNKNOWN." << std::endl;
            }break;

        }
    }
}
