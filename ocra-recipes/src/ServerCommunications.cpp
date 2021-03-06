#include <ocra-recipes/ServerCommunications.h>


using namespace ocra_recipes;

ServerCommunications::ServerCommunications()
{
}

ServerCommunications::ServerCommunications(ocra::Controller::Ptr ctrl, ocra::Model::Ptr mdl)
: model(mdl),
  controller(ctrl)
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
        if (!reply.write(*returnToSender)) {
            OCRA_ERROR("Error writing reply to sender");
            return false;
        }
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
                OCRA_INFO("Got message: GET_IS_FLOATING_BASE.");
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
                OCRA_INFO("Got message: CHANGE_FIXED_LINK_RIGHT.");
                this->controller->setFixedLinkForOdometry("r_sole");
                int isInLeftSupport = input.get(++i).asInt();
                OCRA_INFO("Got message: " << isInLeftSupport);
                int isInRightSupport = input.get(++i).asInt();
                OCRA_INFO("Got message: " << isInRightSupport);
                this->controller->setContactState(isInLeftSupport, isInRightSupport);
                reply.addInt(SUCCESS);
            } break;
            
            case CHANGE_FIXED_LINK_LEFT:
            {
                OCRA_INFO("Got message: CHANGE_FIXED_LINK_LEFT.");
                this->controller->setFixedLinkForOdometry("l_sole");
                int isInLeftSupport = input.get(++i).asInt();
                OCRA_INFO("Got message: " << isInLeftSupport);
                int isInRightSupport = input.get(++i).asInt();
                OCRA_INFO("Got message: " << isInRightSupport);
                this->controller->setContactState(isInLeftSupport, isInRightSupport);
                reply.addInt(SUCCESS);
            } break;


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
                OCRA_INFO("Got message: REMOVE_TASK.");
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
                OCRA_INFO("Got message: GET_TASK_LIST.");
                for(auto taskName : controller->getTaskNames()) {
                    reply.addString(taskName);
                }
            }break;

            case GET_TASK_PORT_LIST:
            {
                OCRA_INFO("Got message: GET_TASK_PORT_LIST.");
                for(auto taskPort : controller->getTaskPortNames()) {
                    reply.addString(taskPort);
                }
            }break;

            case GET_TASK_PORT_NAME:
            {
                std::string taskName = input.get(++i).asString();
                OCRA_INFO("Got message: GET_TASK_PORT_NAME.");
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
