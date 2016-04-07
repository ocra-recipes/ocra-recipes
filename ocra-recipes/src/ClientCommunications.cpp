#include <ocra-recipes/ClientCommunications.h>


using namespace ocra_recipes;

int ClientCommunications::CONTROLLER_CLIENT_COUNT = 0;


ClientCommunications::ClientCommunications()
{
    clientNumber = ++ClientCommunications::CONTROLLER_CLIENT_COUNT;

    rpcClientPort_Name = "/ControllerClient/"+ std::to_string(clientNumber) +"/rpc:o";
    inputPort_Name = "/ControllerClient/"+ std::to_string(clientNumber) +"/:i";
}

ClientCommunications::~ClientCommunications()
{
    close();
}

bool ClientCommunications::open(const bool connectToTaskManagers)
{
    rpcClientPort.open(rpcClientPort_Name.c_str());
    rpcClientPort.setReader(*this);
    inputPort.open(inputPort_Name.c_str());

    bool isConOpen = openServerConnections();
    if(isConOpen && connectToTaskManagers)
    {
        isConOpen &= openTaskManagerConnections();

        if(isConOpen)
        {
            std::cout << "Checking task manager rpc server connections..." << std::endl;
            for(auto rpc_i : taskRpcClients)
            {
                yarp::os::Bottle message, reply;
                message.addInt(ocra::TASK_MESSAGE::GET_TYPE);
                rpc_i.second->write(message, reply);
                std::cout << reply.toString() << std::endl;
            }
            std::cout << "All set!" << std::endl;
        }else{
            yLog.error() << "Couldn't connect to the individual task ports.";
        }
    }
    return isConOpen;
}

bool ClientCommunications::close()
{
    rpcClientPort.close();
    inputPort.close();

    for(auto rpc_i : taskRpcClients)
    {
        rpc_i.second->close();
    }
    taskRpcClients.clear();
}

void ClientCommunications::close(const std::string& taskName)
{
    if(taskRpcClients.find(taskName) != taskRpcClients.end())
    {
        taskRpcClients[taskName]->close();
        taskRpcClients.erase(taskName);
    }
}

bool ClientCommunications::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;

    if (!input.read(connection)){
        return false;
    }
    else{
        parseMessage(input);
        return true;
    }
}

bool ClientCommunications::openServerConnections()
{
    if (!yarp.checkNetwork()) {
        yLog.error() << "Yarp network isn't running.";
        return false;
    }
    else{
        bool connected = false;
        double timeDelayed = 0.0;
        double delayTime = 0.1;
        while(!connected && timeDelayed < CONNECTION_TIMEOUT)
        {
            connected = yarp.connect(rpcClientPort_Name.c_str(), "/ControllerServer/rpc:i");
            yarp::os::Time::delay(delayTime);
            timeDelayed += delayTime;
            if (timeDelayed>= CONNECTION_TIMEOUT) {
                yLog.error() << "Could not connect to the ocra controller server. Are you sure it is running?";
            }
        }

        connected = false;
        timeDelayed = 0.0;
        while(!connected && timeDelayed < CONNECTION_TIMEOUT)
        {
            connected = yarp.connect("/ControllerServer:o", inputPort_Name.c_str());
            yarp::os::Time::delay(delayTime);
            timeDelayed += delayTime;
            if (timeDelayed>= CONNECTION_TIMEOUT) {
                yLog.error() << "Could not connect to the ocra controller port. Are you sure it is running?";
            }
        }
        return connected;
    }
}
std::vector<std::string> ClientCommunications::getTaskPortNames()
{
    std::vector<std::string> portNameVec;
    yarp::os::Bottle message, reply;
    message.addInt(GET_TASK_PORT_LIST);
    rpcClientPort.write(message, reply);
    for(auto i=0; i<reply.size(); ++i)
    {
        portNameVec.push_back(reply.get(i).asString());
    }
    return portNameVec;
}

std::string ClientCommunications::getTaskPortName(const std::string& taskName)
{
    yarp::os::Bottle message, reply;
    message.addInt(GET_TASK_PORT_NAME);
    message.addString(taskName);
    rpcClientPort.write(message, reply);

    std::string portName = "";
    if (reply.size()>0) {
        portName = reply.get(0).asString();
    }
    return portName;
}

std::vector<std::string> ClientCommunications::getTaskNames()
{
    std::vector<std::string> nameVec;
    yarp::os::Bottle message, reply;
    message.addInt(GET_TASK_LIST);
    rpcClientPort.write(message, reply);
    for(auto i=0; i<reply.size(); ++i)
    {
        nameVec.push_back(reply.get(i).asString());
    }
    return nameVec;
}

std::shared_ptr<yarp::os::RpcClient> ClientCommunications::getTaskClient(const std::string& taskName)
{
    if(taskRpcClients.find(taskName) != taskRpcClients.end())
    {
        return taskRpcClients[taskName];
    }else{
        //TODO: return a proper null pointer.
    }
}


yarp::os::Bottle ClientCommunications::queryController(yarp::os::Bottle& requestBottle)
{
    yarp::os::Bottle reply;
    rpcClientPort.write(requestBottle, reply);
    return reply;
}

yarp::os::Bottle ClientCommunications::queryController(const SERVER_COMMUNICATIONS_MESSAGE request)
{
    yarp::os::Bottle requestBottle, reply;
    requestBottle.addInt(request);
    rpcClientPort.write(requestBottle, reply);
    return reply;
}

yarp::os::Bottle ClientCommunications::queryController(const std::vector<SERVER_COMMUNICATIONS_MESSAGE> requestVector)
{
    yarp::os::Bottle requestBottle, reply;
    for(auto request : requestVector){
        requestBottle.addInt(request);
    }
    rpcClientPort.write(requestBottle, reply);
    return reply;
}

bool ClientCommunications::openTaskManagerConnections()
{
    std::vector<std::string> taskNames = getTaskNames();
    std::vector<std::string> taskPortNames = getTaskPortNames();

    bool taskConnected = taskNames.size() == taskPortNames.size();

    if(taskConnected)
    {
        for(auto i=0; i<taskPortNames.size(); ++i)
        {
            std::string tmpTaskPortName = "/ControllerClient/" + std::to_string(clientNumber) + "/" + taskNames[i] + ":o";
            taskRpcClients[taskNames[i]] = std::make_shared<yarp::os::RpcClient>();
            taskRpcClients[taskNames[i]]->open(tmpTaskPortName.c_str());
            taskConnected &= yarp.connect(tmpTaskPortName.c_str(), taskPortNames[i].c_str());
        }
    }else{
        yLog.error() << "The number of task ports and names does not match! Can't connect to task RPC ports.";
    }

    return taskConnected;
}

void ClientCommunications::parseMessage(yarp::os::Bottle& input)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        switch (input.get(i).asInt()) {
            case REMOVE_TASK_PORT:
                {
                    ++i;
                    std::cout << "Got message: REMOVE_TASK_PORT - " << input.get(i).asString() << std::endl;
                    close(input.get(i).asString());
                    ++i;
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
