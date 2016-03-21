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
    }
    return isConOpen;
}
bool ClientCommunications::close()
{
    rpcClientPort.close();
    inputPort.close();
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
bool ClientCommunications::openTaskManagerConnections()
{
    // bool taskConnected = taskNames.size() == taskPortNames.size();
    //
    // if(taskConnected)
    // {
    //     for(auto i=0; i<taskPortNames.size(); ++i)
    //     {
    //         std::string tmpTaskPortName = "/OCRA/" + controllerConnectionName + "/" + taskNames[i] + ":o";
    //         taskRpcClients[taskNames[i]] = std::make_shared<yarp::os::RpcClient>();
    //         taskRpcClients[taskNames[i]]->open(tmpTaskPortName.c_str());
    //         taskConnected &= yarp.connect(tmpTaskPortName.c_str(), taskPortNames[i].c_str());
    //     }
    // }else{
    //     yLog.error() << "The number of task ports and names does not match! Can't connect to task RPC ports.";
    // }
    //
    // return taskConnected;
    return true;
}


void ClientCommunications::parseMessage(yarp::os::Bottle& input)
{
    int btlSize = input.size();
    for (int i=0; i<btlSize;)
    {
        switch (input.get(i).asInt()) {

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
