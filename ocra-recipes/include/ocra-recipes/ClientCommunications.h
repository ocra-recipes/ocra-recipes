#ifndef CLIENT_COMMUNICATIONS_H
#define CLIENT_COMMUNICATIONS_H

#include <iostream>
#include <memory>


#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/Time.h>


#include <ocra-recipes/MessageVocabulary.h>

namespace ocra_recipes
{
class ClientCommunications : public yarp::os::PortReader
{
public:
    ClientCommunications();
    virtual ~ClientCommunications();

    bool open(const bool connectToTaskManagers = true);
    bool close();

    virtual bool read(yarp::os::ConnectionReader& connection);
    void parseMessage(yarp::os::Bottle& input);

private:
    bool openServerConnections();
    bool openTaskManagerConnections();

private:
    yarp::os::RpcClient rpcClientPort;
    yarp::os::Port         inputPort;

    std::string     rpcClientPort_Name;
    std::string        inputPort_Name;

    int clientNumber;
    static int CONTROLLER_CLIENT_COUNT;

    yarp::os::Network yarp;
    yarp::os::Log yLog;

    static constexpr double CONNECTION_TIMEOUT = 20.0;


};
} // namespace ocra_recipes
#endif // CLIENT_COMMUNICATIONS_H
