#ifndef CLIENT_COMMUNICATIONS_H
#define CLIENT_COMMUNICATIONS_H

#include <iostream>
#include <memory>


#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>


#include <ocra-recipes/MessageVocabulary.h>

namespace ocra_recipes
{
class ClientCommunications : public yarp::os::PortReader
{
public:
    ClientCommunications();
    virtual ~ClientCommunications();

    bool open();
    bool close();

    virtual bool read(yarp::os::ConnectionReader& connection);

private:
    yarp::os::RpcServer rpcClientPort;
    yarp::os::Port         inputPort;

    std::string     rpcClientPort_Name;
    std::string        inputPort_Name;
};
} // namespace ocra_recipes
#endif // CLIENT_COMMUNICATIONS_H
