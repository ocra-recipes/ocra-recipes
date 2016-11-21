#ifndef SERVER_COMMUNICATIONS_H
#define SERVER_COMMUNICATIONS_H

#include <iostream>
#include <memory>

#include <ocra/util/Macros.h>

#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskBuilders/TaskConstructionManager.h>

#include <ocra/optim/OneLevelSolver.h>
#include <wocra/WocraController.h>

#include <Eigen/Dense>
#include <Eigen/Lgsm>


// TODO: Should put in defines for yarp independent builds
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>


#include <ocra-recipes/MessageVocabulary.h>
#include <ocra/util/ErrorsHelper.h>

namespace ocra_recipes
{
class ServerCommunications : public yarp::os::PortReader
{
    DEFINE_CLASS_POINTER_TYPEDEFS(ServerCommunications)
public:
    ServerCommunications();
    ServerCommunications(ocra::Controller::Ptr ctrl, ocra::Model::Ptr mdl);

    virtual ~ServerCommunications();

    bool open();
    void close();

    virtual bool read(yarp::os::ConnectionReader& connection);
    void parseMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);


private:
    ocra::Model::Ptr                     model;
    ocra::Controller::Ptr           controller;

    yarp::os::RpcServer rpcServerPort;
    yarp::os::Port         outputPort;

    std::string     rpcServerPort_Name;
    std::string        outputPort_Name;
};
} // namespace ocra_recipes
#endif // SERVER_COMMUNICATIONS_H
