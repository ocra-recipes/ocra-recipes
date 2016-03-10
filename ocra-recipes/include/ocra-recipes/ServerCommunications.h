#ifndef SERVER_COMMUNICATIONS_H
#define SERVER_COMMUNICATIONS_H

#include <iostream>
#include <memory>

#include <ocra/control/Controller.h>
#include <ocra/control/Model.h>
#include <ocra/control/TaskManagers/TaskManagerSet.h>
#include <ocra/control/TaskManagers/TaskParser.h>

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

namespace ocra_recipes
{
class ServerCommunications : public yarp::os::PortReader
{
public:
    ServerCommunications();
    ServerCommunications(std::shared_ptr<ocra::Controller> ctrl, std::shared_ptr<ocra::Model> mdl, std::shared_ptr<TaskManagerSet> tms);

    ServerCommunications(const ServerCommunications& that);
    ServerCommunications& operator=(const ServerCommunications& that);

    virtual ~ServerCommunications();

    bool open();
    bool close();

    virtual bool read(yarp::os::ConnectionReader& connection);
    void parseMessage(yarp::os::Bottle& input, yarp::os::Bottle& reply);


private:
    std::shared_ptr<ocra::Model>                     model;
    std::shared_ptr<ocra::Controller>           controller;
    std::shared_ptr<ocra::TaskManagerSet>   taskManagerSet;

    yarp::os::RpcServer rpcServerPort;
    yarp::os::Port         outputPort;

    std::string     rpcServerPort_Name;
    std::string        outputPort_Name;
};
} // namespace ocra_recipes
#endif // SERVER_COMMUNICATIONS_H
