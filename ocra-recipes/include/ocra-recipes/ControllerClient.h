#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include <ocra-recipes/ClientCommunications.h>
#include <ocra/control/Model.h>
#include <ocra-recipes/RobotState.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

namespace ocra_recipes
{

class ControllerClient {

public:
    ControllerClient ();
    ControllerClient (std::shared_ptr<ocra::Model> derivedModelPtr);
    virtual ~ControllerClient ();

private:
    std::shared_ptr<ClientCommunications> clientComs;
    std::shared_ptr<ocra::Model> model;

    yarp::os::Port statesPort;
    StateListener stateCallback;

    static int CONTROLLER_CLIENT_COUNT;
    int clientNumber;

    yarp::os::Network yarp;
};


} // namespace ocra_recipes
#endif // CONTROLLER_CLIENT_H
