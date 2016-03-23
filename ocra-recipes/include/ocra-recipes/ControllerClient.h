#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include <ocra-recipes/ClientCommunications.h>
#include <ocra/control/Model.h>
#include <ocra-recipes/RobotState.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>

namespace ocra_recipes
{

class ControllerClient : public yarp::os::RateThread
{

public:
    ControllerClient ();
    ControllerClient (std::shared_ptr<ocra::Model> derivedModelPtr, const int loopPeriod = DEFAULT_LOOP_PERIOD);
    virtual ~ControllerClient ();

    // RateThread virtual functions
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    virtual void printHelp(){std::cout << "Hey I am the base ControllerClient class!" << std::endl;}

    int getExpectedPeriod(){return expectedPeriod;}

protected:
    virtual bool initialize(){return true;}
    virtual void release(){/* Do nothing. */}
    virtual void loop() = 0;

private:
    std::shared_ptr<ClientCommunications> clientComs;
    std::shared_ptr<ocra::Model> model;

    bool isReady;

    yarp::os::Port statesPort;
    StateListener stateCallback;

    static int CONTROLLER_CLIENT_COUNT;
    int clientNumber;

    yarp::os::Network yarp;

    int expectedPeriod;
    static const int DEFAULT_LOOP_PERIOD = 10; // ms
};


} // namespace ocra_recipes
#endif // CONTROLLER_CLIENT_H
