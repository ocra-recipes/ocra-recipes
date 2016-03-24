#include <ocra-recipes/ControllerClient.h>

using namespace ocra_recipes;

int ControllerClient::CONTROLLER_CLIENT_COUNT = 0;

ControllerClient::ControllerClient()
: yarp::os::RateThread(DEFAULT_LOOP_PERIOD)
, isReady(false)
, expectedPeriod(DEFAULT_LOOP_PERIOD)
{

}

ControllerClient::ControllerClient(std::shared_ptr<ocra::Model> derivedModelPtr, const int loopPeriod)
: yarp::os::RateThread(loopPeriod)
, model(derivedModelPtr)
, expectedPeriod(loopPeriod)
{
    clientNumber = ++ControllerClient::CONTROLLER_CLIENT_COUNT;

    clientComs = std::make_shared<ClientCommunications>();
    clientComs->open();

    std::string statesPort_Name = "/ControllerClient/"+ std::to_string(clientNumber) +"/states:i";

    statesPort.open(statesPort_Name.c_str());

    stateCallback = StateListener(model);

    statesPort.setReader(stateCallback);

    yarp.connect("/ControllerServer/states:o", statesPort_Name.c_str());

    isReady = true;
}

ControllerClient::~ControllerClient()
{

}

bool ControllerClient::threadInit()
{
    if(isReady)
        return initialize();
    else
        return false;
}

void ControllerClient::run()
{
    loop();
}

void ControllerClient::threadRelease()
{
    release();
}
