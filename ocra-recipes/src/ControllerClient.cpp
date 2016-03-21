#include <ocra-recipes/ControllerClient.h>

using namespace ocra_recipes;

ControllerClient::ControllerClient()
{
    std::cout << "Hey" << std::endl;

    clientComs = std::make_shared<ClientCommunications>();
    clientComs->open();
}

ControllerClient::~ControllerClient()
{

}
