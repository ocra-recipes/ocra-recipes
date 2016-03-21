#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include <ocra-recipes/ClientCommunications.h>

namespace ocra_recipes
{

class ControllerClient {

public:
    ControllerClient ();
    virtual ~ControllerClient ();

private:
    std::shared_ptr<ClientCommunications> clientComs;
    // ClientCommunications clientComs;
};


} // namespace ocra_recipes
#endif // CONTROLLER_CLIENT_H
