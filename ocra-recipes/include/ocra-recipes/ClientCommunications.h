#ifndef CLIENT_COMMUNICATIONS_H
#define CLIENT_COMMUNICATIONS_H

#include <iostream>
#include <memory>
#include <map>

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

#include <ocra/control/TaskManagers/TaskManagerMessageVocab.h>
#include <ocra-recipes/MessageVocabulary.h>

namespace ocra_recipes
{
class ClientCommunications : public yarp::os::PortReader
{
using TaskPortMap = std::map<std::string, std::shared_ptr<yarp::os::RpcClient> >;

public:
    ClientCommunications();
    virtual ~ClientCommunications();

    bool open(const bool connectToTaskManagers = true);
    bool close();
    void close(const std::string& taskName);

    virtual bool read(yarp::os::ConnectionReader& connection);
    void parseMessage(yarp::os::Bottle& input);

    std::vector<std::string> getTaskTypes();


    yarp::os::Bottle queryController(yarp::os::Bottle& requestBottle);
    yarp::os::Bottle queryController(const SERVER_COMMUNICATIONS_MESSAGE request);
    yarp::os::Bottle queryController(const std::vector<SERVER_COMMUNICATIONS_MESSAGE> requestVector);
    // void queryController(const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);

    // void queryTask(const std::string& taskName, const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTask(const int taskIndex, const SERVER_COMMUNICATIONS_MESSAGE request, yarp::os::Bottle& reply);
    // void queryTasks(const SERVER_COMMUNICATIONS_MESSAGE request, std::vector<yarp::os::Bottle&>& replies);
    // void queryTasks(const std::vector<SERVER_COMMUNICATIONS_MESSAGE>& requests, std::vector<yarp::os::Bottle&>& replies);
    // void queryTasks(const std::vector<SERVER_COMMUNICATIONS_MESSAGE>& requests, std::vector<yarp::os::Bottle&>& replies);

    std::vector<std::string> getTaskPortNames();
    std::string getTaskPortName(const std::string& taskName);

    std::vector<std::string> getTaskNames();
    std::shared_ptr<yarp::os::RpcClient> getTaskClient(const std::string& taskName);


    bool parseInput(yarp::os::Bottle& input);


    /*! \class ControlInputCallback
     *  \brief a short description
     *
     *  a long description
     */
    class InputCallback : public yarp::os::PortReader {
    private:
        ClientCommunications& coms;

    public:
        InputCallback(ClientCommunications& comsRef);

        virtual bool read(yarp::os::ConnectionReader& connection);
    };


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

    TaskPortMap taskRpcClients;

    InputCallback inputCallback;


};
} // namespace ocra_recipes
#endif // CLIENT_COMMUNICATIONS_H
